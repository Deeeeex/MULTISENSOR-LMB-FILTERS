"""Create an isolated native runner with a single registered fusion splice."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    source=OUT.parent/'icra_range_detection/runRangeDetectionReplay.m'
    target=OUT/'runRecursionIntervention.m';receipt=OUT/'RUNNER_PATCH.json'
    assert not target.exists() and not receipt.exists()
    text=source.read_text();changes=[]
    def replace(old,new):
        nonlocal text
        assert text.count(old)==1,old
        text=text.replace(old,new);changes.append(dict(before=old,after=new))
    replace('function runRangeDetectionReplay(configPath,unitIndex)','function runRecursionIntervention(configPath,unitIndex)')
    replace("addpath(fullfile(root,'trials','icra_temporal_association'));", "addpath(fullfile(root,'trials','icra_temporal_association'));\naddpath(fullfile(root,'trials','icra_range_detection'));\naddpath(out);")
    replace("qualityPath=fullfile(out,'range_quality');", "qualityPath=fullfile(root,'trials','icra_range_detection','range_quality');")
    replace('checkRangeDetection(baseQuality);','checkRangeDetection(baseQuality);checkRecursionIntervention();')
    replace('model.rangeDetection=unit.range_detection_model;', 'model.rangeDetection=unit.range_detection_model;\nmodel.recursionInterventionFrame=unit.intervention_frame;')
    replace("for condition={'reliable','intermittent'}", "runArms=unit.arms;\nfor condition=reshape(config.conditions,1,[])")
    replace('for a=1:numel(config.arms)\n        arm=config.arms{a};', 'for a=1:numel(runArms)\n        arm=runArms{a};')
    replace("'protocol','icra-range-detection-v1'", "'protocol','icra-recursion-intervention-v1'")
    replace("associationMode='';", """interventionMode='none';
for mode={'joint','existence','spatial'}
    suffix=['_once_',mode{1}];
    if endsWith(rule,suffix),interventionMode=mode{1};rule=erase(rule,suffix);end
end
assert(strcmp(interventionMode,'none') || strcmp(rule,'gaussian_evidence_guarded_scalar'));
associationMode='';""")
    replace('run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);',
            "run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);\nrun.interventionMode=interventionMode;run.interventionFrame=model.recursionInterventionFrame;\nrun.interventionRecords=zeros(0,54);run.interventionMask=false(2,T);")
    replace('[posterior{n},stats]=fuseReviewerEvidence(inputs,[.5,.5],model,details,cfg,rule,t);',
            """[posterior{n},stats]=fuseReviewerEvidence(inputs,[.5,.5],model,details,cfg,rule,t);
                if t==model.recursionInterventionFrame && ~strcmp(interventionMode,'none')
                    [joint,jointStats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,'gaussian_evidence',t);
                    [posterior{n},stats,intervention]=spliceRecursionIntervention( ...
                        posterior{n},stats,joint,jointStats,interventionMode,t,n);
                    run.interventionRecords=[run.interventionRecords;intervention]; %#ok<AGROW>
                    run.interventionMask(n,t)=true;
                end""")
    target.write_text(text)
    receipt.write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
        target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=changes),indent=2)+'\n')
    print('ISOLATED INTERVENTION RUNNER',len(changes),'bounded replacements',flush=True)


if __name__=='__main__':main()
