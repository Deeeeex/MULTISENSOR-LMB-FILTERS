"""Make a bounded nominal replay from the complete verified range recorder."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT.parent/'icra_range_detection/runRangeDetectionReplay.m';destination=OUT/'runNominalOriginReplay.m'
    assert not destination.exists()
    changes=[
        ('function runRangeDetectionReplay(configPath,unitIndex)','function runNominalOriginReplay(configPath,unitIndex)'),
        ("addpath(fullfile(root,'trials','icra_temporal_association'));", "addpath(fullfile(root,'trials','icra_temporal_association'));\naddpath(fullfile(root,'trials','icra_range_detection'));addpath(out);"),
        ("qualityPath=fullfile(out,'range_quality');addpath(qualityPath);", "qualityPath=fullfile(root,'trials','icra_range_detection','range_quality');addpath(qualityPath);"),
        ('checkRangeDetection(baseQuality);','checkRangeDetection(baseQuality);checkInitialNegative();'),
        ('model.rangeDetection=unit.range_detection_model;','model.rangeDetection=unit.range_detection_model;\nmodel.initialNegativeFrame=unit.intervention_frame;'),
        ("for a=1:numel(config.arms)\n        arm=config.arms{a};","for a=1:numel(unit.arms)\n        arm=unit.arms{a};"),
        ("for condition={'reliable','intermittent'}","for condition=reshape(unit.conditions,1,[])"),
        ("'protocol','icra-range-detection-v1'","'protocol','icra-nominal-origin-intervention-v1'"),
        ("associationMode='';", "initialNegativeEnabled=endsWith(rule,'_once_initial_negative');\nif initialNegativeEnabled,rule=erase(rule,'_once_initial_negative');end\nassert(~initialNegativeEnabled || strcmp(rule,'gaussian_evidence'));\nassociationMode='';"),
        ("run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);", "run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);\nrun.initialNegativeEnabled=initialNegativeEnabled;run.initialNegativeFrame=model.initialNegativeFrame;\nrun.initialNegativeRecords=zeros(0,18);run.initialNegativeMask=false(2,T);"),
        ("                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);", "                [posterior{n},stats]=fuseGaussianEvidence(inputs,[.5,.5],model,details,cfg,rule,t);\n                if initialNegativeEnabled && t==model.initialNegativeFrame\n                    [posterior{n},stats,event]=removeInitialNegative(posterior{n},stats,t,n);\n                    run.initialNegativeRecords=[run.initialNegativeRecords;event]; %#ok<AGROW>\n                    run.initialNegativeMask(n,t)=true;\n                end"),
    ]
    content=source.read_text()
    for before,after in changes:
        assert content.count(before)==1,before;content=content.replace(before,after)
    destination.write_text(content)
    receipt=dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),target=str(destination.relative_to(ROOT)),target_sha256=sha(destination),
        changes=[dict(before=a,after=b) for a,b in changes])
    (OUT/'RUNNER_PATCH.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('NOMINAL EVENT RUNNER GENERATED',len(changes),'exact replacements',flush=True)

if __name__=='__main__':main()
