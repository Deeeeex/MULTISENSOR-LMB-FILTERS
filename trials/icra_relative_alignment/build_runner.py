"""Port only the input boundary and output provenance of the native runner."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    original=OUT.parent/'icra_reviewer_revision/runReviewerReplay.m'
    destination=OUT/'runAlignmentReplay.m'
    assert not destination.exists()
    text=original.read_text();changes=[]
    def replace(before,after):
        nonlocal text
        assert text.count(before)==1,before
        text=text.replace(before,after);changes.append(dict(before=before,after=after))
    replace('function runReviewerReplay(configPath,unitIndex)','function runAlignmentReplay(configPath,unitIndex)')
    replace("config=jsondecode(fileread(configPath));unit=config.units(unitIndex);",
            "addpath(fullfile(root,'trials','icra_reviewer_revision'));\nconfig=jsondecode(fileread(configPath));\nif iscell(config.units),unit=config.units{unitIndex};else,unit=config.units(unitIndex);end")
    replace('model=makeModel(positions,T,baseQuality,config.pd);',
'''alignment=zeros(2,T);gridSeconds=zeros(2,T);solveSeconds=zeros(2,T);
if strcmp(config.alignment_mode,'occupancy')
    prepared=load(fullfile(root,unit.alignment_path));
    alignment=prepared.translations;gridSeconds=prepared.gridSeconds;solveSeconds=prepared.solveSeconds;
else
    assert(strcmp(config.alignment_mode,'zero'));
end
assert(isequal(size(alignment),[2,T]) && all(isfinite(alignment),'all'));
modelPositions=positions;
for t=1:T
    measurements{2,t}=measurements{2,t}+alignment(:,t);
    modelPositions(:,2,t)=modelPositions(:,2,t)+alignment(:,t);
end
% Evaluation positions remain the original benchmark crop.
model=makeModel(modelPositions,T,baseQuality,config.pd);''')
    replace("for condition={'reliable','intermittent'}","for condition=reshape(config.conditions,1,[])")
    replace('run=scoreOutputs(run,data.truth);',
'''run=scoreOutputs(run,data.truth);
        extra=double(strcmp(config.alignment_mode,'occupancy'));
        run.alignmentPayloadBytes=repmat(extra*8064,1,T);
        run.alignmentWireBytes=repmat(extra*33024,1,T);
        run.totalWithAlignmentWireBytes=run.totalWireBytes+sum(run.alignmentWireBytes);
        run.alignmentComputeSeconds=sum(gridSeconds,'all')+sum(solveSeconds,'all');''')
    replace("'protocol','icra-reviewer-revision-v1'","'protocol','icra-relative-alignment-v1'")
    replace("'pd',config.pd,'time',double(data.time),'positions',positions, ...",
            "'pd',config.pd,'time',double(data.time),'positions',positions, ...\n            'modelPositions',modelPositions,'measurements',{measurements},'alignmentTranslations',alignment, ...\n            'alignmentMode',config.alignment_mode, ...")
    original_body=original.read_text().split('function model=makeModel',1)[1]
    assert text.split('function model=makeModel',1)[1]==original_body
    destination.write_text(text)
    report=dict(passed=True,original=str(original.relative_to(ROOT)),original_sha256=sha(original),
                result=str(destination.relative_to(ROOT)),result_sha256=sha(destination),changes=changes,
                all_local_update_fusion_reduction_extraction_and_scoring_functions_byte_identical=True)
    (OUT/'RUNNER_PORT.json').write_text(json.dumps(report,indent=2)+'\n')
    print('ALIGNMENT RUNNER PORT VERIFIED',len(changes),'boundary substitutions',flush=True)

if __name__=='__main__':main()
