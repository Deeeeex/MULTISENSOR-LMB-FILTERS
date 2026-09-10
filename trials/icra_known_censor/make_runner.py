"""Port the verified recorder with current local scalar caching and one hook."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT.parent/'icra_range_detection/runRangeDetectionReplay.m';target=OUT/'runKnownCensorReplay.m'
    assert not target.exists()
    changes=[
        ('function runRangeDetectionReplay(configPath,unitIndex)','function runKnownCensorReplay(configPath,unitIndex)'),
        ("addpath(fullfile(root,'trials','icra_temporal_association'));", "addpath(fullfile(root,'trials','icra_temporal_association'));\naddpath(fullfile(root,'trials','icra_range_detection'));addpath(out);"),
        ("qualityPath=fullfile(out,'range_quality');addpath(qualityPath);", "qualityPath=fullfile(root,'trials','icra_range_detection','range_quality');addpath(qualityPath);"),
        ('checkRangeDetection(baseQuality);','checkRangeDetection(baseQuality);checkKnownCensor();'),
        ("for a=1:numel(config.arms)\n        arm=config.arms{a};","for a=1:numel(unit.arms)\n        arm=unit.arms{a};"),
        ("for condition={'reliable','intermittent'}","for condition=reshape(unit.conditions,1,[])"),
        ("'protocol','icra-range-detection-v1'","'protocol','icra-known-censor-v1'"),
        ("associationMode='';", "knownCensorEnabled=endsWith(rule,'_known_censor');\nif knownCensorEnabled,rule=erase(rule,'_known_censor');end\nassert(ismember(rule,{'lineage','gaussian_evidence','gaussian_evidence_guarded_scalar'}));\nassociationMode='';"),
        ("run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);", "run.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);\nrun.knownCensorEnabled=knownCensorEnabled;run.knownCensorRecords=zeros(0,25);"),
        ('    local=cell(1,2);','    local=cell(1,2);knownLocal=cell(1,2);'),
        ('        local{n}=reduce(local{n},model);', '        knownLocal{n}=zeros(numel(local{n}),6);\n        for j=1:numel(local{n})\n            o=local{n}(j);knownLocal{n}(j,:)=[o.birthTime,o.birthLocation,predicted(j).r,o.r,o.lastDirectOpportunity==t,predictedPd(j)];\n        end\n        local{n}=reduce(local{n},model);'),
        ('            for k=1:size(stats.records,1)', '            if knownCensorEnabled\n                [posterior{n},stats,events]=refineKnownCensor(posterior{n},stats,inputs,knownLocal{n},rule,t,n);\n                run.knownCensorRecords=[run.knownCensorRecords;events]; %#ok<AGROW>\n            end\n            for k=1:size(stats.records,1)'),
    ]
    content=source.read_text()
    for before,after in changes:assert content.count(before)==1,before;content=content.replace(before,after)
    target.write_text(content)
    (OUT/'RUNNER_PATCH.json').write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
        target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=[dict(before=a,after=b) for a,b in changes]),indent=2)+'\n')
    print('KNOWN CENSOR RUNNER GENERATED',len(changes),flush=True)

if __name__=='__main__':main()
