"""Bounded copy of the unchanged recursive replay with observation-model logs."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=OUT.parent/'icra_temporal_association/runScreenedAssociation.m'
text=source.read_text()
changes=[
 ('function runScreenedAssociation(configPath,unitIndex)','function runRangeDetectionReplay(configPath,unitIndex)'),
 ('config=jsondecode(fileread(configPath));',"addpath(fullfile(root,'trials','icra_temporal_association'));\nconfig=jsondecode(fileread(configPath));"),
 ("qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);","qualityPath=fullfile(out,'range_quality');addpath(qualityPath);"),
 ("assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));\ndata=load", "assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));\ncheckRangeDetection(baseQuality);\ndata=load"),
 ('model=makeModel(positions,T,baseQuality,config.pd);','model=makeModel(positions,T,baseQuality,config.pd);\nmodel.rangeDetection=unit.range_detection_model;'),
 ("'protocol','icra-screened-association-v1'","'protocol','icra-range-detection-v1'"),
 ("marked=startsWith(arm,'marked_');rule=erase(arm,'marked_');", "marked=startsWith(arm,'marked_');rule=erase(arm,'marked_');\nmodel.rangeDetectionMode='nominal';\nif endsWith(rule,'_range'),model.rangeDetectionMode='range';rule=erase(rule,'_range');\nelseif endsWith(rule,'_constant'),model.rangeDetectionMode='constant';rule=erase(rule,'_constant');end"),
 ("run=emptyRun(arm,T);cfg=buildMixtureAwareKlaReferenceConfig();", "run=emptyRun(arm,T);run.rangeDetectionMode=model.rangeDetectionMode;\nrun.rangeDetectionModel=model.rangeDetection;run.qualityRecords=zeros(0,10);\nrun.fusionSourceRecords=zeros(0,8);run.fusionOutputRecords=zeros(0,19);\ncfg=buildMixtureAwareKlaReferenceConfig();"),
 ('o=predicted(j);pd=evaluateSensorQuality(model,n,o.mu{1},t);predictedPd(j)=pd;', "o=predicted(j);[pd,~,quality]=evaluateSensorQuality(model,n,o.mu{1},t);predictedPd(j)=pd;\n            run.qualityRecords(end+1,:)=[t,n,o.birthTime,o.birthLocation,o.mu{1}',quality.range,pd]; %#ok<AGROW>"),
 ('            run.iterationRecords=[run.iterationRecords;stats.records];', """            for k=1:size(stats.records,1)
                label=stats.records(k,3:4);originals=zeros(1,4);
                for s=1:2
                    index=find([inputs{s}.birthTime]==label(1) & [inputs{s}.birthLocation]==label(2));
                    assert(numel(index)<=1);
                    if ~isempty(index),originals(2*s-1:2*s)=details.originalLabels{s}(:,index)';end
                end
                if startsWith(rule,'gaussian_evidence'),assert(isequal(originals,stats.records(k,32:35)));end
                run.fusionSourceRecords(end+1,:)=[t,n,label,originals]; %#ok<AGROW>
                index=find([posterior{n}.birthTime]==label(1) & [posterior{n}.birthLocation]==label(2));
                assert(numel(index)==1);o=posterior{n}(index);assert(o.numberOfGmComponents==1);
                covariance=o.Sigma{1};lower=find(tril(true(4)));
                run.fusionOutputRecords(end+1,:)=[t,n,label,o.r,o.mu{1}',covariance(lower)']; %#ok<AGROW>
            end
            run.iterationRecords=[run.iterationRecords;stats.records];"""),
]
for before,after in changes:
    assert text.count(before)==1,before
    text=text.replace(before,after)
destination=OUT/'runRangeDetectionReplay.m'
assert not destination.exists()
destination.write_text(text)
audit_source=OUT.parent/'icra_reviewer_revision/review_probability_audit.py'
audit=audit_source.read_text()
audit_changes=[
 ('from review_gaussian_audit import check_gaussians','from review_gaussian_audit import check_gaussians\nfrom range_audit import audit_actual_pd'),
 ("    assert np.isin(pd, [0, data['pd']]).all() and np.array_equal(pd > 0, local[:, 7].astype(bool))", "    audit_actual_pd(run, data)\n    assert np.array_equal(pd > 0, local[:, 7].astype(bool))"),
]
for before,after in audit_changes:
    assert audit.count(before)==1,before
    audit=audit.replace(before,after)
audit_destination=OUT/'probability_audit.py'
assert not audit_destination.exists()
audit_destination.write_text(audit)
receipt=dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
    destination=str(destination.relative_to(ROOT)),destination_sha256=sha(destination),replacements=changes,
    audit_source=str(audit_source.relative_to(ROOT)),audit_source_sha256=sha(audit_source),
    audit_destination=str(audit_destination.relative_to(ROOT)),audit_destination_sha256=sha(audit_destination),audit_replacements=audit_changes)
(OUT/'RUNNER_PATCH.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('GENERATED RANGE REPLAY AND PROBABILITY AUDIT')
