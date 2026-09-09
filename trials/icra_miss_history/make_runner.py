"""Bounded instrumentation of the previously verified replay and audit."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=OUT.parent/'icra_temporal_association/runScreenedAssociation.m'
text=source.read_text()
changes=[
 ('function runScreenedAssociation(configPath,unitIndex)', 'function runMissHistoryReplay(configPath,unitIndex)'),
 ("config=jsondecode(fileread(configPath));", "addpath(fullfile(root,'trials','icra_temporal_association'));\nconfig=jsondecode(fileread(configPath));"),
 ("checkPersistentAssociation();checkColumnAssociation();checkScreenedAssociation();", "checkPersistentAssociation();checkColumnAssociation();checkScreenedAssociation();checkMissHistory();"),
 ("'protocol','icra-screened-association-v1'", "'protocol','icra-miss-history-v1'"),
 ("marked=startsWith(arm,'marked_');rule=erase(arm,'marked_');", "marked=startsWith(arm,'marked_');rule=erase(arm,'marked_');\nmissMode='original';\nif endsWith(rule,'_miss_history'),missMode='history';rule=erase(rule,'_miss_history');\nelseif endsWith(rule,'_miss_half'),missMode='half';rule=erase(rule,'_miss_half');end"),
 ("run=emptyRun(arm,T);cfg=buildMixtureAwareKlaReferenceConfig();", "run=emptyRun(arm,T);run.negativeHistoryMode=missMode;run.negativeHistoryRecords=zeros(0,12);\nmissState={containers.Map('KeyType','char','ValueType','any'),containers.Map('KeyType','char','ValueType','any')};\ncfg=buildMixtureAwareKlaReferenceConfig();"),
 ("        negativeSupport=negativeInnovationSupport(associationMass,predictedPd);", """        missBefore=zeros(size(associationMass));
        for j=1:numel(local{n})
            labelKey=key(local{n}(j));
            if isKey(missState{n},labelKey)
                old=missState{n}(labelKey);
                if old(1)==t-1,missBefore(j)=old(2);end
            end
        end
        nominalNegative=negativeInnovationSupport(associationMass,predictedPd);
        [negativeSupport,missAfter,effectivePd,missDiscount]=missHistorySupport(associationMass,predictedPd,missBefore,missMode);
        for j=1:numel(local{n})
            o=local{n}(j);missState{n}(key(o))=[t,missAfter(j)];
            run.negativeHistoryRecords(end+1,:)=[t,n,o.birthTime,o.birthLocation,missBefore(j), ...
                associationMass(j),predictedPd(j),effectivePd(j),missDiscount(j),missAfter(j), ...
                nominalNegative(j),negativeSupport(j)]; %#ok<AGROW>
        end"""),
]
for before,after in changes:
    assert text.count(before)==1,before
    text=text.replace(before,after)
destination=OUT/'runMissHistoryReplay.m'
assert not destination.exists()
destination.write_text(text)
audit_source=OUT.parent/'icra_reviewer_revision/review_probability_audit.py'
audit=audit_source.read_text()
audit_changes=[('from review_gaussian_audit import check_gaussians', 'from review_gaussian_audit import check_gaussians\nfrom history_audit import audit_history'),
               ('    expected_negative = (1-local[:, 9])*pd/(2-pd)', '    expected_negative = audit_history(run, data)')]
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
print('GENERATED MISS HISTORY REPLAY AND PROBABILITY AUDIT')

