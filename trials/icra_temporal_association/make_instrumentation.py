"""Create an isolated, observation-only extension of the frozen replay."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
source = OUT.parent / 'icra_reviewer_revision/runReviewerReplay.m'
text = source.read_text()
changes = [
    ('function runReviewerReplay(configPath,unitIndex)',
     'function recordAssociationReplay(configPath,unitIndex)'),
    ("original=fullfile(root,'trials','icra_external_fusion');",
     "original=fullfile(root,'trials','icra_external_fusion');\naddpath(fullfile(root,'trials','icra_reviewer_revision'));"),
    ('checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();',
     'checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();checkDirectObservationSummary();'),
    ("'packetGaussianRecords',zeros(0,19),'iterationRecords',zeros(0,60)",
     "'packetGaussianRecords',zeros(0,19),'localAssociationWeights',{cell(2,T)},'localDirectRecords',zeros(0,12),'iterationRecords',zeros(0,60)"),
    ('        predictedPd=reshape(predictedPd,size(opportunity));',
     "        run.localAssociationWeights{n,t}=W;\n        directValues=directObservationSummary(W,measurements{n,t},model.Q{n},opportunity);\n        predictedPd=reshape(predictedPd,size(opportunity));"),
    ('                before=predicted(indices(j));after=local{n}(j);',
     "                before=predicted(indices(j));after=local{n}(j);\n                run.localDirectRecords(end+1,:)=[t,n,after.birthTime,after.birthLocation,directValues(indices(j),:)]; %#ok<AGROW>")]
for old, new in changes:
    assert text.count(old) == 1, old
    text = text.replace(old, new)
destination = OUT / 'recordAssociationReplay.m'
assert not destination.exists()
destination.write_text(text)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(OUT / 'INSTRUMENTATION_PATCH.json').write_text(json.dumps(dict(
    source=str(source.relative_to(ROOT)), source_sha256=sha(source),
    destination=str(destination.relative_to(ROOT)), destination_sha256=sha(destination),
    bounded_replacements=changes, purpose='Record current LBP weights and detection geometry without changing the posterior, packet or matching.'), indent=2) + '\n')
