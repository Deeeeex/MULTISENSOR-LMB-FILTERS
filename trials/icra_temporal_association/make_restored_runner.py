"""Restore the frozen replay domain and dispatch unchanged V1/V2 matchers."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
source = OUT / 'runPersistentReplay.m'
changes = [
    ('function runPersistentReplay(configPath,unitIndex)',
     'function runRestoredAssociation(configPath,unitIndex)'),
    ('checkPersistentAssociation();',
     """checkPersistentAssociation();
% The unit check adds common/ before the replay adapter. Restore that adapter.
addpath(qualityPath,'-begin');addpath(runtimePath,'-begin');
assert(strcmp(which('evaluateSensorQuality'),fullfile(qualityPath,'evaluateSensorQuality.m')));
assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));"""),
    ("'protocol','icra-persistent-association-v2'", "'protocol','icra-restored-association-v1'"),
    ("if endsWith(rule,'_assoc_reopen')", "if endsWith(rule,'_assoc_direct'),associationMode='direct';rule=erase(rule,'_assoc_direct');\nelseif endsWith(rule,'_assoc_temporal'),associationMode='temporal';rule=erase(rule,'_assoc_temporal');\nelseif endsWith(rule,'_assoc_reopen')"),
    ('                [remote,matching,associationMemory{n}]=alignPersistentLmbPair(local{n},decoded{other},50,associationMemory{n},associationMode,t,n);',
     """                if ismember(associationMode,{'direct','temporal'})
                    [remote,matching,associationMemory{n}.snapshots]=alignObservationLmbPair(local{n},decoded{other},50,associationMemory{n}.snapshots,associationMode,t);
                    matching.remoteOnlyLabels=zeros(2,0);matching.splitRecords=zeros(0,4);matching.conflictState=zeros(2,0);
                else
                    [remote,matching,associationMemory{n}]=alignPersistentLmbPair(local{n},decoded{other},50,associationMemory{n},associationMode,t,n);
                end""")]
text = source.read_text()
for old, new in changes:
    assert text.count(old) == 1, old
    text = text.replace(old, new)
destination = OUT / 'runRestoredAssociation.m'
assert not destination.exists()
destination.write_text(text)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(OUT / 'RESTORED_RUNNER_PATCH.json').write_text(json.dumps(dict(
    source=str(source.relative_to(ROOT)), source_sha256=sha(source),
    destination=str(destination.relative_to(ROOT)), destination_sha256=sha(destination),
    bounded_replacements=changes), indent=2) + '\n')
