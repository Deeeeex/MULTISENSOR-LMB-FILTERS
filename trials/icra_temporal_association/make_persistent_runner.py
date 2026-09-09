"""Bounded V2 adaptation; every V1 artifact remains immutable."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
changes = [
    ('function runAssociationReplay(configPath,unitIndex)',
     'function runPersistentReplay(configPath,unitIndex)'),
    ('checkObservationAssociation();', 'checkPersistentAssociation();'),
    ("'protocol','icra-reviewer-revision-v1'", "'protocol','icra-persistent-association-v2'"),
    ("'associationReopenings',zeros(0,8),'iterationRecords'",
     "'associationReopenings',zeros(0,8),'associationRemoteAbstentions',zeros(0,4),'associationSplits',zeros(0,6),'associationConflicts',zeros(0,4),'iterationRecords'"),
    ("if endsWith(rule,'_assoc_direct'),associationMode='direct';rule=erase(rule,'_assoc_direct');\nelseif endsWith(rule,'_assoc_temporal'),associationMode='temporal';rule=erase(rule,'_assoc_temporal');end",
     "if endsWith(rule,'_assoc_reopen'),associationMode='reopen';rule=erase(rule,'_assoc_reopen');\nelseif endsWith(rule,'_assoc_split'),associationMode='split';rule=erase(rule,'_assoc_split');end"),
    ('associationMemory={emptyHistory,emptyHistory};',
     "emptyState=struct('snapshots',emptyHistory,'blocked',zeros(2,0));\nassociationMemory={emptyState,emptyState};"),
    ('alignObservationLmbPair(local{n},decoded{other},50,associationMemory{n},associationMode,t)',
     'alignPersistentLmbPair(local{n},decoded{other},50,associationMemory{n},associationMode,t,n)'),
    ("                for k=1:size(matching.abstainLabels,2)",
     """                if ~isempty(matching.conflictState)
                    run.associationConflicts=[run.associationConflicts;repmat([t,n],size(matching.conflictState,2),1),matching.conflictState']; %#ok<AGROW>
                end
                if ~isempty(matching.splitRecords)
                    run.associationSplits=[run.associationSplits;repmat([t,n],size(matching.splitRecords,1),1),matching.splitRecords]; %#ok<AGROW>
                end
                for k=1:size(matching.abstainLabels,2)"""),
    ("                    run.associationAbstentions(end+1,:)=[t,n,label']; %#ok<AGROW>\n                end",
     """                    run.associationAbstentions(end+1,:)=[t,n,label']; %#ok<AGROW>
                end
                for k=1:size(matching.remoteOnlyLabels,2)
                    label=matching.remoteOnlyLabels(:,k);index=size(matching.abstainLabels,2)+k;
                    details.labelSpecificWeightOverrides(index)=struct('label',label,'sourceInputIndex',2, ...
                        'replacedInputIndex',1,'mode','dominant-nonself-transfer');
                    run.associationRemoteAbstentions(end+1,:)=[t,n,label']; %#ok<AGROW>
                end""")]
source = OUT / 'runAssociationReplay.m'
text = source.read_text()
for old, new in changes:
    assert text.count(old) == 1, old
    text = text.replace(old, new)
destination = OUT / 'runPersistentReplay.m'
assert not destination.exists()
destination.write_text(text)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(OUT / 'PERSISTENT_RUNNER_PATCH.json').write_text(json.dumps(dict(
    source=str(source.relative_to(ROOT)), source_sha256=sha(source),
    destination=str(destination.relative_to(ROOT)), destination_sha256=sha(destination),
    bounded_replacements=changes), indent=2) + '\n')
