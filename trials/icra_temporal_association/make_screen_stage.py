"""Create the V3 registration and audit entry points without altering V1/V2."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
changes = []


def port(old_name, new_name, edits):
    old, new = OUT / old_name, OUT / new_name
    assert not new.exists()
    value = old.read_text()
    for before, after in edits:
        assert value.count(before) == 1, (new_name, before, value.count(before))
        value = value.replace(before, after)
    new.write_text(value)
    changes.append(dict(source=old_name, source_sha256=sha(old), destination=new_name,
        destination_sha256=sha(new), replacements=[dict(before=a, after=b) for a, b in edits]))


port('register_restored_stage.py', 'register_screen_stage.py', [
    ("'PATH_RESTORATION.md', 'RESTORED_RUNNER_PATCH.json', 'PATH_FAILURE.json'",
     "'PATH_RESTORATION.md', 'RESTORED_RUNNER_PATCH.json', 'PATH_FAILURE.json',\n        'CANDIDATES_V3.md', 'SCREENED_RUNNER_PATCH.json', 'SCREENED_AUDIT_MATH_PATCH.json',\n        'SCREENED_STAGE_PATCH.json', 'SPLIT_EVENT_DIAGNOSTIC.json', 'COLUMN_SHAPE_PATCH.json'"),
    ("['direct', 'temporal', 'reopen', 'split']", "['quality', 'nis', 'quality_nis']"),
    ("assert sequences == ['0001', '0008']\n        arms.insert(0, 'marked_gaussian_evidence')",
     "assert sequences == ['0001', '0006', '0007']\n        arms[:0] = ['marked_gaussian_evidence', 'marked_gaussian_evidence_assoc_split']"),
    ("protocol='icra-restored-association-v1'", "protocol='icra-screened-association-v1'")
])

old = (OUT / 'audit_restored_stage.py').read_text()
parity_start = old.index("                if mode in ['reopen', 'split'] and seq == '0008':")
parity_end = old.index("                hashes[str(path.relative_to(ROOT))] = sha(path)", parity_start)
old_parity = old[parity_start:parity_end]
new_parity = '''                if mode == 'split':
                    prior_stage = 'association_restored_preflight' if seq == '0001' else 'association_restored_development_rest'
                    prior_audit = json.loads((OUT / ('audit_' + prior_stage + '.json')).read_text())
                    assert prior_audit['passed']
                    oldpath = OUT / 'results' / prior_stage / path.name
                    assert sha(oldpath) == prior_audit['inputs'][str(oldpath.relative_to(ROOT))]
                    old_run = read(oldpath)['runs']
                    keys = [k for k in old_run if k != 'runtimeSeconds']
                    for key in keys:
                        assert run[key] == old_run[key], (seq, condition, arm, key, 'exact restored S recursion')
                    parity.append(dict(sequence=seq, condition=condition, arm=arm,
                        exact_fields=keys, robot_frames=2*T))
                    hashes[str(oldpath.relative_to(ROOT))] = sha(oldpath)
'''
port('audit_restored_stage.py', 'audit_screen_stage.py', [
    ('from audit_observation_domain import audit_domain',
     'from audit_observation_domain import audit_domain\nfrom audit_screened_run import associations as screened_associations'),
    ("run['arm'].replace('_assoc_reopen', '').replace('_assoc_split', '').replace('_assoc_direct', '').replace('_assoc_temporal', '')",
     "run['arm'].split('_assoc_', 1)[0]"),
    ("assert 'PERSISTENT ASSOCIATION CHECK PASSED' in content",
     "assert 'SCREENED ASSOCIATION CHECK PASSED' in content"),
    ("'icra-restored-association-v1'", "'icra-screened-association-v1'"),
    ("                if mode in ['reopen', 'split']:\n                    matching = persistent_associations(run, data, frames, mode)",
     "                if mode in ['quality', 'nis', 'quality_nis']:\n                    matching = screened_associations(run, data, frames, mode)\n                elif mode == 'split':\n                    matching = persistent_associations(run, data, frames, mode)"),
    (old_parity, new_parity),
    ("OUT / 'audit_observation_domain.py', OUT / 'persistent_math.py'",
     "OUT / 'screen_observation_math.py', OUT / 'screen_persistent_math.py', OUT / 'audit_screened_run.py',\n            OUT / 'audit_observation_domain.py', OUT / 'persistent_math.py'")
])

destination = OUT / 'SCREENED_STAGE_PATCH.json'
assert not destination.exists()
destination.write_text(json.dumps(dict(changes=changes, generator_sha256=sha(Path(__file__))), indent=2) + '\n')
print('SCREENED STAGE ENTRY POINTS CREATED')
