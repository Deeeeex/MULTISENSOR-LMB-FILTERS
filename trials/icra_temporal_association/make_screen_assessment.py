"""Port the completed independent audit to V3 and the original No-age control."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
old=OUT/'audit_restored_assessment.py'
new=OUT/'audit_screen_assessment.py'
assert not new.exists()
value=old.read_text()
changes=[
    ('from audit_observation_domain import audit_domain',
     'from audit_observation_domain import audit_domain\nfrom audit_screened_run import associations as screened_associations\nfrom analyze_holdout import audit_existence\nfrom analyze_full import baseline'),
    ('width = 416 if mode else 352',"width = 416 if mode else (352 if 'gaussian_evidence' in run['arm'] else 216)"),
    ("run['arm'].replace('_assoc_reopen', '').replace('_assoc_split', '').replace('_assoc_direct', '').replace('_assoc_temporal', '')",
     "run['arm'].split('_assoc_', 1)[0]"),
    ("base_view['packetBytes'] = (32 + 352 * counts).tolist()",
     "base_width = 352 if 'gaussian_evidence' in base_view['arm'] else 216\n    base_view['packetBytes'] = (32 + base_width * counts).tolist()"),
    ("assert cfg['cohort'].startswith('association_assessment_')\n    assert sha(OUT / 'RESTORED_DEVELOPMENT_SELECTION.json') == cfg['selection_sha256']",
     "assert cfg['cohort'].startswith('association_screen_assessment_')\n    if not cfg['preflight']:\n        assert sha(OUT / 'SCREENED_DEVELOPMENT_SELECTION.json') == cfg['selection_sha256']"),
    ("assert 'PERSISTENT ASSOCIATION CHECK PASSED' in content","assert 'SCREENED ASSOCIATION CHECK PASSED' in content"),
    ("'icra-restored-association-v1'","'icra-screened-association-v1'"),
    ('                density = audit_probability(base_view, data)',
     "                if arm == 'marked_lineage':\n                    short_view = dict(base_view, iterationRecords=np.asarray(run['iterationRecords'], float).reshape(-1, 60)[:, :26].tolist())\n                    density = audit_existence(short_view, data, 'marked_lineage')\n                else:\n                    density = audit_probability(base_view, data)"),
    ("                if mode in ['reopen', 'split']:\n                    matching = persistent_associations(run, data, frames, mode)",
     "                if mode in ['quality', 'nis', 'quality_nis']:\n                    matching = screened_associations(run, data, frames, mode)"),
    ("                assert mode in ['direct', 'temporal', 'reopen', 'split']",
     """                assert mode in ['', 'quality', 'nis', 'quality_nis']
                if cfg['preflight']:
                    assert arm == 'marked_lineage' and seq == '0000'
                    original, _, _, oldpath = baseline('development', seq, condition, arm)
                    oldrun = next(r for r in original['runs'] if r['arm'] == arm)
                    keys = ['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount',
                        'rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages',
                        'deliveredMessages','maximumBernoulliCount']
                    for key in keys:
                        assert run[key] == oldrun[key], (seq, condition, arm, key, 'exact No-age recursion')
                    assert np.array_equal(np.asarray(run['iterationRecords'])[:, :26], np.asarray(oldrun['iterationRecords']).reshape(-1, 26))
                    parity.append(dict(sequence=seq, condition=condition, arm=arm, exact_fields=keys + ['iterationRecords_first26'], robot_frames=2*T))
                    hashes[str(oldpath.relative_to(ROOT))] = sha(oldpath)"""),
    ("OUT / 'audit_observation_domain.py', OUT / 'persistent_math.py'",
     "OUT / 'screen_observation_math.py', OUT / 'screen_persistent_math.py', OUT / 'audit_screened_run.py',\n            OUT.parent / 'icra_fusion_holdout/analyze_holdout.py',\n            OUT / 'audit_observation_domain.py', OUT / 'persistent_math.py'")
]
for before,after in changes:
    assert value.count(before)==1,(before,value.count(before))
    value=value.replace(before,after)
new.write_text(value)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
receipt=OUT/'SCREENED_ASSESSMENT_PATCH.json';assert not receipt.exists()
receipt.write_text(json.dumps(dict(source=old.name,source_sha256=sha(old),destination=new.name,
    destination_sha256=sha(new),replacements=[dict(before=a,after=b) for a,b in changes],
    generator_sha256=sha(Path(__file__))),indent=2)+'\n')
print('SCREENED ASSESSMENT AUDITOR CREATED')
