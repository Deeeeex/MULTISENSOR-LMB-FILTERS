"""Freeze association candidates and unchanged original development inputs."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('stage')
    p.add_argument('--sequences', nargs='+', required=True)
    p.add_argument('--preflight', action='store_true')
    args = p.parse_args()
    instrumentation = OUT / 'audit_association_instrumentation_development.json'
    audit = json.loads(instrumentation.read_text())
    assert audit['passed'] and audit['audited_robot_frames'] == 7972 and len(audit['parity']) == 18
    previous = REVIEW / 'stages/controls_development.json'
    prior = json.loads(previous.read_text())
    sequences = [f'{int(s):04d}' for s in args.sequences]
    units = [u for u in prior['units'] if u['sequence'] in sequences]
    assert [u['sequence'] for u in units] == sequences
    sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
    for name, expected in sources.items():
        assert sha(ROOT / name) == expected, name
    for unit in units:
        for field in ['data_path', 'marks_path', 'ratios_path']:
            if field in unit:
                name = unit[field]
                assert sha(ROOT / name) == prior['source_sha256'][name], name
                sources[name] = sha(ROOT / name)
    paths = list(OUT.glob('*.m')) + list(OUT.glob('*.py'))
    paths += [OUT / p for p in ['PROTOCOL.md', 'CANDIDATES_V1.md', 'ASSOCIATION_DIAGNOSTIC.json',
        'ASSOCIATION_RUNNER_PATCH.json', 'INSTRUMENTATION_PATCH.json', 'CANDIDATES_V2.md',
        'PERSISTENT_RUNNER_PATCH.json', 'PERSISTENT_EVIDENCE_EXTRACTION.json']]
    paths += [instrumentation, previous, REVIEW / 'runReviewerReplay.m', REVIEW / 'checkReviewerEvidence.m',
        REVIEW / 'fuseReviewerEvidence.m', REVIEW / 'review_probability_audit.py', REVIEW / 'review_gaussian_audit.py']
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in paths})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
    assert len(set(mapped)) == len(mapped)
    arms = ['marked_gaussian_evidence_assoc_reopen', 'marked_gaussian_evidence_assoc_split']
    if args.preflight:
        assert sequences == ['0001', '0008']
        arms.insert(0, 'marked_gaussian_evidence')
    cfg = dict(protocol='icra-persistent-association-v2', stage=args.stage, cohort='development',
        created_utc=datetime.now(timezone.utc).isoformat(), arms=arms, pd=.9, preflight=args.preflight,
        conditions=['reliable', 'intermittent'], units=units, source_sha256=sources)
    path = OUT / 'stages' / (args.stage + '.json')
    assert not path.exists() and not (OUT / 'results' / args.stage).exists()
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(cfg, indent=2, allow_nan=False) + '\n')
    print('REGISTERED', args.stage, len(units), 'sequences', len(arms), 'arms', 2 * len(units) * len(arms), 'outputs', flush=True)


if __name__ == '__main__':
    main()
