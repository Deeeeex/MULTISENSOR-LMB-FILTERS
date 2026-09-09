"""Register an admission experiment on existing immutable prepared inputs."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage')
    parser.add_argument('--cohort', choices=['development', 'seen_transfer', 'new_recording_diagnostic', 'new_validation', 'remaining_train'], required=True)
    parser.add_argument('--sequences', nargs='+')
    parser.add_argument('--arms', nargs='+', required=True)
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    path = OUT / 'stages' / f'{args.stage}.json'
    assert not path.exists() and not (OUT / 'results' / args.stage).exists()
    if args.cohort == 'development':
        previous = REVIEW / 'stages/controls_development.json'
    elif args.cohort == 'seen_transfer':
        previous = REVIEW / 'stages/gs_seen_transfer.json'
    elif args.cohort == 'remaining_train':
        previous = OUT.parent / 'icra_full_coverage/stages/coverage_remaining_train.json'
    else:
        previous = REVIEW / 'stages/new_validation_primary.json'
    prior = json.loads(previous.read_text())
    units = prior['units']
    selected = [f'{int(x):04d}' for x in args.sequences] if args.sequences else None
    if selected:
        units = [u for u in units if u['sequence'] in selected]
        assert [u['sequence'] for u in units] == selected
    if args.cohort == 'new_recording_diagnostic':
        assert [u['sequence'] for u in units] == ['0000']
    sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
    for name, expected in sources.items():
        assert sha(ROOT / name) == expected, name
    for unit in units:
        assert sha(ROOT / unit['data_path']) == unit['input_sha256']
        for field in ['data_path', 'marks_path', 'ratios_path']:
            if field in unit:
                name = unit[field]
                assert sha(ROOT / name) == prior['source_sha256'][name], name
                sources[name] = sha(ROOT / name)
    paths = list(OUT.glob('*.py')) + list(OUT.glob('*.m'))
    paths += [OUT / 'PROTOCOL.md', OUT / 'SOURCE_REVIEW_CN.md', OUT / 'REJECTION_DIAGNOSTIC.json', previous]
    paths += [REVIEW / p for p in ['runReviewerReplay.m', 'checkReviewerEvidence.m', 'fuseReviewerEvidence.m', 'review_probability_audit.py', 'review_gaussian_audit.py']]
    paths += [OUT.parent / 'icra_ceiling_iteration/calibration.json', OUT.parent / 'icra_marked_iteration/likelihood_manifest.json']
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in paths})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
    assert len(set(mapped)) == len(mapped)
    config = dict(protocol='icra-admission-revision-v1', stage=args.stage, cohort=args.cohort,
                  created_utc=datetime.now(timezone.utc).isoformat(), arms=args.arms, pd=.9,
                  preflight=args.preflight, conditions=['reliable', 'intermittent'], units=units, source_sha256=sources)
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(config, indent=2, allow_nan=False) + '\n')
    print('REGISTERED', args.stage, len(units), 'sequences', len(args.arms), 'arms', len(units) * 2 * len(args.arms), 'outputs', flush=True)


if __name__ == '__main__':
    main()
