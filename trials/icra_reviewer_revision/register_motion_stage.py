"""Register the immutable v2 runner, coordinate mode, and all exact inputs."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def register(stage, cohort, arms, adapter, preflight=False):
    path = OUT / 'stages' / f'{stage}.json'
    assert not path.exists() and not (OUT / 'results' / stage).exists()
    template = OUT / 'stages' / ('gs_seen_transfer.json' if cohort == 'seen_transfer' else 'controls_development.json')
    cfg = json.loads(template.read_text()); sources = cfg['source_sha256']
    for key, expected in sources.items():
        assert sha(ROOT / key) == expected, key
    if cohort == 'new_validation':
        manifest_path = OUT / 'NEW_INPUT_MANIFEST.json'; manifest = json.loads(manifest_path.read_text())
        assert manifest['passed']
        cfg['units'] = [dict(sequence=r['sequence'], data_path=r['data_path'], input_sha256=r['input_sha256'],
                             radio_seed=r['radio_seed'], pose_path=r['pose_path'], pose_sha256=r['pose_sha256'])
                        for r in manifest['sequences']]
        for name in ['NEW_INPUT_MANIFEST.json', 'NEW_DATA_FREEZE.json', 'NEW_DETECTION_MANIFEST.json', 'FIXED_SELECTION.json']:
            p = OUT / name; sources[str(p.relative_to(ROOT))] = sha(p)
    elif adapter == 'planar':
        manifest_path = OUT / f'POSE_INPUTS_{cohort}.json'; manifest = json.loads(manifest_path.read_text()); assert manifest['passed']
        indexed = {r['sequence']: r for r in manifest['sequences']}
        for unit in cfg['units']:
            unit['pose_path'] = indexed[unit['sequence']]['pose_path']
            unit['pose_sha256'] = indexed[unit['sequence']]['pose_sha256']
        sources[str(manifest_path.relative_to(ROOT))] = sha(manifest_path)
    if preflight:
        assert cohort == 'development' and adapter == 'identity'
        cfg['units'] = cfg['units'][:1]
    for unit in cfg['units']:
        assert sha(ROOT / unit['data_path']) == unit['input_sha256']
        if 'pose_path' in unit:
            assert sha(ROOT / unit['pose_path']) == unit['pose_sha256']
    for name in ['runMotionReviewerReplay.m', 'applyReviewerEgoMotion.m', 'checkReviewerEgoMotion.m',
                 'make_motion_runner.py', 'MOTION_RUNNER_PATCH.json', 'register_motion_stage.py', 'run_motion_stage.py']:
        p = OUT / name; sources[str(p.relative_to(ROOT))] = sha(p)
    encoded_keys = [re.sub('[^A-Za-z0-9_]', '_', key)[:63] for key in sources]
    assert len(set(encoded_keys)) == len(encoded_keys), 'MATLAB source-map key collision'
    cfg.update(protocol='icra-reviewer-revision-v2', stage=stage, cohort=cohort, arms=arms,
               coordinate_adapter=adapter, preflight=preflight, pd=.9,
               created_utc=datetime.now(timezone.utc).isoformat())
    path.write_text(json.dumps(cfg, indent=2) + '\n')
    print('REGISTERED V2',stage,cohort,adapter,len(cfg['units']),'sequences',len(arms),'arms',flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--stage', required=True)
    p.add_argument('--cohort', choices=['development','seen_transfer','new_validation'], required=True)
    p.add_argument('--arms', nargs='+', required=True); p.add_argument('--adapter', choices=['none','identity','planar'], required=True)
    p.add_argument('--preflight', action='store_true'); args = p.parse_args()
    register(args.stage, args.cohort, args.arms, args.adapter, args.preflight)
