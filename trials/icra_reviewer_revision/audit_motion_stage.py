"""Independently rescore v2 trajectories and check the coordinate adapter."""
from pathlib import Path
import argparse
import csv
import json
import re

from audit_stage import OUT, ROOT, sha, read, score_run, radio_draws, audit_packets, audit_probability, np, loadmat


def main():
    p = argparse.ArgumentParser(); p.add_argument('stage'); args = p.parse_args()
    cfgpath = OUT / 'stages' / f'{args.stage}.json'; cfg = json.loads(cfgpath.read_text())
    assert cfg['protocol'] == 'icra-reviewer-revision-v2'
    runtime_path = OUT / f'runtime_{args.stage}.json'; runtime = json.loads(runtime_path.read_text())
    assert len(runtime) == len(cfg['units'])
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    source = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in cfg['source_sha256'].items()}
    assert len(source) == len(cfg['source_sha256'])
    rows = []; diagnostics = []; parity = []; hashes = {}
    for unit, native in zip(cfg['units'], runtime):
        name = unit['sequence']; assert native['sequence'] == name
        assert native['returncode'] == 0 and native['completion_line'] and native['files'] == 2 * len(cfg['arms'])
        log = ROOT / 'RUN/ICRA_REVIEWER_REVISION' / args.stage / f'{name}.log'
        text = log.read_text()
        assert f'COMPLETED REVIEW {args.stage} {name}' in text
        assert 'REVIEW CONTROL CHECK PASSED' in text and 'REVIEW EGO MOTION CHECK PASSED' in text
        data_path = ROOT / unit['data_path']; assert sha(data_path) == unit['input_sha256']
        mat = loadmat(data_path); T = int(mat['T'].item()); hashes[str(data_path.relative_to(ROOT))] = sha(data_path)
        motion = np.repeat(np.eye(3)[:, :, None], T, axis=2)
        if cfg['coordinate_adapter'] == 'planar':
            pose_path = ROOT / unit['pose_path']; assert sha(pose_path) == unit['pose_sha256']
            pose_data = loadmat(pose_path); motion = pose_data['egoPrevToCurrent']
            # Independently form SE(2) from the actual raw rotation/translation.
            poses = pose_data['egoLidarToWorld']; expected = motion.copy(); expected[:, :, 0] = np.eye(3)
            planar = []
            for t in range(T):
                yaw = np.arctan2(poses[1, 0, t], poses[0, 0, t]); c, s = np.cos(yaw), np.sin(yaw)
                planar.append(np.array([[c, -s, poses[0, 3, t]], [s, c, poses[1, 3, t]], [0, 0, 1.]]))
                if t:
                    expected[:, :, t] = np.linalg.inv(planar[t]) @ planar[t - 1]
            assert np.allclose(motion, expected, rtol=0, atol=1e-10)
            hashes[str(pose_path.relative_to(ROOT))] = sha(pose_path)
        for condition in cfg['conditions']:
            for arm in cfg['arms']:
                path = OUT / 'results' / args.stage / f'{name}_{condition}_{arm}.json.gz'
                data = read(path); run = data['runs']; hashes[str(path.relative_to(ROOT))] = sha(path)
                assert data['protocol'] == 'icra-reviewer-revision-v2' and data['stage'] == args.stage
                assert data['sourceSha256'] == source and data['inputSha256'] == unit['input_sha256']
                assert data['cohort'] == cfg['cohort'] and data['sequence'] == name and data['condition'] == condition
                assert data['coordinateAdapter'] == cfg['coordinate_adapter'] and np.array_equal(data['egoPrevToCurrent'], motion)
                assert data['pd'] == cfg['pd'] and run['arm'] == arm
                assert np.array_equal(data['time'], mat['time'].ravel()) and np.array_equal(data['positions'], mat['positions'])
                for t in range(T):
                    assert np.array_equal(np.asarray(data['truth'][t]).reshape(4, -1), mat['truth'][0, t])
                    assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(), mat['truthIds'][0, t].ravel())
                delivery = radio_draws(unit['radio_seed'] - 8301, T, condition)
                assert np.array_equal(delivery, data['delivered']); audit_packets(run, delivery, T)
                row, _ = score_run(data, run)
                rows.append(dict(sequence=name, condition=condition, arm=arm, frames=T, **row))
                if arm not in ['marked_lineage', 'marked_er']:
                    diagnostics.append(dict(sequence=name, condition=condition, **audit_probability(run, data)))
                if cfg['preflight']:
                    previous = OUT / 'results/preflight_v1' / f'{name}_{condition}_{arm}.json.gz'
                    old = read(previous)['runs']; keys = [key for key in old if key != 'runtimeSeconds']
                    for key in keys:
                        assert run[key] == old[key], (arm, condition, 'identity full-recursion parity', key)
                    parity.append(dict(arm=arm, condition=condition, exact_fields=keys, node_frames=2 * T))
                    hashes[str(previous.relative_to(ROOT))] = sha(previous)
                print('AUDITED V2', args.stage, name, condition, arm, flush=True)
    result = dict(passed=True, stage=args.stage, cohort=cfg['cohort'], coordinate_adapter=cfg['coordinate_adapter'],
                  sequences=len(cfg['units']), arms=cfg['arms'], pd=cfg['pd'], rows=rows, diagnostics=diagnostics,
                  parity=parity, audited_node_frames=sum(2 * row['frames'] for row in rows), inputs=hashes,
                  config_sha256=sha(cfgpath), runtime_sha256=sha(runtime_path),
                  auditor_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__), OUT / 'audit_stage.py',
                      OUT / 'review_probability_audit.py', OUT / 'review_gaussian_audit.py']})
    destination = OUT / f'audit_{args.stage}.json'; assert not destination.exists()
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / f'scores_{args.stage}.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print('V2 REVIEW STAGE AUDIT PASSED', args.stage, result['audited_node_frames'], 'node-frames', flush=True)


if __name__ == '__main__':
    main()
