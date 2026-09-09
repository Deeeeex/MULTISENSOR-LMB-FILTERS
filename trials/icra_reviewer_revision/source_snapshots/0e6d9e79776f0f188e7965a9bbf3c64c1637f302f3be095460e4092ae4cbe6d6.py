"""Match absolute official poses to every existing released relative matrix."""
from pathlib import Path
import argparse
import json
from datetime import datetime, timezone
from scipy.io import loadmat, savemat

from official_input_adapter import *


def save_mat(path, values):
    assert not path.exists()
    savemat(path, values, do_compression=True)
    with path.open('r+b') as stream:
        stream.write(b'MATLAB 5.0 MAT-file, deterministic reviewer revision input'.ljust(116, b' '))


def main():
    p = argparse.ArgumentParser(); p.add_argument('cohort', choices=['development', 'seen_transfer'])
    args = p.parse_args(); transfer = args.cohort == 'seen_transfer'
    archives = [f'train_{i:02d}' for i in range(1, 9)] if transfer else [f'test_{i:02d}' for i in range(1, 4)]
    for archive in archives:
        assert (CACHE / archive / 'yaml_manifest.json').exists(), ('Raw pose download incomplete', archive)
    scenes = list(scene_directories('train' if transfer else 'test').values())
    assert len(scenes) == (32 if transfer else 9)
    seqmap = AUTHOR.parent / f'AB3DMOT/scripts/KITTI/v2v4real_{"train" if transfer else "val"}_evaluate_tracking.seqmap.val'
    lengths = [int(row.split()[3]) + 1 for row in seqmap.read_text().splitlines()]
    assert len(lengths) == len(scenes)
    offsets = np.r_[0, np.cumsum(lengths)]
    old_dir = OUT.parent / ('icra_fusion_holdout' if transfer else 'icra_external_fusion')
    manifest = json.loads((old_dir / ('input_manifest.json' if transfer else 'v2v4real_input_manifest.json')).read_text())
    selected = {int(row['sequence']) for row in manifest['sequences']}
    destination = OUT / 'pose_inputs' / args.cohort; destination.mkdir(parents=True, exist_ok=True)
    report_path = OUT / f'POSE_INPUTS_{args.cohort}.json'; assert not report_path.exists()
    rows = []; sources = {str(seqmap.relative_to(ROOT)): sha(seqmap)}
    for sequence in sorted(selected):
        scene = scenes[sequence]; T = lengths[sequence]
        stems = [f'{i:06d}' for i in range(T)]
        assert sorted(p.stem for p in (scene / '0').glob('*.yaml')) == stems
        assert sorted(p.stem for p in (scene / '1').glob('*.yaml')) == stems
        poses = np.empty((T, 4, 4)); motion = np.repeat(np.eye(3)[None], T, axis=0)
        relative_error = 0.; gt_error = 0.; matched_gt = 0; rotation_error = 0.
        truth_path = AUTHOR.parent / f'AB3DMOT/scripts/KITTI/v2v4real_{"train" if transfer else "val"}_label/{sequence:04d}.txt'
        truth_rows = [line.split() for line in truth_path.read_text().splitlines()]
        old_truth = np.array([[int(r[0]), int(r[1]), float(r[13]), float(r[15])] for r in truth_rows]).reshape(-1, 4)
        for frame, stem in enumerate(stems):
            raw_paths = [scene / str(n) / f'{stem}.yaml' for n in range(2)]
            raw = [read_yaml(path) for path in raw_paths]
            for path in raw_paths:
                sources[str(path.relative_to(ROOT))] = sha(path)
            pose = raw[0]['lidar_pose']; assert pose.shape == (4, 4) and np.isfinite(pose).all()
            assert np.allclose(pose[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            rotation_error = max(rotation_error, float(np.max(np.abs(pose[:3, :3].T @ pose[:3, :3] - np.eye(3)))))
            # Released calibration matrices have finite decimal precision.
            assert rotation_error < 1e-5 and abs(np.linalg.det(pose[:3, :3]) - 1) < 1e-5
            poses[frame] = pose
            if frame:
                motion[frame] = np.linalg.solve(planar_pose(pose), planar_pose(poses[frame - 1]))
            relative = relative_poses(raw)
            directory = 'v2v_transfer_transforms' if transfer else 'v2v_transforms'
            cached = ROOT / 'tmp/external_baselines' / directory / '1' / f'{offsets[sequence] + frame:04d}_transformation_matrix.npy'
            released = np.load(cached, allow_pickle=False); sources[str(cached.relative_to(ROOT))] = sha(cached)
            discrepancy = float(np.max(np.abs(relative[1] - released)))
            relative_error = max(relative_error, discrepancy)
            assert discrepancy < 1e-5, (sequence, frame, 'relative pose', discrepancy)
            # Old development labels independently verify local/world convention
            # and union IDs. This is conversion QA, not algorithm selection.
            boxes, ids = labels_for_pair(raw, relative)
            truth = old_truth[old_truth[:, 0] == frame]
            assert set(ids) == set(truth[:, 1]), (sequence, frame, 'GT IDs')
            for label, center in zip(ids, boxes[:, :2]):
                target = truth[truth[:, 1] == label, 2:4]; assert len(target) == 1
                gt_error = max(gt_error, float(np.max(np.abs(center - target[0])))); matched_gt += 1
        assert gt_error < 1e-4, (sequence, 'GT coordinates', gt_error)
        path = destination / f'poses_{sequence:04d}.mat'
        save_mat(path, dict(egoLidarToWorld=poses.transpose(1, 2, 0),
                            egoPrevToCurrent=motion.transpose(1, 2, 0)))
        row = dict(sequence=f'{sequence:04d}', scene=scene.name, frames=T,
                   pose_path=str(path.relative_to(ROOT)), pose_sha256=sha(path),
                   relative_transform_max_abs_error=relative_error, label_xy_max_abs_error=gt_error,
                   source_rotation_max_orthogonality_error=rotation_error,
                   label_rows_checked=matched_gt, max_ego_displacement_m=float(np.linalg.norm(np.diff(poses[:, :2, 3], axis=0), axis=1).max(initial=0)),
                   max_ego_yaw_change_deg=float(np.rad2deg(np.abs(np.arctan2(motion[:, 1, 0], motion[:, 0, 0]))).max(initial=0)))
        rows.append(row); print('POSE INPUT VERIFIED', args.cohort, row, flush=True)
    report = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(), cohort=args.cohort,
                  sequences=rows, source_sha256=sources, adapter_sha256=sha(OUT / 'official_input_adapter.py'),
                  preparer_sha256=sha(Path(__file__)),
                  motion='Known planar yaw and translation, applied after CV prediction in the previous ego basis. Velocities rotate; full covariance transforms with blockdiag(R,R). Same current-ego measurement/truth crop. Pitch, roll and height not compensated.')
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print('ALL POSE INPUTS VERIFIED', args.cohort, len(rows), flush=True)


if __name__ == '__main__':
    main()
