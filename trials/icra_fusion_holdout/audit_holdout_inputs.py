"""Independently rebuild every reserved input from pinned public source rows.

This audit never reads tracking outcomes and never fits a calibration model.
"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.io import loadmat
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
AUTHOR = ROOT / 'tmp/external_baselines/DMSTrack'
CACHE = ROOT / 'tmp/external_baselines/v2v_transfer_transforms'
sys.path.insert(0, str(OUT.parent / 'icra_external_fusion'))
from analyze_v2v4real import domain


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    manifest = json.loads((OUT / 'input_manifest.json').read_text())
    source = json.loads((OUT / 'source_sha256_port.json').read_text())
    for name, expected in {**source, **freeze['source_and_selection_evidence_sha256']}.items():
        assert digest(ROOT / name) == expected, name
    for name, expected in manifest['source_files'].items():
        assert digest(AUTHOR / name) == expected, name
    linked = {
        'transforms_manifest_sha256': OUT / 'transform_manifest.json',
        'overlap_audit_sha256': OUT.parent / 'icra_method_iteration/transfer_overlap_audit.json',
        'calibration_sha256': OUT.parent / 'icra_ceiling_iteration/calibration.json',
        'likelihood_prior_manifest_sha256': OUT.parent / 'icra_marked_iteration/likelihood_manifest.json',
    }
    for key, path in linked.items():
        assert digest(path) == manifest[key], key
    transforms = json.loads(linked['transforms_manifest_sha256'].read_text())
    assert transforms['matrix_count'] == len(transforms['source_files']) == 11202
    for entry in transforms['source_files']:
        path = CACHE / entry['sensor'] / f"{entry['frame']:04d}_transformation_matrix.npy"
        assert digest(path) == entry['sha256'], path
    calibration = json.loads(linked['calibration_sha256'].read_text())['full_seen_fit']
    prior = json.loads(linked['likelihood_prior_manifest_sha256'].read_text())['full_seen_positive_prior']
    assert calibration['training_sequences'] == [f'{s:04d}' for s in range(9)]
    assert manifest['selected_sequences'] == freeze['units'] == [s for s in range(32) if s % 5 != 0]
    assert freeze['frames'] == manifest['frames'] == 5601
    offsets = np.r_[0, np.cumsum(transforms['complete_train_lengths'])]
    rows = []
    for entry in manifest['sequences']:
        name = entry['sequence']
        seq = int(name)
        path = OUT / 'data' / f'v2v4real_{name}.mat'
        assert digest(path) == entry['input_sha256'], name
        mat = loadmat(path)
        T = entry['frames']
        assert mat['T'].item() == T == offsets[seq + 1] - offsets[seq]
        assert mat['N'].item() == 2 and mat['dt'].item() == .1
        assert np.array_equal(mat['time'].ravel(), np.arange(T) * .1)
        pose = np.zeros((2, 2, T))
        for t in range(T):
            for n, sensor in enumerate(['ego', '1']):
                matrix = np.load(CACHE / sensor / f'{offsets[seq] + t:04d}_transformation_matrix.npy', allow_pickle=False)
                assert matrix.shape == (4, 4) and np.isfinite(matrix).all()
                assert np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5)
                if sensor == 'ego':
                    assert np.allclose(matrix, np.eye(4), atol=1e-5)
                pose[:, n, t] = matrix[:2, 3]
        assert np.array_equal(pose, mat['positions'])
        detections = []
        for sensor in ['ego', '1']:
            path = AUTHOR / f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_train/{sensor}/{name}.txt'
            rows_raw = np.loadtxt(path, delimiter=',', ndmin=2) if path.stat().st_size else np.empty((0, 15))
            assert rows_raw.shape[1] == 15 and np.isfinite(rows_raw).all()
            assert np.all(rows_raw[:, 1] == 2)
            detections.append(rows_raw)
        path = AUTHOR / f'AB3DMOT/scripts/KITTI/v2v4real_train_label/{name}.txt'
        annotations = [line.split() for line in path.read_text().splitlines()]
        assert all(len(row) == 17 and row[2] == 'Car' for row in annotations)
        ground = np.array([[int(row[0]), int(row[1]), float(row[13]), float(row[15])] for row in annotations]).reshape(-1, 4)
        counts = np.zeros(2, dtype=int)
        targets = 0
        for t in range(T):
            for n, detections_raw in enumerate(detections):
                frame = detections_raw[detections_raw[:, 0] == t]
                xy = frame[:, [10, 12]]
                state = np.c_[xy, np.zeros((len(xy), 2))]
                keep = domain(state, pose[:, :, t]) & (((xy - pose[:, n, t]) ** 2).sum(1) <= 1600)
                xy = xy[keep]
                raw_score = frame[keep, 6].reshape(1, -1)
                assert np.array_equal(mat['measurements'][n, t], xy.T)
                assert np.array_equal(mat['rawScores'][n, t], raw_score)
                bounded = np.clip(raw_score, 1e-6, 1 - 1e-6)
                calibrated = expit(calibration['a'] * (np.log(bounded) - np.log1p(-bounded)) + calibration['b'])
                assert np.array_equal(mat['calibratedScores'][n, t], calibrated)
                p = np.clip(calibrated, 1e-6, 1 - 1e-6)
                likelihood = p / (1 - p) * (1 - prior) / prior
                assert np.array_equal(mat['likelihoodRatios'][n, t], likelihood)
                counts[n] += len(xy)
            g = ground[ground[:, 0] == t]
            keep = domain(np.c_[g[:, 2:4], np.zeros((len(g), 2))], pose[:, :, t])
            g = g[keep]
            assert np.array_equal(mat['truth'][0, t], np.vstack([g[:, 2:4].T, np.zeros((2, len(g)))]))
            assert np.array_equal(mat['truthIds'][0, t].ravel(), g[:, 1])
            assert len(set(g[:, 1])) == len(g)
            targets += len(g)
        assert counts.tolist() == entry['detection_rows'] and targets == entry['ground_truth_rows']
        rows.append(dict(sequence=name, frames=T, detections=counts.tolist(), truth_rows=targets, input_sha256=entry['input_sha256']))
        print('SOURCE INPUT AUDITED', name, T, 'frames', flush=True)
    report = dict(
        protocol='remaining-v2v4real-fusion-selection-v1',
        scope='All frozen inputs reconstructed from source; no tracking outcomes inspected or models fitted.',
        source_hashes_verified=len(source), selection_hashes_verified=len(freeze['source_and_selection_evidence_sha256']),
        matrix_hashes_verified=11202, raw_detection_and_ground_truth_files=len(manifest['source_files']),
        frames=sum(row['frames'] for row in rows), sequences=rows,
        exact_pose_measurement_truth_score_and_likelihood_reconstruction=True,
        method_freeze_sha256=digest(OUT / 'METHOD_FREEZE.json'),
        auditor_sha256=digest(Path(__file__)),
    )
    assert len(rows) == 25 and report['frames'] == 5601
    (OUT / 'input_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print('ALL 5601 SOURCE INPUT FRAMES AUDITED; no tracking outcomes used.', flush=True)


if __name__ == '__main__':
    main()
