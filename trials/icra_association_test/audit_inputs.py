"""Verify every saved new-cohort input against frozen raw and detector data."""
import json
from pathlib import Path
from scipy.io import loadmat
from scipy.special import expit
from input_adapter import *
import ast


def domain(xy, positions):
    distance2 = np.sum((xy[:, None] - positions[None]) ** 2, axis=2)
    nearest = np.min(distance2, axis=1)
    return ((xy[:, 0] >= -70.4) & (xy[:, 0] <= 70.4) &
            (xy[:, 1] >= -40) & (xy[:, 1] <= 40) & (nearest <= 1600) & (nearest > 9))


# Independent pose reference from the pinned author implementation.
author_path = AUTHOR / 'opencood/utils/transformation_utils.py'
node = next(n for n in ast.parse(author_path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'x_to_world')
namespace = {'np': np}
exec(compile(ast.Module(body=[node], type_ignores=[]), str(author_path), 'exec'), namespace)
author_pose = namespace['x_to_world']


def independent_labels(raw_pair):
    labels = {}; camera_to_world = author_pose(raw_pair[0]['lidar_pose'])
    world_to_camera = np.linalg.inv(camera_to_world)
    signs = np.array([[1,-1,-1], [1,1,-1], [-1,1,-1], [-1,-1,-1],
                      [1,-1,1], [1,1,1], [-1,1,1], [-1,-1,1]])
    for raw in raw_pair:
        for label, obj in raw['vehicles'].items():
            if obj['obj_type'] in ['Car', 'PoliceCar', 'LongVehicle']:
                labels.setdefault(int(label), obj)
    boxes = []; kept = []
    for label, obj in sorted(labels.items()):
        center = np.asarray(obj['location']) + obj['center']
        relative = world_to_camera @ author_pose([*center, *obj['angle']])
        local = signs * np.asarray(obj['extent'])
        xyz = (relative[:3, :3] @ local.T).T + relative[:3, 3]
        projected = xyz.astype(np.float32)
        if np.all(projected[:, :2] >= [-100, -40]) and np.all(projected[:, :2] <= [100, 40]):
            boxes.append(relative[:3, 3]); kept.append(label)
    return np.asarray(boxes).reshape(-1, 3), np.asarray(kept, np.int64)


def main():
    freeze = json.loads((OUT / 'INFERENCE_FREEZE.json').read_text())
    manifest = json.loads((OUT / 'NEW_INPUT_MANIFEST.json').read_text())
    detections = json.loads((OUT / 'DETECTION_MANIFEST.json').read_text())
    for item in [freeze['protected_sha256'], freeze['raw_file_sha256'], manifest['source_sha256']]:
        for path, expected in item.items():
            assert sha(ROOT / path) == expected, path
    cal = json.loads((OUT.parent / 'icra_ceiling_iteration/calibration.json').read_text())['full_seen_fit']
    prior = json.loads((OUT.parent / 'icra_marked_iteration/likelihood_manifest.json').read_text())['full_seen_positive_prior']
    rows = []; all_calibration_clamps = 0
    for sequence, frozen, detector in zip(manifest['sequences'], freeze['sequences'], detections['sequences']):
        name = sequence['sequence']; assert name == frozen['sequence'] == detector['sequence']
        data_path = ROOT / sequence['data_path']; assert sha(data_path) == sequence['input_sha256']
        pose_path = ROOT / sequence['pose_path']; assert sha(pose_path) == sequence['pose_sha256']
        detector_path = ROOT / detector['path']; assert sha(detector_path) == detector['sha256']
        mat = loadmat(data_path); pose = loadmat(pose_path); detection = np.load(detector_path, allow_pickle=False)
        T = sequence['frames']; assert int(mat['T'].item()) == T and np.array_equal(mat['time'].ravel(), np.arange(T) * .1)
        scene = ROOT / frozen['scene_path']; nmeasure = 0; ntruth = 0
        for t, stem in enumerate(frozen['paired_stems']):
            raw = [read_yaml(scene / str(n) / f'{stem}.yaml') for n in [1, 2]]
            transforms = [np.linalg.inv(author_pose(raw[0]['lidar_pose'])) @ author_pose(source['lidar_pose']) for source in raw]
            positions = np.vstack([np.zeros(2), transforms[1][:2, 3]])
            assert np.array_equal(mat['positions'][:, :, t].T, positions)
            assert np.array_equal(pose['egoLidarToWorld'][:, :, t], author_pose(raw[0]['lidar_pose']))
            for n in range(2):
                xy = detection[f'{t:06d}_{n}_boxes'][:, :2].astype(float)
                raw_score = detection[f'{t:06d}_{n}_scores'].astype(float)
                keep = domain(xy, positions) & (np.sum((xy - positions[n]) ** 2, axis=1) <= 1600)
                assert np.array_equal(mat['measurements'][n, t], xy[keep].T)
                assert np.array_equal(mat['rawScores'][n, t].ravel(), raw_score[keep])
                score = np.clip(raw_score[keep], 1e-6, 1 - 1e-6)
                probability = expit(cal['a'] * (np.log(score) - np.log1p(-score)) + cal['b'])
                clipped = np.clip(probability, 1e-6, 1 - 1e-6)
                all_calibration_clamps += int(np.count_nonzero(clipped != probability))
                assert np.array_equal(mat['calibratedScores'][n, t].ravel(), clipped)
                expected = clipped / (1 - clipped) * ((1 - prior) / prior)
                assert np.array_equal(mat['likelihoodRatios'][n, t].ravel(), expected)
                nmeasure += len(expected)
            boxes, labels = independent_labels(raw); keep = domain(boxes[:, :2].astype(float), positions)
            assert np.array_equal(mat['truth'][0, t][:2], boxes[keep, :2].astype(float).T)
            assert np.array_equal(mat['truthIds'][0, t].ravel(), labels[keep])
            assert not mat['truth'][0, t][2:].any(); ntruth += int(keep.sum())
        rows.append(dict(sequence=name, frames=T, measurement_rows=nmeasure, ground_truth_rows=ntruth))
    result = dict(passed=True, sequences=rows, frames=sum(r['frames'] for r in rows),
                  manifest_sha256=sha(OUT / 'NEW_INPUT_MANIFEST.json'), checker_sha256=sha(Path(__file__)),
                  adapter_sha256=sha(OUT / 'input_adapter.py'), calibration_probability_clamps=all_calibration_clamps,
                  checks='All source hashes; exact saved poses, domain-filtered measurements, raw/calibrated scores, likelihood ratios, independent world-center and full-corner projection, first-occurrence ID union and annotation-only velocities.')
    destination = OUT / 'NEW_INPUT_AUDIT.json'; assert not destination.exists()
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print('NEW INPUT AUDIT PASSED', result['frames'], 'frames;',all_calibration_clamps,'probability clamps',flush=True)


if __name__ == '__main__':
    main()
