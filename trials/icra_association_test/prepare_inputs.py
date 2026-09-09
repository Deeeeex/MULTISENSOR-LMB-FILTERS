"""Convert frozen detector outputs, then attach labels strictly for scoring."""
from datetime import datetime, timezone
import json
import sys

from scipy.io import savemat
from scipy.special import expit
from input_adapter import *
from prepare_pose_inputs import save_mat

sys.path.insert(0, str(OUT.parent / 'icra_external_fusion'))
from prepare_v2v4real import in_domain


def main():
    freeze_path = OUT / 'INFERENCE_FREEZE.json'; freeze = json.loads(freeze_path.read_text())
    detections_path = OUT / 'DETECTION_MANIFEST.json'; detection_manifest = json.loads(detections_path.read_text())
    assert detection_manifest['passed'] and detection_manifest['freeze_sha256'] == sha(freeze_path)
    assert not detection_manifest['truth_used_for_detections']
    for key, expected in freeze['protected_sha256'].items():
        assert sha(ROOT / key) == expected, key
    cal_path = OUT.parent / 'icra_ceiling_iteration/calibration.json'
    prior_path = OUT.parent / 'icra_marked_iteration/likelihood_manifest.json'
    cal = json.loads(cal_path.read_text())['full_seen_fit']
    prior = json.loads(prior_path.read_text())['full_seen_positive_prior']
    assert cal['training_sequences'] == [f'{s:04d}' for s in range(9)] and 0 < prior < 1
    folder = OUT / 'new_data'; folder.mkdir(exist_ok=True)
    pose_folder = OUT / 'pose_inputs'; pose_folder.mkdir(parents=True, exist_ok=True)
    destination = OUT / 'NEW_INPUT_MANIFEST.json'; assert not destination.exists()
    records = []; hashes = {}
    for sequence, detection_record in zip(freeze['sequences'], detection_manifest['sequences']):
        name = sequence['sequence']; T = sequence['frames']; assert detection_record['sequence'] == name
        path = ROOT / detection_record['path']; assert sha(path) == detection_record['sha256']
        detection = np.load(path, allow_pickle=False); hashes[str(path.relative_to(ROOT))] = sha(path)
        assert len(detection.files) == T * 4
        scene = ROOT / sequence['scene_path']; positions = np.zeros((T, 2, 2))
        poses = np.empty((T, 4, 4)); motion = np.repeat(np.eye(3)[None], T, axis=0)
        measurements = np.empty((2, T), object); truth = np.empty((1, T), object); ids = np.empty((1, T), object)
        rawscores = np.empty((2, T), object); calibrated = np.empty((2, T), object); ratios = np.empty((2, T), object)
        target_counts = []; counts = np.zeros(2, int)
        for t, stem in enumerate(sequence['paired_stems']):
            raw_paths = [scene / str(n) / f'{stem}.yaml' for n in [1, 2]]
            for raw_path in raw_paths:
                assert sha(raw_path) == freeze['raw_file_sha256'][str(raw_path.relative_to(ROOT))]
            raw = [read_yaml(path) for path in raw_paths]; transform = relative_poses(raw)
            poses[t] = pose_matrix(raw[0]['lidar_pose'])
            positions[t, 1] = transform[1][:2, 3]
            if t:
                motion[t] = np.linalg.solve(planar_pose(poses[t]), planar_pose(poses[t - 1]))
            for n in range(2):
                boxes = detection[f'{t:06d}_{n}_boxes']; score = detection[f'{t:06d}_{n}_scores'].astype(float)
                assert len(boxes) == len(score) and np.all((score > .2) & (score <= 1))
                xy = boxes[:, :2].astype(float)
                mask = in_domain(xy, positions[t]) & (((xy - positions[t, n]) ** 2).sum(1) <= 1600)
                measurements[n, t] = xy[mask].T; score = score[mask].reshape(1, -1)
                clipped = np.clip(score, 1e-6, 1 - 1e-6)
                probability = expit(cal['a'] * (np.log(clipped) - np.log1p(-clipped)) + cal['b'])
                probability = np.clip(probability, 1e-6, 1 - 1e-6)
                rawscores[n, t] = score; calibrated[n, t] = probability
                ratios[n, t] = probability / (1 - probability) * ((1 - prior) / prior)
                counts[n] += int(mask.sum())
            boxes, labels = labels_for_pair(raw, transform)
            xy = boxes[:, :2].astype(float); mask = in_domain(xy, positions[t])
            truth[0, t] = np.vstack([xy[mask].T, np.zeros((2, int(mask.sum())))])
            ids[0, t] = labels[mask][None, :]; assert len(set(labels)) == len(labels)
            target_counts.append(int(mask.sum()))
        data = dict(name=name, T=T, dt=.1, N=2, measurements=measurements,
                    positions=positions.transpose(2, 1, 0), time=np.arange(T) * .1,
                    truth=truth, truthIds=ids, rawScores=rawscores,
                    calibratedScores=calibrated, likelihoodRatios=ratios)
        data_path = folder / f'v2xreal_{name}.mat'; save_mat(data_path, data)
        pose_path = pose_folder / f'poses_{name}.mat'
        save_mat(pose_path, dict(egoLidarToWorld=poses.transpose(1, 2, 0), egoPrevToCurrent=motion.transpose(1, 2, 0)))
        records.append(dict(sequence=name, scene=sequence['scene'], collection_date=sequence['collection_date'],
                            source_recording=sequence['scene'].rsplit('_', 2)[0],
                            frames=T, radio_seed=sequence['radio_seed'], measurement_rows_by_sensor=counts.tolist(),
                            ground_truth_rows=sum(target_counts), target_count_min=min(target_counts), target_count_max=max(target_counts),
                            data_path=str(data_path.relative_to(ROOT)), input_sha256=sha(data_path),
                            pose_path=str(pose_path.relative_to(ROOT)), pose_sha256=sha(pose_path),
                            relative_vehicle_range_min=float(np.linalg.norm(positions[:, 1], axis=1).min()),
                            relative_vehicle_range_max=float(np.linalg.norm(positions[:, 1], axis=1).max())))
        print('NEW INPUT READY', records[-1], flush=True)
    sources = [Path(__file__), OUT / 'input_adapter.py', old.OUT / 'official_input_adapter.py', old.OUT / 'prepare_pose_inputs.py',
               freeze_path, detections_path, cal_path, prior_path, OUT.parent / 'icra_external_fusion/prepare_v2v4real.py']
    hashes.update({str(path.relative_to(ROOT)): sha(path) for path in sources})
    result = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(), sequences=records,
                  frames=sum(row['frames'] for row in records), source_sha256=hashes,
                  calibration_refitted=False, positive_prior_refitted=False,
                  model_frame='Current ego LiDAR x/y; optional known planar motion adapter has separate frozen pose inputs.',
                  domain='Identical rectangle, two 40 m sensor disks, and 3 m platform exclusion to original replay.',
                  purpose='External V2X vehicle-pair transfer. Annotation fields enter only scoring after each complete trajectory.', vehicle_class=sorted(VEHICLE_TYPES), time_step_source='Official dataset website: vehicle LiDAR 10 Hz.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('ALL NEW INPUTS FROZEN BEFORE TRACKING', result['frames'], 'paired frames', flush=True)


if __name__ == '__main__':
    main()
