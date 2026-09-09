"""Produce frozen-checkpoint detections first; labels enter conversion later."""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import json
import time

from official_input_adapter import *


def main():
    freeze_path = OUT / 'NEW_DATA_FREEZE.json'
    freeze = json.loads(freeze_path.read_text())
    for key, expected in freeze['protected_sha256'].items():
        assert sha(ROOT / key) == expected, key
    destination = OUT / 'new_detections'; destination.mkdir(exist_ok=True)
    report_path = OUT / 'NEW_DETECTION_MANIFEST.json'; assert not report_path.exists()
    torch.set_num_threads(2); model, post = load_detector('mps')
    rows = []; start = time.time()
    for sequence in freeze['sequences']:
        name = sequence['sequence']; scene = ROOT / sequence['scene_path']
        outputs = {}; sources = {}; T = sequence['frames']; counts = [0, 0]
        path = destination / f'detections_{name}.npz'; assert not path.exists()
        # An arm never controls inference or its source-frame order.
        for frame, stem in enumerate(sequence['paired_stems']):
            raw_paths = [scene / str(n) / f'{stem}.yaml' for n in range(2)]
            for raw_path in raw_paths:
                assert sha(raw_path) == freeze['raw_file_sha256'][str(raw_path.relative_to(ROOT))]
            raw = [read_yaml(p) for p in raw_paths]
            transforms = relative_poses(raw)
            for sensor in range(2):
                pcd = scene / str(sensor) / f'{stem}.pcd'; key = str(pcd.relative_to(ROOT))
                assert sha(pcd) == freeze['raw_file_sha256'][key]
                seed = 20 + 100000 * int(name) + 2 * frame + sensor
                output = infer(model, read_pcd(pcd), seed)
                boxes, scores = post.detections(output, transforms[sensor])
                assert boxes.shape == (len(scores), 7) and np.isfinite(boxes).all() and np.isfinite(scores).all()
                outputs[f'{frame:06d}_{sensor}_boxes'] = boxes
                outputs[f'{frame:06d}_{sensor}_scores'] = scores
                counts[sensor] += len(scores)
            if (frame + 1) % 25 == 0 or frame + 1 == T:
                print('FROZEN DETECTOR INFERENCE', name, frame + 1, '/', T, 'elapsed_s', round(time.time() - start), flush=True)
        np.savez_compressed(path, **outputs)
        rows.append(dict(sequence=name, frames=T, detections_by_sensor=counts,
                         path=str(path.relative_to(ROOT)), sha256=sha(path)))
    report = dict(completed_utc=datetime.now(timezone.utc).isoformat(), passed=True, sequences=rows,
                  freeze_sha256=sha(freeze_path), inference_sha256=sha(Path(__file__)),
                  adapter_sha256=sha(OUT / 'official_input_adapter.py'), elapsed_s=time.time() - start,
                  numpy=np.__version__, torch=torch.__version__, device='MPS float32',
                  truth_used_for_detections=False, model_checkpoint_unchanged=True)
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print('ALL NEW DETECTIONS COMPLETE', len(rows), 'sequences', sum(r['frames'] for r in rows), 'paired frames', flush=True)


if __name__ == '__main__':
    main()
