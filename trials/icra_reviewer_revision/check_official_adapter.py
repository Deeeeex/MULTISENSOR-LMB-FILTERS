"""Input-only parity on already exposed frames, before new evaluation."""
from pathlib import Path
from types import SimpleNamespace
import json
import sys
import time

import numpy as np
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment

from official_input_adapter import *


def reference_voxels(points):
    # Vectorized grid assignment followed by stable group ordering; independent
    # of the dense-hash/Numba implementation. Semantics follow upstream spconv
    # Point2VoxelCPU point_to_voxel_static_template(False), float32 arithmetic.
    grid = np.floor((points[:, :3] - LIDAR_RANGE[:3]) / VOXEL_SIZE).astype(np.int32)
    keep = np.all((grid >= 0) & (grid < np.array([352, 200, 1])), axis=1)
    grid, points = grid[keep, ::-1], points[keep]
    unique, first, inverse = np.unique(grid, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first)[:40000]
    voxels = np.zeros((len(order), 32, 4), np.float32)
    counts = np.zeros(len(order), np.int32)
    groups = np.argsort(inverse, kind='stable')
    boundaries = np.r_[0, np.cumsum(np.bincount(inverse))]
    for i, group in enumerate(order):
        rows = groups[boundaries[group]:boundaries[group + 1]][:32]
        counts[i] = len(rows); voxels[i, :len(rows)] = points[rows]
    return voxels, unique[order], counts


def main():
    destination = OUT / 'OFFICIAL_ADAPTER_PREFLIGHT.json'
    assert not destination.exists()
    torch.set_num_threads(2)
    model, post = load_detector('mps')
    paths = json.loads((OUT / 'OLD_PCD_PREFLIGHT_MANIFEST.json').read_text())['files']
    scenes = list(scene_directories('test').values())
    rows = []; gpu_cpu = []
    # Use the author's complete method with dummy feature tensors; the removed
    # feature crops are not inputs to boxes, scores, or NMS.
    proxy = SimpleNamespace(**{k: getattr(box_utils, k) for k in dir(box_utils)})
    proxy.get_mask_for_boxes_within_range_torch = get_mask
    original = selected_method(AUTHOR / 'opencood/data_utils/post_processor/voxel_postprocessor.py',
                               'VoxelPostprocessor', 'post_process_return_in_dict', dict(namespace, F=F, box_utils=proxy))
    def get_features(unused_a, unused_b, mask):
        return torch.zeros((int(mask.sum()), 1)), torch.zeros((int(mask.sum()), 1))
    post.get_object_features = get_features
    saved = {}
    for index, item in enumerate(paths):
        path = ROOT / item['path']; assert sha(path) == item['sha256']
        scene = path.parents[1]; sensor = int(path.parent.name); frame = int(path.stem)
        sequence = scenes.index(scene)
        raw = [read_yaml(scene / str(n) / f'{frame:06d}.yaml') for n in range(2)]
        transforms = relative_poses(raw)
        points = read_pcd(path); seed = 20 + 2 * frame + sensor
        prepared = prepare_points(points, seed)
        got = voxelize(prepared); expected = reference_voxels(prepared)
        assert all(np.array_equal(a, b) for a, b in zip(got, expected)), ('voxelization', path)
        output = infer(model, points, seed)
        boxes, scores = post.detections(output, transforms[sensor])
        content = {'transformation_matrix': torch.from_numpy(transforms[sensor]).float(),
                   'anchor_box': torch.from_numpy(post.generate_anchor_box())}
        original_boxes, original_scores, _, _ = original(post, {'ego': content},
            {'ego': dict(output, spatial_features=None, spatial_features_2d=None)})
        if len(boxes):
            converted = box_utils.corner_to_center(original_boxes['ego'].numpy(), order='hwl')
            assert np.array_equal(boxes, converted) and np.array_equal(scores, original_scores['ego'].numpy())
        else:
            assert not original_boxes
        sensor_name = 'ego' if sensor == 0 else '1'
        release = AUTHOR.parent / f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_val/{sensor_name}/{sequence:04d}.txt'
        released = np.loadtxt(release, delimiter=',', ndmin=2)
        released = released[released[:, 0] == frame]
        distance = np.linalg.norm(boxes[:, None, :2] - released[:, [10, 12]][None, :, :], axis=-1)
        a, b = linear_sum_assignment(distance)
        rows.append(dict(sequence=f'{sequence:04d}', frame=frame, sensor=sensor,
                         points=len(points), voxels=len(got[0]), new_detections=len(boxes),
                         released_detections=len(released), matched_detections=len(a),
                         max_matched_xy_distance=float(distance[a, b].max(initial=0)),
                         max_matched_score_difference=float(np.abs(scores[a] - released[b, 6]).max(initial=0)),
                         voxel_reference_exact=True, original_postprocess_exact=True))
        saved[f'{sequence:04d}_{frame:06d}_{sensor}_boxes'] = boxes
        saved[f'{sequence:04d}_{frame:06d}_{sensor}_scores'] = scores
        if index == 0:
            cpu_model, _ = load_detector('cpu')
            cpu_output = infer(cpu_model, points, seed, 'cpu')
            for key in output:
                delta = float(torch.max(torch.abs(output[key] - cpu_output[key])))
                assert delta < 2e-3, ('CPU/MPS', key, delta)
                gpu_cpu.append(dict(tensor=key, max_abs_difference=delta, tolerance=.002))
            del cpu_model
        print('INPUT ADAPTER CHECK',sequence,frame,sensor,rows[-1]['max_matched_xy_distance'],flush=True)
    report = dict(passed=True, purpose='Old-data input adapter verification, not new tracking results',
                  checkpoint_sha256=sha(AUTHOR / 'official_models/no_fusion_keep_all/net_epoch60.pth'),
                  config_sha256=sha(AUTHOR / 'official_models/no_fusion_keep_all/config.yaml'),
                  adapter_sha256=sha(OUT / 'official_input_adapter.py'), checker_sha256=sha(Path(__file__)),
                  device='Apple MPS float32; CPU postprocessing', numpy=np.__version__, torch=torch.__version__,
                  rows=rows, cpu_mps=gpu_cpu,
                  voxel_source='https://github.com/traveller59/spconv/blob/master/spconv/csrc/sparse/pointops.py',
                  limitation='Released inference RNG was not saved. New inference uses a declared fixed shuffle seed; released-output deviations above are descriptive, not bitwise parity.')
    np.savez_compressed(OUT / 'old_detector_preflight.npz', **saved)
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('OFFICIAL INPUT ADAPTER PREFLIGHT PASSED',len(rows),'old source frames',flush=True)


if __name__ == '__main__':
    main()
