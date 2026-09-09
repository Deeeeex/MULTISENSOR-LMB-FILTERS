"""V2X file/pose adapter around the unchanged V2V4Real detector."""
from pathlib import Path
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_reviewer_revision'))
import official_input_adapter as old

sha = old.sha
read_yaml = old.read_yaml
load_detector = old.load_detector
infer = old.infer
planar_pose = old.planar_pose
torch = old.torch
AUTHOR = ROOT / 'tmp/external_baselines/V2X-Real'
VEHICLE_TYPES = {'LongVehicle', 'Car', 'PoliceCar'}


def pose_matrix(values):
    values = np.asarray(values, dtype=float)
    assert values.shape == (6,) and np.isfinite(values).all()
    x, y, z, roll, yaw, pitch = values
    r, y_angle, p = np.deg2rad([roll, yaw, pitch])
    cr, sr, cy, sy, cp, sp = np.cos(r), np.sin(r), np.cos(y_angle), np.sin(y_angle), np.cos(p), np.sin(p)
    result = np.eye(4)
    result[:3, :3] = [[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                      [sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                      [sp, -cp * sr, cp * cr]]
    result[:3, 3] = [x, y, z]
    return result


def relative_poses(raw_pair):
    poses = [pose_matrix(raw['lidar_pose']) for raw in raw_pair]
    return [np.linalg.inv(poses[0]) @ pose for pose in poses]


def read_points(path):
    assert path.stat().st_size % 16 == 0
    points = np.fromfile(path, dtype='<f4').reshape(-1, 4)
    # The release's BaseDataset calls load_lidar_bin(..., zero_intensity=True).
    points = points[~np.isnan(points[:, :3]).any(axis=1)]
    assert np.isfinite(points[:, :3]).all()
    points[:, 3] = 0
    assert len(points) > 0
    return points


def labels_for_pair(raw_pair, transforms):
    """Only scoring calls this annotation path; detector inference never does."""
    ego_to_world = pose_matrix(raw_pair[0]['lidar_pose'])
    world_to_ego = np.linalg.inv(ego_to_world)
    merged = {}
    for raw in raw_pair:
        for label, obj in raw['vehicles'].items():
            if obj['obj_type'] not in VEHICLE_TYPES:
                continue
            label = int(label)
            center = np.asarray(obj['location']) + np.asarray(obj['center'])
            extent = np.asarray(obj['extent'])
            assert center.shape == extent.shape == (3,) and np.all(extent > 0)
            if label in merged:
                previous = merged[label]
                assert previous['obj_type'] == obj['obj_type']
                # The release has source-specific annotations for shared IDs.
                # Match the author's first-occurrence union: agent 1 first.
            else:
                merged[label] = obj
    boxes = []; labels = []
    for label, obj in sorted(merged.items()):
        center = np.asarray(obj['location']) + np.asarray(obj['center'])
        extent = np.asarray(obj['extent'])
        transform = world_to_ego @ pose_matrix([*center, *obj['angle']])
        corners = old.box_utils.create_bbx(extent)
        corners = (transform @ np.c_[corners, np.ones(8)].T).T[:, :3]
        # Same final all-corners x/y crop as the unchanged author detector.
        if not bool(old.get_mask(torch.from_numpy(corners[None]).float())[0]):
            continue
        xyz = transform[:3, 3]
        assert np.allclose(xyz, corners.mean(0), rtol=0, atol=1e-11)
        yaw = np.arctan2(transform[1, 0], transform[0, 0])
        boxes.append([*xyz, 2 * extent[2], 2 * extent[1], 2 * extent[0], yaw])
        labels.append(label)
    return np.asarray(boxes, dtype=float).reshape(-1, 7), np.asarray(labels, dtype=np.int64)
