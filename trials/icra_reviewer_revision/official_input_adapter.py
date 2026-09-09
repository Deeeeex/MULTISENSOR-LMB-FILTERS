"""Narrow, non-executing YAML/PCD readers and released inference geometry.

The author repository and checkpoint are never modified. AST extraction below
executes only selected methods from the locally pinned, reviewed author source;
it does not execute YAML tags or remote code. No training/association code is
needed for detector inference. Ground truth has a separate conversion function.
"""
from pathlib import Path
import ast
import base64
import hashlib
import math
import sys
import textwrap

import numpy as np
import torch
import yaml
from numba import njit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
AUTHOR = ROOT / 'tmp/external_baselines/DMSTrack/V2V4Real'
CACHE = ROOT / 'tmp/external_baselines/v2v_official'
sys.path.insert(0, str(AUTHOR))
from opencood.utils import box_utils
from opencood.models.point_pillar import PointPillar

GT_RANGE = [-100, -40, -5, 100, 40, 3]
LIDAR_RANGE = np.array([-70.4, -40, -5, 70.4, 40, 3], dtype=np.float32)
VOXEL_SIZE = np.array([.4, .4, 8], dtype=np.float32)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ArrayLoader(yaml.CSafeLoader):
    """Only literal NumPy ndarray/dtype/scalar encodings, with size limits."""


def tuple_literal(loader, node):
    return tuple(loader.construct_sequence(node, deep=True))


def dtype_literal(loader, node):
    obj = loader.construct_mapping(node, deep=True)
    assert obj['args'][0] in ['f8', 'f4', 'i8', 'i4', 'u8', 'u4']
    state = obj.get('state', (3, '<'))
    assert state[1] in ['<', '>', '=']
    return np.dtype(state[1] + obj['args'][0])


def array_literal(loader, node):
    obj = loader.construct_mapping(node, deep=True)
    assert obj['args'][0] == 'literal-numpy-ndarray'
    version, shape, dtype, fortran, payload = obj['state']
    assert version == 1 and 0 <= np.prod(shape) <= 1000000
    assert isinstance(payload, bytes) and len(payload) == np.prod(shape) * dtype.itemsize
    return np.frombuffer(payload, dtype=dtype).reshape(shape, order='F' if fortran else 'C').copy()


def scalar_literal(loader, node):
    dtype, payload = loader.construct_sequence(node, deep=True)
    assert isinstance(payload, bytes) and len(payload) == dtype.itemsize
    return np.frombuffer(payload, dtype=dtype)[0].item()


ArrayLoader.add_constructor('tag:yaml.org,2002:python/tuple', tuple_literal)
ArrayLoader.add_constructor('tag:yaml.org,2002:python/name:numpy.ndarray', lambda loader, node: 'literal-numpy-ndarray')
ArrayLoader.add_constructor('tag:yaml.org,2002:python/object/apply:numpy.dtype', dtype_literal)
for prefix in ['numpy.core.multiarray', 'numpy._core.multiarray']:
    ArrayLoader.add_constructor('tag:yaml.org,2002:python/object/apply:' + prefix + '._reconstruct', array_literal)
    ArrayLoader.add_constructor('tag:yaml.org,2002:python/object/apply:' + prefix + '.scalar', scalar_literal)


def read_yaml(path):
    return yaml.load(path.read_text(), Loader=ArrayLoader)


def read_pcd(path):
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline().decode('ascii').strip()
            assert line, (path, 'Incomplete PCD header')
            if not line.startswith('#'):
                key, *values = line.split(); header[key] = values
            if line.startswith('DATA '):
                break
        assert header['DATA'] == ['ascii']
        assert header['FIELDS'] == ['x', 'y', 'z', 'rgb']
        assert header['TYPE'] == ['F'] * 4 and header['SIZE'] == ['4'] * 4
        assert header['COUNT'] == ['1'] * 4
        data = np.loadtxt(stream, dtype=np.float32, ndmin=2)
    assert data.shape == (int(header['POINTS'][0]), 4)
    # Open3D's PCD reader unpacks float32 RGB; author uses red / 255 as intensity.
    bits = np.ascontiguousarray(data[:, 3]).view(np.uint32)
    data[:, 3] = ((bits >> 16) & 255).astype(np.float32) / np.float32(255)
    assert np.isfinite(data).all()
    return data


def prepare_points(points, seed):
    # Author inference shuffles before range/ego masking. The released run did
    # not record its RNG state, so this new inference declares a fixed seed.
    points = points[np.random.RandomState(seed).permutation(len(points))]
    mask = np.all(points[:, :3] > LIDAR_RANGE[:3], axis=1) & np.all(points[:, :3] < LIDAR_RANGE[3:], axis=1)
    points = points[mask]
    ego = (points[:, 0] >= -1.95) & (points[:, 0] <= 2.95) & (points[:, 1] >= -1.1) & (points[:, 1] <= 1.1)
    return points[~ego]


@njit(cache=True)
def voxelize(points):
    """Point2VoxelCPU3d: insertion order, first 32 points, zero empty slots."""
    index = np.full((1, 200, 352), -1, dtype=np.int32)
    voxels = np.zeros((40000, 32, 4), dtype=np.float32)
    coordinates = np.zeros((40000, 3), dtype=np.int32)
    counts = np.zeros(40000, dtype=np.int32)
    size = 0
    for i in range(len(points)):
        xyz = np.floor((points[i, :3] - LIDAR_RANGE[:3]) / VOXEL_SIZE).astype(np.int32)
        x, y, z = xyz[0], xyz[1], xyz[2]
        if x < 0 or x >= 352 or y < 0 or y >= 200 or z < 0 or z >= 1:
            continue
        k = index[z, y, x]
        if k == -1:
            if size >= 40000:
                continue
            k = size; size += 1; index[z, y, x] = k
            coordinates[k, 0] = z; coordinates[k, 1] = y; coordinates[k, 2] = x
        count = counts[k]
        if count < 32:
            voxels[k, count] = points[i]; counts[k] += 1
    return voxels[:size], coordinates[:size], counts[:size]


def selected_method(path, class_name, method_name, globals_dict):
    source = path.read_text(); tree = ast.parse(source)
    parent = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name) if class_name else tree
    node = next(n for n in parent.body if isinstance(n, ast.FunctionDef) and n.name == method_name)
    # Preserve source statements verbatim except importing GT_RANGE through a
    # module that requires unavailable visualization/training dependencies.
    code = textwrap.dedent(ast.get_source_segment(source, node))
    code = code.replace('    from opencood.data_utils.datasets import GT_RANGE\n', '')
    namespace = dict(globals_dict)
    exec(compile(code, str(path) + ':' + method_name, 'exec'), namespace)
    return namespace[method_name]


namespace = dict(np=np, torch=torch, math=math, sys=sys, box_utils=box_utils, GT_RANGE=GT_RANGE)
get_mask = selected_method(AUTHOR / 'opencood/utils/box_utils.py', None,
                           'get_mask_for_boxes_within_range_torch', namespace)


class ReleasedPostprocessor:
    def __init__(self, params):
        self.params = params
        self.anchor_num = params['anchor_args']['num']

    generate_anchor_box = selected_method(AUTHOR / 'opencood/data_utils/post_processor/voxel_postprocessor.py',
                                         'VoxelPostprocessor', 'generate_anchor_box', namespace)
    delta_to_boxes3d = staticmethod(selected_method(AUTHOR / 'opencood/data_utils/post_processor/voxel_postprocessor.py',
                                                  'VoxelPostprocessor', 'delta_to_boxes3d', namespace))

    def detections(self, output, transform):
        prob = torch.sigmoid(output['psm'].cpu().permute(0, 2, 3, 1)).reshape(-1)
        decoded = self.delta_to_boxes3d(output['rm'].cpu(), torch.from_numpy(self.generate_anchor_box()))[0]
        mask = prob > self.params['target_args']['score_threshold']
        boxes, scores = decoded[mask], prob[mask]
        if len(boxes) == 0:
            return np.empty((0, 7), np.float32), np.empty(0, np.float32)
        corners = box_utils.boxes_to_corners_3d(boxes, order=self.params['order'])
        projected = box_utils.project_box3d(corners, torch.as_tensor(transform, dtype=torch.float32))
        keep = box_utils.nms_rotated(projected, scores, self.params['nms_thresh'])
        projected, scores = projected[keep], scores[keep]
        keep = get_mask(projected)
        projected, scores = projected[keep], scores[keep]
        return box_utils.corner_to_center(projected.numpy(), order='hwl'), scores.numpy()


def labels_for_pair(raw_pair, transforms):
    """Author GT geometry and ID union; never called by model.forward."""
    corners, ids = [], []
    for sensor, (raw, transform) in enumerate(zip(raw_pair, transforms)):
        objects = {}
        box_utils.project_world_objects(raw['vehicles'], objects, np.eye(4), GT_RANGE, 'hwl')
        assert len(objects) <= 100
        centers = np.array([obj['coord'][0] for obj in objects.values()]).reshape(-1, 7)
        ids.extend(int(obj['ass_id'] if obj['ass_id'] != -1 else key + 100 * sensor) for key, obj in objects.items())
        source_corners = box_utils.boxes_to_corners_3d(torch.from_numpy(centers).float(), 'hwl')
        corners.append(box_utils.project_box3d(source_corners, torch.as_tensor(transform, dtype=torch.float32)))
    corners = torch.vstack(corners)
    chosen = [ids.index(x) for x in set(ids)]
    corners, ids = corners[chosen], np.asarray(ids, dtype=np.int64)[chosen]
    mask = get_mask(corners)
    return box_utils.corner_to_center(corners[mask].numpy(), order='hwl'), ids[mask.numpy()]


def load_detector(device='mps'):
    params = read_yaml(AUTHOR / 'official_models/no_fusion_keep_all/config.yaml')
    assert params['root_dir'] == './data/train' and params['validate_dir'] == './data/test'
    model = PointPillar(params['model']['args'])
    weights = torch.load(AUTHOR / 'official_models/no_fusion_keep_all/net_epoch60.pth', map_location='cpu', weights_only=True)
    model.load_state_dict(weights, strict=True)
    model.eval().to(device)
    return model, ReleasedPostprocessor(params['postprocess'])


def infer(model, points, seed, device='mps'):
    voxels, coordinates, counts = voxelize(prepare_points(points, seed))
    processed = dict(voxel_features=torch.from_numpy(voxels).to(device),
                     voxel_coords=torch.from_numpy(np.pad(coordinates, ((0, 0), (1, 0)))).to(device),
                     voxel_num_points=torch.from_numpy(counts).to(device))
    with torch.inference_mode():
        output = model(dict(processed_lidar=processed))
    return {key: output[key].cpu() for key in ['psm', 'rm']}


def scene_directories(split):
    paths = {}
    names = ['val'] if split == 'val' else ([f'test_{i:02d}' for i in range(1, 4)] if split == 'test'
                                           else [f'train_{i:02d}' for i in range(1, 9)])
    for archive in names:
        for path in (CACHE / archive / 'files').rglob('0'):
            if path.is_dir() and path.parent.name.startswith('testoutput_CAV_data_'):
                assert path.parent.name not in paths
                paths[path.parent.name] = path.parent
    return dict(sorted(paths.items()))


def relative_poses(raw_pair):
    ego = raw_pair[0]['lidar_pose']
    return [np.linalg.inv(ego) @ raw['lidar_pose'] for raw in raw_pair]


def planar_pose(pose):
    yaw = np.arctan2(pose[1, 0], pose[0, 0]); c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, pose[0, 3]], [s, c, pose[1, 3]], [0, 0, 1.]])
