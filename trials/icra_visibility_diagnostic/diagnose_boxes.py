"""Check existing detector-footprint occlusion at saved local predictions."""
from collections import Counter
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np
from scipy.io import loadmat
from shapely.geometry import LineString, Point, Polygon

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
HISTORY = OUT.parent / 'icra_miss_history'
sys.path.insert(0, str(OUT.parent / 'icra_external_fusion'))
from analyze_v2v4real import domain

sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
inputs = {}


def bind(path, expected=None):
    digest = sha(path)
    if expected is not None:
        assert digest == expected, path
    name = str(path.relative_to(ROOT))
    if name in inputs:
        assert inputs[name] == digest
    inputs[name] = digest


def read(path):
    bind(path)
    with gzip.open(path, 'rt') as handle:
        return json.load(handle)


def footprint(box):
    c, s = np.cos(box[6]), np.sin(box[6])
    rotation = np.array([[c, -s], [s, c]])
    half = box[[5, 4]] / 2
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half
    return rotation, half, corners @ rotation.T + box[:2]


def ray_blocked(origin, query, box):
    rotation, half, corners = footprint(box)
    a, b = (origin - box[:2]) @ rotation, (query - box[:2]) @ rotation
    if (np.abs(a) <= half).all() or (np.abs(b) <= half).all():
        analytic = False
    else:
        entry, leave = 0., 1.
        for axis in range(2):
            delta = b[axis] - a[axis]
            if abs(delta) < 1e-12:
                if abs(a[axis]) > half[axis]:
                    entry, leave = 1., 0.
                    break
            else:
                ends = sorted([(-half[axis] - a[axis]) / delta, (half[axis] - a[axis]) / delta])
                entry, leave = max(entry, ends[0]), min(leave, ends[1])
        analytic = entry <= leave and leave < 1. and leave > 0.
    polygon = Polygon(corners)
    independent = (not polygon.covers(Point(origin)) and not polygon.covers(Point(query))
                   and polygon.intersects(LineString([origin, query])))
    assert bool(analytic) == bool(independent), (origin, query, box)
    return bool(analytic)


class Detections:
    def __init__(self, sequence, unit):
        self.sequence = sequence
        path = ROOT / unit['data_path']
        bind(path, unit['input_sha256'])
        self.data = loadmat(path)
        if sequence == '0006':
            self.kind = 'released'
            self.arrays = []
            manifest_path = OUT.parent / 'icra_external_fusion/v2v4real_input_manifest.json'
            bind(manifest_path)
            manifest = json.loads(manifest_path.read_text())
            author = ROOT / 'tmp/external_baselines/DMSTrack'
            for sensor in ['ego', '1']:
                path = author / f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_val/{sensor}/0006.txt'
                bind(path, manifest['source_files'][str(path.relative_to(author))])
                self.arrays.append(np.loadtxt(path, delimiter=',', ndmin=2))
        else:
            self.kind = 'frozen_npz'
            folder = OUT.parent / ('icra_association_test' if sequence.startswith('v2xt') else 'icra_v2x_transfer')
            name = unit['original_sequence']
            manifest_path = folder / 'DETECTION_MANIFEST.json'
            bind(manifest_path)
            manifest = json.loads(manifest_path.read_text())
            record = next(row for row in manifest['sequences'] if row['sequence'] == name)
            path = ROOT / record['path']
            bind(path, record['sha256'])
            self.archive = np.load(path, allow_pickle=False)
        self.cache = {}

    def get(self, frame, source):
        key = (frame, source)
        if key not in self.cache:
            if self.kind == 'released':
                rows = self.arrays[source - 1]
                rows = rows[rows[:, 0] == frame - 1]
                boxes = rows[:, [10, 12, 11, 7, 8, 9, 13]]
                scores = rows[:, 6]
            else:
                boxes = self.archive[f'{frame-1:06d}_{source-1}_boxes']
                scores = self.archive[f'{frame-1:06d}_{source-1}_scores']
            poses = self.data['positions'][:, :, frame - 1]
            keep = (np.sum((boxes[:, :2] - poses[:, source - 1]) ** 2, axis=1) <= 1600)
            keep &= domain(np.c_[boxes[:, :2], np.zeros((len(boxes), 2))], poses)
            boxes, scores = boxes[keep], scores[keep]
            expected = self.data['measurements'][source - 1, frame - 1].T
            assert boxes[:, :2].shape == expected.shape
            assert np.allclose(boxes[:, :2], expected, atol=1e-7, rtol=0), (self.sequence, key)
            assert (boxes[:, 3:6] > 0).all()
            self.cache[key] = (boxes, scores, poses[:, source - 1])
        return self.cache[key]


def native_index(run, name, width):
    values = np.asarray(run[name], float).reshape(-1, width)
    index = {tuple(row[:4].astype(int)): row for row in values}
    assert len(index) == len(values)
    return index


def evaluate(detector, condition, group, key, gaussian, delta, extra):
    frame, source, bt, bl = map(int, key)
    query = gaussian[4:6]
    boxes, scores, origin = detector.get(frame, source)
    blocked = [j for j, box in enumerate(boxes) if ray_blocked(origin, query, box)]
    return dict(sequence=detector.sequence, condition=condition, group=group,
                frame=frame, source=source, label=[bt, bl], predicted_xy=query.tolist(),
                source_xy=origin.tolist(), local_delta=float(delta),
                box_count=len(boxes), center_ray_occluded=bool(blocked),
                blocking_boxes=[dict(index=j, box=boxes[j].tolist(), score=float(scores[j])) for j in blocked],
                **extra)


def main():
    false_path = HISTORY / 'FALSE_SUPPORT_DIAGNOSTIC.json'
    bind(false_path)
    false = json.loads(false_path.read_text())
    assert false['passed']
    for name, expected in false['inputs'].items():
        bind(ROOT / name, expected)
    units = {}
    for stage in ['miss_history_preflight', 'miss_history_screen']:
        path = HISTORY / 'stages' / (stage + '.json')
        bind(path)
        config = json.loads(path.read_text())
        units.update({u['sequence']: u for u in config['units']})
    all_rows, summaries = [], []
    for case in false['cases']:
        sequence, condition = case['sequence'], case['condition']
        detector = Detections(sequence, units[sequence])
        native_path = HISTORY / 'results/miss_history_screen' / f'{sequence}_{condition}_marked_gaussian_evidence_miss_history.json.gz'
        run = read(native_path)['runs']
        fusion = native_index(run, 'iterationRecords', 60)
        local = native_index(run, 'localGaussianRecords', 32)
        seen, rows = set(), []
        for item in false['rows']:
            if item['sequence'] != sequence or item['condition'] != condition:
                continue
            row = fusion[(item['frame'], item['robot'], *item['label'])]
            source_labels = row[31:35].reshape(2, 2).astype(int)
            for slot, attenuated in enumerate(item['attenuated_admitted_negative_sources']):
                if not attenuated:
                    continue
                source = item['robot'] if slot == 0 else 3 - item['robot']
                key = (item['frame'], source, *map(int, source_labels[slot]))
                if key in seen:
                    continue
                seen.add(key)
                assert key in local
                rows.append(evaluate(detector, condition, 'new_false_negative_source', key, local[key],
                                     item['source_deltas'][slot], dict(other_positive_support=item['positive_joint_support'][1-slot])))
        summary = dict(sequence=sequence, condition=condition, group='new_false_negative_source',
                       queries=len(rows), center_ray_occluded=sum(row['center_ray_occluded'] for row in rows),
                       unique_sensor_frames=len(detector.cache))
        all_rows.extend(rows)
        summaries.append(summary)
        print('DETECTOR VISIBILITY', json.dumps(summary), flush=True)
    sequence = 'v2xt_0001'
    for condition in ['reliable', 'intermittent']:
        detector = Detections(sequence, units[sequence])
        native_path = HISTORY / 'results/miss_history_preflight' / f'{sequence}_{condition}_marked_gaussian_evidence_miss_history.json.gz'
        data = read(native_path)
        local = native_index(data['runs'], 'localGaussianRecords', 32)
        increments = native_index(data['runs'], 'localIncrementRecords', 12)
        rows, missing = [], []
        for frame in range(53, 123):
            ids = np.asarray(data['truthIds'][frame - 1]).reshape(-1)
            assert np.count_nonzero(ids == 5) == 1
            truth = np.asarray(data['truth'][frame - 1]).reshape(4, -1)[:2, ids == 5].ravel()
            candidates = [(key, value) for key, value in local.items()
                          if key[:2] == (frame, 2) and np.sum((value[4:6] - truth) ** 2) <= 4]
            if not candidates:
                missing.append(frame)
                continue
            key, gaussian = max(candidates, key=lambda pair: increments[pair[0]][4])
            inc = increments[key]
            rows.append(evaluate(detector, condition, 'repaired_true_source2', key, gaussian, inc[6],
                                 dict(predicted_existence=float(inc[4]), local_existence=float(inc[5]),
                                      predicted_truth_distance=float(np.linalg.norm(gaussian[4:6] - truth)))))
        summary = dict(sequence=sequence, condition=condition, group='repaired_true_source2',
                       declared_frames=70, queries=len(rows), missing_prediction_frames=missing,
                       negative_queries=sum(row['local_delta'] < 0 for row in rows),
                       center_ray_occluded=sum(row['center_ray_occluded'] for row in rows),
                       unique_sensor_frames=len(detector.cache))
        all_rows.extend(rows)
        summaries.append(summary)
        print('DETECTOR VISIBILITY', json.dumps(summary), flush=True)
    answer = dict(passed=True, summaries=summaries, rows=all_rows, inputs=inputs,
                  source_sha256=sha(Path(__file__)), protocol_sha256=sha(OUT / 'PROTOCOL.md'),
                  independent_geometry='Every source-query-box result checked with Shapely polygon intersection.',
                  scope='Fixed post-outcome subsets; current-source prepared detection boxes and saved predicted means. No method run.')
    path = OUT / 'BOX_VISIBILITY_DIAGNOSTIC.json'
    assert not path.exists()
    path.write_text(json.dumps(answer, indent=2, allow_nan=False) + '\n')
    print('BOX VISIBILITY DIAGNOSTIC COMPLETE', len(all_rows), flush=True)


if __name__ == '__main__':
    main()
