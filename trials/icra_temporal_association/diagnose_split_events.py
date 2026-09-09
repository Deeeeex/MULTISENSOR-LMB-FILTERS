"""Describe every restored S branch; annotations are used only offline."""
from collections import defaultdict, Counter
from pathlib import Path
import gzip
import hashlib
import json

import numpy as np

from identity_metrics import map_indices, truth_assignment, domain

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
BASE = 'marked_gaussian_evidence'
ARM = BASE + '_assoc_split'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def closest(xy, truth, ids):
    if not len(truth):
        return None
    distance = np.linalg.norm(truth[:, :2] - xy, axis=1)
    index = int(np.argmin(distance))
    return dict(truth_id=int(ids[index]), distance_m=float(distance[index]))


def main():
    destination = OUT / 'SPLIT_EVENT_DIAGNOSTIC.json'
    assert not destination.exists()
    selection_path = OUT / 'RESTORED_DEVELOPMENT_SELECTION.json'
    selection = json.loads(selection_path.read_text())
    assert selection['passed'] and selection['selected']['arm'] == ARM
    scores = {(r['sequence'], r['condition'], r['arm']): r for r in selection['rows']}
    inputs = {str(selection_path.relative_to(ROOT)): sha(selection_path)}
    events, rows = [], []
    for stage in ['association_restored_preflight', 'association_restored_development_rest']:
        for path in sorted((OUT / 'results' / stage).glob('*_' + ARM + '.json.gz')):
            assert sha(path) == selection['inputs'][str(path.relative_to(ROOT))]
            inputs[str(path.relative_to(ROOT))] = sha(path)
            with gzip.open(path, 'rt') as handle:
                data = json.load(handle)
            run = data['runs']; seq, condition = data['sequence'], data['condition']
            local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
            direct = np.asarray(run['localDirectRecords'], float).reshape(-1, 12)
            inc = {tuple(r[:4].astype(int)): r for r in np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)}
            index = {tuple(r[:4].astype(int)): i for i, r in enumerate(local)}
            frames = defaultdict(list)
            for i, row in enumerate(local):
                frames[tuple(row[:2].astype(int))].append(i)
            exists = np.array([inc[tuple(r[:4].astype(int))][5] for r in local])
            reopen = {tuple(r[:4].astype(int)): r for r in np.asarray(run['associationReopenings'], float).reshape(-1, 8)}
            splits = np.asarray(run['associationSplits'], float).reshape(-1, 6)
            poses = np.asarray(data['positions'])
            maps, outputs = {}, {}
            for t in range(1, len(data['time']) + 1):
                truth = np.asarray(data['truth'][t-1], float).reshape(4, -1).T
                ids = np.asarray(data['truthIds'][t-1]).reshape(-1)
                for n in [1, 2]:
                    group = np.asarray(frames[t, n], int)
                    chosen = group[map_indices(exists[group])]
                    chosen = chosen[domain(local[chosen, 18:22], poses[:, :, t-1])]
                    assignment = truth_assignment(local[chosen, 18:22], truth, ids, 2.)
                    maps[t, n] = {tuple(local[chosen[i], 2:4].astype(int)): int(label) for i, label in assignment.items()}
                    final = np.asarray(run['rawEstimates'][n-1 + 2*(t-1)], float).reshape(-1, 4)
                    keys = np.asarray(run['labels'][n-1 + 2*(t-1)], int).reshape(2, -1).T
                    keep = domain(final, poses[:, :, t-1]); final, keys = final[keep], keys[keep]
                    assignment = truth_assignment(final, truth, ids, 2.)
                    outputs[t, n] = {tuple(keys[i]): int(label) for i, label in assignment.items()}
            labels = Counter()
            for split in splits:
                t, n, bt, bl, abt, abl = split.astype(int)
                source_key, alias = (bt, bl), (abt, abl)
                i, j = index[t, n, bt, bl], index[t, 3-n, bt, bl]
                features = direct[[i, j], 4:]
                r = exists[[i, j]]
                q = r * features[:, 0] * features[:, 6]
                evidence = reopen[t, n, bt, bl][4:]
                identity = [maps[t, source].get(source_key) for source in [n, 3-n]]
                kind = 'unscored' if None in identity else ('different_objects' if identity[0] != identity[1] else 'same_object')
                labels[kind] += 1
                truth = np.asarray(data['truth'][t-1], float).reshape(4, -1).T
                ids = np.asarray(data['truthIds'][t-1]).reshape(-1)
                future = []
                for tt in range(t, min(t + 11, len(data['time']) + 1)):
                    future.append(dict(frame=tt, original_truth=outputs[tt, n].get(source_key),
                                       branch_truth=outputs[tt, n].get(alias)))
                events.append(dict(sequence=seq, condition=condition, frame=int(t), receiver=int(n),
                    original=list(map(int, source_key)), alias=list(map(int, alias)),
                    original_local_map_truth=identity, classification=kind,
                    existence=r.tolist(), majority_confidence=q.tolist(), observation_features=features.tolist(),
                    normalized_discrepancy=float(evidence[0]), qualified_samples=int(evidence[1]),
                    current_nis=float(evidence[2]), summed_nis=float(evidence[3]),
                    posterior_distance_m=float(np.linalg.norm(local[i, 18:20] - local[j, 18:20])),
                    observation_distance_m=float(np.linalg.norm(features[0, 1:3] - features[1, 1:3])),
                    nearest_detection_truth=[closest(f[1:3], truth, ids) for f in features],
                    next_eleven_frames=future))
            rows.append(dict(sequence=seq, condition=condition, branches=len(splits), classification=dict(labels),
                changes={k: scores[seq, condition, ARM][k] - scores[seq, condition, BASE][k]
                         for k in ['ospa', 'loc2', 'miss2', 'false2']}))
            print('SPLIT EVENTS CHECKED', seq, condition, len(splits), dict(labels), flush=True)
    result = dict(passed=True, events=events, rows=rows, branch_count=len(events), inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'identity_metrics.py']},
        interpretation='Offline diagnostic on already exposed development trajectories. Strict 2 m local MAP assignment determines same/different/unscored labels. Nearest-detection truth is descriptive and does not replace that assignment. No new candidate is evaluated or chosen here.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('ALL RESTORED SPLIT EVENTS DIAGNOSED', len(events), flush=True)


if __name__ == '__main__':
    main()
