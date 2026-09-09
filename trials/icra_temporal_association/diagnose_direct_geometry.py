"""Measure current-observation separation on the immutable original trajectory."""
from collections import Counter, defaultdict
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_reviewer_revision'))
from review_gaussian_audit import unpack
from association_math import objective
from diagnose_association_v2 import map_indices, truth_assignment, domain


def main():
    dest = OUT / 'DIRECT_GEOMETRY_DIAGNOSTIC.json'
    assert not dest.exists()
    audit = json.loads((OUT / 'audit_association_instrumentation_development.json').read_text())
    rows, inputs = [], {}
    for seq in range(9):
        for condition in ['reliable', 'intermittent']:
            path = OUT / 'results/association_instrumentation_development' / f'{seq:04d}_{condition}_marked_gaussian_evidence.json.gz'
            name = str(path.relative_to(ROOT))
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            assert digest == audit['inputs'][name]
            inputs[name] = digest
            with gzip.open(path, 'rt') as handle:
                data = json.load(handle)
            run = data['runs']
            local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
            direct = np.asarray(run['localDirectRecords'], float).reshape(-1, 12)
            inc = {tuple(r[:4].astype(int)): r for r in np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)}
            r = np.array([inc[tuple(row[:4].astype(int))][5] for row in local])
            covariance = unpack(local[:, 22:32])
            records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
            groups, fusion = defaultdict(list), defaultdict(list)
            for i, row in enumerate(local):
                groups[int(row[0]), int(row[1])].append(i)
            for i, row in enumerate(records):
                fusion[int(row[0]), int(row[1])].append(i)
            positions = np.asarray(data['positions'])
            histories = {(n, mode): [] for n in [1, 2] for mode in ['direct', 'temporal']}
            counts = {mode: Counter() for mode in ['direct', 'temporal']}
            for t in range(1, len(data['time']) + 1):
                truth = np.asarray(data['truth'][t - 1], float).reshape(4, -1).T
                truth_ids = np.asarray(data['truthIds'][t - 1]).reshape(-1)
                maps = {}
                for n in [1, 2]:
                    index = np.asarray(groups[t, n], int)
                    chosen = index[map_indices(r[index])]
                    chosen = chosen[domain(local[chosen, 18:22], positions[:, :, t - 1])]
                    assigned = truth_assignment(local[chosen, 18:22], truth, truth_ids, 2.)
                    maps[n] = {int(chosen[i]): value for i, value in assigned.items()}
                for n in [1, 2]:
                    if not data['delivered'][n - 1][2 - n][t - 1]:
                        continue
                    left, right = np.asarray(groups[t, n], int), np.asarray(groups[t, 3 - n], int)
                    left_keys, right_keys = local[left, 2:4].astype(int), local[right, 2:4].astype(int)
                    li, ri = {tuple(k): i for i, k in enumerate(left_keys)}, {tuple(k): j for j, k in enumerate(right_keys)}
                    for mode in counts:
                        info = objective(left_keys, right_keys, local[left, 18:22], local[right, 18:22],
                            covariance[left], covariance[right], direct[left, 4:], direct[right, 4:],
                            r[left], r[right], histories[n, mode], mode, t)
                        histories[n, mode] = info['history']
                        for row in records[fusion[t, n]]:
                            keys = row[31:35].reshape(2, 2).astype(int)
                            if not (keys[:, 0] > 0).all():
                                continue
                            i, j = li[tuple(keys[0])], ri[tuple(keys[1])]
                            kind = 'known' if np.array_equal(keys[0], keys[1]) else 'assigned'
                            if left[i] in maps[n] and right[j] in maps[3 - n]:
                                status = 'wrong' if maps[n][left[i]] != maps[3 - n][right[j]] else 'correct'
                            else:
                                status = 'unscored'
                            counts[mode][kind + '_' + status] += 1
                            if info['qualified'][i, j] and info['costs'][i, j] > 1:
                                counts[mode][kind + '_' + status + '_observation_conflict'] += 1
            for mode, counter in counts.items():
                rows.append(dict(sequence=f'{seq:04d}', condition=condition, mode=mode, counts=dict(counter)))
            print('DIRECT GEOMETRY', seq, condition, flush=True)
    aggregate = []
    for condition in ['reliable', 'intermittent']:
        for mode in ['direct', 'temporal']:
            total = Counter()
            for row in rows:
                if row['condition'] == condition and row['mode'] == mode:
                    total.update(row['counts'])
            aggregate.append(dict(condition=condition, mode=mode, counts=dict(total)))
    result = dict(passed=True, rows=rows, aggregate=aggregate, inputs=inputs,
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        interpretation='Original GCE trajectory only. This measures observation separation, not the recursively changed candidate outcome.')
    dest.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('DIRECT GEOMETRY COMPLETE', flush=True)


if __name__ == '__main__':
    main()
