"""Offline identity diagnostics shared by original and revised matching."""
from collections import Counter, defaultdict
import numpy as np

from diagnose_association_v2 import map_indices, truth_assignment, domain


def evaluate_identities(data):
    run = data['runs']
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
    inc = {tuple(r[:4].astype(int)): r for r in increments}
    existence = np.array([inc[tuple(r[:4].astype(int))][5] for r in local])
    frames, fused = defaultdict(list), defaultdict(list)
    for i, r in enumerate(local):
        frames[int(r[0]), int(r[1])].append(i)
    for i, r in enumerate(records):
        fused[int(r[0]), int(r[1])].append(i)
    counts = {c: Counter() for c in [2, 12]}
    previous = {(n, c): {} for n in [1, 2] for c in counts}
    positions = np.asarray(data['positions'])
    delivery = np.asarray(data['delivered'])
    all_pairs = 0
    for t in range(1, len(data['time']) + 1):
        truth = np.asarray(data['truth'][t - 1], float).reshape(4, -1).T
        truth_ids = np.asarray(data['truthIds'][t - 1]).reshape(-1)
        maps = {}
        for n in [1, 2]:
            group = np.asarray(frames[t, n], int)
            chosen = group[map_indices(existence[group])]
            chosen = chosen[domain(local[chosen, 18:22], positions[:, :, t - 1])]
            final = np.asarray(run['rawEstimates'][n - 1 + 2 * (t - 1)], float).reshape(-1, 4)
            labels = np.asarray(run['labels'][n - 1 + 2 * (t - 1)], int).reshape(2, -1).T
            keep = domain(final, positions[:, :, t - 1])
            final, labels = final[keep], labels[keep]
            assert np.array_equal(final, np.asarray(run['estimates'][n - 1 + 2 * (t - 1)], float).reshape(-1, 4))
            for cutoff, stat in counts.items():
                assignment = truth_assignment(local[chosen, 18:22], truth, truth_ids, cutoff)
                maps[n, cutoff] = {tuple(local[chosen[i], 2:4].astype(int)): value for i, value in assignment.items()}
                output_assignment = truth_assignment(final, truth, truth_ids, cutoff)
                now = {tuple(labels[i]): value for i, value in output_assignment.items()}
                last = previous[n, cutoff]
                stat['final_assigned_estimates'] += len(now)
                for label in now.keys() & last.keys():
                    stat['label_truth_transition_opportunities'] += 1
                    stat['label_truth_switches'] += int(now[label] != last[label])
                now_truth, last_truth = {v: k for k, v in now.items()}, {v: k for k, v in last.items()}
                for identity in now_truth.keys() & last_truth.keys():
                    stat['truth_label_transition_opportunities'] += 1
                    stat['truth_label_switches'] += int(now_truth[identity] != last_truth[identity])
                previous[n, cutoff] = now
        for n in [1, 2]:
            rows = records[fused[t, n]]
            if not delivery[n - 1, 2 - n, t - 1]:
                assert not len(rows)
                continue
            pairs = set()
            for row in rows:
                keys = row[31:35].reshape(2, 2).astype(int)
                if (keys[:, 0] > 0).all():
                    pairs.add((tuple(keys[0]), tuple(keys[1])))
            assert len(pairs) == int(run['matchedLabels'][n - 1][t - 1])
            all_pairs += len(pairs)
            for cutoff, stat in counts.items():
                left, right = maps[n, cutoff], maps[3 - n, cutoff]
                invleft, invright = {v: k for k, v in left.items()}, {v: k for k, v in right.items()}
                common = invleft.keys() & invright.keys()
                stat['common_identity_opportunities'] += len(common)
                stat['common_identity_pairs_missed'] += sum((invleft[v], invright[v]) not in pairs for v in common)
                for a, b in pairs:
                    kind = 'known' if a == b else 'assigned'
                    if a in left and b in right:
                        outcome = 'correct' if left[a] == right[b] else 'wrong'
                    else:
                        outcome = 'unscored'
                    stat[kind + '_' + outcome] += 1
    return dict(pairs=all_pairs, cutoffs={str(c): dict(v) for c, v in counts.items()},
        definition='2 m/12 m maximum-cardinality center assignment; adjacent frames only. Both label-to-truth and truth-to-label changes are reported; this is not an official benchmark identity metric.')


def pooled(rows):
    result = {}
    for cutoff in ['2', '12']:
        counts = Counter()
        for row in rows:
            counts.update(row['cutoffs'][cutoff])
        wrong = counts['known_wrong'] + counts['assigned_wrong']
        scored = wrong + counts['known_correct'] + counts['assigned_correct']
        result[cutoff] = dict(counts=dict(counts), scored_pairs=scored, wrong_pairs=wrong,
            wrong_pair_rate=wrong / scored if scored else None,
            missed_common_pair_rate=counts['common_identity_pairs_missed'] / counts['common_identity_opportunities']
                if counts['common_identity_opportunities'] else None)
    return result
