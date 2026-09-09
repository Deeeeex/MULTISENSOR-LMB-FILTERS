"""Offline identity diagnosis of previously audited native trajectories."""
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent
sys.path.insert(0, str(TRIALS / 'icra_reviewer_revision'))
from review_gaussian_audit import unpack
sys.path.insert(0, str(TRIALS / 'icra_external_fusion'))
from analyze_v2v4real import domain

ARMS = ['marked_gaussian_evidence', 'marked_gaussian_evidence_guarded_scalar']
CUTOFFS = [2., 12.]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_indices(probabilities):
    p = np.asarray(probabilities) - 1e-6
    pmf = np.array([1.])
    for value in p:
        pmf = np.convolve(pmf, [1 - value, value])
    return np.argsort(-p, kind='stable')[:int(pmf.argmax())]


def truth_assignment(points, truth, truth_ids, cutoff):
    result = {}
    if not len(points) or not len(truth):
        return result
    cost = ((points[:, None, :2] - truth[None, :, :2]) ** 2).sum(2)
    valid = cost < cutoff ** 2
    # An invalid assignment costs more than all valid edges combined.
    penalized = np.where(valid, cost, (max(cost.shape) + 1) * cutoff ** 2)
    ii, jj = linear_sum_assignment(penalized)
    for i, j in zip(ii, jj):
        if valid[i, j]:
            result[int(i)] = int(truth_ids[j])
    return result


def skl_grid(mu, cov, precision, left, right):
    a, b = mu[left], mu[right]
    p, q, jp, jq = cov[left], cov[right], precision[left], precision[right]
    delta = a[:, None, :] - b[None, :, :]
    trace = np.einsum('aij,bji->ab', jp, q) + np.einsum('bij,aji->ab', jq, p)
    quadratic = np.einsum('abi,aij,abj->ab', delta, jp, delta)
    quadratic += np.einsum('abi,bij,abj->ab', delta, jq, delta)
    return np.maximum(0., .25 * (trace + quadratic - 8))


def diagnostic(path, expected, dataset, sequence, condition):
    assert sha(path) == expected, path
    with gzip.open(path, 'rt') as handle:
        data = json.load(handle)
    run = data['runs']
    arm = run['arm']
    T = len(data['time'])
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
    ids = {tuple(row[:4].astype(int)): i for i, row in enumerate(local)}
    increment_ids = {tuple(row[:4].astype(int)): row for row in increments}
    existence = np.array([increment_ids[tuple(row[:4].astype(int))][5] for row in local])
    means = local[:, 18:22]
    cov = unpack(local[:, 22:32])
    precision = np.linalg.inv(cov)
    local_frame = defaultdict(list)
    record_frame = defaultdict(list)
    for i, row in enumerate(local):
        local_frame[int(row[0]), int(row[1])].append(i)
    for i, row in enumerate(records):
        record_frame[int(row[0]), int(row[1])].append(i)
    counts = Counter()
    distributions = defaultdict(list)
    identity = {str(int(c)): Counter() for c in CUTOFFS}
    previous_final = {(n, c): {} for n in [1, 2] for c in CUTOFFS}
    previous_pairs = {1: {}, 2: {}}
    errors = []
    positions = np.asarray(data['positions'])
    delivered = np.asarray(data['delivered'])
    for t in range(1, T + 1):
        truth = np.asarray(data['truth'][t - 1], float).reshape(4, -1).T
        truth_ids = np.asarray(data['truthIds'][t - 1]).reshape(-1)
        assert len(truth) == len(truth_ids) == len(set(truth_ids))
        source_map = {}
        for n in [1, 2]:
            group = np.asarray(local_frame[t, n], int)
            chosen = group[map_indices(existence[group])]
            chosen = chosen[domain(means[chosen], positions[:, :, t - 1])]
            for cutoff in CUTOFFS:
                assignments = truth_assignment(means[chosen], truth, truth_ids, cutoff)
                source_map[n, cutoff] = {int(chosen[i]): value for i, value in assignments.items()}
            final = np.asarray(run['rawEstimates'][n - 1 + 2 * (t - 1)], float).reshape(-1, 4)
            labels = np.asarray(run['labels'][n - 1 + 2 * (t - 1)], int).reshape(2, -1).T
            assert len(final) == len(labels)
            keep = domain(final, positions[:, :, t - 1])
            final, labels = final[keep], labels[keep]
            saved = np.asarray(run['estimates'][n - 1 + 2 * (t - 1)], float).reshape(-1, 4)
            assert np.array_equal(final, saved)
            for cutoff in CUTOFFS:
                assignments = truth_assignment(final, truth, truth_ids, cutoff)
                now = {tuple(labels[i]): v for i, v in assignments.items()}
                stat = identity[str(int(cutoff))]
                stat['final_assigned_estimates'] += len(now)
                for label in now.keys() & previous_final[n, cutoff].keys():
                    stat['consecutive_identity_opportunities'] += 1
                    stat['consecutive_identity_switches'] += int(now[label] != previous_final[n, cutoff][label])
                previous_final[n, cutoff] = now
        for n in [1, 2]:
            other = 3 - n
            rows = records[record_frame[t, n]]
            if not delivered[n - 1, other - 1, t - 1]:
                assert not len(rows)
                continue
            left, right = local_frame[t, n], local_frame[t, other]
            left_keys = {tuple(local[i, 2:4].astype(int)): i for i in left}
            right_keys = {tuple(local[i, 2:4].astype(int)): i for i in right}
            known = left_keys.keys() & right_keys.keys()
            pairs = []
            pair_info = {}
            partners = {}
            for row in rows:
                originals = row[31:35].reshape(2, 2).astype(int)
                if not (originals[:, 0] > 0).all():
                    continue
                akey, bkey = map(tuple, originals)
                i, j = left_keys[akey], right_keys[bkey]
                kind = 'known' if akey == bkey else 'assigned'
                pairs.append((i, j))
                pair_info[i, j] = (kind, akey, bkey, row)
                partners[akey] = bkey
                counts[kind + '_pairs'] += 1
            assert len(pairs) == int(run['matchedLabels'][n - 1][t - 1])
            assert {(a, b) for (i, j), (_, a, b, _) in pair_info.items() if a == b} == {(k, k) for k in known}
            counts['delivered_receiver_frames'] += 1
            for label in partners.keys() & previous_pairs[n].keys():
                counts['repeated_pair_opportunities'] += 1
                counts['pair_partner_changes'] += int(partners[label] != previous_pairs[n][label])
            previous_pairs[n] = partners
            free_left = [i for key, i in left_keys.items() if key not in known]
            free_right = [i for key, i in right_keys.items() if key not in known]
            if free_left and free_right:
                costs = skl_grid(means, cov, precision, free_left, free_right)
                a, b = costs.shape
                augmented = np.full((a + b, a + b), max(1e6, float(costs.max())) * 100)
                augmented[:a, :b] = costs
                augmented[np.arange(a), b + np.arange(a)] = 50.
                augmented[a + np.arange(b), np.arange(b)] = 50.
                augmented[a:, b:] = 0.
                ii, jj = linear_sum_assignment(augmented)
                optimal = augmented[ii, jj].sum()
                li, ri = {v: k for k, v in enumerate(free_left)}, {v: k for k, v in enumerate(free_right)}
                assigned = [(i, j) for i, j in pairs if i in li]
                observed = sum(costs[li[i], ri[j]] for i, j in assigned) + 50 * (a + b - 2 * len(assigned))
                assert np.isclose(observed, optimal, atol=1e-6, rtol=1e-9), (t, n, observed, optimal)
                counts['assignment_objectives_verified'] += 1
            chosen_pairs = set(pairs)
            for cutoff in CUTOFFS:
                stat = identity[str(int(cutoff))]
                lmap, rmap = source_map[n, cutoff], source_map[other, cutoff]
                invleft, invright = {v: i for i, v in lmap.items()}, {v: j for j, v in rmap.items()}
                common = invleft.keys() & invright.keys()
                stat['common_identity_opportunities'] += len(common)
                stat['common_identity_pairs_missed'] += sum((invleft[v], invright[v]) not in chosen_pairs for v in common)
                for i, j in pairs:
                    kind, akey, bkey, row = pair_info[i, j]
                    if i not in lmap or j not in rmap:
                        stat[kind + '_pairs_unscored'] += 1
                        continue
                    wrong = lmap[i] != rmap[j]
                    label = kind + ('_wrong' if wrong else '_correct')
                    stat[label] += 1
                    delta = means[i] - means[j]
                    distance = float(np.linalg.norm(delta[:2]))
                    mahal = float(delta[:2] @ np.linalg.solve(cov[i, :2, :2] + cov[j, :2, :2], delta[:2]))
                    distributions[f'{int(cutoff)}m_{label}_distance'].append(distance)
                    distributions[f'{int(cutoff)}m_{label}_mahal'].append(mahal)
                    if cutoff == 2. and wrong:
                        errors.append(dict(frame=t, receiver=n, kind=kind,
                            local_label=list(akey), remote_label=list(bkey),
                            local_truth=lmap[i], remote_truth=rmap[j],
                            local_r=float(existence[i]), remote_r=float(existence[j]),
                            distance_m=distance, position_mahalanobis_squared=mahal,
                            local_xy=means[i, :2].tolist(), remote_xy=means[j, :2].tolist(),
                            fused_xy=row[4:6].tolist(), fused_r=float(row[9])))
    summary = dict(dataset=dataset, sequence=sequence, condition=condition, arm=arm,
                   frames=T, counts=dict(counts), identity={k: dict(v) for k, v in identity.items()},
                   distributions={k: dict(count=len(v), mean=float(np.mean(v)),
                       q50=float(np.quantile(v, .5)), q90=float(np.quantile(v, .9)),
                       maximum=float(np.max(v))) for k, v in distributions.items()},
                   strict_wrong_pair_examples=errors)
    print('DIAGNOSED', dataset, sequence, condition, arm, 'pairs', sum(counts[k] for k in ['known_pairs', 'assigned_pairs']),
          'strict wrong', len(errors), 'switches', identity['2']['consecutive_identity_switches'], flush=True)
    return summary


def main():
    destination = OUT / 'ASSOCIATION_DIAGNOSTIC.json'
    assert not destination.exists()
    manifests = {
        'gce': TRIALS / 'icra_gaussian_evidence/summary_development.json',
        'gs': TRIALS / 'icra_reviewer_revision/audit_controls_development.json',
        'v2x': TRIALS / 'icra_projected_admission/audit_final_v2x_evaluation.json'}
    loaded = {k: json.loads(p.read_text()) for k, p in manifests.items()}
    units = []
    for dataset, count in [('v2v', 9), ('v2x', 5)]:
        for seq in range(count):
            sequence = f'{seq:04d}'
            for condition in ['reliable', 'intermittent']:
                for arm in ARMS:
                    if dataset == 'v2x':
                        folder, manifest, prefix = 'icra_projected_admission/results/final_v2x_evaluation', 'v2x', 'v2x_'
                    elif arm == ARMS[0]:
                        folder, manifest, prefix = 'icra_gaussian_evidence/results_development', 'gce', ''
                    else:
                        folder, manifest, prefix = 'icra_reviewer_revision/results/controls_development', 'gs', ''
                    path = TRIALS / folder / f'{prefix}{sequence}_{condition}_{arm}.json.gz'
                    name = str(path.relative_to(ROOT))
                    units.append((path, loaded[manifest]['inputs'][name], dataset, sequence, condition))
    frozen = dict(created_utc=datetime.now(timezone.utc).isoformat(), base_commit='dc6a4c14',
                  diagnostic_sources={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'PROTOCOL.md',
                      TRIALS / 'icra_reviewer_revision/review_gaussian_audit.py',
                      TRIALS / 'icra_external_fusion/analyze_v2v4real.py']},
                  prior_manifests={str(p.relative_to(ROOT)): sha(p) for p in manifests.values()},
                  input_sha256={str(p.relative_to(ROOT)): h for p, h, *_ in units})
    freeze = OUT / 'DIAGNOSTIC_FREEZE.json'
    if freeze.exists():
        old = json.loads(freeze.read_text())
        for k in ['diagnostic_sources', 'prior_manifests', 'input_sha256']:
            assert frozen[k] == old[k], k
    else:
        freeze.write_text(json.dumps(frozen, indent=2) + '\n')
    rows = [diagnostic(*unit) for unit in units]
    aggregate = []
    for dataset in ['v2v', 'v2x']:
        for arm in ARMS:
            for condition in ['reliable', 'intermittent']:
                part = [r for r in rows if (r['dataset'], r['arm'], r['condition']) == (dataset, arm, condition)]
                counts = Counter()
                identity = {str(int(c)): Counter() for c in CUTOFFS}
                for row in part:
                    counts.update(row['counts'])
                    for c in identity:
                        identity[c].update(row['identity'][c])
                aggregate.append(dict(dataset=dataset, arm=arm, condition=condition, sequences=len(part),
                    frames=sum(r['frames'] for r in part), counts=dict(counts),
                    identity={k: dict(v) for k, v in identity.items()}))
    result = dict(passed=True, freeze_sha256=sha(freeze), rows=rows, aggregate=aggregate,
                  interpretation='Annotation identities are an offline diagnostic with explicit 2 m/12 m assignment gates; unscored pairs are retained.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('COMPLETE ASSOCIATION DIAGNOSTIC', len(rows), 'trajectories', flush=True)


if __name__ == '__main__':
    main()
