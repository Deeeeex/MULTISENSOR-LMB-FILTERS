"""Independent NumPy association objective and causal receiver history."""
import numpy as np
from scipy.optimize import linear_sum_assignment

CUTOFFS = np.array([9.21034037197618, 13.2767041359876, 16.8118938297709])


def legacy_costs(means_left, cov_left, means_right, cov_right):
    jp, jq = np.linalg.inv(cov_left), np.linalg.inv(cov_right)
    delta = means_left[:, None, :] - means_right[None, :, :]
    value = np.einsum('aij,bji->ab', jp, cov_right) + np.einsum('bij,aji->ab', jq, cov_left)
    value += np.einsum('abi,aij,abj->ab', delta, jp, delta)
    value += np.einsum('abi,bij,abj->ab', delta, jq, delta)
    return np.maximum(0., .25 * (value - 8)) / 100


def objective(left_keys, right_keys, left_means, right_means, left_cov, right_cov,
              left_features, right_features, left_r, right_r, history, mode, frame, quality_min=.5, cutoffs=CUTOFFS):
    n, m = len(left_keys), len(right_keys)
    assert mode in ['direct', 'temporal']
    qleft = left_r * left_features[:, 0] * left_features[:, 6]
    qright = right_r * right_features[:, 0] * right_features[:, 6]
    qualified = (qleft >= quality_min)[:, None] & (qright >= quality_min)[None, :]
    delta = left_features[:, None, 1:3] - right_features[None, :, 1:3]
    xx = left_features[:, None, 3] + right_features[None, :, 3]
    xy = left_features[:, None, 4] + right_features[None, :, 4]
    yy = left_features[:, None, 5] + right_features[None, :, 5]
    determinant = xx * yy - xy ** 2
    distance = np.zeros((n, m))
    assert (determinant[qualified] > 0).all()
    numerator = yy * delta[:, :, 0] ** 2 - 2 * xy * delta[:, :, 0] * delta[:, :, 1] + xx * delta[:, :, 1] ** 2
    distance[qualified] = np.maximum(0., numerator[qualified] / determinant[qualified])
    summed = distance.copy()
    count = qualified.astype(int)
    if mode == 'temporal':
        for old in history:
            if not 1 <= frame - old['frame'] <= 2:
                continue
            old_left = {tuple(k): i for i, k in enumerate(old['left'])}
            old_right = {tuple(k): i for i, k in enumerate(old['right'])}
            li = [i for i, k in enumerate(left_keys) if tuple(k) in old_left]
            ri = [i for i, k in enumerate(right_keys) if tuple(k) in old_right]
            oi = [old_left[tuple(left_keys[i])] for i in li]
            oj = [old_right[tuple(right_keys[j])] for j in ri]
            current_index, previous_index = np.ix_(li, ri), np.ix_(oi, oj)
            use = qualified[current_index] & old['qualified'][previous_index]
            count[current_index] += use
            summed[current_index] += use * old['distance'][previous_index]
    assert (count <= 3).all()
    costs = legacy_costs(left_means, left_cov, right_means, right_cov)
    costs[qualified] = summed[qualified] / cutoffs[count[qualified] - 1]
    known = (left_keys[:, None, :] == right_keys[None, :, :]).all(2)
    reopened = known & qualified & (costs > 1)
    locked = known & ~reopened
    free_left, free_right = np.flatnonzero(~locked.any(1)), np.flatnonzero(~locked.any(0))
    a, b = len(free_left), len(free_right)
    selected = locked.copy()
    optimal = .5 * (a + b)
    if a and b:
        free_cost = costs[np.ix_(free_left, free_right)]
        augmented = np.full((a + b, a + b), max(1e6, float(free_cost.max())) * 100)
        augmented[:a, :b] = free_cost
        augmented[np.arange(a), b + np.arange(a)] = .5
        augmented[a + np.arange(b), np.arange(b)] = .5
        augmented[a:, b:] = 0.
        ii, jj = linear_sum_assignment(augmented)
        optimal = float(augmented[ii, jj].sum())
        for i, j in zip(ii, jj):
            if i < a and j < b:
                selected[free_left[i], free_right[j]] = True
    snapshot = dict(frame=frame, left=left_keys.copy(), right=right_keys.copy(),
                    distance=distance, qualified=qualified)
    new_history = [h for h in [*history, snapshot] if h['frame'] >= frame - 2][-2:]
    return dict(costs=costs, known=known, reopened=reopened, locked=locked, qualified=qualified,
                distance=distance, summed_distance=summed, history_count=count,
                free_left=free_left, free_right=free_right, optimal_cost=optimal,
                selected=selected, history=new_history)


def audit_selected(info, pairs):
    """Accept any one-to-one optimum, preserving the producer's tie choice."""
    selected = np.zeros_like(info['known'])
    for i, j in pairs:
        assert not selected[i, j]
        selected[i, j] = True
    assert (selected.sum(0) <= 1).all() and (selected.sum(1) <= 1).all()
    assert (selected[info['locked']]).all() and not (selected & info['reopened']).any()
    free = selected & ~info['locked']
    assert (info['costs'][free] <= 1 + 1e-8).all()
    observed = info['costs'][free].sum() + .5 * (len(info['free_left']) + len(info['free_right']) - 2 * free.sum())
    assert np.isclose(observed, info['optimal_cost'], atol=1e-7, rtol=1e-9), (observed, info['optimal_cost'])
    dropped = ~selected.any(0) & info['known'].any(0)
    abstain = info['reopened'].any(1) & ~selected.any(1)
    return selected, dropped, abstain
