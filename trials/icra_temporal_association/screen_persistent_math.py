"""Independent persistent-state, original-cost matching and branch accounting."""
import numpy as np
from scipy.optimize import linear_sum_assignment

from screen_observation_math import objective as observation_objective
from association_math import legacy_costs, audit_selected


def objective(left, right, state, frame, quality_min, cutoffs):
    info = observation_objective(left['keys'], right['keys'], left['means'], right['means'],
        left['covariance'], right['covariance'], left['features'], right['features'],
        left['r'], right['r'], state['history'], 'temporal', frame, quality_min, cutoffs)
    evidence_cost = np.where(info['qualified'], info['costs'], 0.)
    eligible = info['known'] & info['qualified'] & (info['history_count'] >= 2)
    enter = eligible & (evidence_cost > 1.)
    clear = eligible & (evidence_cost <= 1.)
    entered = {tuple(k) for k in left['keys'][enter.any(1)]}
    cleared = {tuple(k) for k in left['keys'][clear.any(1)]}
    present = {tuple(k) for k in np.concatenate([left['keys'], right['keys']])}
    blocked = ((state['blocked'] | entered) - cleared) & present
    local_blocked = np.array([tuple(k) in blocked for k in left['keys']], bool)
    reopened = info['known'] & local_blocked[:, None]
    locked = info['known'] & ~reopened
    costs = legacy_costs(left['means'], left['covariance'], right['means'], right['covariance'])
    free_left, free_right = np.flatnonzero(~locked.any(1)), np.flatnonzero(~locked.any(0))
    a, b = len(free_left), len(free_right)
    optimal = .5 * (a + b)
    if a and b:
        free_costs = costs[np.ix_(free_left, free_right)].copy()
        free_costs[reopened[np.ix_(free_left, free_right)]] = 1e8
        augmented = np.full((a + b, a + b), max(1e6, float(free_costs.max())) * 100.)
        augmented[:a, :b] = free_costs
        augmented[np.arange(a), b + np.arange(a)] = .5
        augmented[a + np.arange(b), np.arange(b)] = .5
        augmented[a:, b:] = 0.
        i, j = linear_sum_assignment(augmented)
        optimal = float(augmented[i, j].sum())
    info.update(costs=costs, evidence_cost=evidence_cost, enter=enter,
        locked=locked, reopened=reopened, free_left=free_left, free_right=free_right,
        optimal_cost=optimal, local_blocked=local_blocked, blocked=blocked)
    new_state = dict(history=info['history'], blocked=blocked)
    return info, new_state


def audit_branches(info, pairs, left_keys, right_keys, mode, receiver):
    selected, _, _ = audit_selected(info, pairs)
    abstain = info['local_blocked'] & ~selected.any(1)
    aligned = right_keys.copy()
    for i, j in pairs:
        aligned[j] = left_keys[i]
    keep = np.ones(len(right_keys), bool)
    splits = []
    collisions = ~selected.any(0) & info['known'].any(0)
    for j in np.flatnonzero(collisions):
        i = np.flatnonzero(info['known'][:, j]).item()
        assert info['reopened'][i, j]
        retained = False
        if mode == 'split' and info['enter'][i, j] and right_keys[j, 1] < 1_000_000:
            alias = np.array([right_keys[j, 0], 1_000_000_000 + 1_000_000 * receiver + right_keys[j, 1]])
            other = keep.copy()
            other[j] = False
            occupied = np.concatenate([left_keys, aligned[other]])
            if not (occupied == alias).all(1).any():
                aligned[j] = alias
                splits.append((tuple(right_keys[j]), tuple(alias)))
                retained = True
        keep[j] = retained
    assert len(np.unique(aligned[keep], axis=0)) == keep.sum()
    return dict(selected=selected, dropped=~keep, abstain=abstain, splits=splits,
                aligned=aligned, remote_only={alias for _, alias in splits})
