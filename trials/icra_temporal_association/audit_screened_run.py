"""Audit every received-frame state transition and each source-specific abstention."""
import numpy as np

from screen_persistent_math import objective, audit_branches
from audit_observation_run import by_receiver_frame


def associations(run, data, frames, mode):
    assert mode in ['quality', 'nis', 'quality_nis']
    quality_min = .9 if mode in ['quality', 'quality_nis'] else .5
    cutoffs = np.array([13.815510557964274, 18.46682695290317, 22.457744484825326]
        if mode in ['nis', 'quality_nis'] else [9.21034037197618, 13.2767041359876, 16.8118938297709])
    pairs_at = by_receiver_frame(run['associationRecords'], 10)
    stats_at = by_receiver_frame(run['associationStats'], 11)
    abstain_at = by_receiver_frame(run['associationAbstentions'], 4)
    remote_at = by_receiver_frame(run['associationRemoteAbstentions'], 4)
    reopen_at = by_receiver_frame(run['associationReopenings'], 8)
    splits_at = by_receiver_frame(run['associationSplits'], 6)
    conflicts_at = by_receiver_frame(run['associationConflicts'], 4)
    fused_at = by_receiver_frame(run['iterationRecords'], 60)
    delivery = np.asarray(data['delivered'])
    states = {n: dict(history=[], blocked=set()) for n in [1, 2]}
    totals = dict(receiver_frames=0, accepted_pairs=0, reopened_known_pairs=0,
                  abstained_labels=0, split_branches=0, active_conflict_label_frames=0)
    for t in range(1, len(data['time']) + 1):
        for n in [1, 2]:
            pair_rows, stat_rows = pairs_at(t, n), stats_at(t, n)
            if not delivery[n - 1, 2 - n, t - 1]:
                assert all(not len(f(t, n)) for f in [pairs_at, stats_at, abstain_at,
                    remote_at, reopen_at, splits_at, conflicts_at, fused_at])
                continue
            left, right = frames[t, n], frames[t, 3 - n]
            info, states[n] = objective(left, right, states[n], t, quality_min, cutoffs)
            li = {tuple(k): i for i, k in enumerate(left['keys'])}
            ri = {tuple(k): j for j, k in enumerate(right['keys'])}
            pairs = [(li[tuple(row[2:4].astype(int))], ri[tuple(row[4:6].astype(int))]) for row in pair_rows]
            branch = audit_branches(info, pairs, left['keys'], right['keys'], 'split', n)
            selected, dropped, abstain = [branch[k] for k in ['selected', 'dropped', 'abstain']]
            for (i, j), row in zip(pairs, pair_rows):
                assert row[6] == info['locked'][i, j]
                assert np.isclose(row[7], info['costs'][i, j], atol=1e-7, rtol=1e-9)
                assert row[8] == info['history_count'][i, j]
                assert np.isclose(row[9], info['distance'][i, j], atol=1e-10, rtol=1e-10)
            expected_stats = [t, n, len(li), len(ri), info['locked'].sum(),
                (selected & ~info['locked']).sum(), info['reopened'].sum(), dropped.sum(),
                abstain.sum(), info['qualified'].sum(), (info['history_count'] > 1).sum()]
            assert len(stat_rows) == 1 and np.array_equal(stat_rows[0], expected_stats)
            expected_abstain = {tuple(k) for k in left['keys'][abstain]}
            assert {tuple(r[2:4].astype(int)) for r in abstain_at(t, n)} == expected_abstain
            assert {tuple(r[2:4].astype(int)) for r in remote_at(t, n)} == branch['remote_only']
            assert {tuple(r[2:4].astype(int)) for r in conflicts_at(t, n)} == info['blocked']
            native_splits = [(tuple(r[2:4].astype(int)), tuple(r[4:6].astype(int))) for r in splits_at(t, n)]
            assert native_splits == branch['splits']
            reopenings = reopen_at(t, n)
            assert len(reopenings) == info['reopened'].sum()
            for row in reopenings:
                i = li[tuple(row[2:4].astype(int))]
                j = np.flatnonzero(info['reopened'][i]).item()
                assert np.allclose(row[4:], [info['evidence_cost'][i, j], info['history_count'][i, j],
                    info['distance'][i, j], info['summed_distance'][i, j]], atol=1e-9, rtol=1e-10)
            fused = fused_at(t, n)
            originals = fused[:, 31:35].reshape(-1, 2, 2).astype(int)
            fused_pairs, present_right = set(), set()
            seen_local, seen_remote = set(), set()
            for row, keys in zip(fused, originals):
                output_key = tuple(row[2:4].astype(int))
                if (keys[:, 0] > 0).all():
                    fused_pairs.add((li[tuple(keys[0])], ri[tuple(keys[1])]))
                if keys[1, 0] > 0:
                    j = ri[tuple(keys[1])]
                    present_right.add(j)
                    assert output_key == tuple(branch['aligned'][j])
                if output_key in expected_abstain:
                    assert tuple(keys[0]) == output_key and not keys[1].any()
                    assert np.array_equal(row[13:15], [1, 0])
                    i = li[tuple(keys[0])]
                    assert abs(row[9] - np.clip(left['r'][i], 1e-9, 1-1e-9)) < 1e-10
                    assert np.allclose(np.r_[row[4:6], row[40:42]], left['means'][i], atol=1e-8, rtol=0)
                    seen_local.add(output_key)
                if output_key in branch['remote_only']:
                    assert not keys[0].any() and keys[1, 0] > 0
                    assert np.array_equal(row[13:15], [0, 1])
                    j = ri[tuple(keys[1])]
                    assert abs(row[9] - np.clip(right['r'][j], 1e-9, 1-1e-9)) < 1e-10
                    assert np.allclose(np.r_[row[4:6], row[40:42]], right['means'][j], atol=1e-8, rtol=0)
                    seen_remote.add(output_key)
            assert seen_local == expected_abstain and seen_remote == branch['remote_only']
            assert fused_pairs == set(pairs)
            assert present_right == set(np.flatnonzero(~dropped))
            assert int(run['matchedLabels'][n - 1][t - 1]) == len(pairs)
            totals['receiver_frames'] += 1
            totals['accepted_pairs'] += len(pairs)
            totals['reopened_known_pairs'] += int(info['reopened'].sum())
            totals['abstained_labels'] += int(abstain.sum())
            totals['split_branches'] += len(branch['splits'])
            totals['active_conflict_label_frames'] += len(info['blocked'])
    return totals
