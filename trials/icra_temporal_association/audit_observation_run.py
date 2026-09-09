"""Verify current observation summaries and every receiver association."""
from collections import defaultdict
import numpy as np

from association_math import objective, audit_selected
from observation_math import direct_moments
from review_gaussian_audit import unpack


def current_records(run, mat, packet):
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    direct = np.asarray(run['localDirectRecords'], float).reshape(-1, 12)
    increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    assert np.array_equal(local[:, :4], direct[:, :4])
    if packet:
        received = np.asarray(run['packetDirectRecords'], float).reshape(-1, 12)
        assert np.array_equal(received, direct), 'actual observation packet round trip'
    else:
        assert not run['packetDirectRecords']
    inc_index = {tuple(r[:4].astype(int)): r for r in increments}
    existence = np.array([inc_index[tuple(r[:4].astype(int))][5] for r in local])
    by_frame, inc_frames = defaultdict(list), defaultdict(list)
    for i, r in enumerate(local):
        by_frame[int(r[0]), int(r[1])].append(i)
    for i, r in enumerate(increments):
        inc_frames[int(r[0]), int(r[1])].append(i)
    frames = {}
    weight_values = 0
    covariance = unpack(local[:, 22:32])
    for t in range(1, int(mat['T'].item()) + 1):
        for n in [1, 2]:
            rows = increments[inc_frames[t, n]]
            z = np.asarray(mat['measurements'][n - 1, t - 1], float).reshape(2, -1).T
            raw = np.asarray(run['localAssociationWeights'][n - 1 + 2 * (t - 1)], float)
            W = raw.reshape(len(rows), len(z) + 1) if raw.size else np.zeros((0, 0))
            weight_values += W.size
            values = direct_moments(W, z, rows[:, 7].astype(bool))
            assert np.allclose(values[:, 0], rows[:, 9], atol=1e-12, rtol=0)
            reconstructed = {tuple(r[:4].astype(int)): v for r, v in zip(rows, values)}
            indices = np.asarray(by_frame[t, n], int)
            if len(indices):
                expected = np.array([reconstructed[tuple(local[i, :4].astype(int))] for i in indices])
                assert np.allclose(direct[indices, 4:], expected, atol=1e-10, rtol=1e-10)
            frames[t, n] = dict(keys=local[indices, 2:4].astype(int), means=local[indices, 18:22],
                covariance=covariance[indices], features=direct[indices, 4:], r=existence[indices])
    return frames, dict(direct_records=len(direct), raw_association_weight_values=weight_values)


def by_receiver_frame(records, width):
    matrix = np.asarray(records, float).reshape(-1, width)
    indices = defaultdict(list)
    for i, row in enumerate(matrix):
        indices[int(row[0]), int(row[1])].append(i)
    return lambda t, n: matrix[indices[t, n]]


def associations(run, data, frames, mode):
    pairs_at = by_receiver_frame(run['associationRecords'], 10)
    stats_at = by_receiver_frame(run['associationStats'], 11)
    abstain_at = by_receiver_frame(run['associationAbstentions'], 4)
    reopen_at = by_receiver_frame(run['associationReopenings'], 8)
    fused_at = by_receiver_frame(run['iterationRecords'], 60)
    delivery = np.asarray(data['delivered'])
    history = {1: [], 2: []}
    total_pairs = total_reopened = total_abstained = receiver_frames = 0
    for t in range(1, len(data['time']) + 1):
        for n in [1, 2]:
            pair_rows, stat_rows = pairs_at(t, n), stats_at(t, n)
            if not delivery[n - 1, 2 - n, t - 1]:
                assert not len(pair_rows) and not len(stat_rows)
                assert not len(abstain_at(t, n)) and not len(reopen_at(t, n))
                continue
            left, right = frames[t, n], frames[t, 3 - n]
            info = objective(left['keys'], right['keys'], left['means'], right['means'],
                left['covariance'], right['covariance'], left['features'], right['features'],
                left['r'], right['r'], history[n], mode, t)
            history[n] = info['history']
            li = {tuple(k): i for i, k in enumerate(left['keys'])}
            ri = {tuple(k): j for j, k in enumerate(right['keys'])}
            pairs = [(li[tuple(row[2:4].astype(int))], ri[tuple(row[4:6].astype(int))]) for row in pair_rows]
            selected, dropped, abstain = audit_selected(info, pairs)
            for (i, j), row in zip(pairs, pair_rows):
                assert row[6] == info['locked'][i, j]
                assert np.isclose(row[7], info['costs'][i, j], atol=1e-7, rtol=1e-9)
                assert row[8] == info['history_count'][i, j]
                assert np.isclose(row[9], info['distance'][i, j], atol=1e-10, rtol=1e-10)
            assert len(stat_rows) == 1
            expected_stats = [t, n, len(left['keys']), len(right['keys']), info['locked'].sum(),
                (selected & ~info['locked']).sum(), info['reopened'].sum(), dropped.sum(),
                abstain.sum(), info['qualified'].sum(), (info['history_count'] > 1).sum()]
            assert np.array_equal(stat_rows[0], expected_stats)
            expected_abstain = {tuple(k) for k in left['keys'][abstain]}
            assert {tuple(r[2:4].astype(int)) for r in abstain_at(t, n)} == expected_abstain
            reopenings = reopen_at(t, n)
            assert len(reopenings) == info['reopened'].sum()
            for row in reopenings:
                i = li[tuple(row[2:4].astype(int))]
                j = np.flatnonzero(info['reopened'][i]).item()
                assert np.allclose(row[4:], [info['costs'][i, j], info['history_count'][i, j],
                    info['distance'][i, j], info['summed_distance'][i, j]], atol=1e-9, rtol=1e-10)
            fused = fused_at(t, n)
            originals = fused[:, 31:35].reshape(-1, 2, 2).astype(int)
            fused_pairs = set()
            for row, keys in zip(fused, originals):
                if (keys[:, 0] > 0).all():
                    fused_pairs.add((li[tuple(keys[0])], ri[tuple(keys[1])]))
                if tuple(row[2:4].astype(int)) in expected_abstain:
                    assert tuple(keys[0]) == tuple(row[2:4].astype(int)) and not keys[1].any()
                    assert np.array_equal(row[13:15], [1, 0]), 'identity rejection must abstain'
                    i = li[tuple(keys[0])]
                    assert abs(row[9] - np.clip(left['r'][i], 1e-9, 1 - 1e-9)) < 1e-10
                    assert np.allclose(np.r_[row[4:6], row[40:42]], left['means'][i], atol=1e-8, rtol=0)
            assert fused_pairs == set(pairs), 'fusion inputs must follow accepted associations'
            present_right = {ri[tuple(keys[1])] for keys in originals if keys[1, 0] > 0}
            assert not present_right.intersection(np.flatnonzero(dropped))
            assert int(run['matchedLabels'][n - 1][t - 1]) == len(pairs)
            total_pairs += len(pairs)
            total_reopened += int(info['reopened'].sum())
            total_abstained += int(abstain.sum())
            receiver_frames += 1
    return dict(receiver_frames=receiver_frames, accepted_pairs=total_pairs,
                reopened_known_pairs=total_reopened, abstained_labels=total_abstained)
