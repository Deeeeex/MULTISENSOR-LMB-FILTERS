"""Locate unextracted target components and trace selected native labels."""
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import gzip
import json

import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment
from scipy.special import expit

from diagnose_gap import (OUT, ROOT, TRIALS, GCE, NOAGE, STAGES, sha, read, domain,
                          alternatives, extract, measurement_support)


def assignments(truth, states, cutoff=2.):
    xy = np.asarray(truth, float).reshape(4, -1).T[:, :2]
    states = np.asarray(states, float).reshape(-1, 4)
    distance = np.linalg.norm(xy[:, None, :]-states[None, :, :2], axis=2)
    i, j = linear_sum_assignment(np.where(distance <= cutoff, distance, 1e6))
    return {int(ii): int(jj) for ii, jj in zip(i, j) if distance[ii, jj] <= cutoff}


def make_index(run):
    result = {}
    for key, columns in [('iterationRecords', 60), ('localIncrementRecords', 12),
                         ('localGaussianRecords', 32), ('packetGaussianRecords', 19)]:
        rows = np.asarray(run[key], float).reshape(-1, columns)
        by_frame = defaultdict(list)
        by_label = {}
        for row in rows:
            by_frame[int(row[0]), int(row[1])].append(row)
            label = tuple(row[:4].astype(int))
            assert label not in by_label
            by_label[label] = row
        result[key] = dict(rows=rows, frames=by_frame, labels=by_label, columns=columns)
    return result


def pool(data, index, t, n):
    if data['delivered'][n-1][2-n][t-1]:
        records = np.asarray(index['iterationRecords']['frames'][t, n], float).reshape(-1, 60)
    else:
        local = index['localGaussianRecords']['frames'][t, n]
        records = np.zeros((len(local), 60))
        for j, spatial in enumerate(local):
            record = index['localIncrementRecords']['labels'][tuple(spatial[:4].astype(int))]
            records[j, :4] = spatial[:4]
            records[j, 4:6] = spatial[18:20]
            records[j, 9] = record[5]
            records[j, 7] = record[5]
    positions = np.asarray(data['positions'])[:, :, t-1]
    states, selected, summary = extract(records, records[:, 9], records[:, 4:6], positions)
    expected = np.asarray(data['runs']['estimates'][n-1+2*(t-1)], float).reshape(-1, 4)
    assert np.allclose(states[:, :2], expected[:, :2], atol=1e-8, rtol=1e-9), (data['sequence'], t, n)
    ranks = np.zeros(len(records), int)
    retained = np.flatnonzero(records[:, 9] > .001)
    ranks[retained[np.argsort(-records[retained, 9], kind='stable')]] = np.arange(1, len(retained)+1)
    return records, set(map(int, selected)), ranks, summary


def candidate(records, selected, ranks, xy):
    distance = np.linalg.norm(records[:, 4:6]-xy, axis=1)
    close = np.flatnonzero(distance <= 2.)
    if not len(close):
        return None, dict(candidate=False, near_components=0)
    which = int(close[np.argmax(records[close, 9])])
    z = records[which]
    answer = dict(candidate=True, near_components=len(close), r=float(z[9]), r0=float(z[7]),
                  distance=float(distance[which]), selected=which in selected, rank=int(ranks[which]),
                  below_pruning=bool(z[9] <= .001), birth_time=int(z[2]), birth_location=int(z[3]))
    return z, answer


def mechanism(z):
    if z is None:
        return {}
    kept, delta = z[52:54], z[19:21]
    active = z[13:15] > 0
    logs = np.zeros(2)
    r = np.clip(z[17:19][active], 1e-9, 1-1e-9)
    logs[active] = np.log(r)-np.log1p(-r)
    originals = z[31:35].reshape(2, 2)
    positive, negative = float(np.sum(kept*np.maximum(delta, 0))), float(np.sum(kept*np.minimum(delta, 0)))
    age = float(np.sum((z[28:30]-z[13:15])*logs))
    return dict(positive_log_odds=positive, negative_log_odds=negative, age_log_odds=age,
                normalizer_change=float(z[56]-z[10]), both_present=bool((originals[:, 0] > 0).all()),
                negative_admitted=bool(((kept > 0) & (delta < 0)).any()),
                positive_admitted=bool(((kept > 0) & (delta > 0)).any()),
                positive_gate_zero=bool(((delta > 1e-8) & (z[26:28] <= 1e-12)).any()),
                positive_curvature_rejected=bool(((delta > 1e-8) & (z[26:28] > 1e-12) & (z[57:59] == 0)).any()))


def sanitize(value):
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [sanitize(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    return value


def detailed_record(z, index, arm, truth_xy):
    if z is None:
        return None
    t, n, bt, bl = map(int, z[:4])
    result = dict(label=[bt, bl], r=z[9], r0=z[7], xy=z[4:6], distance=np.linalg.norm(z[4:6]-truth_xy),
                  input_r=z[17:19], b=z[13:15], q=z[11:13], old_log_integral=z[10])
    local = index['localIncrementRecords']['labels'].get((t, n, bt, bl))
    result['self_local'] = None if local is None else dict(prior_r=local[4], post_r=local[5], delta=local[6],
                                                        current=local[7], positive_gate=local[8], mass=local[9],
                                                        pd=local[10], negative_gate=local[11])
    if arm == GCE:
        result.update(mechanism(z))
        result.update(delta=z[19:21], positive_gate=z[26:28], negative_gate=z[35:37], kept=z[52:54],
                      beta=z[28:30], allowed=z[57:59], new_log_integral=z[56], original_labels=z[31:35].reshape(2, 2))
        sources = []
        for side, original in enumerate(z[31:35].reshape(2, 2)):
            if original[0] == 0:
                sources.append(None); continue
            sensor = n if side == 0 else 3-n
            key = (t, sensor, *map(int, original))
            inc = index['localIncrementRecords']['labels'][key]
            spatial = index['packetGaussianRecords']['labels'][key]
            lower = np.tril_indices(4)
            order = np.argsort(lower[1]*4+lower[0])
            J = np.zeros((4, 4)); J[lower[0][order], lower[1][order]] = spatial[4:14]
            J = J+np.tril(J, -1).T
            sources.append(dict(sensor=sensor, label=list(map(int, original)), prior_r=inc[4], post_r=inc[5],
                                delta=inc[6], current=inc[7], positive_gate=inc[8], mass=inc[9], pd=inc[10],
                                negative_gate=inc[11], precision_increment_min_eigenvalue=np.linalg.eigvalsh(J).min()))
        result['sources'] = sources
    return sanitize(result)



def false_alarm_trace(cohort, unit, condition, reason, window, data, indices, cached, compact):
    sequence = unit['sequence']
    counts = Counter()
    for t in range(window['first_frame'], window['last_frame']+1):
        for n in [1, 2]:
            run = data[GCE]['runs']
            raw = np.asarray(run['rawEstimates'][n-1+2*(t-1)], float).reshape(-1, 4)
            labels = np.asarray(run['labels'][n-1+2*(t-1)], int).reshape(2, -1).T
            keep = domain(raw, compact['positions'][:, :, t-1])
            states, labels = raw[keep], labels[keep]
            matched = set(assignments(data[GCE]['truth'][t-1], states, 12.).values())
            for j, label in enumerate(labels):
                if j not in matched: counts[tuple(label)] += 1
    assert counts, (sequence, condition, 'no unmatched output in false-alarm case')
    anchor = tuple(sorted(counts, key=lambda k:(-counts[k], k))[0])
    rows = []
    for t in range(1, unit['frames']+1):
        truth = np.asarray(data[GCE]['truth'][t-1], float).reshape(4, -1)[:2].T
        for n in [1, 2]:
            pools, unused = cached[t, n]
            for arm in [GCE, NOAGE]:
                records, selected, ranks, summary = pools[arm]
                chosen = np.flatnonzero((records[:, 2]==anchor[0]) & (records[:, 3]==anchor[1]))
                z = records[chosen[0]] if len(chosen) else None
                observations = []
                if z is not None:
                    xy = z[4:6]
                    for sensor in [0, 1]:
                        meas = compact['measurements'][sensor, t-1].T
                        distances = np.linalg.norm(meas-xy, axis=1)
                        close = np.flatnonzero(distances <= 2.)
                        observations.append(dict(sensor=sensor+1, detections=len(meas), near_2m=len(close),
                                                 nearby=[dict(index=int(j)+1, distance=float(distances[j]),
                                                              probability=float(compact['calibratedScores'][sensor, t-1].ravel()[j]),
                                                              likelihood_ratio=float(compact['likelihoodRatios'][sensor, t-1].ravel()[j])) for j in close]))
                nearest = float(np.linalg.norm(truth-z[4:6], axis=1).min()) if z is not None and len(truth) else None
                detail = detailed_record(z, indices[arm], arm, z[4:6]) if z is not None else None
                if detail is not None: detail.pop('distance')
                rows.append(dict(frame=t, robot=n, arm=arm, nearest_truth_distance=nearest,
                                 map_count=summary['map_count'], sum_r=summary['sum_r'],
                                 anchor_selected=bool(len(chosen) and int(chosen[0]) in selected),
                                 anchor_rank=int(ranks[chosen[0]]) if len(chosen) else None,
                                 observations_near_estimate=observations, anchor=detail))
    return dict(cohort=cohort, sequence=sequence, scene=unit['scene'], condition=condition, reason=reason,
                case_type='scored_false_alarm', window=window, truth_id=None, anchor_label=list(map(int, anchor)), rows=rows,
                note='The selected loss window has no strict No-age-only target. Trace its most frequent GCE output unmatched within 12 m instead. This is an annotation-relative false alarm; no claim that the physical object does not exist.')

def main():
    destination = OUT/'TRACK_DIAGNOSIS.json'
    assert not destination.exists()
    gap_path = OUT/'GAP_DIAGNOSIS.json'
    gap = json.loads(gap_path.read_text()); assert gap['passed']
    requested = {}
    for cohort in ['v2x_val', 'v2x_test']:
        for condition in ['reliable', 'intermittent']:
            part = [r for r in gap['sequence_rows'] if r['cohort'] == cohort and r['condition'] == condition]
            worst = max(part, key=lambda r:r['means']['delta_ospa'])
            requested[worst['sequence'], condition] = 'largest_sequence_loss'
    strict = [r for r in gap['noage_only_intervals'] if r['cohort']=='v2x_test' and r['condition']=='reliable' and r['cutoff']==2]
    longest = max(strict, key=lambda r:r['frames'])
    requested[longest['sequence'], 'reliable'] = 'longest_strict_interval'
    part = sorted([r for r in gap['sequence_rows'] if r['cohort']=='v2x_test' and r['condition']=='reliable'], key=lambda r:-r['means']['delta_ospa'])
    requested[part[1]['sequence'], 'reliable'] = 'second_largest_sequence_loss'
    requested[part[-1]['sequence'], 'reliable'] = 'largest_sequence_gain'
    selection = dict(created_utc=datetime.now(timezone.utc).isoformat(), gap_sha256=sha(gap_path),
                     cases=[dict(sequence=s, condition=c, reason=r) for (s, c), r in requested.items()],
                     rule='Use dominant strict-matched target ID in the selected 15-frame window; use the longest strict interval target for that case. Select one modal native output label from the winning method, then trace it through the full segment.')
    select_path = OUT/'TRACE_SELECTION_V2.json'; assert not select_path.exists()
    select_path.write_text(json.dumps(selection, indent=2)+'\n')
    inputs = {str(gap_path.relative_to(ROOT)): sha(gap_path), str(Path(__file__).relative_to(ROOT)): sha(Path(__file__)),
              str(select_path.relative_to(ROOT)): sha(select_path), str((OUT/'diagnose_gap.py').relative_to(ROOT)): sha(OUT/'diagnose_gap.py')}
    targets, traces = [], []
    for cohort, folder, stage, count in STAGES:
        base = TRIALS/folder
        cfg_path = base/'stages'/(stage+'.json')
        assert sha(cfg_path) == gap['inputs'][str(cfg_path.relative_to(ROOT))]
        cfg = json.loads(cfg_path.read_text())
        for unit in cfg['units']:
            compact_path = ROOT/unit['data_path']
            assert sha(compact_path) == gap['inputs'][unit['data_path']]
            compact = loadmat(compact_path)
            T, sequence = unit['frames'], unit['sequence']
            for condition in ['reliable', 'intermittent']:
                data, indices = {}, {}
                for arm in [GCE, NOAGE]:
                    path = base/'results'/stage/f'{sequence}_{condition}_{arm}.json.gz'
                    expected = gap['inputs'][str(path.relative_to(ROOT))]
                    assert sha(path) == expected
                    inputs[str(path.relative_to(ROOT))] = expected
                    data[arm] = read(path); indices[arm] = make_index(data[arm]['runs'])
                target_part, cached = [], {}
                for t in range(1, T+1):
                    truth = data[GCE]['truth'][t-1]
                    ids = np.asarray(data[GCE]['truthIds'][t-1]).reshape(-1)
                    xy = np.asarray(truth, float).reshape(4, -1)[:2].T
                    support, opportunities, unused = measurement_support(compact, t-1, 2.)
                    for n in [1, 2]:
                        pools = {arm:pool(data[arm], indices[arm], t, n) for arm in [GCE, NOAGE]}
                        states = {arm:data[arm]['runs']['estimates'][n-1+2*(t-1)] for arm in [GCE, NOAGE]}
                        matches = {arm:assignments(truth, states[arm]) for arm in [GCE, NOAGE]}
                        post, selected, ranks, summary = pools[GCE]
                        delivered = bool(data[GCE]['delivered'][n-1][2-n][t-1])
                        intervention_matches = {}
                        if delivered:
                            variants, unused = alternatives(post)
                            for variant in ['without_negative_scalar', 'without_positive_scalar', 'without_age', 'noage_same_input']:
                                r, means = variants[variant]
                                proposed, unused, unused = extract(post, r, means, compact['positions'][:, :, t-1])
                                intervention_matches[variant] = assignments(truth, proposed)
                        else:
                            intervention_matches = {v:matches[GCE] for v in ['without_negative_scalar', 'without_positive_scalar', 'without_age', 'noage_same_input']}
                        for kind, subset in [('noage_only', set(matches[NOAGE])-set(matches[GCE])),
                                             ('gce_only', set(matches[GCE])-set(matches[NOAGE]))]:
                            for k in sorted(subset):
                                z, candidate_info = candidate(post, selected, ranks, xy[k])
                                entry = dict(cohort=cohort, sequence=sequence, condition=condition, frame=t, robot=n,
                                             truth_id=int(ids[k]), kind=kind, delivered=delivered,
                                             support_sources=int(support[k].sum()), geometric_sources=int(opportunities[k].sum()),
                                             map_count=summary['map_count'], **candidate_info)
                                if delivered: entry.update(mechanism(z))
                                for v in intervention_matches:
                                    entry['matched_'+v] = k in intervention_matches[v]
                                target_part.append(entry)
                        if (sequence, condition) in requested:
                            cached[t, n] = (pools, matches)
                targets.extend(target_part)
                key = sequence, condition
                if key in requested:
                    reason = requested[key]
                    gain = reason == 'largest_sequence_gain'
                    window_kind = 'largest_gain' if gain else 'largest_loss'
                    window = next(r for r in gap['windows'] if r['sequence']==sequence and r['condition']==condition and r['kind']==window_kind)
                    wanted = 'gce_only' if gain else 'noage_only'
                    subset = [r for r in target_part if r['kind']==wanted and window['first_frame']<=r['frame']<=window['last_frame']]
                    counts = Counter(r['truth_id'] for r in subset)
                    if not counts and reason != 'longest_strict_interval':
                        assert not gain
                        traces.append(false_alarm_trace(cohort, unit, condition, reason, window, data, indices, cached, compact))
                        print('FALSE-ALARM WINDOW TRACED', sequence, condition, flush=True)
                        continue
                    truth_id = longest['truth_id'] if reason == 'longest_strict_interval' else sorted(counts, key=lambda k:(-counts[k], k))[0]
                    winner = GCE if gain else NOAGE
                    labels = Counter()
                    for t in range(window['first_frame'], window['last_frame']+1):
                        ids = np.asarray(data[GCE]['truthIds'][t-1]).reshape(-1)
                        where = np.flatnonzero(ids == truth_id)
                        if not len(where): continue
                        for n in [1, 2]:
                            pools, matches = cached[t, n]
                            if int(where[0]) not in matches[winner]: continue
                            raw = np.asarray(data[winner]['runs']['rawEstimates'][n-1+2*(t-1)], float).reshape(-1, 4)
                            labs = np.asarray(data[winner]['runs']['labels'][n-1+2*(t-1)], int).reshape(2, -1).T
                            labs = labs[domain(raw, compact['positions'][:, :, t-1])]
                            labels[tuple(labs[matches[winner][int(where[0])]])] += 1
                    anchor = tuple(sorted(labels, key=lambda k:(-labels[k], k))[0])
                    rows = []
                    for t in range(1, T+1):
                        ids = np.asarray(data[GCE]['truthIds'][t-1]).reshape(-1)
                        where = np.flatnonzero(ids == truth_id)
                        if not len(where): continue
                        k = int(where[0]); xy = np.asarray(data[GCE]['truth'][t-1], float).reshape(4, -1)[:2, k]
                        observations = []
                        for sensor in [0, 1]:
                            meas = compact['measurements'][sensor, t-1].T
                            distances = np.linalg.norm(meas-xy, axis=1)
                            close = np.flatnonzero(distances <= 2.)
                            observations.append(dict(sensor=sensor+1, in_range=bool(np.linalg.norm(xy-compact['positions'][:, sensor, t-1])<=40),
                                                     detections=len(meas), near_2m=len(close),
                                                     nearby=[dict(index=int(j)+1, distance=float(distances[j]),
                                                                  probability=float(compact['calibratedScores'][sensor, t-1].ravel()[j]),
                                                                  likelihood_ratio=float(compact['likelihoodRatios'][sensor, t-1].ravel()[j])) for j in close]))
                        for n in [1, 2]:
                            pools, matches = cached[t, n]
                            for arm in [GCE, NOAGE]:
                                records, selected, ranks, summary = pools[arm]
                                chosen = np.flatnonzero((records[:, 2] == anchor[0]) & (records[:, 3] == anchor[1]))
                                z = records[chosen[0]] if len(chosen) else None
                                best, candidate_info = candidate(records, selected, ranks, xy)
                                rows.append(dict(frame=t, robot=n, arm=arm, truth_xy=xy.tolist(), matched=k in matches[arm],
                                                 observations=observations, map_count=summary['map_count'], sum_r=summary['sum_r'],
                                                 anchor_selected=bool(len(chosen) and int(chosen[0]) in selected),
                                                 anchor_rank=int(ranks[chosen[0]]) if len(chosen) else None,
                                                 anchor=detailed_record(z, indices[arm], arm, xy),
                                                 best_candidate=candidate_info))
                    traces.append(dict(cohort=cohort, sequence=sequence, scene=unit['scene'], condition=condition,
                                       reason=reason, window=window, truth_id=int(truth_id), anchor_label=list(map(int, anchor)), rows=rows))
                print('TARGETS TRACED', sequence, condition, len(target_part), flush=True)
    aggregate = []
    for cohort in ['v2x_val', 'v2x_test']:
        for condition in ['reliable', 'intermittent']:
            for kind in ['noage_only', 'gce_only']:
                part = [r for r in targets if r['cohort']==cohort and r['condition']==condition and r['kind']==kind]
                c = Counter()
                for r in part:
                    c['target_robot_frames'] += 1
                    c['candidate_within_2m'] += int(r['candidate'])
                    c['candidate_unselected'] += int(r['candidate'] and not r['selected'])
                    c['candidate_pruned'] += int(r.get('below_pruning', False))
                    c['one_sensor_support'] += int(r['support_sources']==1)
                    c['two_sensor_support'] += int(r['support_sources']==2)
                    c['one_support_both_geometric'] += int(r['support_sources']==1 and r['geometric_sources']==2)
                    for k in ['negative_admitted', 'positive_admitted', 'positive_gate_zero', 'positive_curvature_rejected',
                              'matched_without_negative_scalar', 'matched_without_positive_scalar', 'matched_without_age', 'matched_noage_same_input']:
                        c[k] += int(r.get(k, False))
                aggregate.append(dict(cohort=cohort, condition=condition, kind=kind, counts=dict(c)))
    frame_path = OUT/'target_robot_frames.csv.gz'
    fields = list(dict.fromkeys(k for r in targets for k in r))
    with gzip.open(frame_path, 'wt', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator='\n')
        writer.writeheader(); writer.writerows(targets)
    result = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), target_robot_frames=len(targets),
                  aggregate=aggregate, traces=traces, inputs=inputs,
                  target_file=str(frame_path.relative_to(ROOT)), target_sha256=sha(frame_path),
                  interpretation='Truth associations and case selection are post-outcome diagnostics. The anchor denotes one saved native label, not every component for a target; the best-candidate field checks all nearby components. Both native pools are reconstructed before tracing.')
    destination.write_text(json.dumps(sanitize(result), indent=2, allow_nan=False)+'\n')
    print('TARGET DIAGNOSIS COMPLETE', len(targets), 'target robot-frames;', len(traces), 'case traces', flush=True)


if __name__ == '__main__':
    main()
