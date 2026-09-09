"""Keep a false-alarm-only window when there is no missed target to trace."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
original = OUT/'diagnose_tracks.py'
target = OUT/'diagnose_tracks_v2.py'
assert not target.exists()
source = original.read_text()
helper = '''
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

'''
replacements = [
    ("select_path = OUT/'TRACE_SELECTION.json'", "select_path = OUT/'TRACE_SELECTION_V2.json'"),
    ("def main():\n", helper+"def main():\n"),
    ("                    counts = Counter(r['truth_id'] for r in subset)\n                    truth_id =", 
     "                    counts = Counter(r['truth_id'] for r in subset)\n                    if not counts and reason != 'longest_strict_interval':\n                        assert not gain\n                        traces.append(false_alarm_trace(cohort, unit, condition, reason, window, data, indices, cached, compact))\n                        print('FALSE-ALARM WINDOW TRACED', sequence, condition, flush=True)\n                        continue\n                    truth_id ="),
    ("    if isinstance(value, (np.integer, int)):\n", "    if isinstance(value, (np.bool_, bool)):\n        return bool(value)\n    if isinstance(value, (np.integer, int)):\n")]
for old, new in replacements:
    assert source.count(old) == 1, old
    source = source.replace(old, new)
target.write_text(source)
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report = dict(created_utc=datetime.now(timezone.utc).isoformat(), original_sha256=sha(original), corrected_sha256=sha(target),
              failed_log_sha256=sha(ROOT/'RUN/ICRA_V2X_GCE_DIAGNOSIS/track_diagnosis.log'),
              replacements=[dict(old=old,new=new) for old,new in replacements],
              reason='The old v2x_0003 intermittent loss window is a pure false-alarm increase, with no strict No-age-only target. Preserve it and trace a scored false output rather than silently excluding the case or inventing a missed target.',
              native_trajectories_changed=False, phase1_outputs_changed=False)
(OUT/'CASE_SELECTION_REPAIR.json').write_text(json.dumps(report,indent=2)+'\n')
print('PRESERVED FALSE-ALARM CASE IN TRACE V2')
