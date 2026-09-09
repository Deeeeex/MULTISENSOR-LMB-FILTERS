"""Read-only paired trajectory decomposition and fixed-input sensitivities."""
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
import argparse
import csv
import gzip
import hashlib
import json
import subprocess
import sys

import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent
sys.path.insert(0, str(TRIALS / 'icra_external_fusion'))
from analyze_v2v4real import domain
from analyze_case_studies import score as reference_score

GCE = 'marked_gaussian_evidence'
NOAGE = 'marked_lineage'
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
VARIANTS = ['native', 'noage_same_input', 'guarded_scalar_same_input',
            'without_negative_scalar', 'without_positive_scalar', 'without_age',
            'old_normalizer', 'old_mean']
COMPONENTS = ['localization', 'missed', 'false', 'cardinality_gap', 'normalization']
STAGES = [('v2x_val', 'icra_projected_admission', 'final_v2x_evaluation', 5),
          ('v2x_test', 'icra_temporal_association', 'association_screen_selected_test', 14)]
BASE_COMMIT = 'e661f2e88764dd90a2d6bfcab6a000ef3bcaba05'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(4 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    with gzip.open(path, 'rt') as handle:
        return json.load(handle)


def scoring(truth, estimates):
    x = np.asarray(truth, float).reshape(4, -1).T
    y = np.asarray(estimates, float).reshape(-1, 4)
    n, m = len(x), len(y)
    costs = np.sum((x[:, None, :2] - y[None, :, :2]) ** 2, axis=2)
    i, j = linear_sum_assignment(np.minimum(costs, 144.))
    d2 = costs[i, j]
    valid = d2 < 144.
    matched = np.full(n, np.nan)
    matched[i[valid]] = d2[valid]
    support = int(valid.sum())
    loc = float(d2[valid].sum())
    missed, false = (n-support)*72., (m-support)*72.
    denominator = max(n, m)
    numerator = loc + missed + false + 72*abs(n-m)
    ospa2 = numerator/denominator if denominator else 0.
    return dict(ospa=float(np.sqrt(ospa2)), gospa=float(np.sqrt(loc+missed+false)),
                loc2=loc, miss2=missed, false2=false, countError=abs(n-m),
                truth_count=n, output_count=m, matches=support, matched_d2=matched,
                denominator=denominator, numerator=numerator, ospa2=ospa2)


def exact_ospa_decomposition(a, b):
    """Symmetric product decomposition, in additive metres of OSPA difference."""
    inv_a = 1/a['denominator'] if a['denominator'] else 0.
    inv_b = 1/b['denominator'] if b['denominator'] else 0.
    divisor = a['ospa'] + b['ospa']
    if divisor == 0:
        return dict.fromkeys(COMPONENTS, 0.)
    scale = .5*(inv_a+inv_b)/divisor
    answer = {name: (a[key]-b[key])*scale for name, key in
              [('localization', 'loc2'), ('missed', 'miss2'), ('false', 'false2')]}
    answer['cardinality_gap'] = 72*(a['countError']-b['countError'])*scale
    answer['normalization'] = .5*(a['numerator']+b['numerator'])*(inv_a-inv_b)/divisor
    assert np.isclose(sum(answer.values()), a['ospa']-b['ospa'], atol=2e-13, rtol=1e-12)
    return answer


def extract(rows, probabilities, means, positions):
    keep = probabilities > .001
    rr = probabilities[keep]-1e-6
    pmf = np.array([1.])
    for value in rr:
        pmf = np.convolve(pmf, [1-value, value])
    count = int(pmf.argmax())
    chosen = np.flatnonzero(keep)[np.argsort(-rr, kind='stable')[:count]]
    states = np.c_[means[chosen], np.zeros((count, 2))]
    cropped = domain(states, positions)
    return states[cropped], chosen[cropped], dict(map_count=count, retained=len(rr),
                                                sum_r=float(probabilities[keep].sum()))


def alternatives(records):
    b = records[:, 13:15]
    beta = records[:, 28:30]
    active = b > 0
    logits = np.zeros_like(b)
    r = np.clip(records[:, 17:19][active], 1e-9, 1-1e-9)
    logits[active] = np.log(r)-np.log1p(-r)
    kept = records[:, 52:54]
    delta = records[:, 19:21]
    positive = np.sum(kept*np.maximum(delta, 0), axis=1)
    negative = np.sum(kept*np.minimum(delta, 0), axis=1)
    age = np.sum((beta-b)*logits, axis=1)
    base = np.sum(b*logits, axis=1)+records[:, 10]
    final = np.sum(beta*logits, axis=1)+positive+negative+records[:, 56]
    assert np.allclose(expit(base), records[:, 7], atol=2e-12, rtol=0)
    assert np.allclose(expit(final), records[:, 9], atol=2e-12, rtol=0)
    new_mean, old_mean = records[:, 4:6], records[:, 38:40]
    result = {
        'native': (records[:, 9], new_mean),
        'noage_same_input': (records[:, 7], old_mean),
        'guarded_scalar_same_input': (expit(base+age+positive+negative), old_mean),
        'without_negative_scalar': (expit(final-negative), new_mean),
        'without_positive_scalar': (expit(final-positive), new_mean),
        'without_age': (expit(final-age), new_mean),
        'old_normalizer': (expit(final-records[:, 56]+records[:, 10]), new_mean),
        'old_mean': (records[:, 9], old_mean)}
    raw_kappa = (active-beta)*np.where(delta >= 0, records[:, 26:28], records[:, 35:37])
    originals = records[:, 31:35].reshape(-1, 2, 2)
    present = originals[:, :, 0] > 0
    joint = (active.sum(1) == 2) & present.all(1)
    raw_kappa[~joint] = 0
    rejected = (raw_kappa > 1e-12) & (kept < 1e-12)
    counts = dict(records=len(records), both_present=int(present.all(1).sum()),
                  missing_source=int((~present.all(1)).sum()),
                  admitted_positive=int(((kept > 0) & (delta > 0)).sum()),
                  admitted_negative=int(((kept > 0) & (delta < 0)).sum()),
                  rejected_positive=int((rejected & (delta > 0)).sum()),
                  rejected_negative=int((rejected & (delta < 0)).sum()),
                  negative_age=int((age < -1e-12).sum()),
                  aggregate_fallback=int((records[:, 59] > 0).sum()))
    return result, counts


def measurement_support(data, t, cutoff):
    truth = data['truth'][0, t][:2].T
    support = np.zeros((len(truth), 2), bool)
    opportunities = np.zeros_like(support)
    counts = np.zeros(3, int)
    for n in [0, 1]:
        indices = np.flatnonzero(np.sum((truth-data['positions'][:, n, t])**2, axis=1) <= 1600)
        opportunities[indices, n] = True
        measurements = data['measurements'][n, t].T
        costs = np.linalg.norm(truth[indices, None, :]-measurements[None, :, :], axis=2)
        i, j = linear_sum_assignment(np.where(costs <= cutoff, costs, 1e6))
        keep = costs[i, j] <= cutoff
        support[indices[i[keep]], n] = True
        counts += [len(indices), len(measurements), int(keep.sum())]
    return support, opportunities, counts


def matched_truth(truth, states, cutoff):
    x = np.asarray(truth, float).reshape(4, -1).T[:, :2]
    y = np.asarray(states, float).reshape(-1, 4)[:, :2]
    costs = np.linalg.norm(x[:, None, :]-y[None, :, :], axis=2)
    i, j = linear_sum_assignment(np.where(costs <= cutoff, costs, 1e6))
    keep = costs[i, j] <= cutoff
    answer = np.zeros(len(x), bool)
    answer[i[keep]] = True
    return answer


def averages(rows, keys):
    return {key: float(np.mean([r[key] for r in rows])) for key in keys}


def summarize(frames):
    keys = [key for key in frames[0] if key.startswith(('gce_', 'noage_', 'delta_', 'cf_', 'attribution_'))]
    rows = []
    for cohort in ['v2x_val', 'v2x_test', 'v2x_all']:
        for condition in ['reliable', 'intermittent']:
            part = [r for r in frames if (cohort == 'v2x_all' or r['cohort'] == cohort) and r['condition'] == condition]
            seqs = sorted({r['sequence'] for r in part})
            dates = sorted({r['recording'] for r in part})
            seq_means = [averages([r for r in part if r['sequence'] == seq], keys) for seq in seqs]
            date_means = []
            for date in dates:
                dseqs = [seq for seq in seqs if next(r['recording'] for r in part if r['sequence'] == seq) == date]
                date_means.append(averages([seq_means[seqs.index(seq)] for seq in dseqs], keys))
            sequence_macro = averages(seq_means, keys)
            assert np.isclose(sequence_macro['delta_ospa'], sum(sequence_macro['attribution_'+k] for k in COMPONENTS), atol=1e-12)
            rows.append(dict(cohort=cohort, condition=condition, sequences=len(seqs), collection_dates=len(dates),
                             robot_frames=len(part), sequence_macro=sequence_macro,
                             frame_weighted=averages(part, keys), date_macro=averages(date_means, keys)))
    return rows


def unit_analysis(unit, cohort, cfg, audit, result_dir, verify):
    sequence = unit['sequence']
    input_path = ROOT/unit['data_path']
    verify(input_path, unit['input_sha256'])
    compact = loadmat(input_path)
    frames = int(unit['frames'])
    detector_counts = Counter()
    support_by_frame = {}
    for t in range(frames):
        for cutoff in [2, 12]:
            support, opportunities, counts = measurement_support(compact, t, cutoff)
            support_by_frame[t, cutoff] = support, opportunities
            for key, count in zip(['opportunities', 'detections', 'matches'], counts):
                detector_counts[f'{cutoff}m_{key}'] += int(count)
    frame_rows, sequence_rows, events, windows = [], [], [], []
    audit_rows = {(r['sequence'], r['condition'], r['arm']): r for r in audit['rows']}
    for condition in ['reliable', 'intermittent']:
        source = {}
        for arm in [GCE, NOAGE]:
            path = result_dir/f'{sequence}_{condition}_{arm}.json.gz'
            verify(path, audit['inputs'][str(path.relative_to(ROOT))])
            source[arm] = read(path)
            assert source[arm]['inputSha256'] == unit['input_sha256']
        a, b = source[GCE], source[NOAGE]
        for field in ['truth', 'truthIds', 'positions', 'time', 'delivered', 'inputSha256']:
            assert a[field] == b[field], (sequence, condition, field)
        assert len(a['time']) == frames
        records = np.asarray(a['runs']['iterationRecords'], float).reshape(-1, 60)
        rec_by_frame = defaultdict(list)
        for index, row in enumerate(records):
            rec_by_frame[int(row[0])-1, int(row[1])-1].append(index)
        variants, mechanism_counts = alternatives(records)
        paired, native_values = [], {arm: [] for arm in [GCE, NOAGE]}
        active_events = {}
        target_counts = Counter()
        for t in range(frames):
            assert np.array_equal(np.asarray(a['truth'][t]).reshape(4, -1), compact['truth'][0, t])
            ids = np.asarray(a['truthIds'][t]).reshape(-1)
            positions = np.asarray(a['positions'])[:, :, t]
            for n in [0, 1]:
                values = {}
                for arm in [GCE, NOAGE]:
                    run = source[arm]['runs']
                    values[arm] = scoring(a['truth'][t], run['estimates'][n+2*t])
                    v = values[arm]
                    for metric, native in [('ospa', 'ospa'), ('countError', 'countError'),
                                           ('loc2', 'matchedSquaredError'), ('matches', 'matchedCount')]:
                        assert np.isclose(v[metric], run[native][n][t], atol=1e-8, rtol=1e-9), (sequence, condition, t, n, metric)
                    if t in [0, frames//2, frames-1]:
                        reference = reference_score(a['truth'][t], run['estimates'][n+2*t])
                        assert all(np.isclose(v[k], reference[k], atol=1e-10) for k in METRICS)
                    native_values[arm].append(v)
                va, vb = values[GCE], values[NOAGE]
                delivered = bool(a['delivered'][n][1-n][t])
                row = dict(cohort=cohort, sequence=sequence, scene=unit['scene'], recording=unit['recording'],
                           condition=condition, frame=t+1, robot=n+1, delivered=delivered,
                           truth_count=va['truth_count'], gce_output_count=va['output_count'], noage_output_count=vb['output_count'])
                for prefix, value in [('gce', va), ('noage', vb)]:
                    row.update({prefix+'_'+key: value[key] for key in METRICS})
                row.update({'delta_'+key: va[key]-vb[key] for key in METRICS})
                row.update({'attribution_'+key: value for key, value in exact_ospa_decomposition(va, vb).items()})
                indices = np.asarray(rec_by_frame[t, n], int)
                if delivered:
                    for variant in VARIANTS:
                        rr, means = variants[variant]
                        states, chosen, extraction = extract(records[indices], rr[indices], means[indices], positions)
                        value = scoring(a['truth'][t], states)
                        if variant == 'native':
                            assert all(np.isclose(value[k], va[k], atol=1e-8, rtol=1e-9) for k in METRICS), (sequence, condition, t, n, 'reconstruct')
                        for key in METRICS:
                            row[f'cf_{variant}_{key}'] = value[key]-va[key]
                        row[f'cf_{variant}_output_count'] = len(states)-va['output_count']
                else:
                    assert not len(indices)
                    for variant in VARIANTS:
                        for key in METRICS+['output_count']:
                            row[f'cf_{variant}_{key}'] = 0.
                for cutoff in [2, 12]:
                    aa = matched_truth(a['truth'][t], a['runs']['estimates'][n+2*t], cutoff)
                    bb = matched_truth(a['truth'][t], b['runs']['estimates'][n+2*t], cutoff)
                    support, opportunities = support_by_frame[t, cutoff]
                    for kind, mask in [('noage_only', bb & ~aa), ('gce_only', aa & ~bb)]:
                        target_counts[f'{cutoff}m_{kind}'] += int(mask.sum())
                        for sources in [0, 1, 2]:
                            target_counts[f'{cutoff}m_{kind}_support_{sources}'] += int((mask & (support.sum(1) == sources)).sum())
                    current = {int(ids[i]) for i in np.flatnonzero(bb & ~aa)}
                    previous_keys = [key for key in active_events if key[:2] == (n, cutoff)]
                    for key in previous_keys:
                        if key[2] not in current:
                            events.append(active_events.pop(key))
                    for identity in current:
                        key = n, cutoff, identity
                        if key not in active_events:
                            active_events[key] = dict(cohort=cohort, sequence=sequence, condition=condition, robot=n+1,
                                                     cutoff=cutoff, truth_id=identity, first_frame=t+1, last_frame=t+1,
                                                     frames=0, supported_frames=0, both_supported_frames=0)
                        event = active_events[key]
                        event['last_frame'] = t+1
                        event['frames'] += 1
                        index = np.flatnonzero(ids == identity)
                        assert len(index) == 1
                        num = int(support[index[0]].sum())
                        event['supported_frames'] += int(num > 0)
                        event['both_supported_frames'] += int(num == 2)
                paired.append(row)
        events.extend(active_events.values())
        for arm in [GCE, NOAGE]:
            expected = audit_rows[sequence, condition, arm]
            for key in METRICS:
                assert np.isclose(np.mean([r[key] for r in native_values[arm]]), expected[key], atol=1e-9, rtol=1e-9)
        numeric = [key for key in paired[0] if key.startswith(('gce_', 'noage_', 'delta_', 'cf_', 'attribution_'))]
        sequence_rows.append(dict(cohort=cohort, sequence=sequence, scene=unit['scene'], recording=unit['recording'],
                                  condition=condition, frames=frames, means=averages(paired, numeric),
                                  mechanism_counts=mechanism_counts, target_counts=dict(target_counts)))
        series = np.asarray([.5*sum(r['delta_ospa'] for r in paired if r['frame'] == t+1) for t in range(frames)])
        width = min(15, frames)
        rolling = np.convolve(series, np.ones(width), mode='valid')/width
        for kind, first in [('largest_loss', int(rolling.argmax())), ('largest_gain', int(rolling.argmin()))]:
            windows.append(dict(cohort=cohort, sequence=sequence, scene=unit['scene'], condition=condition, kind=kind,
                                first_frame=first+1, last_frame=first+width, mean_ospa_difference=float(rolling[first])))
        frame_rows.extend(paired)
        print('PAIRED DIAGNOSED', sequence, condition, frames, 'frames', flush=True)
    return frame_rows, sequence_rows, events, windows, dict(cohort=cohort, sequence=sequence, **detector_counts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight', metavar='SEQUENCE')
    args = parser.parse_args()
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE_COMMIT
    destination = OUT/'GAP_DIAGNOSIS.json'
    assert not destination.exists()
    checked = {}

    def verify(path, expected=None):
        path = Path(path)
        name = str(path.relative_to(ROOT))
        if name not in checked:
            checked[name] = sha(path)
        if expected is not None:
            assert checked[name] == expected, name
        return checked[name]

    plans = []
    for cohort, folder, stage, count in STAGES:
        base = TRIALS/folder
        cfg_path, audit_path = base/'stages'/(stage+'.json'), base/('audit_'+stage+'.json')
        cfg, audit = json.loads(cfg_path.read_text()), json.loads(audit_path.read_text())
        assert audit['passed'] and len(cfg['units']) == count
        verify(cfg_path, audit['config_sha256'])
        verify(audit_path)
        for name, expected in cfg['source_sha256'].items():
            verify(ROOT/name, expected)
        for unit in cfg['units']:
            if args.preflight is None or unit['sequence'] == args.preflight:
                plans.append((unit, cohort, cfg, audit, base/'results'/stage))
    assert len(plans) == (1 if args.preflight else 19)
    for path in [Path(__file__), OUT/'PROTOCOL.md', TRIALS/'icra_external_fusion/analyze_v2v4real.py',
                 TRIALS/'icra_external_fusion/analyze_case_studies.py']:
        verify(path)
    if not args.preflight:
        freeze = OUT/'DIAGNOSIS_FREEZE.json'
        assert not freeze.exists()
        freeze.write_text(json.dumps(dict(created_utc=datetime.now(timezone.utc).isoformat(), base_commit=BASE_COMMIT,
                                         study='post-outcome diagnosis', sources=checked,
                                         sequences=[u[0]['sequence'] for u in plans]), indent=2)+'\n')
    frames, sequences, events, windows, support = [], [], [], [], []
    for unit, cohort, cfg, audit, result_dir in plans:
        ff, ss, ee, ww, dd = unit_analysis(unit, cohort, cfg, audit, result_dir, verify)
        frames.extend(ff); sequences.extend(ss); events.extend(ee); windows.extend(ww); support.append(dd)
    if args.preflight:
        print(json.dumps(sequences, allow_nan=False))
        print('PREFLIGHT PASSED', args.preflight, len(frames), 'paired robot-frames')
        return
    assert len(frames) == 4*(619+2172) and len(sequences) == 38
    frame_path = OUT/'paired_robot_frames.csv.gz'
    with gzip.open(frame_path, 'wt', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(frames[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(frames)
    aggregate = summarize(frames)
    result = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), base_commit=BASE_COMMIT,
                  sequences=19, paired_frames=2791, paired_robot_frames=len(frames), native_results=76,
                  aggregate=aggregate, sequence_rows=sequences, support=support,
                  noage_only_intervals=sorted(events, key=lambda r:(-r['frames'],r['sequence'],r['first_frame'])),
                  windows=windows, inputs=checked, frame_file=str(frame_path.relative_to(ROOT)), frame_sha256=sha(frame_path),
                  counterfactual_definition='Differences from native GCE at its visited current inputs. Undelivered frames unchanged. These are not recursive method runs.',
                  attribution_definition='Exact symmetric product decomposition of OSPA squared numerator times inverse max count, divided by sum of paired OSPAs. Terms sum to the OSPA difference in metres.',
                  support_definition='One-to-one measurement-center matches within known 40 m sensor supports; annotated targets include occlusion. Identity intervals use maximal valid matching at the indicated cutoff.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for r in aggregate:
        m = r['sequence_macro']
        print(r['cohort'], r['condition'], 'GAP', m['delta_ospa'], 'CF', {v:m['cf_'+v+'_ospa'] for v in VARIANTS}, flush=True)
    print('GAP DIAGNOSIS COMPLETE', len(frames), 'paired robot-frames', flush=True)


if __name__ == '__main__':
    main()
