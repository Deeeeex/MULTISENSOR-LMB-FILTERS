"""Independent arithmetic and native-field checks without study-code imports."""
from collections import Counter, defaultdict
from pathlib import Path
import argparse
import csv
import gzip
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for data in iter(lambda: handle.read(4*1024**2), b''):
            h.update(data)
    return h.hexdigest()


def close(a, b):
    assert np.isclose(a, b, atol=1e-10, rtol=1e-10), (a, b)


def flag(value):
    assert value in ['', 'True', 'False']
    return value == 'True'


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--full', action='store_true')
    args = parser.parse_args()
    gap = json.loads((OUT/'GAP_DIAGNOSIS.json').read_text())
    track = json.loads((OUT/'TRACK_DIAGNOSIS.json').read_text())
    assert gap['passed'] and track['passed'] and gap['native_results'] == 76
    for obj, field, expected in [(gap, 'frame_file', 'frame_sha256'), (track, 'target_file', 'target_sha256')]:
        path = ROOT/obj[field]
        assert digest(path) == obj[expected]
    with gzip.open(ROOT/gap['frame_file'], 'rt') as f:
        frames = list(csv.DictReader(f))
    assert len(frames) == gap['paired_robot_frames'] == 11164
    keys = [k for k in frames[0] if k.startswith(('gce_', 'noage_', 'delta_', 'cf_', 'attribution_'))]
    for row in frames:
        for key in keys: row[key] = float(row[key])
        native = {}
        for arm in ['gce', 'noage']:
            n, m = int(row['truth_count']), int(row[arm+'_output_count'])
            count = abs(n-m); loc = row[arm+'_loc2']; missed = row[arm+'_miss2']; false = row[arm+'_false2']
            close(count, row[arm+'_countError'])
            close(missed/72, round(missed/72)); close(false/72, round(false/72))
            close(n-missed/72, m-false/72)
            numerator = loc+missed+false+72*count; denominator = max(n, m)
            close(row[arm+'_ospa']**2, numerator/denominator if denominator else 0)
            close(row[arm+'_gospa']**2, loc+missed+false)
            native[arm] = numerator, 1/denominator if denominator else 0
        close(row['delta_ospa'], row['gce_ospa']-row['noage_ospa'])
        close(sum(row['attribution_'+k] for k in ['localization','missed','false','cardinality_gap','normalization']), row['delta_ospa'])
        for key in keys:
            if key.startswith('cf_native_'): close(row[key], 0)
        a, ia = native['gce']; b, ib = native['noage']; total = row['gce_ospa']+row['noage_ospa']
        norm = (a+b)*(ia-ib)/(2*total) if total else 0
        close(norm, row['attribution_normalization'])
    sequence = {}
    for row in gap['sequence_rows']:
        part = [r for r in frames if r['sequence']==row['sequence'] and r['condition']==row['condition']]
        assert len(part) == 2*row['frames']
        means = {k:float(np.mean([r[k] for r in part])) for k in keys}
        for k in keys: close(means[k], row['means'][k])
        sequence[row['sequence'], row['condition']] = means
    for row in gap['aggregate']:
        part = [r for r in frames if (row['cohort']=='v2x_all' or r['cohort']==row['cohort']) and r['condition']==row['condition']]
        names = sorted({r['sequence'] for r in part}); dates = sorted({r['recording'] for r in part})
        assert len(names)==row['sequences'] and len(dates)==row['collection_dates'] and len(part)==row['robot_frames']
        for k in keys:
            close(np.mean([r[k] for r in part]), row['frame_weighted'][k])
            close(np.mean([sequence[s,row['condition']][k] for s in names]), row['sequence_macro'][k])
            date_means = []
            for date in dates:
                dseq = {r['sequence'] for r in part if r['recording']==date}
                date_means.append(np.mean([sequence[s,row['condition']][k] for s in dseq]))
            close(np.mean(date_means), row['date_macro'][k])
    with gzip.open(ROOT/track['target_file'], 'rt') as f:
        targets = list(csv.DictReader(f))
    assert len(targets)==track['target_robot_frames']==5771
    for row in track['aggregate']:
        part = [r for r in targets if all(r[k]==row[k] for k in ['cohort','condition','kind'])]
        counts = Counter()
        for r in part:
            counts['target_robot_frames'] += 1
            counts['candidate_within_2m'] += flag(r['candidate'])
            counts['candidate_unselected'] += flag(r['candidate']) and not flag(r['selected'])
            counts['candidate_pruned'] += flag(r['below_pruning'])
            counts['one_sensor_support'] += int(r['support_sources'])==1
            counts['two_sensor_support'] += int(r['support_sources'])==2
            counts['one_support_both_geometric'] += int(r['support_sources'])==1 and int(r['geometric_sources'])==2
            for k in ['negative_admitted','positive_admitted','positive_gate_zero','positive_curvature_rejected',
                      'matched_without_negative_scalar','matched_without_positive_scalar','matched_without_age','matched_noage_same_input']:
                counts[k] += flag(r[k])
        assert dict(counts)==row['counts'], row
    checked = 0; trace_fields = 0
    if args.full:
        source_hashes = dict(gap['inputs'])
        for name, value in track['inputs'].items():
            if name in source_hashes: assert source_hashes[name] == value
            source_hashes[name] = value
        for name, value in source_hashes.items():
            assert digest(ROOT/name)==value, name
            checked += 1
        for case in track['traces']:
            for arm in ['marked_gaussian_evidence','marked_lineage']:
                suffix = f"/{case['sequence']}_{case['condition']}_{arm}.json.gz"
                paths = [p for p in track['inputs'] if p.endswith(suffix)]; assert len(paths)==1
                with gzip.open(ROOT/paths[0], 'rt') as f: data = json.load(f)
                run = data['runs']; records = np.asarray(run['iterationRecords'],float).reshape(-1,60)
                local = np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
                native = {tuple(z[:4].astype(int)):z for z in records}
                updates = {tuple(z[:4].astype(int)):z for z in local}
                for row in [r for r in case['rows'] if r['arm']==arm]:
                    t, n = row['frame'], row['robot']; key = (t,n,*case['anchor_label'])
                    detail = row['anchor']
                    if detail is not None:
                        if data['delivered'][n-1][2-n][t-1]:
                            z = native[key]
                            close(detail['r'], z[9]); close(detail['r0'], z[7])
                            assert np.allclose(detail['xy'], z[4:6], atol=1e-12, rtol=0)
                            if arm == 'marked_gaussian_evidence':
                                close(detail['negative_log_odds'], sum(z[52:54]*np.minimum(z[19:21],0)))
                                close(detail['positive_log_odds'], sum(z[52:54]*np.maximum(z[19:21],0)))
                                close(detail['normalizer_change'], z[56]-z[10])
                        else:
                            close(detail['r'], updates[key][5])
                        trace_fields += 1
                    labs = np.asarray(run['labels'][n-1+2*(t-1)],int).reshape(2,-1).T
                    raw = np.asarray(run['rawEstimates'][n-1+2*(t-1)],float).reshape(-1,4)
                    pos = np.asarray(data['positions'])[:,:,t-1]
                    distance = np.min(np.sum((raw[:,:2,None]-pos[None,:,:])**2,axis=1),axis=1) if len(raw) else np.array([])
                    keep = (abs(raw[:,0])<=70.4)&(abs(raw[:,1])<=40)&(distance<=1600)&(distance>9)
                    selected = any(np.array_equal(label,case['anchor_label']) for label in labs[keep])
                    assert selected == row['anchor_selected'], (case['sequence'],arm,t,n)
            print('NATIVE TRACE VERIFIED',case['sequence'],case['condition'],flush=True)
    report = dict(passed=True, full=args.full, sequence_comparisons=len(sequence), aggregate_rows=len(gap['aggregate']),
                  paired_robot_frames=len(frames), target_robot_frames=len(targets), native_trace_records=trace_fields,
                  source_files_checked=checked, verifier_sha256=digest(Path(__file__)))
    if args.full:
        destination = OUT/'DIAGNOSIS_VERIFICATION.json'; assert not destination.exists()
        destination.write_text(json.dumps(report,indent=2)+'\n')
    print('DIAGNOSIS VERIFIED',json.dumps(report),flush=True)


if __name__ == '__main__':
    main()
