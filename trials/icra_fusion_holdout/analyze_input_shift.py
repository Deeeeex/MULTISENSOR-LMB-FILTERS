"""Post-registration input diagnostic; it cannot select or fit a tracker.

Use all nine development and all 25 reserved sequences. Reconstruct the same
per-sensor 12 m assignment labels used by the frozen score calibration, then
report sequence-balanced and detection-pooled calibration/density summaries.
Development predictions are leave-one-sequence-out; reserved predictions use
the already frozen full-development fit. No tracking outputs are read.
"""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(y, raw, calibrated):
    result = dict(detections=len(y), positives=int(y.sum()), positive_fraction=float(y.mean()))
    for name, p in [('raw', raw), ('calibrated', calibrated)]:
        p = np.clip(p, 1e-12, 1 - 1e-12)
        result[name + '_brier'] = float(np.mean((p - y) ** 2))
        result[name + '_log_loss'] = float(np.mean(-y * np.log(p) - (1 - y) * np.log1p(-p)))
    return result


def main():
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    for name, expected in freeze['source_and_selection_evidence_sha256'].items():
        assert sha(ROOT / name) == expected, name
    audit = json.loads((OUT / 'input_audit.json').read_text())
    assert audit['exact_pose_measurement_truth_score_and_likelihood_reconstruction']
    assert audit['method_freeze_sha256'] == sha(OUT / 'METHOD_FREEZE.json')
    calibration_path = OUT.parent / 'icra_ceiling_iteration/calibration.json'
    calibration = json.loads(calibration_path.read_text())
    diagnosis_path = OUT.parent / 'icra_evidence_iteration/detection_score_diagnosis.csv'
    assert sha(diagnosis_path) == calibration['diagnosis_input_sha256']
    with diagnosis_path.open() as stream:
        development = list(csv.DictReader(stream))
    reports, pooled, sources = [], {}, {str(diagnosis_path.relative_to(ROOT)): sha(diagnosis_path)}
    cohorts = [('development', calibration['inputs']),
               ('reserved', json.loads((OUT / 'input_manifest.json').read_text())['sequences'])]
    for cohort, entries in cohorts:
        observations = []
        for entry in entries:
            name = entry['sequence']
            if cohort == 'development':
                path = OUT.parent / 'icra_external_fusion/data' / f'v2v4real_{name}.mat'
                assert sha(path) == entry['original_input_sha256']
                marks_path = OUT.parent / 'icra_ceiling_iteration/data_marks' / f'marks_{name}.mat'
                assert sha(marks_path) == entry['marks_sha256']
                marks = loadmat(marks_path)
                sources[str(marks_path.relative_to(ROOT))] = sha(marks_path)
            else:
                path = OUT / 'data' / f'v2v4real_{name}.mat'
                assert sha(path) == entry['input_sha256']
                marks = loadmat(path)
            data = loadmat(path)
            sources[str(path.relative_to(ROOT))] = sha(path)
            T = int(data['T'].item())
            seq_rows, counts = [], []
            for n, sensor in enumerate(['ego', '1']):
                for t in range(T):
                    xy = data['measurements'][n, t].T
                    truth = data['truth'][0, t][:2].T
                    cost = ((truth[:, None, :] - xy[None, :, :]) ** 2).sum(-1)
                    r, c = linear_sum_assignment(np.minimum(cost, 144))
                    y = np.zeros(len(xy), dtype=int)
                    y[c[cost[r, c] < 144]] = 1
                    raw = marks['rawScores'][n, t].ravel()
                    cal = marks['calibratedScores'][n, t].ravel()
                    assert len(xy) == len(raw) == len(cal)
                    fit = (next(f for f in calibration['folds'] if f['excluded_sequence'] == name)
                           if cohort == 'development' else calibration['full_seen_fit'])
                    bounded = np.clip(raw, 1e-6, 1 - 1e-6)
                    expected = expit(fit['a'] * (np.log(bounded) - np.log1p(-bounded)) + fit['b'])
                    assert np.allclose(cal, expected, atol=1e-14, rtol=0)
                    if cohort == 'development':
                        prior = [v for v in development if v['sequence'] == name and v['sensor'] == sensor and int(v['frame']) == t]
                        assert raw.tolist() == [float(v['score']) for v in prior]
                        assert y.tolist() == [int(v['positive_12m']) for v in prior]
                    seq_rows.extend(zip(y, raw, cal))
                    counts.append(len(xy))
            values = np.asarray(seq_rows)
            assert len(values) > 0
            observations.extend(seq_rows)
            gt_counts = np.array([data['truth'][0, t].shape[1] for t in range(T)])
            reports.append(dict(cohort=cohort, sequence=name, frames=T,
                                truth_per_frame=float(gt_counts.mean()),
                                detections_per_sensor_frame=float(np.mean(counts)),
                                **metrics(values[:, 0], values[:, 1], values[:, 2])))
        values = np.asarray(observations)
        cohort_rows = [r for r in reports if r['cohort'] == cohort]
        keys = ['positive_fraction', 'truth_per_frame', 'detections_per_sensor_frame',
                'raw_brier', 'calibrated_brier', 'raw_log_loss', 'calibrated_log_loss']
        pooled[cohort] = dict(sequences=len(entries), frames=sum(r['frames'] for r in cohort_rows),
                              sequence_macro={key: float(np.mean([r[key] for r in cohort_rows])) for key in keys},
                              detection_pooled=metrics(values[:, 0], values[:, 1], values[:, 2]))
    assert [pooled[c]['sequences'] for c, _ in cohorts] == [9, 25]
    assert pooled['reserved']['frames'] == freeze['frames'] == 5601
    report = dict(scope=__doc__, assignment_cutoff_m=12, post_registration_diagnostic=True,
                  no_parameter_fitting=True, no_tracking_outputs_read=True,
                  cohorts=pooled, sequences=reports, source_sha256=sources,
                  calibration_sha256=sha(calibration_path), method_freeze_sha256=sha(OUT / 'METHOD_FREEZE.json'),
                  analyzer_sha256=sha(Path(__file__)))
    (OUT / 'input_shift.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (OUT / 'input_shift_sequences.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(reports[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(reports)
    print(json.dumps(pooled, indent=2), flush=True)
    print('All 34 input sequences diagnosed; calibration remains frozen; no tracking results read.', flush=True)


if __name__ == '__main__':
    main()
