"""Describe frozen detector support; no parameters or tracking inputs change."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def assignment(truth, measurements, cutoff):
    if not len(truth) or not len(measurements):
        return np.empty(0, int), np.empty(0, int), np.empty(0)
    distances = cdist(truth, measurements)
    # Maximize valid cardinality first; minimize distance within that count.
    i, j = linear_sum_assignment(np.where(distances <= cutoff, distances, 1e6))
    keep = distances[i, j] <= cutoff
    return i[keep], j[keep], distances[i[keep], j[keep]]


def main():
    destination = OUT / 'DETECTION_SUPPORT_DIAGNOSTIC.json'
    assert not destination.exists()
    sources = {}; units = []
    stages = [('v2v', 'icra_projected_admission/stages/projected_v1_preflight.json'),
              ('v2v', 'icra_projected_admission/stages/projected_v1_development_rest.json'),
              ('v2v', 'icra_projected_admission/stages/final_v2v_evaluation_v2.json'),
              ('v2x', 'icra_projected_admission/stages/final_v2x_evaluation.json')]
    for dataset, name in stages:
        path = TRIALS / name; sources[str(path.relative_to(ROOT))] = sha(path)
        cfg = json.loads(path.read_text())
        for r in cfg['units']:
            units.append(dict(dataset=dataset, **r))
    assert sum(r['dataset'] == 'v2v' for r in units) == 43
    assert sum(r['dataset'] == 'v2x' for r in units) == 5
    rows = []; score_bins = []
    for unit in units:
        data_path = ROOT / unit['data_path']; digest = sha(data_path)
        assert digest == unit['input_sha256']; sources[unit['data_path']] = digest
        data = loadmat(data_path)
        if 'calibratedScores' not in data:
            mark_path = ROOT / unit['marks_path']
            sources[unit['marks_path']] = sha(mark_path); data.update(loadmat(mark_path))
        T = int(data['T'].item())
        for cutoff in [2.0, 12.0]:
            counts = np.zeros(3, int); squared = 0.; predicted = []; outcomes = []
            for t in range(T):
                truth = data['truth'][0, t][:2].T
                for n in range(2):
                    position = data['positions'][:, n, t]
                    local = truth[np.sum((truth - position) ** 2, axis=1) <= 1600]
                    measurements = data['measurements'][n, t].T
                    probabilities = data['calibratedScores'][n, t].ravel()
                    assert len(probabilities) == len(measurements)
                    _, matched, distances = assignment(local, measurements, cutoff)
                    counts += [len(local), len(measurements), len(matched)]
                    squared += float(distances @ distances)
                    y = np.zeros(len(measurements)); y[matched] = 1
                    predicted.extend(probabilities); outcomes.extend(y)
            nt, nm, match = map(int, counts)
            p = np.asarray(predicted); y = np.asarray(outcomes)
            assert len(p) == nm and int(y.sum()) == match
            record = dict(dataset=unit['dataset'], sequence=unit['sequence'], frames=T,
                cutoff_m=cutoff, truth_sensor_opportunities=nt, detections=nm, matches=match,
                recall=match / nt if nt else None, precision=match / nm if nm else None,
                matched_rmse_m=float(np.sqrt(squared / match)) if match else None,
                mean_calibrated_probability=float(p.mean()) if len(p) else None,
                brier=float(np.mean((p-y)**2)) if len(p) else None)
            rows.append(record)
            for lower, upper in zip([0, .2, .4, .6, .8], [.2, .4, .6, .8, 1.000001]):
                keep = (p >= lower) & (p < upper)
                if keep.any():
                    score_bins.append(dict(dataset=unit['dataset'], sequence=unit['sequence'], cutoff_m=cutoff,
                        lower=lower, upper=min(upper, 1), detections=int(keep.sum()), matches=int(y[keep].sum()),
                        mean_probability=float(p[keep].mean()), observed_fraction=float(y[keep].mean())))
    aggregate = []
    for dataset in ['v2v', 'v2x']:
        for cutoff in [2., 12.]:
            part = [r for r in rows if r['dataset'] == dataset and r['cutoff_m'] == cutoff]
            nt = sum(r['truth_sensor_opportunities'] for r in part)
            nm = sum(r['detections'] for r in part); match = sum(r['matches'] for r in part)
            aggregate.append(dict(dataset=dataset, cutoff_m=cutoff, sequences=len(part),
                truth_sensor_opportunities=nt, detections=nm, matches=match,
                pooled_recall=match / nt, pooled_precision=match / nm,
                sequence_macro_recall=float(np.mean([r['recall'] for r in part if r['recall'] is not None])),
                sequence_macro_precision=float(np.mean([r['precision'] for r in part if r['precision'] is not None])),
                mean_calibrated_probability=float(sum(r['mean_calibrated_probability'] * r['detections'] for r in part) / nm)))
    sources[str(Path(__file__).relative_to(ROOT))] = sha(Path(__file__))
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), rows=rows,
        aggregate=aggregate, score_bins=score_bins, source_sha256=sources,
        scope='Post-outcome diagnostic only. Per-sensor one-to-one center assignment inside each frozen 40 m support. The 12 m cutoff matches calibration labeling; 2 m describes strict center support. Annotated opportunities include occluded targets. These are detector support statistics, not per-target visibility estimates or a tracking oracle.')
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for r in aggregate:
        print(r, flush=True)


if __name__ == '__main__':
    main()
