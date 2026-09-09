"""Exact baseline parity and independent reconstruction of observation moments."""
from pathlib import Path
import argparse
import json
import re
import sys

import numpy as np
from scipy.io import loadmat

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
for directory in ['icra_gaussian_evidence', 'icra_marked_control', 'icra_fusion_holdout', 'icra_reviewer_revision']:
    sys.path.insert(0, str(OUT.parent / directory))
from analyze_control import sha, read, score_run
from analyze_holdout import radio_draws
from review_probability_audit import audit_probability
from audit_stage import audit_packets


def direct_moments(weights, measurements, opportunity):
    result = np.zeros((len(opportunity), 8))
    if not weights.size or not len(measurements):
        return result
    assert weights.shape == (len(opportunity), len(measurements) + 1)
    assert np.isfinite(weights).all() and (weights >= 0).all()
    for j in range(len(weights)):
        total = weights[j].sum()
        if not opportunity[j] or total <= 0:
            continue
        detection = weights[j, 1:] / total
        mass = detection.sum()
        if mass <= 0:
            continue
        conditional = detection / mass
        mean = conditional @ measurements
        delta = measurements - mean
        covariance = np.eye(2) + np.einsum('n,ni,nj->ij', conditional, delta, delta)
        entropy = -sum(v * np.log(v) for v in conditional if v > 0)
        result[j] = [mass, *mean, covariance[0, 0], covariance[1, 0], covariance[1, 1], conditional.max(), entropy]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage')
    args = parser.parse_args()
    cfgpath = OUT / 'stages' / (args.stage + '.json')
    cfg = json.loads(cfgpath.read_text())
    runtimepath = OUT / ('runtime_' + args.stage + '.json')
    runtime = json.loads(runtimepath.read_text())
    assert len(runtime) == len(cfg['units']) == 9
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    source = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in cfg['source_sha256'].items()}
    assert len(source) == len(cfg['source_sha256'])
    original = OUT.parent / 'icra_gaussian_evidence'
    previous = json.loads((original / 'summary_development.json').read_text())
    hashes, rows, diagnostics, parity = {}, [], [], []
    direct_rows, weight_values = 0, 0
    for unit, native in zip(cfg['units'], runtime):
        name = unit['sequence']
        assert native['sequence'] == name
        assert native['returncode'] == 0 and native['completion_line'] and native['files'] == 2
        log = ROOT / 'RUN/ICRA_TEMPORAL_ASSOCIATION' / args.stage / (name + '.log')
        assert 'DIRECT OBSERVATION CHECK PASSED' in log.read_text()
        assert f'COMPLETED REVIEW {args.stage} {name}' in log.read_text()
        hashes[str(log.relative_to(ROOT))] = sha(log)
        mat = loadmat(ROOT / unit['data_path'])
        T = int(mat['T'].item())
        for condition in cfg['conditions']:
            arm = cfg['arms'][0]
            filename = f'{name}_{condition}_{arm}.json.gz'
            path = OUT / 'results' / args.stage / filename
            oldpath = original / 'results_development' / filename
            assert sha(oldpath) == previous['inputs'][str(oldpath.relative_to(ROOT))]
            data, olddata = read(path), read(oldpath)
            run, old = data['runs'], olddata['runs']
            assert data['sourceSha256'] == source and data['inputSha256'] == unit['input_sha256']
            assert data['stage'] == args.stage and data['cohort'] == 'development' and data['pd'] == .9
            for key in ['time', 'truth', 'truthIds', 'positions', 'delivered', 'sequence', 'condition']:
                assert data[key] == olddata[key], key
            keys = [k for k in old if k != 'runtimeSeconds']
            for key in keys:
                assert run[key] == old[key], (name, condition, 'exact original recursion', key)
            parity.append(dict(sequence=name, condition=condition, exact_fields=keys, robot_frames=2 * T))
            delivery = radio_draws(unit['radio_seed'] - 8301, T, condition)
            assert np.array_equal(delivery, data['delivered'])
            audit_packets(run, delivery, T)
            diagnostics.append(dict(sequence=name, condition=condition, **audit_probability(run, data)))
            value, _ = score_run(data, run)
            rows.append(dict(sequence=name, condition=condition, arm=arm, frames=T, **value))
            increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
            local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
            recorded = np.asarray(run['localDirectRecords'], float).reshape(-1, 12)
            assert np.array_equal(recorded[:, :4], local[:, :4])
            direct_rows += len(recorded)
            reconstructed = {}
            for t in range(1, T + 1):
                for n in [1, 2]:
                    subset = increments[(increments[:, 0] == t) & (increments[:, 1] == n)]
                    z = np.asarray(mat['measurements'][n - 1, t - 1]).reshape(2, -1).T
                    raw = np.asarray(run['localAssociationWeights'][n - 1 + 2 * (t - 1)], float)
                    W = raw.reshape(len(subset), len(z) + 1) if raw.size else np.zeros((0, 0))
                    weight_values += W.size
                    values = direct_moments(W, z, subset[:, 7].astype(bool))
                    assert np.allclose(values[:, 0], subset[:, 9], atol=1e-12, rtol=0)
                    for row, feature in zip(subset, values):
                        reconstructed[tuple(row[:4].astype(int))] = feature
            expected = np.array([reconstructed[tuple(r[:4].astype(int))] for r in recorded])
            assert np.allclose(recorded[:, 4:], expected, atol=1e-10, rtol=1e-10)
            for p in [path, oldpath]:
                hashes[str(p.relative_to(ROOT))] = sha(p)
            print('AUDITED INSTRUMENTATION', name, condition, len(recorded), 'direct records; exact baseline parity', flush=True)
    result = dict(passed=True, stage=args.stage, audited_robot_frames=sum(r['robot_frames'] for r in parity),
        local_direct_records=direct_rows, raw_association_weight_values=weight_values,
        config_sha256=sha(cfgpath), runtime_sha256=sha(runtimepath), inputs=hashes,
        rows=rows, diagnostics=diagnostics, parity=parity,
        auditor_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__),
            OUT.parent / 'icra_reviewer_revision/review_probability_audit.py',
            OUT.parent / 'icra_reviewer_revision/review_gaussian_audit.py',
            OUT.parent / 'icra_reviewer_revision/audit_stage.py']})
    destination = OUT / ('audit_' + args.stage + '.json')
    assert not destination.exists()
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('INSTRUMENTATION AUDIT PASSED', result['audited_robot_frames'], 'robot-frames', flush=True)


if __name__ == '__main__':
    main()
