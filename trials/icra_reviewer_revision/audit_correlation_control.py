"""Check the scalar producer using full joint Gaussian matrix calculations."""
import json
from pathlib import Path
import hashlib

import numpy as np
from scipy.stats import chi2

OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    spec = json.loads((OUT / 'CORRELATION_FREEZE.json').read_text())
    report = json.loads((OUT / 'correlation_control.json').read_text())
    assert report['freeze_sha256'] == sha(OUT / 'CORRELATION_FREEZE.json')
    assert spec['code_sha256'] == sha(OUT / 'correlation_control.py')
    assert report['samples_sha256'] == sha(OUT / 'correlation_samples.npz')
    samples = np.load(OUT / 'correlation_samples.npz', allow_pickle=False)
    I = np.eye(2); P0 = 25 * I; H = np.vstack([I, I]); q = chi2.ppf(.95, 2)
    children = np.random.SeedSequence(spec['seed']).spawn(len(spec['rhos']))
    checks = []
    for rho, child in zip(spec['rhos'], children):
        random = np.random.default_rng(child); N = spec['samples_per_rho']
        x = random.normal(size=(N, 2)) * 5
        common = random.normal(size=(N, 2)); independent = random.normal(size=(N, 2, 2))
        y = x[:, None] + np.sqrt(rho) * common[:, None] + np.sqrt(1 - rho) * independent
        Y = y.reshape(N, 4); R = np.block([[I, rho * I], [rho * I, I]])
        P_local = np.linalg.inv(np.linalg.inv(P0) + I)
        P_gce = np.linalg.inv(np.linalg.inv(P0) + H.T @ H)
        P_oracle = np.linalg.inv(np.linalg.inv(P0) + H.T @ np.linalg.solve(R, H))
        matrices = [(P_local, np.hstack([P_local / 2, P_local / 2])),
                    (P_gce, P_gce @ H.T), (P_oracle, np.linalg.solve(R, H @ P_oracle).T)]
        for arm, (P, K) in zip(spec['arms'], matrices):
            error = Y @ K.T - x
            squared = np.sum(error ** 2, axis=1)
            nees = np.einsum('ij,ji->i', error, np.linalg.solve(P, error.T))
            covered = nees <= q
            previous = samples[f'rho_{rho}_{arm.replace(" ", "_")}']
            assert np.allclose(squared, previous[:, 0], rtol=0, atol=1e-11)
            assert np.allclose(nees, previous[:, 1], rtol=0, atol=1e-10)
            assert np.array_equal(covered, previous[:, 2])
            actual = (K @ H - I) @ P0 @ (K @ H - I).T + K @ R @ K.T
            normalized = np.linalg.solve(P, actual)
            assert np.allclose(normalized, normalized[0, 0] * I, atol=1e-12)
            expected_coverage = float(chi2.cdf(q / normalized[0, 0], 2))
            row = next(r for r in report['rows'] if r['rho'] == rho and r['arm'] == arm)
            assert abs(row['expected_coverage_95'] - expected_coverage) < 1e-12
            assert abs(row['expected_rmse'] - np.sqrt(np.trace(actual))) < 1e-12
            assert abs(row['coverage_95'] - covered.mean()) < 1e-12
            assert abs(row['mean_nees_per_dimension'] - nees.mean() / 2) < 1e-12
            checks.append(dict(rho=rho, arm=arm, samples=N, full_joint_matrix_check=True))
    result = dict(passed=True, checks=checks, tested_samples=50000,
                  producer_sha256=sha(OUT / 'correlation_control.py'), source_report_sha256=sha(OUT / 'correlation_control.json'),
                  samples_sha256=sha(OUT / 'correlation_samples.npz'), auditor_sha256=sha(Path(__file__)))
    path = OUT / 'CORRELATION_AUDIT.json'; assert not path.exists()
    path.write_text(json.dumps(result, indent=2) + '\n')
    print('CORRELATION MATRIX AUDIT PASSED', len(checks), 'comparisons, 50000 shared draws', flush=True)


if __name__ == '__main__':
    main()
