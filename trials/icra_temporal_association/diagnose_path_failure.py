"""Preserve the failed-path evidence and invalidate affected comparisons."""
from datetime import datetime, timezone
from pathlib import Path
import gzip
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def domain_mismatches(data):
    run = data['runs']
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    inc = {tuple(r[:4]): r for r in np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)}
    poses = np.asarray(data['positions'])
    mismatches = []
    for row in local:
        t, n = row[:2].astype(int)
        xy = row[4:6]
        d = ((poses[:, :, t-1].T - xy) ** 2).sum(1)
        current = abs(xy[0]) <= 70.4 and abs(xy[1]) <= 40 and (d > 9).all() and d[n-1] <= 1600
        actual = inc[tuple(row[:4])]
        expected = data['pd'] if current else 0.
        if actual[10] != expected or bool(actual[7]) != current:
            mismatches.append(dict(key=row[:4].astype(int).tolist(), predicted_xy=xy.tolist(),
                vehicle_squared_distances=d.tolist(), expected_pd=expected, executed_pd=actual[10]))
    return len(local), mismatches


def main():
    destination = OUT / 'PATH_FAILURE.json'
    assert not destination.exists()
    probe = ROOT / 'RUN/ICRA_TEMPORAL_ASSOCIATION/path_probe.log'
    text = probe.read_text()
    assert f'QUALITY BEFORE UNIT {ROOT}/trials/icra_external_fusion/replay_quality/evaluateSensorQuality.m' in text
    assert f'QUALITY AFTER UNIT {ROOT}/common/evaluateSensorQuality.m' in text
    assert f'QUALITY RESTORED {ROOT}/trials/icra_external_fusion/replay_quality/evaluateSensorQuality.m' in text
    inputs = {str(probe.relative_to(ROOT)): sha(probe)}
    rows = []
    for stage in ['association_v1_preflight', 'association_v1_development_rest', 'association_v2_preflight',
                  'association_instrumentation_development']:
        paths = sorted((OUT / 'results' / stage).glob('*.json.gz'))
        assert len(paths) == dict(association_v1_preflight=6, association_v1_development_rest=32,
            association_v2_preflight=12, association_instrumentation_development=18)[stage]
        for path in paths:
            with gzip.open(path, 'rt') as handle:
                data = json.load(handle)
            count, mismatches = domain_mismatches(data)
            if stage == 'association_instrumentation_development':
                assert not mismatches
            rows.append(dict(stage=stage, sequence=data['sequence'], condition=data['condition'],
                arm=data['runs']['arm'], retained_local_predictions=count,
                domain_mismatches=len(mismatches), first_mismatch=mismatches[0] if mismatches else None))
            inputs[str(path.relative_to(ROOT))] = sha(path)
        print('PATH DIAGNOSED', stage, len(paths), flush=True)
    for name in ['DEVELOPMENT_SELECTION.json', 'V1_FAILURE_ATTRIBUTION.json', 'V1_RESULTS_CN.md']:
        path = OUT / name
        inputs[str(path.relative_to(ROOT))] = sha(path)
    result = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(),
        native_probe_returncode=0, invalidated_candidate_runs=50, unaffected_instrumentation_runs=18,
        cause='The association unit check moved common/ ahead of replay_quality/. The restored runner reinstalls the adapter after checks.',
        failed_parity=dict(stage='association_v2_preflight', sequence='0001', condition='reliable',
            field='estimates', first_material_increment_key=[8, 2, 2, 200001], original_pd=0., incorrect_pd=.9),
        interpretation='All V1 candidate selection and V2 preflight method comparisons are withdrawn. Rerun the unchanged four candidate definitions with the original domain.',
        rows=rows, inputs=inputs, source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__),
            OUT / 'checkObservationAssociation.m', OUT / 'probeAssociationPath.m', OUT / 'audit_observation_domain.py']})
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('PATH FAILURE PROVEN; 50 runs invalidated; 18 instrumentation runs preserve the original domain', flush=True)


if __name__ == '__main__':
    main()
