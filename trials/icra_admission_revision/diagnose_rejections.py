"""Identify discarded scalar evidence at frozen, previously scored GCE inputs."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys

import numpy as np
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'
sys.path.insert(0, str(REVIEW))
from review_gaussian_audit import unpack, natural
sys.path.insert(0, str(OUT.parent / 'icra_fusion_holdout'))
from analyze_holdout import counterfactual, score

ARM = 'marked_gaussian_evidence'
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
RULES = ['original', 'restore_all_scalar', 'restore_positive_scalar', 'restore_negative_scalar']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def diagnose(path, expected, cohort, sequence, condition):
    assert sha(path) == expected
    with gzip.open(path, 'rt') as handle:
        data = json.load(handle)
    run = data['runs']
    assert run['arm'] == ARM
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    jp, _, _ = natural(local[:, 4:8], unpack(local[:, 8:18]))
    ju, _, _ = natural(local[:, 18:22], unpack(local[:, 22:32]))
    dj = ju - jp
    smallest = np.linalg.eigvalsh(dj)[:, 0]
    tolerance = 1e-10 * np.maximum(1., np.maximum(np.linalg.norm(ju, 2, axis=(-2, -1)), np.linalg.norm(jp, 2, axis=(-2, -1))))
    local_ids = {tuple(row[:4].astype(int)): i for i, row in enumerate(local)}
    original = records[:, 31:35].reshape(-1, 2, 2)
    present = original[:, :, 0] > 0
    b, beta, delta = records[:, 13:15], records[:, 28:30], records[:, 19:21]
    active = b > 0
    joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
    raw = joint[:, None] * (active - beta) * np.where(delta >= 0, records[:, 26:28], records[:, 35:37])
    kept = records[:, 52:54]
    discarded = raw - kept
    assert np.all(discarded >= -1e-14)
    discarded = np.maximum(discarded, 0)
    rejected = (raw > 0) & ~records[:, 57:59].astype(bool)
    source_rows = []
    for i, side in np.argwhere(rejected):
        sensor = int(records[i, 1]) if side == 0 else 3 - int(records[i, 1])
        key = (int(records[i, 0]), sensor, *original[i, side].astype(int))
        idx = local_ids[key]
        assert smallest[idx] < -tolerance[idx]
        source_rows.append(dict(cohort=cohort, sequence=sequence, condition=condition,
            frame=int(records[i, 0]), receiver=int(records[i, 1]), source=sensor,
            source_birth_time=int(original[i, side, 0]), source_birth_location=int(original[i, side, 1]),
            delta=float(delta[i, side]), raw_kappa=float(raw[i, side]),
            discarded_log_odds=float(discarded[i, side] * delta[i, side]),
            smallest_precision_eigenvalue=float(smallest[idx]), curvature_tolerance=float(tolerance[idx])))
    logits = np.zeros_like(b)
    r = np.clip(records[:, 17:19][active], 1e-9, 1 - 1e-9)
    logits[active] = np.log(r) - np.log1p(-r)
    log_odds = (beta * logits).sum(1) + (kept * delta).sum(1) + records[:, 56]
    assert np.allclose(expit(log_odds), records[:, 9], atol=2e-10, rtol=0)
    additions = {'original': np.zeros(len(records)),
                 'restore_all_scalar': (discarded * delta).sum(1),
                 'restore_positive_scalar': (discarded * np.maximum(delta, 0)).sum(1),
                 'restore_negative_scalar': (discarded * np.minimum(delta, 0)).sum(1)}
    alternatives = {}
    for rule, extra in additions.items():
        candidate = records.copy()
        candidate[:, 9] = expit(log_odds + extra)
        alternatives[rule] = candidate
    scores = {rule: {key: [] for key in METRICS} for rule in RULES}
    changed_frames = {rule: 0 for rule in RULES}
    poses = np.asarray(data['positions'])
    T = len(data['time'])
    for t in range(T):
        for n in range(2):
            mask = (records[:, 0] == t + 1) & (records[:, 1] == n + 1)
            baseline = np.asarray(run['estimates'][n + 2 * t], float).reshape(-1, 4)
            for rule in RULES:
                if np.asarray(data['delivered'])[n, 1 - n, t]:
                    output = counterfactual(alternatives[rule][mask], 9, poses[:, :, t])
                else:
                    assert not mask.any()
                    output = baseline
                value = score(data['truth'][t], output)
                if rule == 'original':
                    assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                else:
                    changed_frames[rule] += int(not np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-10, rtol=0))
                for key in METRICS:
                    scores[rule][key].append(value[key])
    summaries = [dict(cohort=cohort, sequence=sequence, condition=condition, rule=rule, frames=T,
                      changed_robot_frames=changed_frames[rule], **{key: float(np.mean(v)) for key, v in scores[rule].items()}) for rule in RULES]
    stats = dict(cohort=cohort, sequence=sequence, condition=condition, fusion_labels=len(records),
                 eligible_source_uses=int((raw > 0).sum()), rejected_source_uses=int(rejected.sum()),
                 rejected_positive_sources=int((rejected & (delta > 0)).sum()),
                 rejected_negative_sources=int((rejected & (delta < 0)).sum()),
                 rejected_zero_sources=int((rejected & (delta == 0)).sum()),
                 discarded_positive_log_odds=float((discarded * np.maximum(delta, 0)).sum()),
                 discarded_negative_log_odds=float((discarded * np.minimum(delta, 0)).sum()),
                 aggregate_fallbacks=int(records[:, 59].sum()))
    print('DIAGNOSED', cohort, sequence, condition, stats, flush=True)
    for row in summaries:
        print('FIXED INPUT', row['rule'], 'OSPA', round(row['ospa'], 6), 'miss2', round(row['miss2'], 6), 'false2', round(row['false2'], 6), flush=True)
    return summaries, source_rows, stats


def main():
    dest = OUT / 'REJECTION_DIAGNOSTIC.json'
    assert not dest.exists()
    development = json.loads((OUT.parent / 'icra_gaussian_evidence/summary_development.json').read_text())
    validation = json.loads((REVIEW / 'audit_new_validation_primary.json').read_text())
    assert validation['passed']
    rows, sources, stats, inputs = [], [], [], {}
    units = [('new_recording_diagnostic', '0000', condition,
              REVIEW / 'results/new_validation_primary' / f'0000_{condition}_{ARM}.json.gz', validation['inputs'])
             for condition in ['reliable', 'intermittent']]
    units += [('development', f'{seq:04d}', condition,
               OUT.parent / 'icra_gaussian_evidence/results_development' / f'{seq:04d}_{condition}_{ARM}.json.gz', development['inputs'])
              for seq in range(9) for condition in ['reliable', 'intermittent']]
    for cohort, sequence, condition, path, manifest in units:
        expected = manifest[str(path.relative_to(ROOT))]
        a, b, c = diagnose(path, expected, cohort, sequence, condition)
        rows.extend(a); sources.extend(b); stats.append(c)
        inputs[str(path.relative_to(ROOT))] = expected
    result = dict(passed=True, diagnostic='Fixed frozen GCE input; restore discarded scalar evidence and retain its spatial posterior.',
                  rows=rows, rejection_statistics=stats, inputs=inputs, code_sha256=sha(Path(__file__)),
                  scope='All nine original development sequences plus the reviewer-identified 409-frame recording; both link conditions.')
    dest.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for name, records in [('fixed_input_scores.csv', rows), ('rejected_sources.csv', sources)]:
        with (OUT / name).open('w') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(records[0]))
            writer.writeheader(); writer.writerows(records)
    print('COMPLETE REJECTION DIAGNOSTIC', len(stats), 'full sequence-conditions', len(sources), 'rejected source uses', flush=True)


if __name__ == '__main__':
    main()
