"""Fixed-input spatial-projection screen on previously exposed recordings."""
from datetime import datetime, timezone
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
sys.path.insert(0, str(OUT.parent / 'icra_fusion_holdout'))
from analyze_holdout import counterfactual, score
from projected_ratio import project, natural, integrate, unpack

METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
RULES = ['original', 'projected_space', 'projected_all', 'projected_consensus']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT / 'PROJECTION_DIAGNOSTIC.json'; assert not destination.exists()
    source = OUT.parent / 'icra_admission_revision/REJECTION_DIAGNOSTIC.json'
    previous = json.loads(source.read_text()); assert previous['passed']
    summaries = []; diagnostics = []
    for key, expected in previous['inputs'].items():
        path = ROOT / key; assert sha(path) == expected
        with gzip.open(path, 'rt') as handle:
            data = json.load(handle)
        run = data['runs']; cohort = 'development' if data.get('cohort') == 'development' or 'results_development' in key else 'new_recording_diagnostic'
        sequence = data['sequence']; condition = data['condition']
        records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
        local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
        prior_mean, prior_cov = local[:, 4:8], unpack(local[:, 8:18])
        post_mean, post_cov = local[:, 18:22], unpack(local[:, 22:32])
        dj, dh, dc, good, rank = project(prior_mean, prior_cov, post_mean, post_cov)
        ju, hu, cu = natural(post_mean, post_cov)
        local_ids = {tuple(row[:4].astype(int)): i for i, row in enumerate(local)}
        original = records[:, 31:35].reshape(-1, 2, 2)
        present = original[:, :, 0] > 0
        indices = np.zeros(present.shape, int)
        for i, side in np.argwhere(present):
            sensor = int(records[i, 1]) if side == 0 else 3 - int(records[i, 1])
            indices[i, side] = local_ids[(int(records[i, 0]), sensor, *original[i, side].astype(int))]
        b, beta, delta = records[:, 13:15], records[:, 28:30], records[:, 19:21]
        active = b > 0; joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
        raw = joint[:, None] * (active - beta) * np.where(delta >= 0, records[:, 26:28], records[:, 35:37])
        old_kept = records[:, 52:54]
        space_kept = raw * (good[indices] | (rank[indices] > 0))
        alpha = records[:, 54:56]
        j0 = np.einsum('ns,nsij->nij', alpha, ju[indices])
        h0 = np.einsum('ns,nsi->ni', alpha, hu[indices])
        c0 = np.einsum('ns,ns->n', alpha, cu[indices])
        precision = j0 + np.einsum('ns,nsij->nij', space_kept, dj[indices])
        information = h0 + np.einsum('ns,nsi->ni', space_kept, dh[indices])
        constant = c0 + np.einsum('ns,ns->n', space_kept, dc[indices])
        mean, covariance, log_i = integrate(precision, information, constant)
        assert np.all(np.linalg.cond(precision, 1) < 1e12)
        unchanged = ~(space_kept > 0).any(1)
        log_i[unchanged] = records[unchanged, 10]
        logits = np.zeros_like(b)
        r = np.clip(records[:, 17:19][active], 1e-9, 1 - 1e-9)
        logits[active] = np.log(r) - np.log1p(-r)
        current = records[:, 21:23] == records[:, 0, None]
        agreed = joint & ((~active) | (current & (delta < 0))).all(1)
        consensus = old_kept + (raw - old_kept) * ((delta > 0) & (space_kept > 0))
        consensus[agreed] = raw[agreed]
        scalar = dict(projected_space=old_kept, projected_all=raw, projected_consensus=consensus)
        alternatives = {'original': records}
        for rule, value in scalar.items():
            candidate = records.copy()
            candidate[:, 4:6] = mean[:, :2]
            candidate[:, 9] = expit((beta * logits).sum(1) + (value * delta).sum(1) + log_i)
            alternatives[rule] = candidate
        scores = {rule: {k: [] for k in METRICS} for rule in RULES}
        poses = np.asarray(data['positions']); T = len(data['time'])
        for t in range(T):
            for n in range(2):
                mask = (records[:, 0] == t + 1) & (records[:, 1] == n + 1)
                baseline = np.asarray(run['estimates'][n + 2 * t], float).reshape(-1, 4)
                for rule in RULES:
                    output = counterfactual(alternatives[rule][mask], 9, poses[:, :, t]) if np.asarray(data['delivered'])[n, 1 - n, t] else baseline
                    value = score(data['truth'][t], output)
                    if rule == 'original':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for k in METRICS:
                        scores[rule][k].append(value[k])
        for rule in RULES:
            summaries.append(dict(cohort=cohort, sequence=sequence, condition=condition, rule=rule,
                                  frames=T, **{k: float(np.mean(v)) for k, v in scores[rule].items()}))
        rejected = (raw > 0) & ~good[indices]
        diagnostics.append(dict(cohort=cohort, sequence=sequence, condition=condition,
            rejected_source_uses=int(rejected.sum()),
            recovered_contracting_source_uses=int((rejected & (rank[indices] > 0)).sum()),
            retained_rank_counts={str(k): int((rejected & (rank[indices] == k)).sum()) for k in range(5)}))
        print('PROJECTION DIAGNOSTIC', cohort, sequence, condition, diagnostics[-1], flush=True)
    aggregate = []
    for cohort in ['development', 'new_recording_diagnostic']:
        for condition in ['reliable', 'intermittent']:
            for rule in RULES:
                rows = [r for r in summaries if r['cohort'] == cohort and r['condition'] == condition and r['rule'] == rule]
                aggregate.append(dict(cohort=cohort, condition=condition, rule=rule,
                                      **{k: float(np.mean([r[k] for r in rows])) for k in METRICS}))
                print('FIXED INPUT', aggregate[-1], flush=True)
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), rows=summaries,
                  aggregate=aggregate, diagnostics=diagnostics, inputs=previous['inputs'],
                  source_diagnostic_sha256=sha(source), protocol_sha256=sha(OUT / 'PROTOCOL.md'),
                  source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'projected_ratio.py']},
                  meaning='Fixed inputs from the previously scored GCE; all candidate spatial/scalar changes shown, no recursive-performance claim.')
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print('COMPLETE PROJECTION SCREEN', len(summaries), 'rows', flush=True)


if __name__ == '__main__':
    main()
