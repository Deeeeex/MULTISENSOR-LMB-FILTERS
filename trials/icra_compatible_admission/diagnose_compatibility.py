"""Screen fixed, truth-free compatibility gates on existing GCE inputs."""
from datetime import datetime, timezone
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_reviewer_revision'))
from review_gaussian_audit import natural, integrate, unpack
sys.path.insert(0, str(OUT.parent / 'icra_fusion_holdout'))
from analyze_holdout import counterfactual, score

RULES = ['agreement_all', 'agreement_positive', 'agreement_conflict', 'positive_consensus', 'positive_only']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def factors(records, delta, active, joint):
    overlap = np.exp(np.minimum(0, records[:, 10]))
    assert np.all(records[:, 10] < 1e-7) and np.all((overlap >= 0) & (overlap <= 1))
    opposing = joint & ((delta > 0) & active).any(1) & ((delta < 0) & active).any(1)
    positive_agreement = joint & ((~active) | (delta > 0)).all(1)
    return dict(agreement_all=np.broadcast_to(overlap[:, None], delta.shape),
                agreement_positive=np.where(delta > 0, overlap[:, None], 1),
                agreement_conflict=np.broadcast_to(np.where(opposing, overlap, 1)[:, None], delta.shape),
                positive_consensus=np.where(delta > 0, positive_agreement[:, None], 1),
                positive_only=(delta >= 0).astype(float))


def main():
    destination = OUT / 'COMPATIBILITY_DIAGNOSTIC.json'; assert not destination.exists()
    oldpath = OUT.parent / 'icra_admission_revision/REJECTION_DIAGNOSTIC.json'
    old = json.loads(oldpath.read_text()); assert old['passed']
    rows = []
    for key, expected in old['inputs'].items():
        assert sha(ROOT / key) == expected
        with gzip.open(ROOT / key, 'rt') as handle:
            data = json.load(handle)
        cohort = 'development' if 'results_development' in key else 'new_recording_diagnostic'
        run = data['runs']; records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
        local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
        jp, hp, cp = natural(local[:, 4:8], unpack(local[:, 8:18]))
        ju, hu, cu = natural(local[:, 18:22], unpack(local[:, 22:32]))
        dj, dh, dc = ju - jp, hu - hp, cu - cp
        tolerance = 1e-10 * np.maximum(1., np.maximum(np.linalg.norm(ju, 2, axis=(-2, -1)), np.linalg.norm(jp, 2, axis=(-2, -1))))
        good = np.linalg.eigvalsh(dj)[:, 0] >= -tolerance
        lookup = {tuple(row[:4].astype(int)): i for i, row in enumerate(local)}
        original = records[:, 31:35].reshape(-1, 2, 2); present = original[:, :, 0] > 0
        indices = np.zeros(present.shape, int)
        for i, side in np.argwhere(present):
            source = int(records[i, 1]) if side == 0 else 3 - int(records[i, 1])
            indices[i, side] = lookup[(int(records[i, 0]), source, *original[i, side].astype(int))]
        b, beta, delta, alpha = records[:, 13:15], records[:, 28:30], records[:, 19:21], records[:, 54:56]
        active = b > 0; joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
        raw = joint[:, None] * (active - beta) * np.where(delta >= 0, records[:, 26:28], records[:, 35:37])
        j0 = np.einsum('ns,nsij->nij', alpha, ju[indices]); h0 = np.einsum('ns,nsi->ni', alpha, hu[indices]); c0 = np.einsum('ns,ns->n', alpha, cu[indices])
        logits = np.zeros_like(b); probability = np.clip(records[:, 17:19][active], 1e-9, 1 - 1e-9)
        logits[active] = np.log(probability) - np.log1p(-probability)
        alternatives = {'original': records}
        for rule, factor in factors(records, delta, active, joint).items():
            kept = raw * factor * good[indices]
            j = j0 + np.einsum('ns,nsij->nij', kept, dj[indices])
            h = h0 + np.einsum('ns,nsi->ni', kept, dh[indices]); c = c0 + np.einsum('ns,ns->n', kept, dc[indices])
            j = (j + j.swapaxes(-2, -1)) / 2
            assert np.all(np.linalg.cond(j, 1) < 1e12)
            mean, covariance, log_i = integrate(j, h, c)
            unchanged = ~(kept > 0).any(1); log_i[unchanged] = records[unchanged, 10]
            candidate = records.copy(); candidate[:, 4:6] = mean[:, :2]
            candidate[:, 9] = expit((beta * logits).sum(1) + (kept * delta).sum(1) + log_i)
            alternatives[rule] = candidate
        scores = {rule: {k: [] for k in METRICS} for rule in alternatives}
        positions = np.asarray(data['positions']); delivery = np.asarray(data['delivered']); T = len(data['time'])
        for t in range(T):
            for n in range(2):
                mask = (records[:, 0] == t + 1) & (records[:, 1] == n + 1)
                for rule, candidate in alternatives.items():
                    estimate = counterfactual(candidate[mask], 9, positions[:, :, t]) if delivery[n, 1 - n, t] else run['estimates'][n + 2 * t]
                    value = score(data['truth'][t], estimate)
                    if rule == 'original':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for k in METRICS:
                        scores[rule][k].append(value[k])
        rows += [dict(cohort=cohort, sequence=data['sequence'], condition=data['condition'], rule=rule,
                      frames=T, **{k: float(np.mean(v)) for k, v in scores[rule].items()}) for rule in scores]
        print('COMPATIBILITY SCREEN', cohort, data['sequence'], data['condition'], flush=True)
    aggregate = []
    for cohort in ['development', 'new_recording_diagnostic']:
        for rule in ['original', *RULES]:
            conditions = {}
            for condition in ['reliable', 'intermittent']:
                selected = [r for r in rows if r['cohort'] == cohort and r['rule'] == rule and r['condition'] == condition]
                conditions[condition] = {k: float(np.mean([r[k] for r in selected])) for k in METRICS}
            aggregate.append(dict(cohort=cohort, rule=rule, conditions=conditions,
                                  selection_mean_ospa=float(np.mean([r['ospa'] for r in conditions.values()]))))
    candidates = [r for r in aggregate if r['cohort'] == 'development' and r['rule'] in RULES]
    candidates.sort(key=lambda r: (r['selection_mean_ospa'], RULES.index(r['rule'])))
    advance = [r['rule'] for r in candidates[:2]]
    if 'positive_only' not in advance:
        advance.append('positive_only')
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), rows=rows, aggregate=aggregate,
                  advance_to_recursion=advance, screening_rule='Top two fixed-input dual-condition development means, plus positive_only control.',
                  original_inputs=old['inputs'], source_diagnostic_sha256=sha(oldpath),
                  code_sha256=sha(Path(__file__)), protocol_sha256=sha(OUT / 'PROTOCOL.md'),
                  current_external_candidate_scores_inspected=False)
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for row in [r for r in aggregate if r['cohort'] == 'development']:
        print('FIXED INPUT', row['rule'], {c: v['ospa'] for c, v in row['conditions'].items()}, row['selection_mean_ospa'], flush=True)
    print('ADVANCE TO RECURSION', advance, flush=True)


if __name__ == '__main__':
    main()
