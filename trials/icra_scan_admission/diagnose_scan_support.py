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

from scan_support import support_from_records
RULES = ['negative_scan_r', 'negative_scan_r2']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()




def main():
    destination = OUT / 'SCAN_DIAGNOSTIC.json'; assert not destination.exists()
    oldpath = OUT.parent / 'icra_admission_revision/REJECTION_DIAGNOSTIC.json'
    old = json.loads(oldpath.read_text()); assert old['passed']
    rows = []; scan_rates = []
    files = {k:v for k,v in old['inputs'].items() if 'results_development' in k}
    external_path = OUT.parent / 'icra_admission_final/EXTERNAL_ANALYSIS.json'
    external = json.loads(external_path.read_text()); assert external['passed']
    files.update({k:v for k,v in external['inputs'].items() if k.endswith('_marked_gaussian_evidence.json.gz') and '/final_v2x_evaluation/' in k})
    assert len(files) == 28
    for key, expected in files.items():
        assert sha(ROOT / key) == expected
        with gzip.open(ROOT / key, 'rt') as handle:
            data = json.load(handle)
        cohort = 'development' if 'results_development' in key else 'v2x_development'
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
        local_increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
        increment_lookup = {tuple(row[:4].astype(int)): i for i, row in enumerate(local_increments)}
        increment_indices = np.zeros(present.shape, int)
        for i, side in np.argwhere(present):
            source = int(records[i, 1]) if side == 0 else 3 - int(records[i, 1])
            increment_indices[i, side] = increment_lookup[(int(records[i, 0]), source, *original[i, side].astype(int))]
        for rule, exponent in zip(RULES, [1, 2]):
            negative, rates = support_from_records(local_increments, exponent)
            scan_rates += [dict(cohort=cohort, sequence=data['sequence'], condition=data['condition'], rule=rule, **r) for r in rates]
            source_support = negative[increment_indices] * present
            old_negative = records[:, 35:37]
            scale = np.divide(source_support, old_negative, out=np.ones_like(source_support), where=old_negative > 0)
            assert np.all((scale >= 0) & (scale <= 1 + 1e-12))
            factor = np.where(delta < 0, scale, 1)
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
        print('SCAN SUPPORT SCREEN', cohort, data['sequence'], data['condition'], flush=True)
    aggregate = []
    for cohort in ['development', 'v2x_development']:
        for rule in ['original', *RULES]:
            conditions = {}
            for condition in ['reliable', 'intermittent']:
                selected = [r for r in rows if r['cohort'] == cohort and r['rule'] == rule and r['condition'] == condition]
                conditions[condition] = {k: float(np.mean([r[k] for r in selected])) for k in METRICS}
            aggregate.append(dict(cohort=cohort, rule=rule, conditions=conditions,
                                  selection_mean_ospa=float(np.mean([r['ospa'] for r in conditions.values()]))))
    candidates = [r for r in aggregate if r['cohort'] == 'development' and r['rule'] in RULES]
    candidates.sort(key=lambda r: (r['selection_mean_ospa'], RULES.index(r['rule'])))
    originals = {r['cohort']: r['selection_mean_ospa'] for r in aggregate if r['rule'] == 'original'}
    eligible = [r for r in candidates if r['selection_mean_ospa'] < originals['development'] and
                next(x['selection_mean_ospa'] for x in aggregate if x['cohort'] == 'v2x_development' and x['rule'] == r['rule']) < originals['v2x_development']]
    advance = [eligible[0]['rule']] if eligible else []
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), rows=rows, aggregate=aggregate,
                  advance_to_recursion=advance, screening_rule='Strict fixed-input mean improvement in both development datasets; lowest original-nine mean wins, then exponent one.',
                  original_inputs=files, source_diagnostic_sha256=sha(oldpath), external_report_sha256=sha(external_path),
                  code_sha256=sha(Path(__file__)), support_code_sha256=sha(OUT / 'scan_support.py'),
                  protocol_sha256=sha(OUT / 'PROTOCOL.md'), scan_rates=scan_rates,
                  current_external_candidate_scores_inspected=True)
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for row in aggregate:
        print('FIXED INPUT', row['cohort'], row['rule'], {c: v['ospa'] for c, v in row['conditions'].items()}, row['selection_mean_ospa'], flush=True)
    print('ADVANCE TO RECURSION', advance, flush=True)


if __name__ == '__main__':
    main()
