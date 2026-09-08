"""Measure missing-label censor effects on fixed complete saved inputs."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_fusion_holdout'))
from analyze_holdout import counterfactual, score, interval
sys.path.insert(0, str(OUT.parent / 'icra_marked_control'))
from analyze_full import baseline

ARMS = ['marked_lineage', 'marked_er']
RULES = ['retain', 'omit_self', 'omit_peer', 'omit_both']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous = OUT.parent / 'icra_tempered_iteration'
    sources = json.loads((previous / 'source_sha256.json').read_text())
    registration = json.loads((previous / 'ROUND_FREEZE.json').read_text())
    for name, expected in {**sources, **registration['source_and_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    units = [u for u in registration['units']]
    rows, inputs, audited = [], {}, 0
    for unit in units:
        cohort, name = unit['cohort'], f"{unit['sequence']:04d}"
        for condition in ['reliable', 'intermittent']:
            for arm in ARMS:
                data, baseline_row, _, path = baseline('development' if cohort == 'development' else 'holdout', name, condition, arm)
                inputs[str(path.relative_to(ROOT))] = sha(path)
                run = data['runs'] if isinstance(data['runs'], dict) else next(r for r in data['runs'] if r['arm'] == arm)
                records = np.asarray(run['iterationRecords'], float).reshape(-1, 26)
                T = len(data['time'])
                delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
                values = {rule: {key: [] for key in METRICS} for rule in RULES}
                counts = dict(label_events=len(records), self_censor_events=0, peer_censor_events=0,
                              node_frames_with_censor=0, maximum_abs_single_source_log_eta=0)
                for t in range(T):
                    for n in range(2):
                        part = records[(records[:, 0] == t + 1) & (records[:, 1] == n + 1)]
                        if not delivery[n, 1 - n, t]:
                            assert not len(part)
                            value = score(data['truth'][t], run['estimates'][n + 2 * t])
                            for rule in RULES:
                                for key in METRICS:
                                    values[rule][key].append(value[key])
                            audited += 1
                            continue
                        probabilities, weights = part[:, 17:19], part[:, 13:15]
                        stamps, factors = part[:, 21:23], part[:, 15:17]
                        missing = (weights > 0) & (stamps == 0) & (factors == 1) & (probabilities == .001)
                        any_missing = missing.any(1)
                        assert np.all(missing.sum(1) <= 1)
                        if any_missing.any():
                            counts['maximum_abs_single_source_log_eta'] = max(
                                counts['maximum_abs_single_source_log_eta'], float(np.abs(part[any_missing, 10]).max()))
                        counts['self_censor_events'] += int(missing[:, 0].sum())
                        counts['peer_censor_events'] += int(missing[:, 1].sum())
                        counts['node_frames_with_censor'] += int(any_missing.any())
                        modified = {rule: part.copy() for rule in RULES}
                        for rule in RULES:
                            modified[rule][:, 9] = part[:, 6]
                        for rule, which in [('omit_self', [0]), ('omit_peer', [1]), ('omit_both', [0, 1])]:
                            for side in which:
                                mask = missing[:, side]
                                assert np.all(weights[mask, 1 - side] > 0)
                                p = np.clip(probabilities[mask, 1 - side], 1e-9, 1 - 1e-9)
                                modified[rule][mask, 9] = expit(np.log(p) - np.log1p(-p) + part[mask, 10])
                        for rule in RULES:
                            value = score(data['truth'][t], counterfactual(modified[rule], 9, poses[:, :, t]))
                            if rule == 'retain':
                                assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                            for key in METRICS:
                                values[rule][key].append(value[key])
                        audited += 1
                for rule in RULES:
                    metrics = {key: float(np.mean(v)) for key, v in values[rule].items()}
                    if rule == 'retain':
                        for key in METRICS:
                            assert np.isclose(metrics[key], baseline_row[key], atol=1e-10, rtol=0)
                    rows.append(dict(cohort=cohort, sequence=name, condition=condition, arm=arm,
                                     rule=rule, frames=T, **metrics, **counts))
        print('CENSOR DIAGNOSTIC COMPLETE', cohort, name, flush=True)
    assert audited == 60752
    result = dict(protocol='fixed-input-missing-label-diagnostic-v1', all_outcomes_seen=True,
                  no_tracking_replay=True, no_parameter_fitting=True, same_input_only=True,
                  verified_original_node_frames=audited, scored_rule_node_frames=len(RULES) * audited,
                  source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'PROTOCOL.md',
                                                                         previous / 'source_sha256.json', previous / 'ROUND_FREEZE.json']},
                  input_sha256=inputs, runs=rows, cohorts={})
    for cohort in ['development', 'seen_transfer']:
        names = [f"{u['sequence']:04d}" for u in units if u['cohort'] == cohort]
        lookup = {(r['sequence'], r['condition'], r['arm'], r['rule']): r for r in rows if r['cohort'] == cohort}
        samples = np.random.default_rng(8301).integers(0, len(names), (10000, len(names)))
        aggregate, paired = [], []
        for condition in ['reliable', 'intermittent']:
            for arm in ARMS:
                for rule in RULES:
                    group = [lookup[name, condition, arm, rule] for name in names]
                    aggregate.append(dict(condition=condition, arm=arm, rule=rule,
                                          **{key: interval([r[key] for r in group], samples) for key in METRICS}))
                    if rule != 'retain':
                        original = [lookup[name, condition, arm, 'retain'] for name in names]
                        comparison = dict(condition=condition, arm=arm, rule=rule,
                                          **{key: interval([r[key] - b[key] for r, b in zip(group, original)], samples) for key in METRICS},
                                          ospa_wins=sum(r['ospa'] < b['ospa'] - 1e-10 for r, b in zip(group, original)))
                        paired.append(comparison)
                        if rule == 'omit_both':
                            print('REMOVE BOTH CENSORS', cohort, condition, arm, comparison['ospa'],
                                  'miss', comparison['miss2']['mean'], 'false', comparison['false2']['mean'], flush=True)
        result['cohorts'][cohort] = dict(sequences=len(names), aggregate=aggregate, paired=paired)
    (OUT / 'summary_absence.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('ALL CENSOR COUNTERFACTUALS COMPLETE:', audited, 'original and', len(RULES) * audited, 'rule node-frames.', flush=True)


if __name__ == '__main__':
    main()
