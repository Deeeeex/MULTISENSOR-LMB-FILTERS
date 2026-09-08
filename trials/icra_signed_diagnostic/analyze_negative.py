"""Diagnose all current negative-evidence rules on two complete saved histories."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PREVIOUS = OUT.parent / 'icra_selective_innovation'
sys.path.insert(0, str(PREVIOUS))
from analyze_selective import counterfactual, read, score, score_run, interval

BACKENDS = ['marked_selective', 'marked_selective_signed']
RULES = ['retain', 'positive', 'mark_signed', 'negative_all', 'negative_miss']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (OUT / 'summary_negative.json').exists(), 'Do not overwrite a completed diagnostic.'
    previous = json.loads((PREVIOUS / 'summary_development.json').read_text())
    assert previous['sequences'] == 9 and not previous['continuation_gate_passed']
    source = json.loads((PREVIOUS / 'source_sha256.json').read_text())
    frozen = json.loads((PREVIOUS / 'ROUND_FREEZE.json').read_text())
    inputs = {**source, **frozen['source_and_evidence_sha256'],
              str((PREVIOUS / 'summary_development.json').relative_to(ROOT)): sha(PREVIOUS / 'summary_development.json'),
              str((OUT / 'PROTOCOL.md').relative_to(ROOT)): sha(OUT / 'PROTOCOL.md'),
              str(Path(__file__).relative_to(ROOT)): sha(Path(__file__))}
    for name, expected in inputs.items():
        assert sha(ROOT / name) == expected, name
    runs, count_original, count_rules = [], 0, 0
    for sequence in range(9):
        name = f'{sequence:04d}'
        for condition in ['reliable', 'intermittent']:
            for backend in BACKENDS:
                path = PREVIOUS / 'results_development' / f'{name}_{condition}_{backend}.json.gz'
                assert sha(path) == previous['inputs'][str(path.relative_to(ROOT))]
                inputs[str(path.relative_to(ROOT))] = sha(path)
                data = read(path)
                original = data['runs']
                baseline, _ = score_run(data, original)
                expected = next(r for r in previous['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == backend)
                for key in METRICS:
                    assert np.isclose(baseline[key], expected[key], atol=1e-10, rtol=0)
                records = np.asarray(original['iterationRecords'], float).reshape(-1, 35)
                local = np.asarray(original['localIncrementRecords'], float).reshape(-1, 10)
                lookup = {tuple(row[:4].astype(int)): row for row in local}
                b, q = records[:, 13:15], records[:, 11:13]
                r, delta, gates = records[:, 17:19], records[:, 19:21], records[:, 26:28]
                active = b > 0
                logits = np.zeros_like(b)
                rr = np.clip(r[active], 1e-9, 1-1e-9)
                logits[active] = np.log(rr)-np.log1p(-rr)
                age = ((q-b)*logits).sum(1)
                beta = np.where((age < -1e-12)[:, None], q, b)
                assert np.allclose(beta, records[:, 28:30], atol=2e-14, rtol=0)
                base = (beta*logits).sum(1)+records[:, 10]
                ids = records[:, 31:35].reshape(-1, 2, 2)
                present = ids[:, :, 0] > 0
                current, mass = np.zeros_like(b), np.zeros_like(b)
                for row, side in np.argwhere(present):
                    source_id = int(records[row, 1]) if side == 0 else 3-int(records[row, 1])
                    entry = lookup[(int(records[row, 0]), source_id, *ids[row, side].astype(int))]
                    assert delta[row, side] == entry[6]
                    current[row, side], mass[row, side] = entry[7], entry[9]
                joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
                coefficient = joint[:, None]*(active-beta)
                positive = (coefficient*gates*np.maximum(delta, 0)).sum(1)
                signed = (coefficient*gates*delta).sum(1)
                negative = coefficient*current*np.minimum(delta, 0)
                outputs = {'retain': records[:, 6], 'positive': expit(base+positive),
                           'mark_signed': expit(base+signed),
                           'negative_all': expit(base+positive+negative.sum(1)),
                           'negative_miss': expit(base+positive+(negative*(1-mass)*.9/(2-.9)).sum(1))}
                original_rule = 'positive' if backend == 'marked_selective' else 'mark_signed'
                assert np.allclose(outputs[original_rule], outputs['retain'], atol=2e-12, rtol=0)
                T, poses = len(data['time']), np.asarray(data['positions'])
                values = {rule: {key: [] for key in METRICS} for rule in RULES}
                for t in range(T):
                    for node in range(2):
                        mask = (records[:, 0] == t+1) & (records[:, 1] == node+1)
                        for rule in RULES:
                            subset = records[mask].copy()
                            if data['delivered'][node][1-node][t]:
                                subset[:, 9] = outputs[rule][mask]
                                estimates = counterfactual(subset, 9, poses[:, :, t])
                            else:
                                assert not len(subset)
                                estimates = original['estimates'][node+2*t]
                            metric = score(data['truth'][t], estimates)
                            if rule in ['retain', original_rule]:
                                assert np.isclose(metric['ospa'], original['ospa'][node][t], atol=1e-8, rtol=1e-9)
                            for key in METRICS:
                                values[rule][key].append(metric[key])
                            count_rules += 1
                        count_original += 1
                for rule in RULES:
                    runs.append(dict(sequence=name, condition=condition, backend=backend, rule=rule, frames=T,
                                     **{key: float(np.mean(v)) for key, v in values[rule].items()}))
        print('NEGATIVE DIAGNOSTIC COMPLETE', name, flush=True)
    samples = np.random.default_rng(8301).integers(0, 9, (10000, 9))
    lookup = {(r['sequence'], r['condition'], r['backend'], r['rule']): r for r in runs}
    aggregate, paired = [], []
    for condition in ['reliable', 'intermittent']:
        for backend in BACKENDS:
            for rule in RULES:
                a = [lookup[f'{s:04d}', condition, backend, rule] for s in range(9)]
                b = [lookup[f'{s:04d}', condition, backend, 'retain'] for s in range(9)]
                aggregate.append(dict(condition=condition, backend=backend, rule=rule,
                    **{key: interval([r[key] for r in a], samples) for key in METRICS}))
                if rule != 'retain':
                    paired.append(dict(condition=condition, backend=backend, rule=rule,
                        **{key: interval([x[key]-y[key] for x,y in zip(a,b)], samples) for key in METRICS}))
    result = dict(fixed_input_only=True, all_development_sequences=9, tracker_rerun=False,
                  original_node_frames=count_original, rule_node_frames=count_rules,
                  aggregate=aggregate, paired=paired, runs=runs, input_sha256=inputs)
    assert count_original == 15944 and count_rules == 79720
    (OUT / 'summary_negative.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for r in paired:
        print(r['condition'], r['backend'], r['rule'], r['ospa'], 'miss', r['miss2']['mean'], 'false', r['false2']['mean'], flush=True)
    print('NEGATIVE FIXED-INPUT DIAGNOSTIC PASSED', count_original, count_rules, flush=True)


if __name__ == '__main__':
    main()
