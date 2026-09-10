"""Original No-age scalar audit with explicit final-r refinement."""
import numpy as np
from scipy.special import expit
from analyze_holdout import counterfactual, score, METRICS

def audit_existence(run, data, primary):
    arm = run['arm']
    records = np.asarray(run['iterationRecords'], dtype=float).reshape(-1, 26)
    if arm.endswith('local') or 'tc_ospa2' in arm or 'mil_support' in arm:
        assert not len(records)
        return None
    if not len(records):
        return dict(arm=arm, labels=0)
    assert np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, evidence = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    active = b > 0
    assert np.isfinite(r[active]).all()
    assert np.all((r[active] >= 0) & (r[active] <= 1))
    assert np.allclose(b.sum(1), 1, atol=1e-14)
    assert np.isin(b, [0, .5, 1]).all()
    expected_factors = np.where(stamps > 0, .25 + .75 * np.exp(-(records[:, 0, None] - stamps) * .1 / 5), .25)
    assert np.all(stamps <= records[:, 0, None])
    # A missing label has no object stamp and retains the neutral factor 1;
    # a represented object without direct history uses .25. The compact
    # record has no separate represented-label mask, so distinguish only
    # the two permitted zero-stamp cases, not an invented age for absence.
    assert np.allclose(factors[stamps > 0], expected_factors[stamps > 0], rtol=0, atol=2e-14)
    assert np.isin(factors[stamps == 0], [.25, 1]).all()
    expected_q = b * factors
    expected_q /= expected_q.sum(1, keepdims=True)
    assert np.allclose(q, expected_q, rtol=0, atol=2e-14)
    logits = np.zeros_like(b)
    rr = np.clip(r[active], 1e-9, 1 - 1e-9)
    logits[active] = np.log(rr) - np.log1p(-rr)
    base = (b * logits).sum(1) + records[:, 10]
    age = ((q - b) * logits).sum(1)
    assert np.all((evidence >= 0) & (evidence <= 1))
    assert np.all(evidence[stamps != records[:, 0, None]] == 0)
    eligible = active & (q > b + 1e-12) & (r >= .5)
    cap = np.where(eligible, evidence, 0).max(1)
    r0, rER = expit(base), expit(base + age)
    if arm.endswith('lineage'):
        expected = r0
    elif arm in ['qualified_exist', 'marked_er']:
        expected = rER
    else:
        expected = np.minimum(rER, np.maximum(r0, cap))
    from event_audit import check_censor_event
    expected = check_censor_event(run, records, expected, b, logits, np.zeros_like(b), np.zeros_like(b), records[:, 10])
    for column, value in [(6, expected), (7, r0), (8, rER), (9, expected)]:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'existence equation', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23] - records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    if not arm.endswith('lineage'):
        assert np.all(expected >= np.minimum(r0, rER) - 2e-12)
        assert np.all(expected <= rER + 2e-12)
        assert np.allclose(expected[age < 0], rER[age < 0], atol=2e-12, rtol=0)
    diagnostic = dict(arm=arm, labels=len(records),
                      positive_age_labels=int((age > 1e-8).sum()),
                      support_above_noage=int((cap > r0 + 1e-8).sum()),
                      constrained_below_er=int((expected < rER - 1e-8).sum()),
                      preserved_above_cr=int((expected > np.minimum(r0, rER) + 1e-8).sum()))
    if arm == primary:
        delivery = np.asarray(data['delivered'])
        poses = np.asarray(data['positions'])
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in ['no_age', 'ER', 'candidate']}
        for t in range(len(data['time'])):
            for n in range(2):
                subset = records[(records[:, 0] == t + 1) & (records[:, 1] == n + 1)]
                if not delivery[n, 1 - n, t]:
                    assert not len(subset)
                    continue
                for rule, column in [('no_age', 7), ('ER', 8), ('candidate', 9)]:
                    value = score(data['truth'][t], counterfactual(subset, column, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(values)) for key, values in metrics.items()}
                                    for rule, metrics in effects.items()}
    return diagnostic
