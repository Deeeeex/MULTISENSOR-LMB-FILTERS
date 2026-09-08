def audit_probability(run, data):
    arm = run['arm']
    assert arm in ARMS + ['marked_er']
    local = np.asarray(run['localIncrementRecords'], float).reshape(-1, 8)
    assert len(local) and np.isfinite(local).all()
    assert np.all(local[:, 0] >= 1) and np.all(local[:, 0] <= len(data['time']))
    assert np.isin(local[:, 1], [1, 2]).all() and np.isin(local[:, 7], [0, 1]).all()
    assert np.all((local[:, 4:6] >= 0) & (local[:, 4:6] <= 1))
    assert len(np.unique(local[:, :4], axis=0)) == len(local)
    before, after = np.clip(local[:, 4], 1e-9, 1-1e-9), np.clip(local[:, 5], 1e-9, 1-1e-9)
    expected_delta = np.log(after)-np.log1p(-after)-np.log(before)+np.log1p(-before)
    assert np.allclose(local[:, 6], expected_delta, atol=2e-12, rtol=0)
    assert np.all(np.abs(local[local[:, 7] == 0, 6]) < 1e-10)
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 26)
    assert len(records) and np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, delta = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    active = b > 0
    assert np.isfinite(r[active]).all() and np.all((r[active] >= 0) & (r[active] <= 1))
    assert np.allclose(b.sum(1), 1, atol=1e-14) and np.isin(b, [0, .5, 1]).all()
    expected_factors = np.where(stamps > 0, .25+.75*np.exp(-(records[:, 0, None]-stamps)*.1/5), .25)
    assert np.all(stamps <= records[:, 0, None])
    assert np.allclose(factors[stamps > 0], expected_factors[stamps > 0], atol=2e-14, rtol=0)
    assert np.isin(factors[stamps == 0], [.25, 1]).all()
    expected_q = b*factors
    expected_q /= expected_q.sum(1, keepdims=True)
    assert np.allclose(q, expected_q, atol=2e-14, rtol=0)
    logits = np.zeros_like(b)
    rr = np.clip(r[active], 1e-9, 1-1e-9)
    logits[active] = np.log(rr)-np.log1p(-rr)
    base = (b*logits).sum(1)+records[:, 10]
    age = ((q-b)*logits).sum(1)
    joint = (active.sum(1) >= 2) & ((~active) | (r > .001)).all(1)
    recent = arm in ['marked_er', 'marked_joint_evidence_recency']
    correction = age.copy() if recent else np.zeros(len(records))
    if arm != 'marked_er':
        w = q if recent else b
        correction += joint*((active-w)*delta).sum(1)
    expected, r0, er = expit(base+correction), expit(base), expit(base+age)
    for column, value in [(6, expected), (7, r0), (8, er), (9, expected)]:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'joint scalar', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 24], correction, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23]-records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    diagnostic = dict(arm=arm, local_update_records=len(local), labels=len(records), joint_labels=int(joint.sum()),
                      positive_shift_from_er=int((correction-age > 1e-8).sum()),
                      negative_shift_from_er=int((correction-age < -1e-8).sum()))
    if arm == PRIMARY:
        delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in ['no_age', 'ER', 'candidate']}
        for t in range(len(data['time'])):
            for n in range(2):
                subset = records[(records[:, 0] == t+1) & (records[:, 1] == n+1)]
                if not delivery[n, 1-n, t]:
                    assert not len(subset)
                    continue
                for rule, column in [('no_age', 7), ('ER', 8), ('candidate', 9)]:
                    value = score(data['truth'][t], counterfactual(subset, column, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(v)) for key, v in values.items()}
                                    for rule, values in effects.items()}
    return diagnostic
