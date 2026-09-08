def audit_probability(run, data):
    arm = run['arm']
    assert arm in ARMS + ['marked_conservative']
    local = np.asarray(run['localIncrementRecords'], float).reshape(-1, 10)
    assert len(local) and np.isfinite(local).all()
    assert np.all(local[:, 0] >= 1) and np.all(local[:, 0] <= len(data['time']))
    assert np.isin(local[:, 1], [1, 2]).all() and np.isin(local[:, 7], [0, 1]).all()
    assert np.all((local[:, 4:6] >= 0) & (local[:, 4:6] <= 1))
    assert len(np.unique(local[:, :4], axis=0)) == len(local)
    before, after = np.clip(local[:, 4], 1e-9, 1-1e-9), np.clip(local[:, 5], 1e-9, 1-1e-9)
    expected_delta = np.log(after)-np.log1p(-after)-np.log(before)+np.log1p(-before)
    assert np.allclose(local[:, 6], expected_delta, atol=2e-12, rtol=0)
    assert np.all(np.abs(local[local[:, 7] == 0, 6]) < 1e-10)
    assert np.all((local[:, 8:10] >= 0) & (local[:, 8:10] <= 1))
    assert np.all(local[:, 8] <= local[:, 9]+1e-14)
    assert np.all(local[local[:, 7] == 0, 8:10] == 0)
    if arm == 'marked_selective_no_mark':
        assert np.array_equal(local[:, 8], local[:, 9])
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 35)
    assert len(records) and np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, delta = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    gates, beta = records[:, 26:28], records[:, 28:30]
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
    joined = 0
    if arm == 'marked_conservative':
        assert np.all(records[:, 26:] == 0) and np.all(delta == 0)
        correction = np.minimum(age, 0)
        joint = np.zeros(len(records), bool)
        boost = np.zeros(len(records))
    else:
        assert np.all((gates >= 0) & (gates <= 1))
        assert np.all(gates[stamps != records[:, 0, None]] == 0)
        original = records[:, 31:35].reshape(-1, 2, 2)
        present = original[:, :, 0] > 0
        assert np.all(original[~present] == 0) and np.all(original[present] > 0)
        assert np.all(delta[~present] == 0) and np.all(gates[~present] == 0)
        local_lookup = {tuple(row[:4].astype(int)): row for row in local}
        for i, side in np.argwhere(present):
            source = int(records[i, 1]) if side == 0 else 3-int(records[i, 1])
            key = (int(records[i, 0]), source, *original[i, side].astype(int))
            row = local_lookup[key]
            assert np.isclose(delta[i, side], row[6], atol=2e-12, rtol=0), (arm, key, 'received delta')
            expected_gate = row[8] if stamps[i, side] == records[i, 0] else 0
            assert gates[i, side] == expected_gate, (arm, key, 'received gate')
            if active[i, side]:
                assert np.isclose(r[i, side], row[5], atol=1e-9, rtol=0), (arm, key, 'received existence')
            joined += 1
        joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
        expected_beta = b.copy()
        if arm != 'marked_selective_no_history':
            expected_beta[age < -1e-12] = q[age < -1e-12]
        assert np.allclose(beta, expected_beta, atol=2e-14, rtol=0)
        fresh = delta if arm == 'marked_selective_signed' else np.maximum(delta, 0)
        boost = joint*((active-beta)*gates*fresh).sum(1)
        assert np.allclose(records[:, 30], boost, atol=1e-10, rtol=0)
        correction = ((beta-b)*logits).sum(1)+boost
    expected, r0, er = expit(base+correction), expit(base), expit(base+age)
    for column, value in [(6, expected), (7, r0), (8, er), (9, expected)]:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'selective scalar', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 24], correction, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23]-records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    if arm in [PRIMARY, 'marked_selective_no_mark']:
        assert np.all(expected >= np.minimum(r0, er)-2e-12)
    diagnostic = dict(arm=arm, local_update_records=len(local), labels=len(records), joined_source_records=joined,
                      joint_labels=int(joint.sum()), boosted_labels=int((boost > 1e-8).sum()),
                      negative_boost_labels=int((boost < -1e-8).sum()),
                      mean_gate=float(gates[active].mean()), mean_boost=float(boost.mean()))
    if arm == PRIMARY:
        delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in ['no_age', 'ER', 'CR', 'candidate']}
        conservative_records = records.copy()
        conservative_records[:, 9] = np.minimum(r0, er)
        for t in range(len(data['time'])):
            for n in range(2):
                mask = (records[:, 0] == t+1) & (records[:, 1] == n+1)
                subset = records[mask]
                if not delivery[n, 1-n, t]:
                    assert not len(subset)
                    continue
                for rule, column in [('no_age', 7), ('ER', 8), ('CR', 9), ('candidate', 9)]:
                    selected = conservative_records[mask] if rule == 'CR' else subset
                    value = score(data['truth'][t], counterfactual(selected, column, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(v)) for key, v in values.items()}
                                    for rule, values in effects.items()}
    return diagnostic
