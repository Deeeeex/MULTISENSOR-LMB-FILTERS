"""Reconstruct the declared one-step distributions from local moments."""
from pathlib import Path
from itertools import product
import sys
import numpy as np
from scipy.special import expit

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'icra_reviewer_revision'))
from review_gaussian_audit import natural, integrate, unpack, LOWER, check_gaussians

RULES = ['original', 'joint_mark', 'conditional', 'joint']


def enumerate_fixture(prior, pd, likelihood):
    """Exact mutually exclusive assignment enumeration, including nonexistence."""
    prior = np.asarray(prior, float); pd = np.asarray(pd, float)
    likelihood = np.asarray(likelihood, float)
    n, m = likelihood.shape
    assert n == len(prior) == len(pd)
    states = []; weights = []
    for assignment in product(range(-1, m + 1), repeat=n):
        detections = [x for x in assignment if x > 0]
        if len(set(detections)) != len(detections): continue
        weight = 1.
        for j, x in enumerate(assignment):
            weight *= (1 - prior[j] if x == -1 else prior[j] * (1 - pd[j]) if x == 0
                       else prior[j] * pd[j] * likelihood[j, x - 1])
        states.append(assignment); weights.append(weight)
    states = np.asarray(states); weights = np.asarray(weights); weights /= weights.sum()
    existence = np.sum(weights[:, None] * (states >= 0), axis=0)
    joint = np.sum(weights[:, None] * (states > 0), axis=0)
    W = np.zeros((n, m + 1))
    for j in range(n):
        for d in range(m + 1):
            W[j, d] = weights[states[:, j] == d].sum() / existence[j]
    assert np.allclose(existence * W[:, 1:].sum(1), joint, atol=1e-14, rtol=0)
    assert np.all((joint >= 0) & (joint <= existence))
    assert np.all(np.sum(existence[:, None] * W[:, 1:], axis=0) <= 1 + 1e-14)
    return existence, W, joint


def check_fixtures():
    cases = [([.01], [.9], [[.01]]), ([.01], [.9], [[1e6]]),
             ([.8, .2], [.9, .9], [[10], [10]]),
             ([.05, .05, .05], [.9] * 3, [[1e4]] * 3),
             ([.7, .4], [.9, 0], [[3, 8], [2, 5]]),
             ([.7, .4], [.9, .9], np.empty((2, 0)))]
    summaries = []
    for prior, pd, likelihood in cases:
        r, W, joint = enumerate_fixture(prior, pd, likelihood)
        summaries.append(dict(posterior=r.tolist(), conditional=W[:, 1:].sum(1).tolist(),
                              joint=joint.tolist()))
    assert summaries[3]['conditional'][0] > .98
    assert summaries[3]['joint'][0] < .34
    return summaries


def source_state(run):
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
    increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    assert len(records) and len(local) and np.isfinite(increments).all()
    original = records[:, 31:35].reshape(-1, 2, 2).astype(int)
    present = original[:, :, 0] > 0
    indices = np.zeros(present.shape, int); inc_indices = np.zeros(present.shape, int)
    lookup = {tuple(r[:4].astype(int)): i for i, r in enumerate(local)}
    inc_lookup = {tuple(r[:4].astype(int)): i for i, r in enumerate(increments)}
    assert len(lookup) == len(local) and len(inc_lookup) == len(increments)
    for i, s in np.argwhere(present):
        t, n = map(int, records[i, :2]); key = (t, n if s == 0 else 3 - n, *original[i, s])
        indices[i, s] = lookup[key]; inc_indices[i, s] = inc_lookup[key]
    b = records[:, 13:15]; active = b > 0
    joint = (active.sum(1) >= 2) & ((~active) | present).all(1)
    beta = records[:, 28:30]; delta = records[:, 19:21]; gates = records[:, 26:28]
    negative = records[:, 35:37]; current = present & (records[:, 21:23] == records[:, 0, None])
    received = increments[inc_indices]
    assert np.allclose(delta[present], received[:, :, 6][present], atol=2e-12, rtol=0)
    assert np.array_equal(gates, np.where(current, received[:, :, 8], 0))
    assert np.array_equal(negative, np.where(current, received[:, :, 11], 0))
    assert np.allclose(records[:, 17:19][active & present], received[:, :, 5][active & present], atol=1e-9, rtol=0)
    a = np.where(current, received[:, :, 9], 0)
    rplus = np.where(present, received[:, :, 5], 0)
    assert np.all((a >= 0) & (a <= 1)) and np.all((rplus >= 0) & (rplus <= 1))
    assert np.all((increments[:, 10] == 0) | (increments[:, 10] == .9))
    assert np.array_equal(increments[:, 7].astype(bool), increments[:, 10] > 0)
    assert np.allclose(increments[:, 11], (1-increments[:, 9])*increments[:, 10]/(2-increments[:, 10]), atol=1e-14, rtol=0)
    logits = np.zeros_like(b); r = np.clip(records[:, 17:19][active], 1e-9, 1-1e-9)
    logits[active] = np.log(r) - np.log1p(-r)
    original_audit = check_gaussians(run, records, original, present, active, joint, beta,
                                     logits, delta, gates, negative)
    jp, hp, cp = natural(local[:, 4:8], unpack(local[:, 8:18]))
    ju, hu, cu = natural(local[:, 18:22], unpack(local[:, 22:32]))
    dj, dh, dc = ju-jp, hu-hp, cu-cp
    tol = 1e-10*np.maximum(1., np.maximum(np.linalg.norm(ju, 2, axis=(-2,-1)), np.linalg.norm(jp, 2, axis=(-2,-1))))
    good = np.linalg.eigvalsh(dj)[:, 0] >= -tol
    alpha = records[:, 54:56]
    j0 = np.einsum('ns,nsij->nij', alpha, ju[indices])
    h0 = np.einsum('ns,nsi->ni', alpha, hu[indices])
    c0 = np.einsum('ns,ns->n', alpha, cu[indices])
    m0, p0, log0 = integrate(j0, h0, c0)
    assert np.allclose(m0[:, :2], records[:, 38:40], atol=1e-7, rtol=0)
    assert np.allclose(log0, records[:, 10], atol=1e-8, rtol=0)
    return dict(records=records, increments=increments, local=local, indices=indices,
                present=present, active=active, joint=joint, beta=beta, delta=delta,
                gates=gates, negative=negative, a=a, rplus=rplus, logits=logits,
                good=good, dj=dj, dh=dh, dc=dc, j0=j0, h0=h0, c0=c0,
                m0=m0, p0=p0, original_audit=original_audit)


def calculate(state, rule, scalar=False):
    s = state; rec = s['records']; idx = s['indices']
    positive = {'original': s['gates'], 'joint_mark': s['rplus']*s['gates'],
                'conditional': s['a'], 'joint': s['rplus']*s['a']}[rule]
    raw = s['joint'][:, None]*(s['active']-s['beta'])*np.where(s['delta'] >= 0, positive, s['negative'])
    called = (raw > 0).any(1)
    allowed = (~called[:, None]) | (~s['present']) | s['good'][idx]
    kept = raw*allowed
    J = s['j0'] + np.einsum('ns,nsij->nij', kept, s['dj'][idx])
    H = s['h0'] + np.einsum('ns,nsi->ni', kept, s['dh'][idx])
    C = s['c0'] + np.einsum('ns,ns->n', kept, s['dc'][idx])
    J = (J + J.swapaxes(-1,-2))/2
    fallback = np.zeros(len(rec), bool)
    for i in np.flatnonzero(called):
        try:
            np.linalg.cholesky(J[i])
            fallback[i] = (not np.isfinite(J[i]).all() or not np.isfinite(H[i]).all()
                           or not np.isfinite(C[i]) or 1/np.linalg.cond(J[i], 1) < 1e-12)
        except np.linalg.LinAlgError: fallback[i] = True
    kept[fallback] = 0
    J[fallback], H[fallback], C[fallback] = s['j0'][fallback], s['h0'][fallback], s['c0'][fallback]
    mean, cov, log_i = integrate(J, H, C)
    log_i[~(kept > 0).any(1)] = rec[~(kept > 0).any(1), 10]
    if scalar: mean, cov, log_i = s['m0'], s['p0'], rec[:, 10].copy()
    r = expit((s['beta']*s['logits']).sum(1) + (kept*s['delta']).sum(1) + log_i)
    if rule == 'original':
        assert np.allclose(r, rec[:, 9], atol=2e-10, rtol=0)
        assert np.allclose(mean, np.c_[rec[:,4:6], rec[:,40:42]], atol=1e-7, rtol=0)
        assert np.allclose(cov, unpack(rec[:,42:52]), atol=1e-7, rtol=0)
        assert np.allclose(kept, rec[:,52:54], atol=2e-14, rtol=0)
        assert np.array_equal(allowed, rec[:,57:59]) and np.array_equal(fallback, rec[:,59])
    candidate = rec.copy()
    if rule != 'original':
        candidate[:, 9] = r; candidate[:, 4:6] = mean[:, :2]
        candidate[:, 40:42] = mean[:, 2:]; candidate[:,42:52] = cov[:, LOWER[0], LOWER[1]]
    values = dict(r=r, mean=mean, covariance=cov, log_integral=log_i, kept=kept,
                  allowed=allowed, fallback=fallback, positive=positive)
    diagnostics = dict(fusion_labels=len(rec), corrected_labels=int((kept > 0).any(1).sum()),
        rejected_sources=int(((raw > 0) & ~allowed).sum()), aggregate_fallbacks=int(fallback.sum()),
        admitted_positive_sources=int(((kept > 0) & (s['delta'] >= 0)).sum()),
        positive_gate_changed=int((np.abs(positive-s['gates']) > 1e-14).sum()),
        mean_absolute_existence_change=float(np.mean(np.abs(r-rec[:,9]))),
        mean_positive_gate=float(positive[s['active']].mean()))
    return candidate, values, diagnostics


def audit_raw_weights(data, state, ratios):
    run = data['runs']; local = state['increments']
    if 'localAssociationWeights' not in run:
        return dict(available=False, reason='Older source run has only audited association masses; raw W was not recorded')
    groups = {}
    for i, row in enumerate(local): groups.setdefault(tuple(row[:2].astype(int)), []).append(i)
    checked = 0; values = 0; maximum_column_sum = 0.
    for (t, n), idx in groups.items():
        rows = local[idx]; raw = np.asarray(run['localAssociationWeights'][n-1+2*(t-1)], float)
        marks = np.maximum(0, np.tanh(.5*np.log(np.asarray(ratios[n-1,t-1], float).ravel())))
        if not raw.size:
            assert not rows[:,8:10].any(); continue
        W = raw.reshape(len(rows), len(marks)+1)
        assert np.isfinite(W).all() and (W >= 0).all()
        assert np.allclose(W.sum(1), 1, atol=1e-12, rtol=0)
        W = W/W.sum(1, keepdims=True)
        opportunity = rows[:,7].astype(bool)
        a = np.clip(W[:,1:].sum(1), 0, 1)*opportunity
        old_mark = np.clip(W[:,1:] @ marks, 0, 1)*opportunity
        assert np.allclose(a, rows[:,9], atol=2e-14, rtol=0)
        assert np.allclose(old_mark, rows[:,8], atol=2e-14, rtol=0)
        joint = rows[:,5,None]*W[:,1:]
        assert np.allclose(joint.sum(1)*opportunity, rows[:,5]*rows[:,9], atol=2e-14, rtol=0)
        if joint.size: maximum_column_sum = max(maximum_column_sum, float(joint.sum(0).max()))
        checked += len(rows); values += W.size
    return dict(available=True, local_rows_with_W=checked, values=values,
                maximum_joint_detection_column_sum=maximum_column_sum,
                note='LBP column residual is reported without clipping or an exact-assignment claim')
