"""Reconstruct recorded Gaussian densities from means/covariances in NumPy.

Uses local prior/posterior moments rather than the producer's encoded ratios.
Absolute checks: scalar probabilities 2e-10, log integrals 1e-8,
means/covariances 1e-7. Encoded natural parameters use rtol 2e-10,
atol 1e-8 to account for subtraction of large log-density constants.
"""
import numpy as np
from scipy.special import expit

LOWER = tuple(np.asarray(v) for v in zip(*[(r, c) for c in range(4) for r in range(c, 4)]))
LOG2PI4 = 4 * np.log(2 * np.pi)


def unpack(values):
    out = np.zeros((*values.shape[:-1], 4, 4))
    out[..., LOWER[0], LOWER[1]] = values
    out[..., LOWER[1], LOWER[0]] = values
    return out


def natural(mean, covariance):
    np.linalg.cholesky(covariance)
    precision = np.linalg.inv(covariance)
    precision = (precision + precision.swapaxes(-1, -2)) / 2
    information = np.einsum('...ij,...j->...i', precision, mean)
    constant = -.5 * (LOG2PI4 + np.linalg.slogdet(covariance)[1]
                       + np.einsum('...i,...i->...', mean, information))
    return precision, information, constant


def integrate(precision, information, constant):
    np.linalg.cholesky(precision)
    covariance = np.linalg.inv(precision)
    covariance = (covariance + covariance.swapaxes(-1, -2)) / 2
    mean = np.linalg.solve(precision, information[..., None])[..., 0]
    log_integral = constant + .5 * (LOG2PI4 - np.linalg.slogdet(precision)[1]
                                   + np.einsum('...i,...i->...', information, mean))
    return mean, covariance, log_integral


def check_gaussians(run, records, original, present, active, joint, beta, logits, delta, gates, negative):
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    assert len(local) and np.isfinite(local).all()
    assert len(np.unique(local[:, :4], axis=0)) == len(local)
    prior_mean, prior_cov = local[:, 4:8], unpack(local[:, 8:18])
    post_mean, post_cov = local[:, 18:22], unpack(local[:, 22:32])
    jp, hp, cp = natural(prior_mean, prior_cov)
    ju, hu, cu = natural(post_mean, post_cov)
    dj, dh, dc = ju-jp, hu-hp, cu-cp
    encoded = np.c_[dj[:, LOWER[0], LOWER[1]], dh, dc]
    local_ids = {tuple(row[:4].astype(int)): i for i, row in enumerate(local)}
    scalar_ids = {tuple(row[:4]) for row in np.asarray(run['localIncrementRecords'], float)}
    assert set(local_ids) <= scalar_ids
    packets = np.asarray(run['packetGaussianRecords'], float).reshape(-1, 19)
    old = run['arm'] == 'marked_asymmetric'
    if old:
        assert not len(packets) and np.all(records[:, 37:] == 0)
    else:
        assert packets.shape[0] == local.shape[0]
        assert np.array_equal(packets[:, :4], local[:, :4])
        assert np.allclose(packets[:, 4:], encoded, rtol=2e-10, atol=1e-8), 'encoded ratio'
    counts = np.zeros_like(np.asarray(run['packetBytes']))
    for t, source in local[:, :2].astype(int):
        counts[source-1, t-1] += 1
    width = 232 if old else 352
    assert np.array_equal(32 + width*counts, run['packetBytes'])
    diagnostic = dict(local_gaussian_records=len(local), packet_gaussian_records=len(packets),
                      gaussian_source_joins=0, corrected_labels=0, curvature_rejected_sources=0,
                      curvature_rejected_labels=0, aggregate_fallbacks=0,
                      mean_spatial_shift_m=0., max_spatial_shift_m=0., mean_log_integral_change=0.)
    if old:
        return diagnostic
    indices = np.zeros(present.shape, dtype=int)
    for i, side in np.argwhere(present):
        source = int(records[i, 1]) if side == 0 else 3-int(records[i, 1])
        key = (int(records[i, 0]), source, *original[i, side].astype(int))
        indices[i, side] = local_ids[key]
    alpha = records[:, 54:56]
    expected_alpha = (active & present).astype(float)
    expected_alpha /= expected_alpha.sum(1, keepdims=True)
    assert np.allclose(alpha, expected_alpha, rtol=0, atol=2e-14), 'eligible spatial weights'
    j0 = np.einsum('ns,nsij->nij', alpha, ju[indices])
    h0 = np.einsum('ns,nsi->ni', alpha, hu[indices])
    c0 = np.einsum('ns,ns->n', alpha, cu[indices])
    base_mean, base_cov, base_i = integrate(j0, h0, c0)
    assert np.allclose(base_mean[:, :2], records[:, 38:40], rtol=0, atol=1e-7), 'old mean'
    assert np.allclose(base_i, records[:, 10], rtol=0, atol=1e-8), 'old integral'
    raw = joint[:, None]*(active-beta)*np.where(delta >= 0, gates, negative)
    called = (raw > 0).any(1)
    kept, allowed = raw.copy(), np.ones_like(present)
    if run['arm'] != 'marked_gaussian_evidence_no_curvature':
        tolerance = 1e-10*np.maximum(1., np.maximum(np.linalg.norm(ju, 2, axis=(-2, -1)),
                                                   np.linalg.norm(jp, 2, axis=(-2, -1))))
        good = np.linalg.eigvalsh(dj)[:, 0] >= -tolerance
        allowed = (~called[:, None]) | (~present) | good[indices]
        kept[~allowed] = 0
    precision = j0 + np.einsum('ns,nsij->nij', kept, dj[indices])
    information = h0 + np.einsum('ns,nsi->ni', kept, dh[indices])
    constant = c0 + np.einsum('ns,ns->n', kept, dc[indices])
    precision = (precision + precision.swapaxes(-1, -2))/2
    fallback = np.zeros(len(records), bool)
    for i in np.flatnonzero(called):
        try:
            np.linalg.cholesky(precision[i])
            fallback[i] = not np.isfinite(precision[i]).all() or 1/np.linalg.cond(precision[i], 1) < 1e-12
        except np.linalg.LinAlgError:
            fallback[i] = True
    kept[fallback] = 0
    precision[fallback], information[fallback], constant[fallback] = j0[fallback], h0[fallback], c0[fallback]
    mean, cov, log_i = integrate(precision, information, constant)
    unchanged = ~(kept > 0).any(1)
    log_i[unchanged] = records[unchanged, 10]
    if run['arm'] == 'marked_gaussian_evidence_guarded_scalar':
        mean, cov, log_i = base_mean, base_cov, records[:, 10].copy()
    assert np.array_equal(records[:, 57:59], allowed), 'per-source curvature admission'
    assert np.array_equal(records[:, 59], fallback), 'aggregate integrability fallback'
    assert np.allclose(records[:, 52:54], kept, rtol=0, atol=2e-14), 'admitted current exponent'
    assert np.allclose(records[:, 56], log_i, rtol=0, atol=1e-8), 'new log integral'
    saved_mean = np.c_[records[:, 4:6], records[:, 40:42]]
    saved_cov = unpack(records[:, 42:52])
    assert np.allclose(saved_mean, mean, rtol=0, atol=1e-7), 'new full mean'
    assert np.allclose(saved_cov, cov, rtol=0, atol=1e-7), 'new full covariance'
    np.linalg.cholesky(saved_cov)
    expected_r = expit((beta*logits).sum(1)+(kept*delta).sum(1)+log_i)
    for column in [6, 9]:
        assert np.allclose(records[:, column], expected_r, rtol=0, atol=2e-10), 'coherent existence'
    shift = np.linalg.norm(saved_mean[:, :2]-base_mean[:, :2], axis=1)
    diagnostic.update(gaussian_source_joins=int(present.sum()), corrected_labels=int((~unchanged).sum()),
                      curvature_rejected_sources=int(((raw > 0) & ~allowed).sum()),
                      curvature_rejected_labels=int(((raw > 0) & ~allowed).any(1).sum()),
                      aggregate_fallbacks=int(fallback.sum()), mean_spatial_shift_m=float(shift.mean()),
                      max_spatial_shift_m=float(shift.max()), mean_log_integral_change=float((log_i-records[:, 10]).mean()))
    return diagnostic
