"""Independent Gaussian marginal ratios from local prior/posterior moments."""
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'icra_reviewer_revision'))
from review_gaussian_audit import natural, integrate, unpack


def project(prior_mean, prior_cov, post_mean, post_cov):
    jp, hp, cp = natural(prior_mean, prior_cov)
    ju, hu, cu = natural(post_mean, post_cov)
    dj, dh, dc = ju - jp, hu - hp, cu - cp
    tolerance = 1e-10 * np.maximum(1., np.maximum(np.linalg.norm(ju, 2, axis=(-2, -1)),
                                                 np.linalg.norm(jp, 2, axis=(-2, -1))))
    original_good = np.linalg.eigvalsh(dj)[:, 0] >= -tolerance
    rank = np.full(len(dj), 4, int)
    for index in np.flatnonzero(~original_good):
        lower = np.linalg.cholesky(prior_cov[index])
        white_j = lower.T @ ju[index] @ lower
        white_j = (white_j + white_j.T) / 2
        value, vector = np.linalg.eigh(white_j)
        assert np.all(value > 0)
        selected = value > 1 + 1e-10 * max(1., np.max(np.abs(value)))
        rank[index] = int(selected.sum())
        if not selected.any():
            dj[index] = 0; dh[index] = 0; dc[index] = 0
            continue
        basis = vector[:, selected]; eigenvalue = value[selected]
        white_h = lower.T @ ju[index] @ (post_mean[index] - prior_mean[index])
        one_h = basis.T @ white_h
        white_a = (basis * (eigenvalue - 1)) @ basis.T
        selected_h = basis @ one_h
        inverse = np.linalg.inv(lower)
        update_j = inverse.T @ white_a @ inverse
        update_j = (update_j + update_j.T) / 2
        shift_h = inverse.T @ selected_h
        mean = prior_mean[index]
        update_h = update_j @ mean + shift_h
        marginal_c = .5 * np.sum(np.log(eigenvalue) - one_h ** 2 / eigenvalue)
        update_c = marginal_c - .5 * mean @ update_j @ mean - shift_h @ mean
        dj[index], dh[index], dc[index] = update_j, update_h, update_c
        # The retained factor is a normalized posterior/prior marginal ratio.
        _, _, log_mass = integrate(jp[index] + update_j, hp[index] + update_h, cp[index] + update_c)
        assert abs(log_mass) < 2e-8, ('marginal normalization', log_mass)
    return dj, dh, dc, original_good, rank
