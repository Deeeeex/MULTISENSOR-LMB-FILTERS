"""Independent reconstruction of current observation moments."""
import numpy as np


def direct_moments(weights, measurements, opportunity):
    result = np.zeros((len(opportunity), 8))
    if not weights.size or not len(measurements):
        return result
    assert weights.shape == (len(opportunity), len(measurements) + 1)
    assert np.isfinite(weights).all() and (weights >= 0).all()
    for j in range(len(weights)):
        total = weights[j].sum()
        if not opportunity[j] or total <= 0:
            continue
        detection = weights[j, 1:] / total
        mass = detection.sum()
        if mass <= 0:
            continue
        conditional = detection / mass
        mean = conditional @ measurements
        delta = measurements - mean
        covariance = np.eye(2) + np.einsum('n,ni,nj->ij', conditional, delta, delta)
        entropy = -sum(v * np.log(v) for v in conditional if v > 0)
        result[j] = [mass, *mean, covariance[0, 0], covariance[1, 0], covariance[1, 1], conditional.max(), entropy]
    return result

