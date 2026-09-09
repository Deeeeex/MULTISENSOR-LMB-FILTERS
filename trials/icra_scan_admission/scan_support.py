"""Independent current-scan negative-support reconstruction."""
import numpy as np


def support_from_records(local, exponent):
    assert exponent in [1, 2]
    local = np.asarray(local, float).reshape(-1, 12)
    support = np.zeros(len(local)); rates = []
    groups = np.unique(local[:, :2].astype(int), axis=0)
    for t, n in groups:
        index = np.flatnonzero((local[:, 0] == t) & (local[:, 1] == n))
        values = local[index]; mask = values[:, 7] > 0
        nominal = values[mask, 10]
        if not len(nominal):
            continue
        assert np.all(nominal == nominal[0]); pd = float(nominal[0])
        weights = values[:, 4] ** exponent * mask
        denominator = float(weights.sum())
        numerator = float(weights @ values[:, 9])
        estimate = min(pd, numerator / denominator) if denominator > 0 else pd
        assert 0 <= estimate <= pd
        support[index] = (1-values[:, 9]) * mask * estimate / (2-estimate)
        assert np.all(support[index] <= values[:, 11] + 1e-14)
        rates.append(dict(time=int(t), source=int(n), numerator=numerator,
                          denominator=denominator, p_hat=estimate, nominal_pd=pd))
    return support, rates
