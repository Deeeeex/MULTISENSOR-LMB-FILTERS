import numpy as np

def factors(records, delta, active, joint):
    overlap = np.exp(np.minimum(0, records[:, 10]))
    assert np.all(records[:, 10] < 1e-7) and np.all((overlap >= 0) & (overlap <= 1))
    opposing = joint & ((delta > 0) & active).any(1) & ((delta < 0) & active).any(1)
    positive_agreement = joint & ((~active) | (delta > 0)).all(1)
    return dict(agreement_all=np.broadcast_to(overlap[:, None], delta.shape),
                agreement_positive=np.where(delta > 0, overlap[:, None], 1),
                agreement_conflict=np.broadcast_to(np.where(opposing, overlap, 1)[:, None], delta.shape),
                positive_consensus=np.where(delta > 0, positive_agreement[:, None], 1),
                positive_only=(delta >= 0).astype(float))

