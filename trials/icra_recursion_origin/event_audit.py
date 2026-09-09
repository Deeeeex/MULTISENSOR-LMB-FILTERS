"""Validate every component of the declared event from reconstructed densities."""
import numpy as np
from scipy.special import expit

LOWER=tuple(np.asarray(v) for v in zip(*[(r,c) for c in range(4) for r in range(c,4)]))


def apply_expected_event(run,records,base_mean,base_cov,joint_mean,joint_cov,joint_i,beta,logits,kept,delta):
    mode=run['interventionMode'];frame=run['interventionFrame']
    assert mode in ['none','joint','existence','spatial'] and frame==3
    old_i=records[:,10].copy()
    if mode=='none':return base_mean,base_cov,old_i
    select=records[:,0]==frame
    events=np.asarray(run['interventionRecords'],float).reshape(-1,54)
    assert len(events)==select.sum() and len(events)>0 and np.isfinite(events).all()
    assert np.array_equal(events[:,:4],records[select,:4])
    assert np.all(events[:,4]==['joint','existence','spatial'].index(mode)+1)
    mean=base_mean.copy();cov=base_cov.copy();log_i=old_i.copy()
    if mode in ['joint','existence']:log_i[select]=joint_i[select]
    if mode in ['joint','spatial']:mean[select]=joint_mean[select];cov[select]=joint_cov[select]
    inherited=(beta*logits).sum(1)+(kept*delta).sum(1)
    for column,expected in [(5,expit(inherited+old_i)),(6,expit(inherited+joint_i)),(7,expit(inherited+log_i))]:
        assert np.allclose(events[:,column],expected[select],atol=2e-10,rtol=0),('event existence',column)
    for start,expected in [(8,base_mean),(12,joint_mean),(16,mean)]:
        assert np.allclose(events[:,start:start+4],expected[select],atol=1e-7,rtol=0),('event mean',start)
    for start,expected in [(20,base_cov),(30,joint_cov),(40,cov)]:
        assert np.allclose(events[:,start:start+10],expected[select][:,LOWER[0],LOWER[1]],atol=1e-7,rtol=0),('event covariance',start)
    assert np.array_equal(events[:,50:52],records[select,52:54])
    assert np.array_equal(events[:,52],old_i[select])
    assert np.allclose(events[:,53],joint_i[select],atol=1e-8,rtol=0)
    assert np.array_equal(events[:,7],records[select,6])
    return mean,cov,log_i
