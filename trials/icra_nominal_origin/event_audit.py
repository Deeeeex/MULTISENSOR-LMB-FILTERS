"""Independent expected scalar event using the fully reconstructed native GCE terms."""
import numpy as np
from scipy.special import expit

def check_initial_event(run,records,original_r,beta,logits,kept,delta,log_i):
    assert run['initialNegativeEnabled'] and run['initialNegativeFrame']==2
    select=records[:,0]==2;events=np.asarray(run['initialNegativeRecords'],float).reshape(-1,18)
    assert len(events)==select.sum() and np.isfinite(events).all()
    assert np.array_equal(events[:,:4],records[select,:4])
    negative=np.sum(kept*np.minimum(delta,0),axis=1);positive=np.sum(kept*np.maximum(delta,0),axis=1)
    inherited=np.sum(beta*logits,axis=1);expected=original_r.copy()
    altered=select&(negative<0);expected[altered]=expit(inherited[altered]+positive[altered]+log_i[altered])
    for column,values,tolerance in [(4,original_r,2e-10),(5,expected,2e-10),(6,negative,1e-10),(7,positive,1e-10),
                                   (8,inherited,1e-10),(9,log_i,1e-8)]:
        assert np.allclose(events[:,column],values[select],atol=tolerance,rtol=0),('initial event scalar',column)
    for start,values in [(10,kept),(12,delta),(14,beta),(16,records[:,13:15])]:
        assert np.array_equal(events[:,start:start+2],values[select])
    assert np.array_equal(events[:,5],records[select,6])
    unchanged=events[:,6]==0;assert np.array_equal(events[unchanged,4],events[unchanged,5])
    return expected
