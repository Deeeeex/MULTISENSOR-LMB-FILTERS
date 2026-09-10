"""Derive exact refinement eligibility from native scalar and source records."""
import numpy as np
from scipy.special import expit

def eligible_rows(run,records):
    inc=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    sources=np.asarray(run['fusionSourceRecords'],float).reshape(-1,8)
    assert np.array_equal(sources[:,:4],records[:,:4])
    lookup={tuple(map(int,r[:4])):r for r in inc};assert len(lookup)==len(inc)
    local={tuple(map(int,r[:4])) for r in run['localGaussianRecords']}
    eligible=[];joined=[]
    for i,(record,source) in enumerate(zip(records,sources)):
        key=tuple(map(int,record[:4]));current=lookup.get(key)
        if source[4]>0 or current is None or current[5]>.001 or not current[7] or record[13]<=0:continue
        assert key not in local and not source[4:6].any() and source[6]>0
        assert record[17]==.001 and record[14]>0 and current[10]>0
        eligible.append(i);joined.append(current)
    return np.asarray(eligible,int),np.asarray(joined,float).reshape(-1,12)

def check_censor_event(run,records,original_r,beta,logits,kept,delta,log_i):
    expected=original_r.copy();events=np.asarray(run['knownCensorRecords'],float).reshape(-1,25)
    enabled=run['knownCensorEnabled'];assert isinstance(enabled,bool)
    if not enabled:assert not len(events);return expected
    ix,inc=eligible_rows(run,records)
    assert len(ix)==len(events) and np.isfinite(events).all()
    assert np.array_equal(events[:,:4],records[ix,:4])
    assert not kept[ix].any(),'missing local Gaussian cannot admit extra exponents'
    b=records[ix,13:15];q=records[ix,11:13];newlo=logits[ix].copy()
    rr=np.clip(inc[:,5],1e-9,1-1e-9);newlo[:,0]=np.log(rr)-np.log1p(-rr)
    newage=((q-b)*newlo).sum(1);oldage=((q-b)*logits[ix]).sum(1)
    newbeta=b.copy()
    if 'lineage' not in run['arm']:newbeta[newage < -1e-12]=q[newage < -1e-12]
    oldinherited=(beta[ix]*logits[ix]).sum(1);newinherited=(newbeta*newlo).sum(1)
    refined=expit(newinherited+log_i[ix]);equal=inc[:,5]==records[ix,17]
    refined[equal]=original_r[ix[equal]];expected[ix]=refined
    assert np.all(refined<=original_r[ix]+2e-10)
    values=np.c_[original_r[ix],refined,inc[:,4:6],inc[:,7],inc[:,10],records[ix,17:19],b,q,
                 beta[ix],newbeta,oldage,newage,log_i[ix],oldinherited,newinherited]
    tolerances=np.array([2e-10,2e-10,0,0,0,0,0,0,0,0,0,0,2e-14,2e-14,2e-14,2e-14,1e-10,1e-10,1e-8,1e-10,1e-10])
    assert np.all(abs(events[:,4:]-values)<=tolerances),'native event operands and scalar equation'
    assert np.array_equal(events[:,5],records[ix,6]) and np.array_equal(events[:,5],records[ix,9])
    return expected
