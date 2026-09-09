"""Independent scalar reconstruction from the entire local update history."""
from collections import defaultdict
import numpy as np


def audit_history(run,data):
    mode=run['negativeHistoryMode']
    assert mode in ['original','history','half']
    local=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    logged=np.asarray(run['negativeHistoryRecords'],float).reshape(-1,12)
    assert np.array_equal(local[:,:4],logged[:,:4])
    assert np.isfinite(logged).all()
    state={}
    expected=[]
    for row in local:
        t,n,bt,bl=map(int,row[:4]);key=(n,bt,bl)
        a,p=row[9:11]
        last,h=state.get(key,(0,0.))
        before=h if last==t-1 else 0.
        if p==0:
            effective=0.;discount=1.;after=0.
        else:
            assert 0<p<1
            after=(1-a)*(before+1)
            effective=p;discount=1.
            if mode=='half':discount=.5
            elif mode=='history' and before>0:
                # Algebraically independent form of the Beta predictive mean.
                effective=p/(1+(1-p)*before)
                discount=np.log1p(-effective)/np.log1p(-p)
        nominal=(1-a)*p/(2-p)
        expected.append([t,n,bt,bl,before,a,p,effective,discount,after,nominal,nominal*discount])
        state[key]=(t,after)
    expected=np.asarray(expected,float).reshape(-1,12)
    assert np.allclose(logged,expected,atol=2e-12,rtol=0),'source-private consecutive history'
    return expected[:,-1]


def audit_positive_marks(run,mat,ratios):
    local=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    frames=defaultdict(list)
    for i,row in enumerate(local):frames[int(row[0]),int(row[1])].append(i)
    checked=0
    for t in range(1,int(mat['T'].item())+1):
        for n in [1,2]:
            rows=local[frames[t,n]]
            marks=np.maximum(0,np.tanh(.5*np.log(np.asarray(ratios[n-1,t-1],float).reshape(-1))))
            raw=np.asarray(run['localAssociationWeights'][n-1+2*(t-1)],float)
            if not len(rows):assert not raw.size;continue
            W=raw.reshape(len(rows),len(marks)+1)
            W=np.where(np.isfinite(W),np.maximum(W,0),0)
            sums=W.sum(1,keepdims=True)
            weights=np.divide(W,sums,out=np.zeros_like(W),where=sums>0)
            expected=np.clip(weights[:,1:]@marks,0,1)*rows[:,7]
            assert np.allclose(rows[:,8],expected,atol=1e-12,rtol=0),'unchanged positive mark support'
            checked+=len(rows)
    return checked


def audit_matching(run,data,frames):
    from association_math import objective,audit_selected
    records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    indexed=defaultdict(list)
    for i,row in enumerate(records):indexed[int(row[0]),int(row[1])].append(i)
    delivered=np.asarray(data['delivered'],bool)
    checked=0
    for t in range(1,len(data['time'])+1):
        for n in [1,2]:
            rows=records[indexed[t,n]]
            if not delivered[n-1,2-n,t-1]:assert not len(rows);continue
            left,right=frames[t,n],frames[t,3-n]
            info=objective(left['keys'],right['keys'],left['means'],right['means'],left['covariance'],right['covariance'],
                np.zeros_like(left['features']),np.zeros_like(right['features']),left['r'],right['r'],[],'direct',t)
            li={tuple(k):i for i,k in enumerate(left['keys'])};ri={tuple(k):i for i,k in enumerate(right['keys'])}
            originals=rows[:,31:35].reshape(-1,2,2).astype(int)
            pairs=[(li[tuple(k[0])],ri[tuple(k[1])]) for k in originals if (k[:,0]>0).all()]
            selected,dropped,abstain=audit_selected(info,pairs)
            assert not dropped.any() and not abstain.any()
            assert len(pairs)==run['matchedLabels'][n-1][t-1]
            assert set(ri)=={tuple(k[1]) for k in originals if k[1,0]>0}
            assert set(li)=={tuple(k[0]) for k in originals if k[0,0]>0}
            checked+=len(pairs)
    return checked
