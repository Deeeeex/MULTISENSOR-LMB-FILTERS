"""Independent executed-probability, local odds, and complete No-age Gaussian audit."""
from collections import defaultdict
import numpy as np
from scipy.special import expit
from review_gaussian_audit import unpack,natural,integrate

def base_arm(arm):
    for suffix in ['_range','_constant']:
        if arm.endswith(suffix):return arm[:-len(suffix)]
    return arm

def audit_actual_pd(run,data):
    logged=np.asarray(run['qualityRecords'],float).reshape(-1,10)
    local=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    assert np.isfinite(logged).all() and np.array_equal(logged[:,:4],local[:,:4])
    assert len(np.unique(logged[:,:4],axis=0))==len(logged)
    mode=run['rangeDetectionMode'];fit=run['rangeDetectionModel'];poses=np.asarray(data['positions'])
    assert mode in ['nominal','range','constant'] and fit['slope']<=0
    expected=[];excluded=0
    for row in logged:
        t,n=map(int,row[:2]);xy=row[4:6];squared=np.sum((poses[:,:,t-1].T-xy)**2,axis=1)
        distance=np.sqrt(squared[n-1]);inside=abs(xy[0])<=70.4 and abs(xy[1])<=40 and (squared>9).all() and distance<=40
        p=expit(fit['intercept']+fit['slope']*distance/40) if mode=='range' else fit['constant'] if mode=='constant' else data['pd']
        expected.append(p if inside else 0.);excluded+=int(not inside)
        assert abs(distance-row[8])<1e-12
    expected=np.asarray(expected)
    assert np.allclose(logged[:,9],expected,atol=3e-15,rtol=0),'actual shared pD'
    assert np.array_equal(local[:,10],logged[:,9]) and np.array_equal(expected>0,local[:,7].astype(bool))
    retained=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    lookup={tuple(r[:4].astype(int)):r for r in logged}
    for row in retained:assert np.array_equal(row[4:8],lookup[tuple(row[:4].astype(int))][4:8])
    before,after=np.clip(local[:,4],1e-9,1-1e-9),np.clip(local[:,5],1e-9,1-1e-9)
    assert np.allclose(local[:,6],np.log(after/(1-after))-np.log(before/(1-before)),atol=3e-12,rtol=0)
    assert np.allclose(local[:,11],(1-local[:,9])*expected/(2-expected),atol=1e-14,rtol=0)
    grouped=defaultdict(list)
    for i,row in enumerate(local):grouped[tuple(row[:2].astype(int))].append(i)
    for (t,n),indices in grouped.items():
        rows=local[indices];raw=np.asarray(run['localAssociationWeights'][n-1+2*(t-1)],float)
        w0=raw.reshape(len(rows),-1)[:,0] if raw.size else np.ones(len(rows))
        assert np.isfinite(w0).all() and (w0>=0).all() and (w0<=1+1e-12).all()
        phi=rows[:,4]*(1-rows[:,10]);posterior=phi/(phi+(1-rows[:,4])*w0)
        assert np.allclose(rows[:,5],posterior,atol=3e-12,rtol=0),'executed local odds from conditional association weights'
    return dict(all_predictions=len(logged),retained_predictions=len(retained),excluded_predictions=excluded,
        minimum_positive_pd=float(expected[expected>0].min()),maximum_pd=float(expected.max()),all_local_odds_checked=len(local))

def source_view(run):
    records=np.asarray(run['iterationRecords'],float).reshape(-1,60).copy()
    sources=np.asarray(run['fusionSourceRecords'],float).reshape(-1,8)
    outputs=np.asarray(run['fusionOutputRecords'],float).reshape(-1,19)
    assert np.array_equal(sources[:,:4],records[:,:4]) and np.array_equal(outputs[:,:4],records[:,:4])
    assert np.isfinite(sources).all() and np.isfinite(outputs).all()
    assert np.array_equal(outputs[:,4],records[:,6]) and np.array_equal(outputs[:,5:7],records[:,4:6])
    if 'gaussian_evidence' in run['arm']:
        assert np.array_equal(records[:,31:35],sources[:,4:8])
        assert np.allclose(outputs[:,7:9],records[:,40:42],atol=1e-13,rtol=0)
        assert np.allclose(outputs[:,9:],records[:,42:52],atol=1e-13,rtol=0)
    else:
        assert not records[:,26:].any()
        records[:,31:35]=sources[:,4:8]
    return dict(run,arm=base_arm(run['arm']),iterationRecords=records.tolist())

def audit_noage(run,data):
    from censor_scalar_audit import audit_existence
    records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    scalar=audit_existence(dict(run,iterationRecords=records[:,:26].tolist()),data,'marked_lineage')
    local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    outputs=np.asarray(run['fusionOutputRecords'],float).reshape(-1,19)
    originals=records[:,31:35].reshape(-1,2,2).astype(int);present=originals[:,:,0]>0
    assert not run['packetGaussianRecords']
    index={tuple(r[:4].astype(int)):i for i,r in enumerate(local)}
    indices=np.zeros(present.shape,int)
    for i,side in np.argwhere(present):
        t,n=map(int,records[i,:2]);indices[i,side]=index[t,n if side==0 else 3-n,*originals[i,side]]
    active=records[:,13:15]>0;alpha=(active&present).astype(float)
    assert (alpha.sum(1)>0).all();alpha/=alpha.sum(1,keepdims=True)
    j,h,c=natural(local[:,18:22],unpack(local[:,22:32]))
    mean,cov,normalizer=integrate(np.einsum('ns,nsij->nij',alpha,j[indices]),
        np.einsum('ns,nsi->ni',alpha,h[indices]),np.einsum('ns,ns->n',alpha,c[indices]))
    assert np.allclose(normalizer,records[:,10],atol=1e-8,rtol=0)
    assert np.allclose(mean,outputs[:,5:9],atol=1e-7,rtol=0)
    assert np.allclose(cov,unpack(outputs[:,9:]),atol=1e-7,rtol=0)
    scalar.update(full_gaussians=len(outputs),source_joins=int(present.sum()))
    return scalar
