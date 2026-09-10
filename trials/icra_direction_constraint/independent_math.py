"""Independent encoded-ratio reconstruction with scalar Brent boundaries."""
import math
import numpy as np
from scipy.optimize import brentq
from scipy.special import expit

LOWER=tuple(np.asarray(v) for v in zip(*[(r,c) for c in range(4) for r in range(c,4)]))
_RUN=None
_STATE=None

def matrix(v):
    out=np.zeros((*v.shape[:-1],4,4));out[...,LOWER[0],LOWER[1]]=v;out[...,LOWER[1],LOWER[0]]=v
    return out

def gaussian(mean,cov):
    root=np.linalg.cholesky(cov)
    precision=np.linalg.solve(cov,np.broadcast_to(np.eye(4),cov.shape))
    precision=(precision+precision.swapaxes(-1,-2))/2
    information=np.einsum('...ij,...j->...i',precision,mean)
    constant=-.5*(4*np.log(2*np.pi)+2*np.log(np.diagonal(root,axis1=-2,axis2=-1)).sum(-1)+np.sum(mean*information,axis=-1))
    return precision,information,constant

def integrate(j,h,c):
    j=(j+j.swapaxes(-1,-2))/2;root=np.linalg.cholesky(j)
    forward=np.linalg.solve(root,h[...,None]);mean=np.linalg.solve(root.swapaxes(-1,-2),forward)[...,0]
    covariance=np.linalg.solve(j,np.broadcast_to(np.eye(4),j.shape))
    integral=c+.5*(4*np.log(2*np.pi)-2*np.log(np.diagonal(root,axis1=-2,axis2=-1)).sum(-1)+np.sum(h*mean,axis=-1))
    return mean,covariance,integral

def source(run,scalar):
    global _RUN,_STATE
    if _RUN is run:
        assert _STATE['scalar']==scalar
        return _STATE
    rec=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    packets=np.asarray(run['packetGaussianRecords'],float).reshape(-1,19)
    local=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    gauss=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    packet_map={tuple(v[:4].astype(int)):v[4:] for v in packets}
    local_map={tuple(v[:4].astype(int)):v for v in local}
    gauss_map={tuple(v[:4].astype(int)):v for v in gauss}
    ids=rec[:,31:35].reshape(-1,2,2).astype(int);present=ids[:,:,0]>0;active=rec[:,13:15]>0
    eligible=(active.sum(1)>=2)&((~active)|present).all(1)
    encoded=np.zeros((len(rec),2,15));post_j=np.zeros((len(rec),2,4,4));post_h=np.zeros((len(rec),2,4));post_c=np.zeros((len(rec),2))
    positive=np.zeros((len(rec),2));negative=np.zeros_like(positive);good=np.ones(present.shape,bool)
    logits=np.zeros_like(positive);p=np.clip(rec[:,17:19][active],1e-9,1-1e-9);logits[active]=np.log(p)-np.log1p(-p)
    minus_logits=logits-rec[:,19:21]
    for i,s in np.argwhere(present):
        t,n=map(int,rec[i,:2]);key=(t,n if s==0 else 3-n,*ids[i,s]);row=local_map[key];g=gauss_map[key]
        encoded[i,s]=packet_map[key];post_j[i,s],post_h[i,s],post_c[i,s]=gaussian(g[18:22],matrix(g[22:32]))
        prior_j=post_j[i,s]-matrix(encoded[i,s,:10])
        tol=1e-10*max(1.,np.linalg.norm(post_j[i,s],2),np.linalg.norm(prior_j,2))
        good[i,s]=np.linalg.eigvalsh(matrix(encoded[i,s,:10]))[0]>=-tol
        assert abs(row[6]-rec[i,19+s])<2e-12
        if active[i,s]:
            rp=np.clip(row[4],1e-9,1-1e-9)
            assert abs(minus_logits[i,s]-(np.log(rp)-np.log1p(-rp)))<2e-9
        if rec[i,21+s]==t:positive[i,s],negative[i,s]=row[8],row[11]
    assert np.array_equal(positive,rec[:,26:28]) and np.array_equal(negative,rec[:,35:37])
    dj=matrix(encoded[:,:,:10]);dh=encoded[:,:,10:14];dc=encoded[:,:,14]
    alpha=rec[:,54:56]
    local_base_j=np.einsum('ns,nsij->nij',alpha,post_j)
    local_base_h=np.einsum('ns,nsi->ni',alpha,post_h);local_base_c=np.sum(alpha*post_c,axis=1)
    base_mean,base_cov,base_i=integrate(local_base_j,local_base_h,local_base_c)
    assert np.allclose(base_i,rec[:,10],atol=1e-8,rtol=0)
    raw=eligible[:,None]*(active-rec[:,28:30])*np.where(rec[:,19:21]>=0,positive,negative)
    called=(raw>0).any(1);allowed=(~called[:,None])|(~present)|good;kept=raw*allowed
    proposal_j=local_base_j+np.einsum('ns,nsij->nij',kept,dj)
    proposal_h=local_base_h+np.einsum('ns,nsi->ni',kept,dh);proposal_c=local_base_c+np.sum(kept*dc,axis=1)
    fallback=np.zeros(len(rec),bool)
    for i in np.flatnonzero(called):
        try:
            np.linalg.cholesky(proposal_j[i])
            fallback[i]=not np.isfinite(proposal_j[i]).all() or not np.isfinite(proposal_h[i]).all() or not np.isfinite(proposal_c[i]) or 1/np.linalg.cond(proposal_j[i],1)<1e-12
        except np.linalg.LinAlgError:fallback[i]=True
    kept[fallback]=0
    assert np.array_equal(allowed,rec[:,57:59]) and np.array_equal(fallback,rec[:,59])
    assert np.allclose(kept,rec[:,52:54],atol=2e-14,rtol=0)
    actual_mean=np.c_[rec[:,4:6],rec[:,40:42]];actual_cov=matrix(rec[:,42:52])
    actual_j,actual_h,actual_c=gaussian(actual_mean,actual_cov)
    old_kept=np.zeros_like(kept) if scalar else rec[:,52:54]
    base_j=actual_j-np.einsum('ns,nsij->nij',old_kept,dj)
    base_h=actual_h-np.einsum('ns,nsi->ni',old_kept,dh)
    base_c=actual_c+rec[:,10 if scalar else 56]-np.sum(old_kept*dc,axis=1)
    assert np.allclose(base_j,local_base_j,atol=1e-7,rtol=0)
    assert np.allclose(base_h,local_base_h,atol=1e-7,rtol=0)
    assert np.allclose(base_c,local_base_c,atol=1e-7,rtol=0)
    # Independent prediction reconstruction: remove the encoded local ratio.
    minus_j=np.einsum('ns,nsij->nij',alpha,post_j-dj)
    minus_h=np.einsum('ns,nsi->ni',alpha,post_h-dh);minus_c=np.sum(alpha*(post_c-dc),axis=1)
    _,_,minus_i=integrate(minus_j,minus_h,minus_c)
    base_z=np.sum(rec[:,28:30]*logits,axis=1)+rec[:,10]
    ref_z=np.sum(rec[:,28:30]*minus_logits,axis=1)+minus_i
    extra_scalar=np.sum(rec[:,52:54]*rec[:,19:21],axis=1)
    d0=base_z-ref_z;d1=d0+extra_scalar+rec[:,10 if scalar else 56]-rec[:,10]
    assert np.allclose(expit(ref_z+d1),rec[:,9],atol=2e-10,rtol=0)
    eligible&=(rec[:,52:54]>0).any(1)
    negative_reversal=eligible&(d0>1e-7)&(d1<-1e-7)
    positive_reversal=eligible&(d0<-1e-7)&(d1>1e-7)
    s=dict(scalar=scalar,rec=rec,mean=actual_mean,covariance=actual_cov,base_mean=base_mean,base_covariance=base_cov,
        positive=positive,negative=negative,allowed=allowed,fallback=fallback,
        base_j=base_j,base_h=base_h,base_c=base_c,minus_i=minus_i,ref_z=ref_z,d0=d0,d1=d1,extra_scalar=extra_scalar,
        extra_j=np.einsum('ns,nsij->nij',rec[:,52:54],dj),extra_h=np.einsum('ns,nsi->ni',rec[:,52:54],dh),extra_c=np.sum(rec[:,52:54]*dc,axis=1),
        negative_reversal=negative_reversal,positive_reversal=positive_reversal)
    _RUN=run;_STATE=s;return s

def path(s,indices,x):
    j=s['base_j'][indices]+x[:,None,None]*s['extra_j'][indices]
    h=s['base_h'][indices]+x[:,None]*s['extra_h'][indices]
    c=s['base_c'][indices]+x*s['extra_c'][indices]
    mean,cov,integral=integrate(j,h,c)
    if s['scalar']:mean,cov,integral=s['base_mean'][indices],s['base_covariance'][indices],s['rec'][indices,10]
    d=s['d0'][indices]+x*s['extra_scalar'][indices]+integral-s['rec'][indices,10]
    return mean,cov,integral,d

def roots(s):
    if 'roots' in s:return s['roots']
    answer=np.ones(len(s['rec']))
    for i in np.flatnonzero(s['negative_reversal']|s['positive_reversal']):
        def f(x):return float(path(s,np.array([i]),np.array([x]))[3][0])
        assert f(0)*f(1)<0
        answer[i]=brentq(f,0.,1.,xtol=1e-14,rtol=1e-14,maxiter=100)
        assert abs(f(answer[i]))<1e-8
    s['roots']=answer;return answer

def rebuilt(run,rule,scalar):
    assert rule in ['original','nonreversal','negative_reversal','positive_reversal']
    s=source(run,scalar);rec=s['rec'];n=len(rec)
    changed=np.zeros(n,bool)
    if rule in ['nonreversal','negative_reversal']:changed|=s['negative_reversal']
    if rule in ['nonreversal','positive_reversal']:changed|=s['positive_reversal']
    multiplier=np.ones(n)
    if changed.any():multiplier[changed]=roots(s)[changed]
    mean=s['mean'].copy();cov=s['covariance'].copy();integral=rec[:,10 if scalar else 56].copy()
    r=rec[:,9].copy();direction=s['d1'].copy()
    if changed.any():
        indices=np.flatnonzero(changed);mean[changed],cov[changed],integral[changed],direction[changed]=path(s,indices,multiplier[indices])
        r[changed]=expit(s['ref_z'][changed]+direction[changed])
    return rec,dict(r=r,mean=mean,covariance=cov,log_integral=integral,
        kept=rec[:,52:54]*multiplier[:,None],allowed=s['allowed'],fallback=s['fallback'],unchanged=~changed,
        positive=s['positive'],negative=s['negative'],multiplier=multiplier,reference_log_odds=s['ref_z'],
        prediction_log_integral=s['minus_i'],base_direction=s['d0'],original_direction=s['d1'],final_direction=direction,
        negative_reversal=s['negative_reversal'],positive_reversal=s['positive_reversal'])
