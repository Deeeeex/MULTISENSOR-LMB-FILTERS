"""Bound extra negative evidence by current peer detection support."""
from pathlib import Path
import sys
import numpy as np
from scipy.special import expit

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'icra_joint_admission'))
from joint_math import source_state,audit_raw_weights,enumerate_fixture,integrate,LOWER,unpack

RULES=['original','peer_conditional','peer_joint','no_negative']

def modulation(a,rplus,present,active,rule):
    assert rule in RULES and a.shape==rplus.shape==present.shape==active.shape and a.shape[1]==2
    assert np.all((a>=0)&(a<=1)) and np.all((rplus>=0)&(rplus<=1))
    conditional=np.where(present&active,a,0);joint=conditional*rplus
    if rule=='original':retention=np.ones_like(a)
    elif rule=='no_negative':retention=np.zeros_like(a)
    else:retention=1-(joint if rule=='peer_joint' else conditional)[:,::-1]
    assert np.all((retention>=0)&(retention<=1))
    return retention,conditional,joint

def check_fixtures():
    cases=[([.01],[.9],[[.01]]),([.01],[.9],[[1e6]]),([.8,.2],[.9,.9],[[10],[10]]),
           ([.05,.05,.05],[.9]*3,[[1e4]]*3),([.7,.4],[.9,0],[[3,8],[2,5]]),
           ([.7,.4],[.9,.9],np.empty((2,0)))]
    for prior,pd,L in cases:enumerate_fixture(prior,pd,L)
    a=np.array([[0,0],[1,1],[1,.3],[.7,.8],[.7,.8],[0,.8],[1,1]],float)
    r=np.array([[.7,.8],[1,1],[.01,.9],[.3,.8],[.3,.8],[.3,.8],[0,0]],float)
    present=np.ones_like(a,bool);active=np.ones_like(a,bool);present[3,1]=False;active[4,1]=False
    joint,conditional,pi=modulation(a,r,present,active,'peer_joint')
    expected=np.array([[1,1],[0,0],[.73,.99],[1,.79],[1,.79],[.36,1],[1,1]])
    assert np.allclose(joint,expected,atol=1e-15,rtol=0)
    cond,_,_=modulation(a,r,present,active,'peer_conditional');assert np.all(joint>=cond)
    original,_,_=modulation(a,r,present,active,'original');zero,_,_=modulation(a,r,present,active,'no_negative')
    assert np.all(original==1) and not zero.any()
    flipped,_,_=modulation(a[:,::-1],r[:,::-1],present[:,::-1],active[:,::-1],'peer_joint')
    assert np.array_equal(flipped[:,::-1],joint)
    # For fixed spatial density, attenuating a nonpositive scalar term has
    # ordered endpoints. No general monotonic-r claim is made after integration.
    before=expit(.25-.94);after=expit(.25-.94*.04);none=expit(.25)
    assert before<after<none
    return dict(exact_assignment_cases=len(cases),support_cases=len(a),source_permutation=True,
        excluded_and_absent_zero=True,stale_current_mass_zero=True,conditional_joint_order=True,
        scalar_endpoint_order=True)

def calculate(state,rule,scalar=False):
    assert rule in RULES;s=state;rec=s['records'];idx=s['indices'];positive=s['gates']
    retention,conditional,pi=modulation(s['a'],s['rplus'],s['present'],s['active'],rule)
    negative=s['negative']*retention
    raw=s['joint'][:,None]*(s['active']-s['beta'])*np.where(s['delta']>=0,positive,negative)
    called=(raw>0).any(1);allowed=(~called[:,None])|(~s['present'])|s['good'][idx];kept=raw*allowed
    J=s['j0']+np.einsum('ns,nsij->nij',kept,s['dj'][idx]);H=s['h0']+np.einsum('ns,nsi->ni',kept,s['dh'][idx])
    C=s['c0']+np.einsum('ns,ns->n',kept,s['dc'][idx]);J=(J+J.swapaxes(-1,-2))/2
    fallback=np.zeros(len(rec),bool)
    for i in np.flatnonzero(called):
        try:
            np.linalg.cholesky(J[i]);fallback[i]=not np.isfinite(J[i]).all() or not np.isfinite(H[i]).all() or not np.isfinite(C[i]) or 1/np.linalg.cond(J[i],1)<1e-12
        except np.linalg.LinAlgError:fallback[i]=True
    kept[fallback]=0;J[fallback],H[fallback],C[fallback]=s['j0'][fallback],s['h0'][fallback],s['c0'][fallback]
    mean,cov,log_i=integrate(J,H,C);log_i[~(kept>0).any(1)]=rec[~(kept>0).any(1),10]
    if scalar:mean,cov,log_i=s['m0'].copy(),s['p0'].copy(),rec[:,10].copy()
    r=expit(np.sum(s['beta']*s['logits'],axis=1)+np.sum(kept*s['delta'],axis=1)+log_i)
    unchanged=np.all(kept==rec[:,52:54],axis=1)
    actual_mean=np.c_[rec[:,4:6],rec[:,40:42]];actual_cov=unpack(rec[:,42:52])
    assert np.allclose(r[unchanged],rec[unchanged,9],atol=2e-10,rtol=0)
    assert np.allclose(mean[unchanged],actual_mean[unchanged],atol=1e-7,rtol=0)
    assert np.allclose(cov[unchanged],actual_cov[unchanged],atol=1e-7,rtol=0)
    if rule=='original':assert unchanged.all() and np.array_equal(allowed,rec[:,57:59]) and np.array_equal(fallback,rec[:,59])
    r[unchanged]=rec[unchanged,9];mean[unchanged]=actual_mean[unchanged];cov[unchanged]=actual_cov[unchanged]
    log_i[unchanged]=rec[unchanged,10 if scalar else 56]
    candidate=rec.copy();candidate[:,9]=r;candidate[:,4:6]=mean[:,:2];candidate[:,40:42]=mean[:,2:]
    candidate[:,42:52]=cov[:,LOWER[0],LOWER[1]];assert candidate[unchanged].tobytes()==rec[unchanged].tobytes()
    values=dict(r=r,mean=mean,covariance=cov,log_integral=log_i,kept=kept,allowed=allowed,fallback=fallback,
                positive=positive,negative=negative,retention=retention,conditional=conditional,peer_joint=pi,unchanged=unchanged)
    negative_sources=(s['delta']<0)&(rec[:,52:54]>0)
    diagnostics=dict(fusion_labels=len(rec),changed_labels=int((~unchanged).sum()),
        original_negative_sources=int(negative_sources.sum()),attenuated_original_negative_sources=int((negative_sources&(retention<1)).sum()),
        negative_gate_changed=int((negative!=s['negative']).sum()),admitted_negative_sources=int(((s['delta']<0)&(kept>0)).sum()),
        admitted_positive_sources=int(((s['delta']>=0)&(kept>0)).sum()),rejected_sources=int(((raw>0)&~allowed).sum()),
        aggregate_fallbacks=int(fallback.sum()),mean_absolute_existence_change=float(np.mean(abs(r-rec[:,9]))),
        mean_retention_on_original_negative=float(retention[negative_sources].mean()) if negative_sources.any() else None)
    return candidate,values,diagnostics
