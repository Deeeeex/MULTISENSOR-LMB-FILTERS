"""Follow a complete ratio path up to reversal of matched current evidence."""
from pathlib import Path
import sys
import numpy as np
from scipy.special import expit
from scipy.optimize import brentq
from scipy.integrate import quad

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'icra_joint_admission'))
from joint_math import source_state as old_state,audit_raw_weights,integrate,unpack,LOWER,natural

RULES=['original','nonreversal','negative_reversal','positive_reversal']
DIRECTION_TOL=1e-7
STEPS=60

def signs(d0,d1):
    return (d0>DIRECTION_TOL)&(d1<-DIRECTION_TOL),(d0<-DIRECTION_TOL)&(d1>DIRECTION_TOL)

def select(negative,positive,rule):
    assert rule in RULES
    if rule=='original':return np.zeros_like(negative)
    if rule=='nonreversal':return negative|positive
    return negative if rule=='negative_reversal' else positive

def bracket(f,d0):
    lo=np.zeros(len(d0));hi=np.ones(len(d0));direction=np.sign(d0)
    assert np.all(direction*f(lo)>0) and np.all(direction*f(hi)<0)
    for _ in range(STEPS):
        mid=(lo+hi)/2;safe=direction*f(mid)>=0
        lo=np.where(safe,mid,lo);hi=np.where(safe,hi,mid)
    assert np.all((lo>=0)&(lo<=1)) and np.all(hi-lo<=2e-16)
    return lo

def check_fixtures():
    checks=[]
    # One nontrivial Gaussian coordinate and three unchanged standard coordinates.
    # Independent scalar quadrature integrates the path without natural algebra.
    for d0,linear,precision,shift in [(.4,-1.,1.,0.),(-.4,2.,1.,0.),(.6,-3.,.7,1.2),(-.6,1.4,.3,2.)]:
        def exact(x):return d0+linear*x+.5*((shift*x)**2/(1+precision*x)-np.log1p(precision*x))
        assert exact(0)*exact(1)<0
        result=float(bracket(lambda v:exact(v),np.array([d0]))[0])
        reference=brentq(exact,0,1,xtol=1e-14,rtol=1e-14)
        assert abs(result-reference)<1e-12 and abs(exact(result))<1e-12
        for x in [0.,result,.5,1.]:
            value,error=quad(lambda y:np.exp(-.5*(1+precision*x)*y*y+shift*x*y)/np.sqrt(2*np.pi),-np.inf,np.inf,epsabs=1e-11)
            expected=.5*((shift*x)**2/(1+precision*x)-np.log1p(precision*x))
            assert abs(np.log(value)-expected)<1e-10
        checks.append(dict(d0=d0,d1=float(exact(1)),lambda_value=result,brent_value=reference))
    d0=np.array([.4,-.4,.4,-.4,0.,DIRECTION_TOL/2]);d1=np.array([-.2,.2,.2,-.2,-1.,-1.])
    n,p=signs(d0,d1);assert np.array_equal(n,[1,0,0,0,0,0]) and np.array_equal(p,[0,1,0,0,0,0])
    for rule,mask in [('original',[0]*6),('nonreversal',[1,1,0,0,0,0]),('negative_reversal',[1,0,0,0,0,0]),('positive_reversal',[0,1,0,0,0,0])]:
        assert np.array_equal(select(n,p,rule),mask)
    linear=bracket(lambda v:.4-v,np.array([.4]));assert abs(linear[0]-.4)<1e-15
    return dict(gaussian_quadrature_and_brent=checks,scalar_exact_boundary=True,directional_cases=6,
        numerical_dead_band=DIRECTION_TOL,bisections=STEPS)

def source_state(run):
    s=old_state(run);rec=s['records'];idx=s['indices'];alpha=rec[:,54:56]
    jm,hm,cm=natural(s['local'][:,4:8],unpack(s['local'][:,8:18]))
    minus_j=np.einsum('ns,nsij->nij',alpha,jm[idx]);minus_h=np.einsum('ns,nsi->ni',alpha,hm[idx])
    minus_c=np.einsum('ns,ns->n',alpha,cm[idx]);_,_,minus_i=integrate(minus_j,minus_h,minus_c)
    prior_logits=s['logits']-s['delta']
    increments={tuple(v[:4].astype(int)):v for v in s['increments']}
    for i,side in np.argwhere(s['present']&s['active']):
        t,n=map(int,rec[i,:2]);label=rec[i,31+2*side:33+2*side].astype(int)
        row=increments[t,n if side==0 else 3-n,*label];r=np.clip(row[4],1e-9,1-1e-9)
        assert abs(prior_logits[i,side]-(np.log(r)-np.log1p(-r)))<2e-9
    base_z=np.sum(s['beta']*s['logits'],axis=1)+rec[:,10]
    ref_z=np.sum(s['beta']*prior_logits,axis=1)+minus_i
    extra_scalar=np.sum(rec[:,52:54]*s['delta'],axis=1)
    scalar=run['arm']=='marked_gaussian_evidence_guarded_scalar'
    actual_i=rec[:,10 if scalar else 56]
    d0=base_z-ref_z;d1=d0+extra_scalar+actual_i-rec[:,10]
    assert np.allclose(expit(ref_z+d1),rec[:,9],atol=2e-10,rtol=0)
    eligible=s['joint']&(rec[:,52:54]>0).any(1)
    negative,positive=signs(d0,d1);negative&=eligible;positive&=eligible
    s.update(minus_i=minus_i,base_z=base_z,ref_z=ref_z,d0=d0,d1=d1,extra_scalar=extra_scalar,
        extra_j=np.einsum('ns,nsij->nij',rec[:,52:54],s['dj'][idx]),
        extra_h=np.einsum('ns,nsi->ni',rec[:,52:54],s['dh'][idx]),
        extra_c=np.einsum('ns,ns->n',rec[:,52:54],s['dc'][idx]),
        negative_reversal=negative,positive_reversal=positive,scalar=scalar)
    return s

def density(s,indices,scale):
    j=s['j0'][indices]+scale[:,None,None]*s['extra_j'][indices]
    h=s['h0'][indices]+scale[:,None]*s['extra_h'][indices]
    c=s['c0'][indices]+scale*s['extra_c'][indices]
    mean,cov,log_i=integrate(j,h,c)
    if s['scalar']:mean,cov,log_i=s['m0'][indices],s['p0'][indices],s['records'][indices,10]
    direction=s['d0'][indices]+scale*s['extra_scalar'][indices]+log_i-s['records'][indices,10]
    return mean,cov,log_i,direction

def boundary(s):
    if '_boundary' in s:return s['_boundary']
    indices=np.flatnonzero(s['negative_reversal']|s['positive_reversal'])
    scale=np.ones(len(s['records']))
    if len(indices):
        scale[indices]=bracket(lambda v:density(s,indices,v)[3],s['d0'][indices])
        residual=density(s,indices,scale[indices])[3]
        assert np.max(abs(residual))<1e-8
        assert np.all(np.sign(s['d0'][indices])*residual>=-1e-10)
    s['_boundary']=scale;return scale

def calculate(s,rule,scalar=False):
    assert scalar==s['scalar'];rec=s['records'];n=len(rec)
    changed=select(s['negative_reversal'],s['positive_reversal'],rule)
    scale=np.ones(n)
    if changed.any():scale[changed]=boundary(s)[changed]
    kept=rec[:,52:54]*scale[:,None];unchanged=np.all(kept==rec[:,52:54],axis=1)
    assert np.array_equal(unchanged,~changed)
    mean=np.c_[rec[:,4:6],rec[:,40:42]];cov=unpack(rec[:,42:52]);log_i=rec[:,10 if scalar else 56].copy()
    r=rec[:,9].copy();direction=s['d1'].copy()
    if changed.any():
        indices=np.flatnonzero(changed);mean[changed],cov[changed],log_i[changed],direction[changed]=density(s,indices,scale[changed])
        r[changed]=expit(s['ref_z'][changed]+direction[changed])
        assert np.all(abs(direction[changed])<1e-8)
        assert np.allclose(r[changed],expit(s['ref_z'][changed]),atol=3e-9,rtol=0)
    candidate=rec.copy();candidate[:,9]=r;candidate[:,4:6]=mean[:,:2];candidate[:,40:42]=mean[:,2:]
    candidate[:,42:52]=cov[:,LOWER[0],LOWER[1]]
    assert candidate[unchanged].tobytes()==rec[unchanged].tobytes()
    if rule=='original':assert candidate.tobytes()==rec.tobytes()
    values=dict(r=r,mean=mean,covariance=cov,log_integral=log_i,kept=kept,
        allowed=rec[:,57:59].astype(bool),fallback=rec[:,59].astype(bool),unchanged=unchanged,
        positive=s['gates'],negative=s['negative'],multiplier=scale,reference_log_odds=s['ref_z'],
        prediction_log_integral=s['minus_i'],base_direction=s['d0'],original_direction=s['d1'],final_direction=direction,
        negative_reversal=s['negative_reversal'],positive_reversal=s['positive_reversal'])
    diag=dict(fusion_labels=n,changed_labels=int(changed.sum()),
        negative_reversals=int(s['negative_reversal'].sum()),positive_reversals=int(s['positive_reversal'].sum()),
        changed_negative_reversals=int((changed&s['negative_reversal']).sum()),
        changed_positive_reversals=int((changed&s['positive_reversal']).sum()),
        mean_multiplier_on_changed=float(scale[changed].mean()) if changed.any() else None,
        maximum_boundary_residual=float(abs(direction[changed]).max()) if changed.any() else 0.,
        increased_existence=int((r>rec[:,9]).sum()),decreased_existence=int((r<rec[:,9]).sum()),
        mean_absolute_existence_change=float(abs(r-rec[:,9]).mean()))
    return candidate,values,diag
