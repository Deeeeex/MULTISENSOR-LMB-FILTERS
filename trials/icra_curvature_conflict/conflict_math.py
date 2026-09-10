"""One fixed rule: veto extra evidence made one-sided by curvature filtering."""
from pathlib import Path
import sys
import numpy as np
from scipy.special import expit

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'icra_joint_admission'))
from joint_math import source_state, calculate as original_calculate, LOWER, unpack

RULES = ['original', 'paired_veto', 'positive_rejected_veto', 'negative_rejected_veto']


def events(raw, delta, allowed, final_kept):
    rejected = (raw > 0) & ~allowed
    accepted = final_kept > 0
    positive = ((rejected & (delta > 0)).any(1)
                & (accepted & (delta < 0)).any(1))
    negative = ((rejected & (delta < 0)).any(1)
                & (accepted & (delta > 0)).any(1))
    return positive, negative


def check_fixtures():
    delta = np.array([[2.,-2.],[-2.,2.],[2.,2.],[-2.,-2.],[2.,-2.],
                      [2.,-2.],[0.,-2.],[2.,-2.],[2.,-2.],[2.,-2.]])
    raw = np.full_like(delta, .2); raw[4,0] = 0
    allowed = np.tile([False, True], (len(delta),1)); allowed[7] = True
    allowed[8] = False
    kept = raw*allowed; kept[5] = 0; kept[9] = 0
    p,n = events(raw,delta,allowed,kept)
    assert np.array_equal(p,[1,0,0,0,0,0,0,0,0,0])
    assert np.array_equal(n,[0,1,0,0,0,0,0,0,0,0])
    p2,n2 = events(raw[:,::-1],delta[:,::-1],allowed[:,::-1],kept[:,::-1])
    assert np.array_equal(p,p2) and np.array_equal(n,n2)
    p3,n3 = events(raw,-delta,allowed,kept)
    assert np.array_equal(p,n3) and np.array_equal(n,p3)
    # The veto returns the original conservative pool for both signs.
    beta = np.array([.5,.5]); r = np.array([.2,.8]); integral = -.3
    z = float(np.sum(beta*(np.log(r)-np.log1p(-r))))+integral
    assert np.isclose(expit(z),expit(-.3),atol=1e-15,rtol=0)
    assert expit(z-.4) < expit(z) < expit(z+.4)
    return dict(event_rows=len(delta),positive_events=int(p.sum()),negative_events=int(n.sum()),
                source_permutation_invariant=True,sign_symmetric=True,conservative_endpoint=True)


def calculate(state, rule, scalar=False):
    assert rule in RULES
    s=state; rec=s['records']
    _, original, _=original_calculate(s,'original',scalar)
    raw=s['joint'][:,None]*(s['active']-s['beta'])*np.where(s['delta']>=0,s['gates'],s['negative'])
    p,n=events(raw,s['delta'],original['allowed'],rec[:,52:54])
    trigger={'original':np.zeros(len(rec),bool), 'paired_veto':p|n,
             'positive_rejected_veto':p, 'negative_rejected_veto':n}[rule]
    # Copy the native output exactly at every unaffected label.
    r=rec[:,9].copy(); mean=np.c_[rec[:,4:6],rec[:,40:42]]
    covariance=unpack(rec[:,42:52]); kept=rec[:,52:54].copy()
    log_i=rec[:,10 if scalar else 56].copy()
    kept[trigger]=0
    r[trigger]=expit(np.sum(s['beta'][trigger]*s['logits'][trigger],axis=1)+rec[trigger,10])
    if not scalar:
        mean[trigger]=s['m0'][trigger]; covariance[trigger]=s['p0'][trigger]
    log_i[trigger]=rec[trigger,10]
    candidate=rec.copy(); candidate[:,9]=r; candidate[:,4:6]=mean[:,:2]
    candidate[:,40:42]=mean[:,2:]; candidate[:,42:52]=covariance[:,LOWER[0],LOWER[1]]
    assert np.array_equal(candidate[~trigger],rec[~trigger])
    assert np.array_equal(kept[~trigger],rec[~trigger,52:54])
    assert not kept[trigger].any()
    values=dict(r=r,mean=mean,covariance=covariance,log_integral=log_i,kept=kept,
                allowed=original['allowed'],fallback=original['fallback'],positive=s['gates'],
                positive_rejected=p,negative_rejected=n,trigger=trigger)
    diagnostics=dict(fusion_labels=len(rec),positive_rejected_events=int(p.sum()),
        negative_rejected_events=int(n.sum()),vetoed_labels=int(trigger.sum()),
        existence_increased=int((r>rec[:,9]+1e-12).sum()),
        existence_decreased=int((r<rec[:,9]-1e-12).sum()),
        mean_absolute_existence_change=float(np.mean(abs(r-rec[:,9]))),
        changed_spatial_labels=int((np.max(abs(mean-np.c_[rec[:,4:6],rec[:,40:42]]),axis=1)>1e-10).sum()))
    return candidate,values,diagnostics
