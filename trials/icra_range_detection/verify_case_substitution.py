"""Rebuild the case substitution from saved GS densities and transmitted ratios."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
from scipy.special import expit

OUT=Path(__file__).resolve().parent
for folder in ['icra_reviewer_revision','icra_method_iteration','icra_v2x_gce_diagnosis']:
    sys.path.insert(0,str(OUT.parent/folder))
from review_gaussian_audit import unpack
from analyze_development import counterfactual
from diagnose_gap import read,matched_truth
from analyze_case_studies import score
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT/'SCALAR_CASE_SUBSTITUTION.json';old=json.loads(source.read_text())
    path=OUT/'results/range_detection_preflight/v2xt_0001_intermittent_marked_gaussian_evidence_guarded_scalar_range.json.gz'
    assert old['passed'] and sha(path)==old['native_sha256']
    data=read(path);run=data['runs'];delivery=np.asarray(data['delivered'],bool);poses=np.asarray(data['positions'])
    records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    records=records[(records[:,0]>=53)&(records[:,0]<=122)]
    packets=np.asarray(run['packetGaussianRecords'],float).reshape(-1,19)
    lookup={tuple(row[:4].astype(int)):row[4:] for row in packets}
    delta=np.zeros((len(records),2,15));original=records[:,31:35].reshape(-1,2,2).astype(int)
    for i,s in np.argwhere(original[:,:,0]>0):
        t,n=map(int,records[i,:2]);delta[i,s]=lookup[t,n if s==0 else 3-n,*original[i,s]]
    mean0=np.c_[records[:,4:6],records[:,40:42]];cov0=unpack(records[:,42:52])
    covariance_root=np.linalg.cholesky(cov0)
    precision0=np.linalg.solve(cov0,np.broadcast_to(np.eye(4),cov0.shape))
    information0=np.einsum('nij,nj->ni',precision0,mean0)
    log2pi4=4*np.log(2*np.pi)
    constant0=-.5*(log2pi4+2*np.log(np.diagonal(covariance_root,axis1=-2,axis2=-1)).sum(1)
        +np.einsum('ni,ni->n',mean0,information0))+records[:,10]
    kept=records[:,52:54];assert not kept[records[:,59]>0].any()
    precision=precision0+np.einsum('ns,nsij->nij',kept,unpack(delta[:,:,:10]))
    information=information0+np.einsum('ns,nsi->ni',kept,delta[:,:,10:14])
    constant=constant0+np.sum(kept*delta[:,:,14],axis=1)
    root=np.linalg.cholesky(precision)
    forward=np.linalg.solve(root,information[...,None])
    mean1=np.linalg.solve(root.swapaxes(-1,-2),forward)[...,0]
    log1=constant+.5*(log2pi4-2*np.log(np.diagonal(root,axis1=-2,axis2=-1)).sum(1)
        +np.einsum('ni,ni->n',information,mean1))
    log1[~(kept>0).any(1)]=records[~(kept>0).any(1),10]
    active=records[:,13:15]>0;logits=np.zeros_like(kept);r=np.clip(records[:,17:19][active],1e-9,1-1e-9)
    logits[active]=np.log(r)-np.log1p(-r)
    offset=np.sum(records[:,28:30]*logits,axis=1)+np.sum(kept*records[:,19:21],axis=1)
    new_r=expit(offset+log1)
    assert abs(np.mean(np.abs(log1-records[:,10]))-old['mean_absolute_normalizer_change'])<1e-8
    assert abs(np.mean(np.abs(new_r-records[:,9]))-old['mean_absolute_existence_change'])<1e-10
    by_frame={(r['frame'],r['robot'],r['alternative']):r for r in old['frames']}
    checked=0
    for name in ['actual','normalizer_only','mean_only','joint']:
        alternative=records.copy()
        if name in ['normalizer_only','joint']:alternative[:,9]=new_r
        if name in ['mean_only','joint']:alternative[:,4:6]=mean1[:,:2]
        for t in range(53,123):
            truth=np.asarray(data['truth'][t-1]).reshape(4,-1)
            target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item()
            for n in [1,2]:
                mask=(records[:,0]==t)&(records[:,1]==n)
                output=counterfactual(alternative[mask],9,poses[:,:,t-1]) if delivery[n-1,2-n,t-1] else run['estimates'][n-1+2*(t-1)]
                reference=by_frame[t,n,name];ospa=score(truth,output)['ospa']
                assert abs(ospa-reference['ospa'])<1e-8
                assert bool(matched_truth(truth,output,2.)[target])==reference['detected']
                checked+=1
    result=dict(passed=True,method='Independent algebra: normalized saved GS density plus logged spatial integral plus transmitted ratio, integrated by Cholesky',
        fusion_labels=len(records),counterfactual_robot_frames=checked,substitution_sha256=sha(source),
        native_sha256=sha(path),verifier_sha256=sha(Path(__file__)))
    destination=OUT/'CASE_SUBSTITUTION_VERIFICATION.json';assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2)+'\n')
    print('INDEPENDENT CASE SUBSTITUTION VERIFIED',len(records),'fusion labels',checked,'counterfactual robot frames')

if __name__=='__main__':main()
