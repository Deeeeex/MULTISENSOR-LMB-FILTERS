"""Fixed-input spatial/normalizer substitution on the declared GS mechanism case."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for folder in ['icra_reviewer_revision','icra_method_iteration','icra_v2x_gce_diagnosis']:
    sys.path.insert(0,str(OUT.parent/folder))
from review_gaussian_audit import natural,integrate,unpack
from analyze_development import counterfactual
from diagnose_gap import read,matched_truth
from analyze_case_studies import score
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    path=OUT/'results/range_detection_preflight/v2xt_0001_intermittent_marked_gaussian_evidence_guarded_scalar_range.json.gz'
    auditpath=OUT/'audit_range_detection_preflight.json';audit=json.loads(auditpath.read_text())
    assert audit['passed'] and sha(path)==audit['inputs'][str(path.relative_to(ROOT))]
    data=read(path);run=data['runs'];delivery=np.asarray(data['delivered'],bool);poses=np.asarray(data['positions'])
    local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    all_records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    records=all_records[(all_records[:,0]>=53)&(all_records[:,0]<=122)]
    original=records[:,31:35].reshape(-1,2,2).astype(int);present=original[:,:,0]>0
    lookup={tuple(r[:4].astype(int)):i for i,r in enumerate(local)};indices=np.zeros(present.shape,int)
    for i,s in np.argwhere(present):
        t,n=map(int,records[i,:2]);indices[i,s]=lookup[t,n if s==0 else 3-n,*original[i,s]]
    jp,hp,cp=natural(local[:,4:8],unpack(local[:,8:18]))
    ju,hu,cu=natural(local[:,18:22],unpack(local[:,22:32]))
    alpha=records[:,54:56];kept=records[:,52:54]
    j0=np.einsum('ns,nsij->nij',alpha,ju[indices]);h0=np.einsum('ns,nsi->ni',alpha,hu[indices]);c0=np.einsum('ns,ns->n',alpha,cu[indices])
    mean0,_,log0=integrate(j0,h0,c0)
    assert np.allclose(mean0[:,:2],records[:,4:6],atol=1e-7,rtol=0)
    assert np.allclose(log0,records[:,10],atol=1e-8,rtol=0)
    j=j0+np.einsum('ns,nsij->nij',kept,(ju-jp)[indices])
    h=h0+np.einsum('ns,nsi->ni',kept,(hu-hp)[indices])
    c=c0+np.einsum('ns,ns->n',kept,(cu-cp)[indices])
    mean1,_,log1=integrate(j,h,c)
    unchanged=~(kept>0).any(1);log1[unchanged]=records[unchanged,10]
    active=records[:,13:15]>0;logits=np.zeros_like(alpha)
    probabilities=np.clip(records[:,17:19][active],1e-9,1-1e-9)
    logits[active]=np.log(probabilities)-np.log1p(-probabilities)
    offset=np.sum(records[:,28:30]*logits,axis=1)+np.sum(kept*records[:,19:21],axis=1)
    assert np.allclose(expit(offset+records[:,10]),records[:,9],atol=2e-10,rtol=0)
    new_r=expit(offset+log1);alternatives={}
    for name in ['actual','normalizer_only','mean_only','joint']:
        alternative=records.copy()
        if name in ['normalizer_only','joint']:alternative[:,9]=new_r
        if name in ['mean_only','joint']:alternative[:,4:6]=mean1[:,:2]
        alternatives[name]=alternative
    rows=[]
    for t in range(53,123):
        ids=np.asarray(data['truthIds'][t-1]).ravel();index=np.flatnonzero(ids==5).item()
        truth=np.asarray(data['truth'][t-1]).reshape(4,-1)
        for n in [1,2]:
            mask=(records[:,0]==t)&(records[:,1]==n)
            actual=run['estimates'][n-1+2*(t-1)]
            for name,alternative in alternatives.items():
                output=counterfactual(alternative[mask],9,poses[:,:,t-1]) if delivery[n-1,2-n,t-1] else actual
                value=score(truth,output);detected=bool(matched_truth(truth,output,2.)[index])
                if name=='actual':
                    assert np.isclose(value['ospa'],run['ospa'][n-1][t-1],atol=1e-8,rtol=1e-9)
                    assert detected==bool(matched_truth(truth,actual,2.)[index])
                rows.append(dict(frame=t,robot=n,alternative=name,detected=detected,ospa=value['ospa']))
    summary=[]
    for name in alternatives:
        selected=[r for r in rows if r['alternative']==name]
        summary.append(dict(alternative=name,robot_frames=len(selected),detected=sum(r['detected'] for r in selected),
            mean_ospa=float(np.mean([r['ospa'] for r in selected]))))
    changed=~unchanged
    result=dict(passed=True,scope='Post-outcome fixed-input substitution on GS-visited states; never a recursive method result or selection input',
        window=[53,122],source_arm=run['arm'],source_condition='intermittent',fusion_labels=len(records),
        labels_with_admitted_spatial_increment=int(changed.sum()),
        mean_absolute_normalizer_change=float(np.mean(np.abs(log1-records[:,10]))),
        mean_absolute_existence_change=float(np.mean(np.abs(new_r-records[:,9]))),
        summaries=summary,frames=rows,native_sha256=sha(path),audit_sha256=sha(auditpath),source_sha256=sha(Path(__file__)))
    destination=OUT/'SCALAR_CASE_SUBSTITUTION.json';assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('VERIFIED FIXED-INPUT SCALAR CASE',json.dumps(summary),flush=True)

if __name__=='__main__':main()
