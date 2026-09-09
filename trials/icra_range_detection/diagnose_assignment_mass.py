"""Inspect conditional versus joint association mass at the fixed frame-53 snapshot."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_v2x_gce_diagnosis'))
from diagnose_gap import read
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    cfg=json.loads((OUT/'stages/range_detection_preflight.json').read_text());unit=cfg['units'][1]
    assert unit['sequence']=='v2xt_0001';mat=loadmat(ROOT/unit['data_path'])
    audit=json.loads((OUT/'audit_range_detection_preflight.json').read_text());assert audit['passed']
    t=53;rows=[];sources={}
    for arm in ['marked_gaussian_evidence_range','marked_lineage_range','marked_gaussian_evidence_guarded_scalar_range']:
        path=OUT/'results/range_detection_preflight'/f'v2xt_0001_intermittent_{arm}.json.gz'
        expected=audit['inputs'][str(path.relative_to(ROOT))];assert sha(path)==expected;sources[str(path.relative_to(ROOT))]=expected
        data=read(path);run=data['runs'];alllocal=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
        index=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item()
        truth=np.asarray(data['truth'][t-1]).reshape(4,-1)[:2,index]
        for n in [1,2]:
            local=alllocal[(alllocal[:,0]==t)&(alllocal[:,1]==n)];z=np.asarray(mat['measurements'][n-1,t-1],float).reshape(2,-1)
            raw=np.asarray(run['localAssociationWeights'][n-1+2*(t-1)],float)
            row=dict(arm=arm,source=n,frame=t,predicted_components=len(local),detections=z.shape[1])
            if z.shape[1]==0:
                assert not raw.size and not local[:,8:10].any()
                row['interpretation']='No current detections; conditional detection mass is zero.'
            else:
                W=raw.reshape(len(local),z.shape[1]+1)
                assert np.isfinite(W).all() and (W>=0).all() and np.allclose(W.sum(1),1,atol=1e-12,rtol=0)
                distances=np.sqrt(np.sum((z-truth[:,None])**2,axis=0));column=int(np.argmin(distances))
                conditional=W[:,column+1];joint=local[:,5]*conditional
                mark=float(max(0,np.tanh(.5*np.log(mat['likelihoodRatios'][n-1,t-1].ravel()[column]))))
                high=conditional>.5
                row.update(closest_detection_column_1based=column+1,closest_detection_distance_m=float(distances[column]),
                    conditional_mass=float(conditional.sum()),joint_mass=float(joint.sum()),mark=mark,
                    conditional_mark_mass=float(mark*conditional.sum()),joint_mark_mass=float(mark*joint.sum()),
                    high_conditional_tracks=int(high.sum()),positive_delta_high_conditional_tracks=int((high&(local[:,6]>0)).sum()),
                    largest_joint_mass_per_track=float(joint.max()),
                    all_detection_joint_column_sum_max=float(np.max(np.sum(local[:,5,None]*W[:,1:],axis=0))))
            rows.append(row);print(json.dumps(row),flush=True)
    result=dict(passed=True,scope='Descriptive snapshot only; no change to conditional association definition, admission rule, or current screen',
        note='W is conditional on a label existing. r_post times W gives its approximate joint association mass. A large conditional column sum alone is not a probability violation and does not prove that changing the gate would improve recursion.',
        rows=rows,inputs=sources,data_sha256=sha(ROOT/unit['data_path']),source_sha256=sha(Path(__file__)))
    destination=OUT/'ASSIGNMENT_MASS_DIAGNOSTIC.json';assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
