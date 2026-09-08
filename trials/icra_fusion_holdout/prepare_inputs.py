"""Prepare every holdout frame and frozen optional marks, never tracking outputs."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from scipy.io import savemat
from scipy.special import expit
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from prepare_v2v4real import in_domain
AUTH=ROOT/'tmp/external_baselines/DMSTrack'
CACHE=ROOT/'tmp/external_baselines/v2v_transfer_transforms'
SELECT=[s for s in range(32) if s not in range(0,32,5)]


def main():
    transforms=json.loads((OUT/'transform_manifest.json').read_text())
    lengths=transforms['complete_train_lengths'];offsets=np.r_[0,np.cumsum(lengths)]
    assert transforms['selected_sequences']==SELECT and transforms['frames']==5601
    hashes={(r['sensor'],r['frame']):r['sha256'] for r in transforms['source_files']}
    calpath=OUT.parent/'icra_ceiling_iteration/calibration.json';cal=json.loads(calpath.read_text())['full_seen_fit']
    priorpath=OUT.parent/'icra_marked_iteration/likelihood_manifest.json';prior=json.loads(priorpath.read_text())['full_seen_positive_prior']
    assert cal['training_sequences']==[f'{s:04d}' for s in range(9)] and 0<prior<1
    overlap=OUT.parent/'icra_method_iteration/transfer_overlap_audit.json'
    dest=OUT/'data';dest.mkdir(exist_ok=True);records=[];sources={}
    for seq in SELECT:
        name=f'{seq:04d}';T=lengths[seq];pose=np.zeros((T,2,2))
        for t,frame in enumerate(range(offsets[seq],offsets[seq+1])):
            for n,sensor in enumerate(['ego','1']):
                path=CACHE/sensor/f'{frame:04d}_transformation_matrix.npy'
                assert hashlib.sha256(path.read_bytes()).hexdigest()==hashes[sensor,frame]
                matrix=np.load(path,allow_pickle=False);assert np.allclose(matrix[3],[0,0,0,1],atol=1e-5)
                if sensor=='ego':assert np.allclose(matrix,np.eye(4),atol=1e-5)
                pose[t,n]=matrix[:2,3]
        detections=[]
        for sensor in ['ego','1']:
            path=AUTH/f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_train/{sensor}/{name}.txt'
            raw=np.loadtxt(path,delimiter=',',ndmin=2) if path.stat().st_size else np.empty((0,15))
            assert raw.shape[1]==15 and np.isfinite(raw).all() and np.all(raw[:,1]==2)
            assert np.all((raw[:,6]>=.199999)&(raw[:,6]<=1))
            assert np.all((raw[:,0]>=0)&(raw[:,0]<T)&(raw[:,0]==np.floor(raw[:,0])))
            detections.append(raw);sources[str(path.relative_to(AUTH))]=hashlib.sha256(path.read_bytes()).hexdigest()
        path=AUTH/f'AB3DMOT/scripts/KITTI/v2v4real_train_label/{name}.txt'
        gt=[line.split() for line in path.read_text().splitlines()]
        assert all(len(row)==17 and row[2]=='Car' for row in gt)
        sources[str(path.relative_to(AUTH))]=hashlib.sha256(path.read_bytes()).hexdigest()
        labels=np.array([[int(r[0]),int(r[1]),float(r[13]),float(r[15])] for r in gt],float).reshape(-1,4)
        assert np.isfinite(labels).all() and np.all((labels[:,0]>=0)&(labels[:,0]<T))
        measurements=np.empty((2,T),object);truth=np.empty((1,T),object);ids=np.empty((1,T),object)
        rawscores=np.empty((2,T),object);calibrated=np.empty((2,T),object);ratios=np.empty((2,T),object)
        counts=[];dcounts=[]
        for t in range(T):
            for n,d in enumerate(detections):
                frame=d[d[:,0]==t];xy=frame[:,[10,12]]
                mask=in_domain(xy,pose[t])&(((xy-pose[t,n])**2).sum(1)<=1600)
                measurements[n,t]=xy[mask].T;score=frame[mask,6].reshape(1,-1)
                bounded=np.clip(score,1e-6,1-1e-6)
                probability=expit(cal['a']*(np.log(bounded)-np.log1p(-bounded))+cal['b'])
                rawscores[n,t]=score;calibrated[n,t]=probability
                probability=np.clip(probability,1e-6,1-1e-6)
                ratios[n,t]=probability/(1-probability)*(1-prior)/prior
            g=labels[labels[:,0]==t];assert len(set(g[:,1]))==len(g)
            g=g[in_domain(g[:,2:4],pose[t])]
            truth[0,t]=np.vstack([g[:,2:4].T,np.zeros((2,len(g)))]);ids[0,t]=g[:,1].astype(int)[None,:]
            counts.append(len(g));dcounts.append([measurements[n,t].shape[1] for n in range(2)])
        data=dict(name=name,T=T,dt=.1,N=2,measurements=measurements,positions=pose.transpose(2,1,0),
                  time=np.arange(T)*.1,truth=truth,truthIds=ids,rawScores=rawscores,
                  calibratedScores=calibrated,likelihoodRatios=ratios)
        path=dest/f'v2v4real_{name}.mat';savemat(path,data,do_compression=True)
        with path.open('r+b') as f:f.write(b'MATLAB 5.0 MAT-file, deterministic remaining-cohort V2V4Real input'.ljust(116,b' '))
        records.append(dict(sequence=name,frames=T,role='fusion_selection_holdout',ground_truth_rows=sum(counts),
                            detection_rows=np.array(dcounts).sum(0).tolist(),input_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        print('PREPARED',name,T,'frames',flush=True)
    manifest=dict(protocol='remaining-v2v4real-fusion-selection-v1',selected_sequences=SELECT,
                  frames=sum(r['frames'] for r in records),sequences=records,source_files=sources,
                  source_commit='d3b9949499c8e68ea33060873bd1cb95b6d4d323',
                  transforms_manifest_sha256=hashlib.sha256((OUT/'transform_manifest.json').read_bytes()).hexdigest(),
                  overlap_audit_sha256=hashlib.sha256(overlap.read_bytes()).hexdigest(),
                  calibration_sha256=hashlib.sha256(calpath.read_bytes()).hexdigest(),
                  likelihood_prior_manifest_sha256=hashlib.sha256(priorpath.read_bytes()).hexdigest(),
                  limitation='Unused fusion-selection outcomes only; released detector was trained on this split and routes may be related.')
    assert manifest['frames']==5601 and len(records)==25
    (OUT/'input_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('ALL HOLDOUT INPUTS PREPARED',manifest['frames'],'frames;',len(hashes),'pose hashes checked; no tracking outcomes.')


if __name__=='__main__':main()
