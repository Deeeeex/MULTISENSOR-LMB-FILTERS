"""Prepare every frame of the reserved, ID-selected real-data cohort."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from scipy.io import savemat
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from prepare_v2v4real import in_domain
AUTH=ROOT/'tmp/external_baselines/DMSTrack'
CACHE=ROOT/'tmp/external_baselines/v2v_transfer_transforms'

def main():
    transforms=json.loads((OUT/'transfer_transform_manifest.json').read_text())
    lengths=transforms['complete_train_lengths'];offsets=np.r_[0,np.cumsum(lengths)]
    selected=transforms['selected_sequences'];assert selected==list(range(0,32,5))
    hashes={(r['sensor'],r['frame']):r['sha256'] for r in transforms['source_files']}
    dest=OUT/'data_transfer';dest.mkdir(exist_ok=True);records=[];sources={}
    for seq in selected:
        name=f'{seq:04d}';T=lengths[seq];pose=np.zeros((T,2,2))
        for t,frame in enumerate(range(offsets[seq],offsets[seq+1])):
            for n,sensor in enumerate(['ego','1']):
                p=CACHE/sensor/f'{frame:04d}_transformation_matrix.npy'
                assert hashlib.sha256(p.read_bytes()).hexdigest()==hashes[sensor,frame]
                matrix=np.load(p,allow_pickle=False);assert np.allclose(matrix[3],[0,0,0,1],atol=1e-5)
                if sensor=='ego':assert np.allclose(matrix,np.eye(4),atol=1e-5)
                pose[t,n]=matrix[:2,3]
        detections=[]
        for sensor in ['ego','1']:
            p=AUTH/f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_train/{sensor}/{name}.txt'
            d=np.loadtxt(p,delimiter=',',ndmin=2)
            assert d.shape[1]==15 and np.isfinite(d).all() and np.all(d[:,1]==2) and np.all(d[:,6]>=.199999)
            assert np.all((d[:,0]>=0)&(d[:,0]<T)&(d[:,0]==np.floor(d[:,0])))
            detections.append(d);sources[str(p.relative_to(AUTH))]=hashlib.sha256(p.read_bytes()).hexdigest()
        p=AUTH/f'AB3DMOT/scripts/KITTI/v2v4real_train_label/{name}.txt'
        gt=[line.split() for line in p.read_text().splitlines()]
        assert all(len(row)==17 and row[2]=='Car' for row in gt)
        sources[str(p.relative_to(AUTH))]=hashlib.sha256(p.read_bytes()).hexdigest()
        labels=np.array([[int(r[0]),int(r[1]),float(r[13]),float(r[15])] for r in gt])
        assert np.isfinite(labels).all() and np.all((labels[:,0]>=0)&(labels[:,0]<T))
        measurements=np.empty((2,T),object);truth=np.empty((1,T),object);ids=np.empty((1,T),object)
        counts=[];dcounts=[]
        for t in range(T):
            for n,d in enumerate(detections):
                xy=d[d[:,0]==t][:,[10,12]];mask=in_domain(xy,pose[t])&(((xy-pose[t,n])**2).sum(1)<=1600)
                measurements[n,t]=xy[mask].T
            g=labels[labels[:,0]==t];assert len(set(g[:,1]))==len(g)
            g=g[in_domain(g[:,2:4],pose[t])]
            truth[0,t]=np.vstack([g[:,2:4].T,np.zeros((2,len(g)))]);ids[0,t]=g[:,1].astype(int)[None,:]
            counts.append(len(g));dcounts.append([measurements[n,t].shape[1] for n in range(2)])
        data=dict(name=name,T=T,dt=.1,N=2,measurements=measurements,positions=pose.transpose(2,1,0),
                  time=np.arange(T)*.1,truth=truth,truthIds=ids)
        p=dest/f'v2v4real_{name}.mat';savemat(p,data,do_compression=True)
        with p.open('r+b') as f:f.write(b'MATLAB 5.0 MAT-file, deterministic reserved V2V4Real transfer input'.ljust(116,b' '))
        records.append(dict(sequence=name,frames=T,role='overlap_control' if seq==0 else 'reserved',ground_truth_rows=sum(counts),
                            detection_rows=np.array(dcounts).sum(0).tolist(),input_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    manifest=dict(protocol='reserved-v2v4real-algorithm-transfer-v1',selected_sequences=selected,
                  frames=sum(r['frames'] for r in records),sequences=records,source_files=sources,
                  source_commit='d3b9949499c8e68ea33060873bd1cb95b6d4d323',
                  transforms_manifest_sha256=hashlib.sha256((OUT/'transfer_transform_manifest.json').read_bytes()).hexdigest(),
                  overlap_audit_sha256=hashlib.sha256((OUT/'transfer_overlap_audit.json').read_bytes()).hexdigest(),
                  primary_reserved_sequences=[5,10,15,20,25,30],primary_reserved_frames=1357,
                  limitation='Original detector train split; sequence 0000 overlaps development and is a separate control; related routes possible.')
    assert manifest['frames']==1504
    (OUT/'transfer_input_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('RESERVED INPUTS PREPARED',manifest['frames'],'frames;',len(hashes),'pose hashes checked; no tracking outcomes.')

if __name__=='__main__':main()
