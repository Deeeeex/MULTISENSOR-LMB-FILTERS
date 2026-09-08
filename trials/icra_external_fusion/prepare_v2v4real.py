"""Convert released real detections/labels; keep truth outside tracker input."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.io import savemat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
AUTH=ROOT/'tmp/external_baselines/DMSTrack'
CACHE=ROOT/'tmp/external_baselines/v2v_transforms'
LENGTHS=[147,114,144,198,180,310,304,221,375]


def in_domain(xy,positions):
    xy=np.asarray(xy).reshape(-1,2)
    rectangle=(np.abs(xy[:,0])<=70.4)&(np.abs(xy[:,1])<=40)
    nearest=((xy[:,None,:]-positions[None,:,:])**2).sum(-1).min(1)
    return rectangle & (nearest<=40**2) & (nearest>3**2)


def main():
    matrices=np.load(CACHE/'transforms.npz',allow_pickle=False)
    assert np.allclose(matrices['ego'],np.eye(4),atol=1e-5)
    inputs=OUT/'data'; inputs.mkdir(exist_ok=True)
    seqmap=AUTH/'AB3DMOT/scripts/KITTI/v2v4real_val_evaluate_tracking.seqmap.val'
    rows=[line.split() for line in seqmap.read_text().splitlines()]
    assert [int(r[3])+1 for r in rows]==LENGTHS
    records=[]; source_files={str(seqmap.relative_to(AUTH)):hashlib.sha256(seqmap.read_bytes()).hexdigest()}
    offset=0
    for index,T in enumerate(LENGTHS):
        name=f'{index:04d}'; transforms=matrices['1'][offset:offset+T]
        positions=np.zeros((T,2,2)); positions[:,1,:]=transforms[:,:2,3]
        assert np.isfinite(positions).all()
        detections=[]
        for sensor in ['ego','1']:
            p=AUTH/f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_val/{sensor}/{name}.txt'
            d=np.loadtxt(p,delimiter=',',ndmin=2)
            assert d.shape[1]==15 and np.isfinite(d).all() and np.all(d[:,1]==2)
            assert np.all((d[:,0]>=0)&(d[:,0]<T)&(d[:,0]==np.floor(d[:,0])))
            assert np.all(d[:,6]>=.199999)
            detections.append(d);source_files[str(p.relative_to(AUTH))]=hashlib.sha256(p.read_bytes()).hexdigest()
        label_path=AUTH/f'AB3DMOT/scripts/KITTI/v2v4real_val_label/{name}.txt'
        gt=[line.split() for line in label_path.read_text().splitlines()]
        assert all(len(row)==17 and row[2]=='Car' for row in gt)
        source_files[str(label_path.relative_to(AUTH))]=hashlib.sha256(label_path.read_bytes()).hexdigest()
        labels=np.array([[int(r[0]),int(r[1]),float(r[13]),float(r[15])] for r in gt])
        assert np.isfinite(labels).all() and np.all((labels[:,0]>=0)&(labels[:,0]<T))
        measurements=np.empty((2,T),dtype=object); truth=np.empty((1,T),dtype=object)
        ids=np.empty((1,T),dtype=object); counts=[]; detection_counts=[]
        for t in range(T):
            for n,d in enumerate(detections):
                # Columns 10,12 are planar x,y after the release's y/z swap.
                xy=d[d[:,0]==t][:,[10,12]]
                mask=(((xy-positions[t,n])**2).sum(1)<=40**2)&in_domain(xy,positions[t])
                measurements[n,t]=xy[mask].T
            g=labels[labels[:,0]==t]; assert len(set(g[:,1]))==len(g)
            mask=in_domain(g[:,2:4],positions[t]); selected=g[mask]
            truth[0,t]=np.vstack([selected[:,2:4].T,np.zeros((2,len(selected)))])
            ids[0,t]=selected[:,1].astype(int)[None,:]
            counts.append(int(mask.sum()));detection_counts.append([measurements[n,t].shape[1] for n in range(2)])
        data=dict(name=name,T=T,dt=.1,N=2,measurements=measurements,positions=positions.transpose(2,1,0),
                  time=np.arange(T)*.1,truth=truth,truthIds=ids)
        # Tracking receives the explicit measurement/model fields; ground
        # truth is read only by the runner's score/output stage.
        path=inputs/f'v2v4real_{name}.mat';savemat(path,data,do_compression=True)
        with path.open('r+b') as stream:
            stream.write(b'MATLAB 5.0 MAT-file, deterministic V2V4Real real-detection replay input'.ljust(116,b' '))
        records.append(dict(sequence=name,frames=T,ground_truth_rows=sum(counts),
                            target_count_min=min(counts),target_count_max=max(counts),
                            detection_rows=np.asarray(detection_counts).sum(0).tolist(),
                            relative_vehicle_range_min=float(np.linalg.norm(positions[:,1],axis=1).min()),
                            relative_vehicle_range_max=float(np.linalg.norm(positions[:,1],axis=1).max()),
                            input_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        offset+=T
    report=dict(protocol='v2v4real-2d-real-detection-replay-v1',sequences=records,frames=offset,
                source_commit='d3b9949499c8e68ea33060873bd1cb95b6d4d323',source_files=source_files,
                transforms_manifest_sha256=hashlib.sha256((CACHE/'manifest.json').read_bytes()).hexdigest(),
                coordinate_frame='current ego LiDAR; no ego-motion compensation; planar x/y',
                measurement_source='released no-fusion neural detections, not generated from truth',
                ego_identity_checked=True,selection='all nine released evaluation sequences')
    (OUT/'v2v4real_input_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    for record in records:print(record)
    print('INPUT CHECK PASSED',offset,'paired real-data frames; no tracking run.')


if __name__=='__main__':main()
