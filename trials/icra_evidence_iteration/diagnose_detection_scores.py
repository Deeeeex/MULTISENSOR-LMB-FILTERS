"""Diagnose released detector scores on seen data, without changing inputs."""
from pathlib import Path
import csv,hashlib,json,sys
import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];OLD=OUT.parent/'icra_external_fusion'
sys.path.insert(0,str(OLD));from prepare_v2v4real import in_domain


def main():
    manifest=json.loads((OLD/'v2v4real_input_manifest.json').read_text());rows=[]
    auth=ROOT/'tmp/external_baselines/DMSTrack'
    for seq in manifest['sequences']:
        name=seq['sequence'];T=seq['frames'];data=loadmat(OLD/'data'/f'v2v4real_{name}.mat')
        poses=data['positions'].transpose(2,1,0)
        for n,sensor in enumerate(['ego','1']):
            path=auth/f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_val/{sensor}/{name}.txt'
            assert hashlib.sha256(path.read_bytes()).hexdigest()==manifest['source_files'][str(path.relative_to(auth))]
            raw=np.loadtxt(path,delimiter=',',ndmin=2)
            for t in range(T):
                d=raw[raw[:,0]==t];xy=d[:,[10,12]]
                keep=in_domain(xy,poses[t])&(((xy-poses[t,n])**2).sum(1)<=1600);d=d[keep];xy=xy[keep]
                assert np.array_equal(xy,data['measurements'][n,t].T)
                truth=data['truth'][0,t][:2].T;cost=((truth[:,None,:]-xy[None,:,:])**2).sum(-1)
                matched={}
                for threshold in [3,12]:
                    rr,cc=linear_sum_assignment(np.minimum(cost,threshold**2));mask=np.zeros(len(d),bool)
                    mask[cc[cost[rr,cc]<threshold**2]]=True;matched[threshold]=mask
                for j in range(len(d)):
                    rows.append(dict(sequence=name,sensor=sensor,frame=t,score=float(d[j,6]),
                                     distance_sensor=float(np.linalg.norm(xy[j]-poses[t,n])),
                                     positive_3m=int(matched[3][j]),positive_12m=int(matched[12][j])))
    with (OUT/'detection_score_diagnosis.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    score=np.array([r['score'] for r in rows]);tables=[]
    for threshold in [3,12]:
        y=np.array([r[f'positive_{threshold}m'] for r in rows])
        for a,b in zip([.2,.3,.4,.5,.6,.7,.8,.9],[.3,.4,.5,.6,.7,.8,.9,1.00001]):
            mask=(score>=a)&(score<b)
            tables.append(dict(assignment_cutoff_m=threshold,score_low=a,score_high=min(b,1.),n=int(mask.sum()),
                               positive_fraction=float(y[mask].mean()) if mask.any() else None))
        print('cutoff',threshold,'n',len(rows),'positive',int(y.sum()),'score median true/false',np.median(score[y==1]),np.median(score[y==0]))
    (OUT/'detection_score_diagnosis.json').write_text(json.dumps(dict(scope='Seen development detections only; diagnostics do not define a fitted tracker.',bins=tables),indent=2)+'\n')
    for row in tables:print(row)


if __name__=='__main__':main()
