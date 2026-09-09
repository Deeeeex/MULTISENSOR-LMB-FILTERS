"""Reconstruct MAP extraction and count every target frame with fixed 2 m matching."""
from collections import defaultdict
import numpy as np
from scipy.optimize import linear_sum_assignment


def summarize(data,mat):
    run=data['runs'];T=len(data['time']);group=defaultdict(list);increments=defaultdict(list)
    inc=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    lookup={tuple(row[:4].astype(int)):row for row in inc}
    for row in inc:increments[int(row[0]),int(row[1])].append(row)
    for row in np.asarray(run['localGaussianRecords'],float).reshape(-1,32):
        group['local',int(row[0]),int(row[1])].append(np.r_[row[:4],lookup[tuple(row[:4].astype(int))][5],row[18:32]])
    for row in np.asarray(run['fusionOutputRecords'],float).reshape(-1,19):group['fusion',int(row[0]),int(row[1])].append(row)
    frames=[]
    for t in range(1,T+1):
        truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T
        target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[target,:2]
        for n in [1,2]:
            i=n-1+2*(t-1);phase='fusion' if data['delivered'][n-1][2-n][t-1] else 'local'
            pool=np.asarray(group[phase,t,n]).reshape(-1,19);pool=pool[pool[:,4]>.001]
            rho=np.array([1.])
            for r in pool[:,4]-1e-6:rho=np.convolve(rho,[1-r,r])
            count=int(np.argmax(rho));selected=pool[np.argsort(-pool[:,4],kind='stable')[:count]]
            raw=np.asarray(run['rawEstimates'][i],float).reshape(-1,4)
            labels=np.asarray(run['labels'][i]).reshape(2,-1).T
            assert len(raw)==count and np.array_equal(labels,selected[:,2:4]),('MAP labels',t,n)
            assert np.allclose(raw,selected[:,5:9],atol=1e-13,rtol=0),('MAP states',t,n)
            estimates=np.asarray(run['estimates'][i],float).reshape(-1,4)
            distance=np.sqrt(((truth[:,None,:2]-estimates[None,:,:2])**2).sum(2))
            ti,ei=linear_sum_assignment(np.where(distance<=2,distance,1e6))
            detected=any(a==target and distance[a,b]<=2 for a,b in zip(ti,ei))
            near=pool[np.sum((pool[:,5:7]-xy)**2,axis=1)<=4]
            row=dict(frame=t,robot=n,detected=bool(detected),output_count=len(estimates),active_components=len(pool),
                near_active_components=len(near),near_total_r=float(near[:,4].sum()),near_max_r=float(near[:,4].max(initial=0)),
                association_has_column=False,association_effective_labels=None,association_largest_share=None,
                association_nearest_distance_m=None,association_joint_mass=None)
            local=np.asarray(increments[t,n]).reshape(-1,12);z=np.asarray(mat['measurements'][n-1,t-1],float).reshape(2,-1)
            if len(local) and z.shape[1]:
                column=int(np.argmin(np.sum((z-xy[:,None])**2,axis=0)))
                W=np.asarray(run['localAssociationWeights'][i],float).reshape(len(local),z.shape[1]+1)
                mass=local[:,5]*W[:,column+1];total=float(mass.sum());p=mass/total if total else np.zeros_like(mass)
                positive=p[p>0];entropy=float(-np.sum(positive*np.log(positive)))
                row.update(association_has_column=True,association_effective_labels=float(np.exp(entropy)),
                    association_largest_share=float(p.max(initial=0)),association_nearest_distance_m=float(np.linalg.norm(z[:,column]-xy)),
                    association_joint_mass=total)
            frames.append(row)
    windows={}
    for name,left,right in [('full',1,T),('before_event',1,2),('after_event',3,T),('original_window',53,122)]:
        selected=[r for r in frames if left<=r['frame']<=right]
        windows[name]=dict(detected=sum(r['detected'] for r in selected),robot_frames=len(selected),
            by_robot=[sum(r['detected'] for r in selected if r['robot']==n) for n in [1,2]],
            maximum_near_components=max(r['near_active_components'] for r in selected),
            maximum_components=max(r['active_components'] for r in selected))
    return dict(map_extractions_checked=len(frames),windows=windows,frame53=[r for r in frames if r['frame']==53]),frames
