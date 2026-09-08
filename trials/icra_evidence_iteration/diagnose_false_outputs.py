"""Post-hoc development diagnosis only; no signal is fed into a tracker."""
from pathlib import Path
import gzip,hashlib,json
import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent;OLD=OUT.parent/'icra_external_fusion'


def distance(points,reference):
    if not len(reference):return np.full(len(points),np.inf)
    return np.sqrt(((points[:,None,:]-reference[None,:,:])**2).sum(-1).min(1))


def main():
    summary=json.loads((OLD/'summary_v2v4real.json').read_text());rows=[];examples=[]
    expected={(r['sequence'],r['condition']):r['result_sha256'] for r in summary['inputs']}
    for seq in range(9):
        name=f'{seq:04d}';mat=loadmat(OLD/'data'/f'v2v4real_{name}.mat');T=int(mat['T'].item())
        for condition in ['reliable','intermittent']:
            path=OLD/'results_v2v'/f'{name}_{condition}.json.gz'
            assert hashlib.sha256(path.read_bytes()).hexdigest()==expected[name,condition]
            with gzip.open(path,'rt') as stream:data=json.load(stream)
            for run in data['runs']:
                if run['arm'] not in ['lineage','qualified_exist']:continue
                counts=dict(false=0,false_gt_empty=0,false_near_gt_3m=0,false_near_measurement_3m=0,
                            false_neither_3m=0,matched=0,matched_near_measurement_3m=0)
                for t in range(T):
                    gt=np.asarray(data['truth'][t]).reshape(4,-1)[:2].T
                    measurements=np.concatenate([mat['measurements'][n,t].T for n in range(2)])
                    for n in range(2):
                        y=np.asarray(run['estimates'][n+2*t]).reshape(-1,4)[:,:2]
                        cost=((gt[:,None,:]-y[None,:,:])**2).sum(-1)
                        rr,cc=linear_sum_assignment(np.minimum(cost,144));valid=cost[rr,cc]<144
                        matched=np.zeros(len(y),bool);matched[cc[valid]]=True
                        d_gt=distance(y,gt);d_z=distance(y,measurements)
                        false=~matched;counts['false']+=int(false.sum());counts['matched']+=int(matched.sum())
                        counts['false_gt_empty']+=int(false.sum()) if not len(gt) else 0
                        counts['false_near_gt_3m']+=int((false&(d_gt<=3)).sum())
                        counts['false_near_measurement_3m']+=int((false&(d_z<=3)).sum())
                        counts['false_neither_3m']+=int((false&(d_z>3)&(d_gt>3)).sum())
                        counts['matched_near_measurement_3m']+=int((matched&(d_z<=3)).sum())
                        if false.any() and len(examples)<500:
                            examples.append(dict(sequence=name,condition=condition,arm=run['arm'],frame=t,node=n,
                                                 truth=gt.tolist(),measurements=measurements.tolist(),false_outputs=y[false].tolist(),
                                                 false_measurement_distances=[float(v) if np.isfinite(v) else None for v in d_z[false]],
                                                 false_gt_distances=[float(v) if np.isfinite(v) else None for v in d_gt[false]]))
                rows.append(dict(sequence=name,condition=condition,arm=run['arm'],frames=T,**counts))
    result=dict(scope='Post-hoc diagnosis of already-seen development outputs; 3m bins are descriptive, not a tuned decision rule.',
                runs=rows,first_500_false_node_frames=examples)
    (OUT/'false_output_diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for condition in ['reliable','intermittent']:
        for arm in ['lineage','qualified_exist']:
            group=[r for r in rows if r['condition']==condition and r['arm']==arm]
            counts={k:sum(r[k] for r in group) for k in group[0] if k.startswith(('false','matched'))}
            print(condition,arm,counts)
    print('FALSE OUTPUT DIAGNOSIS COMPLETE: all nine sequences; original result hashes checked.')


if __name__=='__main__':main()
