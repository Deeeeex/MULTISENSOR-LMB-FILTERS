"""Trace the already exposed label, with single-label scalar substitutions only."""
from collections import defaultdict
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
PRIOR=OUT.parent/'icra_prune_information'
LABEL=(3,100004)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def key(r):return tuple(map(int,r[:4]))
def logit(x):return math.log(x)-math.log1p(-x)

def extraction(pool,poses,replace=None):
    pool=np.asarray(pool,float).reshape(-1,19).copy()
    if replace is not None:
        mask=(pool[:,2:4]==LABEL).all(1);assert mask.sum()==1;pool[mask,4]=replace
    pool=pool[pool[:,4]>.001];pmf=np.array([1.]);p=pool[:,4]-1e-6
    for x in p:pmf=np.convolve(pmf,[1-x,x])
    count=int(pmf.argmax());chosen=pool[np.argsort(-p,kind='stable')[:count]]
    raw=chosen.copy();d2=((chosen[:,None,5:7]-poses.T[None,:,:])**2).sum(-1).min(1)
    inside=(abs(chosen[:,5])<=70.4)&(abs(chosen[:,6])<=40)&(d2<=1600)&(d2>9)
    return raw,chosen[inside]

def detected(data,t,chosen):
    truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T[:,:2]
    target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item()
    d=np.sqrt(((truth[:,None,:]-chosen[None,:,5:7])**2).sum(-1))
    a,b=linear_sum_assignment(np.where(d<=2,d,1e6))
    return any(i==target and d[i,j]<=2 for i,j in zip(a,b)),truth[target]

def main():
    destination=OUT/'MOTIVATING_TRACE.json';assert not destination.exists()
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    prior=json.loads((PRIOR/'RESULTS.json').read_text());scores={(d['condition'],d['arm']):d for d in prior['diagnostics']}
    config=json.loads((PRIOR/'stages/prune_info_controls.json').read_text());rows=[];snapshots=[];summaries=[];inputs={}
    for u in config['units']:
        condition=u['execution_id']
        for mode in ['original','local','shared']:
            for base in ['marked_gaussian_evidence','marked_lineage']:
                arm=base+('_known_censor' if mode=='local' else '_prune_info' if mode=='shared' else '')
                path=ROOT/u['original_references'][base]['path'] if mode=='original' else PRIOR/'results'/('prune_info_controls' if mode=='local' else 'prune_info_shared')/f"{u['sequence']}_{condition}_{arm}.json.gz"
                with gzip.open(path,'rt') as stream:data=json.load(stream)
                assert sha(path)==cfg['motivation_inputs'][str(path.relative_to(ROOT))];inputs[str(path.relative_to(ROOT))]=sha(path)
                run=data['runs'];inc={key(r):r for r in run['localIncrementRecords']};records={key(r):r for r in run['iterationRecords']}
                sources={key(r):r for r in run['fusionSourceRecords']};events={key(r):r for r in run.get('knownCensorRecords',[])+run.get('pruneInformationRecords',[])}
                fused=defaultdict(list);local=defaultdict(list)
                for r in run['fusionOutputRecords']:fused[tuple(map(int,r[:2]))].append(r)
                for r in run['localGaussianRecords']:local[tuple(map(int,r[:2]))].append([*r[:4],inc[key(r)][5],*r[18:32]])
                cell_rows=[]
                for t in range(1,241):
                    poses=np.asarray(data['positions'])[:,:,t-1]
                    for n in [1,2]:
                        received=bool(data['delivered'][n-1][2-n][t-1]);pool=fused[t,n] if received else local[t,n]
                        raw,chosen=extraction(pool,poses);actual=np.asarray(run['estimates'][n-1+2*(t-1)],float).reshape(-1,4)
                        assert len(chosen)==len(actual) and np.allclose(chosen[:,5:9],actual,atol=1e-12,rtol=0)
                        saved_labels=np.asarray(run['labels'][n-1+2*(t-1)]).reshape(2,-1).T
                        assert np.array_equal(raw[:,2:4],saved_labels)
                        found,xy=detected(data,t,chosen);k=(t,n,*LABEL);r=records.get(k);root=next((v for v in pool if tuple(v[2:4])==LABEL),None)
                        row=dict(condition=condition,mode=mode,backend='GCE' if 'gaussian' in base else 'No-age',frame=t,robot=n,
                            received=received,root_present=root is not None,root_active=root is not None and root[4]>.001,
                            root_near_target=root is not None and math.hypot(root[5]-xy[0],root[6]-xy[1])<=2,
                            root_r=None if root is None else root[4],target_detected=found,output_count=len(chosen),
                            root_extracted=any(tuple(x[2:4])==LABEL for x in chosen),extra_negative=0.,extra_positive=0.,history=0.,
                            base_r=None,base_I=None,integral_change=None,own_r=None,peer_r=None,own_a=None,peer_a=None,
                            own_positive_gate=None,peer_positive_gate=None,both_curvature_allowed=None,censor_shift=0.,
                            without_negative_r=None,without_negative_root_extracted=None,without_negative_target_detected=None)
                        if r is not None:
                            r=np.asarray(r,float);s=sources[k];b=r[13:15];beta=r[28:30] if 'gaussian' in base else b
                            logits=np.zeros(2);active=b>0;rr=np.clip(r[17:19][active],1e-9,1-1e-9);logits[active]=np.log(rr)-np.log1p(-rr)
                            negative=float(np.dot(r[52:54],np.minimum(r[19:21],0))) if 'gaussian' in base else 0.
                            positive=float(np.dot(r[52:54],np.maximum(r[19:21],0))) if 'gaussian' in base else 0.
                            integral=float(r[56] if 'gaussian' in base else r[10]);censor=events.get(k)
                            shift=logit(censor[5])-logit(censor[4]) if censor is not None else 0.
                            total=float(np.dot(beta,logits))+positive+negative+integral+shift
                            assert abs(expit(total)-root[4])<2e-10
                            without=float(expit(total-negative)) if negative<0 else root[4]
                            if 0<root[4]<1 and negative<0:assert abs(without-expit(logit(root[4])-negative))<2e-10
                            _,alternate=extraction(pool,poses,without);alternate_found,_=detected(data,t,alternate)
                            row.update(extra_negative=negative,extra_positive=positive,history=float(np.dot(beta-b,logits)),base_r=float(expit(np.dot(b,logits)+r[10])),
                                base_I=float(r[10]),integral_change=integral-float(r[10]),censor_shift=shift,without_negative_r=without,
                                without_negative_root_extracted=any(tuple(x[2:4])==LABEL for x in alternate),without_negative_target_detected=alternate_found,
                                both_curvature_allowed=bool(all(r[57:59])) if 'gaussian' in base else None)
                            for side,label in enumerate([s[4:6],s[6:8]]):
                                if label[0]<=0:continue
                                v=inc[t,n if side==0 else 3-n,*map(int,label)];name='own' if side==0 else 'peer'
                                row[name+'_r']=v[5];row[name+'_a']=v[9];row[name+'_positive_gate']=v[8]
                        cell_rows.append(row)
                        if t==53:snapshots.append(row)
                for window,left,right in [('full',1,240),('original_window',53,122)]:
                    selected=[r for r in cell_rows if left<=r['frame']<=right];near=[r for r in selected if r['root_near_target'] and r['root_active']]
                    assert sum(r['target_detected'] for r in selected)==scores[condition,arm]['target']['windows'][window]['detected']
                    summaries.append(dict(condition=condition,mode=mode,backend=cell_rows[0]['backend'],window=window,robot_frames=len(selected),
                        detected=sum(r['target_detected'] for r in selected),near_root_frames=len(near),
                        near_root_misses=sum(not r['root_extracted'] for r in near),
                        near_root_negative_extra=sum(r['extra_negative']<0 for r in near),
                        both_curvature_allowed_with_negative=sum(r['extra_negative']<0 and r['both_curvature_allowed'] is True for r in near),
                        immediate_target_rescues=sum(not r['target_detected'] and r['without_negative_target_detected'] is True for r in selected)))
                rows.extend(cell_rows);print('MOTIVATING TRACE CHECKED',condition,mode,base,480,'robot frames',flush=True)
    assert len(rows)==5760
    table=OUT/'MOTIVATING_TRACE.csv'
    with table.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    destination.write_text(json.dumps(dict(passed=True,exploratory=True,root_label=list(LABEL),anchor_frame=53,
        scope='Already inspected fixed label; one-label scalar substitutions never fed back; not selection or a recursive method',
        checked_robot_frames=len(rows),snapshots=snapshots,summaries=summaries,input_sha256=inputs,table_sha256=sha(table),
        source_sha256=sha(Path(__file__)),freeze_sha256=sha(OUT/'SCREEN_FREEZE.json')),indent=2,allow_nan=False)+'\n')
    print('MOTIVATING TRACE VERIFIED',len(rows),'robot frames',flush=True)

if __name__=='__main__':main()
