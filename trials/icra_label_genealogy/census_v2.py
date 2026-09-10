"""Exact native population ledger and diagnostic birth ancestry with bitsets."""
from collections import Counter
from datetime import datetime,timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_recursion_origin'))
from trace_v2 import prepare,rows_at,pool_at

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
THRESHOLD=.001
A=np.block([[np.eye(2),.1*np.eye(2)],[np.zeros((2,2)),np.eye(2)]])
PROCESS=25*np.block([[(.1**3/3)*np.eye(2),(.1**2/2)*np.eye(2)],[(.1**2/2)*np.eye(2),.1*np.eye(2)]])
LOWER=tuple(np.asarray(x) for x in zip(*[(r,c) for c in range(4) for r in range(c,4)]))


def covariance(v):
    p=np.zeros((4,4));p[LOWER]=v;p[LOWER[::-1]]=v
    return p


def label(row):return tuple(map(int,row[2:4]))


def labels(rows):
    result={label(row):row for row in rows};assert len(result)==len(rows)
    return result


def indices(mask):
    while mask:
        bit=mask&-mask;yield bit.bit_length()-1;mask^=bit


def ancestry_statistics(masks):
    counts=Counter(k for mask in masks for k in indices(mask))
    repeated=[k for k,count in counts.items() if count>1]
    repeated_mask=sum(1<<k for k in repeated)
    pairs=sum(bool(left&right) for i,left in enumerate(masks) for right in masks[i+1:])
    return dict(ancestry_roots=len(counts),ancestry_memberships=sum(counts.values()),
        repeated_roots=len(repeated),labels_with_shared_roots=sum(bool(m&repeated_mask) for m in masks),
        pairs_with_shared_roots=pairs,maximum_root_multiplicity=max(counts.values(),default=0))


def check_fixtures():
    assert ancestry_statistics([1,2,4])==dict(ancestry_roots=3,ancestry_memberships=3,
        repeated_roots=0,labels_with_shared_roots=0,pairs_with_shared_roots=0,maximum_root_multiplicity=1)
    assert ancestry_statistics([3,6])==dict(ancestry_roots=3,ancestry_memberships=4,
        repeated_roots=1,labels_with_shared_roots=2,pairs_with_shared_roots=1,maximum_root_multiplicity=2)
    assert ancestry_statistics([3,3,3])==dict(ancestry_roots=2,ancestry_memberships=6,
        repeated_roots=2,labels_with_shared_roots=3,pairs_with_shared_roots=3,maximum_root_multiplicity=3)
    assert ancestry_statistics([])['ancestry_roots']==0
    assert 4-2==2-1-1+2
    return dict(disjoint=True,shared_root=True,all_shared=True,empty=True,population_identity=True)


def root_roster(measurements,T):
    result=[]
    for t in range(2,T+1):
        for n in [1,2]:
            z=np.asarray(measurements[n-1,t-2],float).reshape(2,-1)
            for j in range(z.shape[1]):
                result.append(dict(birth_frame=t,sensor=n,previous_detection_index=j+1,
                    label=[t,n*100000+j+1],mean=[*map(float,z[:,j]),0.,0.]))
    return result


def pool_record(t,n,phase,pool,masks,xy):
    ordered=sorted(pool);rows=[pool[k] for k in ordered];bits=[masks[k] for k in ordered]
    near=[i for i,row in enumerate(rows) if np.sum((row[5:7]-xy)**2)<=4.]
    statistics=ancestry_statistics(bits)
    target_stats=ancestry_statistics([bits[i] for i in near])
    r=[float(row[4]) for row in rows];near_r=[r[i] for i in near]
    target=dict(frame=t,robot=n,phase=phase,total_count=len(rows),near_count=len(near),
        near_sum_r=float(sum(near_r)),near_max_r=max(near_r,default=0.),
        near_birth_labels=sum(ordered[i][0]==t for i in near),
        near_labels_from_first_three_frames=sum(ordered[i][0]<=3 for i in near),
        near_label_age_sum=sum(t-ordered[i][0] for i in near),**target_stats)
    raw=dict(frame=t,robot=n,phase=phase,
        labels=[dict(label=list(k),r=float(pool[k][4]),mean=list(map(float,pool[k][5:9])),
                     roots=hex(masks[k])) for k in ordered])
    return statistics,target,raw


def first_shared(t,n,phase,pool,masks,source_rows):
    ordered=sorted(pool)
    for i,left in enumerate(ordered):
        for right in ordered[i+1:]:
            shared=masks[left]&masks[right]
            if shared:
                chosen=[]
                for key in [left,right]:
                    source=source_rows.get(key)
                    chosen.append(dict(label=list(key),r=float(pool[key][4]),mean=list(map(float,pool[key][5:9])),
                        roots=hex(masks[key]),source_labels=None if source is None else source[4:].astype(int).tolist()))
                return dict(frame=t,robot=n,phase=phase,shared_roots=hex(shared),labels=chosen)
    return None


def census_cell(data,cell,mat,roots):
    prepared=prepare(data);T=len(data['time']);previous={1:{},2:{}};previous_masks={1:{},2:{}}
    root_index={tuple(root['label']):i for i,root in enumerate(roots)}
    births_by_time={}
    for root in roots:births_by_time.setdefault((root['birth_frame'],root['sensor']),[]).append(root)
    population=[];targets=[];pools=[];births=[];first={'local':None,'posterior':None}
    prediction_count=0;covariance_count=0;source_links=0
    for t in range(1,T+1):
        local={};local_masks={};local_diag={}
        ids=np.asarray(data['truthIds'][t-1]).ravel();target_idx=np.flatnonzero(ids==5).item()
        xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,target_idx]
        for n in [1,2]:
            inc=labels(rows_at(prepared,'inc',t,n));pred=labels(rows_at(prepared,'predicted',t,n))
            gauss=labels(rows_at(prepared,'gaussian',t,n));local[n]=labels(rows_at(prepared,'local',t,n))
            new={tuple(root['label']):root for root in births_by_time.get((t,n),[])}
            assert not previous[n].keys()&new.keys()
            assert inc.keys()==pred.keys()==previous[n].keys()|new.keys()
            assert local[n].keys()=={key for key,row in inc.items() if row[5]>THRESHOLD}==gauss.keys()
            inherited_kept=0;birth_kept=0;local_masks[n]={}
            for key,row in inc.items():
                if key in new:
                    expected_r=.01;mean=np.asarray(new[key]['mean']);p=np.diag([16.,16.,225.,225.])
                    origin=1<<root_index[key]
                    births.append(dict(frame=t,robot=n,label=list(key),prior_r=float(row[4]),
                        prior_mean=list(map(float,pred[key][5:9])),posterior_r=float(row[5]),retained=key in local[n]))
                else:
                    old=previous[n][key];expected_r=.99*old[4];mean=A@old[5:9]
                    p=A@covariance(old[9:])@A.T+PROCESS;origin=previous_masks[n][key]
                assert abs(row[4]-expected_r)<1e-12,(cell,t,n,key,'prediction existence')
                assert np.allclose(pred[key][5:9],mean,atol=1e-7,rtol=0),(cell,t,n,key,'prediction mean')
                prediction_count+=1
                if key in gauss:
                    assert np.allclose(gauss[key][4:8],mean,atol=1e-7,rtol=0)
                    assert np.allclose(covariance(gauss[key][8:18]),p,atol=1e-7,rtol=0),(cell,t,n,key,'prediction covariance')
                    covariance_count+=1;local_masks[n][key]=origin
                    if key in new:birth_kept+=1
                    else:inherited_kept+=1
            local_diag[n]=dict(previous_count=len(previous[n]),birth_count=len(new),predicted_count=len(inc),
                local_count=len(local[n]),birth_local_retained=birth_kept,inherited_local_retained=inherited_kept,
                local_pruned=len(inc)-len(local[n]),birth_local_pruned=len(new)-birth_kept,
                inherited_local_pruned=len(previous[n])-inherited_kept)
            stats,target,raw=pool_record(t,n,'local',local[n],local_masks[n],xy)
            local_diag[n].update({'local_'+k:v for k,v in stats.items()});targets.append(target);pools.append(raw)
            if first['local'] is None:first['local']=first_shared(t,n,'local',local[n],local_masks[n],{})
        current={};current_masks={}
        for n in [1,2]:
            delivered=bool(np.asarray(data['delivered'])[n-1,2-n,t-1])
            fused=labels(pool_at(prepared,t,n));source=labels(rows_at(prepared,'source_labels',t,n))
            masks={};count_self_only=0;count_matched=0;count_remote_only=0
            used={0:set(),1:set()}
            if delivered:
                assert source.keys()==fused.keys()
                for key,row in source.items():
                    parents=[tuple(x) for x in row[4:].astype(int).reshape(2,2)]
                    present=[k[0]>0 for k in parents];assert any(present)
                    origin=0
                    for side in [0,1]:
                        if not present[side]:continue
                        parent=parents[side];sensor=n if side==0 else 3-n
                        assert parent in local[sensor] and parent not in used[side]
                        used[side].add(parent);origin|=local_masks[sensor][parent];source_links+=1
                        if side==0:assert parent==key
                    masks[key]=origin
                    if all(present):count_matched+=1
                    elif present[0]:count_self_only+=1
                    else:count_remote_only+=1
            else:
                assert not source and fused.keys()==local[n].keys()
                masks=local_masks[n].copy();count_self_only=len(local[n])
            current[n]={key:row for key,row in fused.items() if row[4]>THRESHOLD}
            current_masks[n]={key:masks[key] for key in current[n]}
            assert current[n].keys()==labels(pool_at(prepared,t,n)[pool_at(prepared,t,n)[:,4]>THRESHOLD]).keys()
            self_lost=len(local[n].keys()-current[n].keys())
            remote_retained=len(current[n].keys()-local[n].keys())
            diag=dict(condition=cell['condition'],backend=cell['backend'],frame=t,robot=n,
                **local_diag[n],received=delivered,fusion_pre_count=len(fused),
                local_only_outputs=count_self_only,matched_outputs=count_matched,remote_only_outputs=count_remote_only,
                local_lost_at_fusion=self_lost,remote_only_retained=remote_retained,
                fusion_pruned=len(fused)-len(current[n]),retained_count=len(current[n]),
                population_delta=len(current[n])-len(previous[n]),
                posterior_birth_labels=sum(k[0]==t for k in current[n]),
                posterior_label_age_sum=sum(t-k[0] for k in current[n]))
            assert diag['population_delta']==diag['birth_count']-diag['local_pruned']-self_lost+remote_retained,(cell,t,n,'population balance')
            assert count_self_only+count_matched+count_remote_only==len(fused)
            stats,target,raw=pool_record(t,n,'posterior',current[n],current_masks[n],xy)
            diag.update({'posterior_'+k:v for k,v in stats.items()});population.append(diag)
            targets.append(target);pools.append(raw)
            if first['posterior'] is None:first['posterior']=first_shared(t,n,'posterior',current[n],current_masks[n],source)
        previous=current;previous_masks=current_masks
        if t%60==0:print('GENEALOGY',cell['condition'],cell['backend'],t,'/',T,flush=True)
    for target in targets:target.update(condition=cell['condition'],backend=cell['backend'])
    assert len(population)==480 and len(targets)==960
    return dict(cell=cell,population=population,targets=targets,pools=pools,births=births,roots=roots,first=first,
        checked_predictions=prediction_count,checked_predicted_covariances=covariance_count,source_links=source_links)


def write_csv(path,rows):
    assert not path.exists()
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)


def main():
    freeze=OUT/'CENSUS_FREEZE_V2.json';cfg=json.loads(freeze.read_text());destination=OUT/'CENSUS_RESULTS.json'
    assert not destination.exists()
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    check_fixtures();mat=loadmat(ROOT/cfg['unit']['data_path']);roots=root_roster(mat['measurements'],240)
    resultdir=OUT/'results'/'v2';resultdir.mkdir(exist_ok=True)
    population=[];targets=[];cells=[];artifacts={};birth_reference=None
    for cell in cfg['cells']:
        path=ROOT/cell['path'];assert sha(path)==cell['sha256']
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        assert data['pd']==.9 and data['runs']['rangeDetectionMode']=='nominal'
        result=census_cell(data,cell,mat,roots)
        birth_prior=[{k:r[k] for k in ['frame','robot','label','prior_r','prior_mean']} for r in result['births']]
        if birth_reference is None:birth_reference=birth_prior
        else:assert birth_reference==birth_prior
        native=resultdir/f"{cell['condition']}_{cell['backend'].replace(' ','_')}.json.gz";assert not native.exists()
        with gzip.open(native,'wt') as stream:json.dump(result,stream,allow_nan=False)
        artifacts[str(native.relative_to(ROOT))]=sha(native)
        population.extend(result['population']);targets.extend(result['targets'])
        cells.append({k:result[k] for k in ['cell','first','checked_predictions','checked_predicted_covariances','source_links']})
        print('GENEALOGY COMPLETE',cell['condition'],cell['backend'],flush=True)
    write_csv(OUT/'POPULATION.csv',population);write_csv(OUT/'TARGET_ANCESTRY.csv',targets)
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    report=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),source_runs=6,
        robot_frames=len(population),target_phase_rows=len(targets),birth_roots=len(roots),
        birth_priors_identical_across_all_methods=True,cells=cells,artifacts=artifacts,
        population_sha256=sha(OUT/'POPULATION.csv'),target_sha256=sha(OUT/'TARGET_ANCESTRY.csv'),
        freeze_sha256=sha(freeze),source_sha256=cfg['source_sha256'])
    destination.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('CENSUS COMPLETE',len(population),'robot frames;',len(targets),'target phase rows',flush=True)


if __name__=='__main__':main()
