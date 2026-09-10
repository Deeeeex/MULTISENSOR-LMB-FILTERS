"""Independent native population/ancestry reconstruction with ordinary sets."""
from collections import Counter
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def grouped(values,width):
    result={}
    for row in np.asarray(values,float).reshape(-1,width):
        t,n,b,s=map(int,row[:4]);frame=result.setdefault((t,n),{})
        assert (b,s) not in frame;frame[b,s]=row
    return result


def symmetric(v):
    p=np.zeros((4,4));k=0
    for col in range(4):
        for row in range(col,4):p[row,col]=p[col,row]=v[k];k+=1
    return p


def statistics(origins):
    appearances={}
    for label,roots in origins.items():
        for root in roots:appearances.setdefault(root,set()).add(label)
    repeated={root:keys for root,keys in appearances.items() if len(keys)>1}
    involved=set().union(*repeated.values()) if repeated else set()
    keys=list(origins)
    pairs=sum(not origins[left].isdisjoint(origins[right]) for i,left in enumerate(keys) for right in keys[i+1:])
    return dict(ancestry_roots=len(appearances),ancestry_memberships=sum(len(v) for v in origins.values()),
        repeated_roots=len(repeated),labels_with_shared_roots=len(involved),pairs_with_shared_roots=pairs,
        maximum_root_multiplicity=max((len(v) for v in appearances.values()),default=0))


def check_pool(stored,objects,origins,root_positions,xy):
    assert [tuple(x['label']) for x in stored['labels']]==sorted(objects)
    for item in stored['labels']:
        key=tuple(item['label']);obj=objects[key]
        assert item['r']==obj[0] and np.allclose(item['mean'],obj[1],atol=1e-12,rtol=0)
        assert int(item['roots'],16)==sum(2**root_positions[root] for root in origins[key])
    near={k:roots for k,roots in origins.items() if math.hypot(*(objects[k][1][:2]-xy))<=2.}
    t=stored['frame']
    target=dict(frame=t,robot=stored['robot'],phase=stored['phase'],total_count=len(objects),near_count=len(near),
        near_sum_r=math.fsum(objects[k][0] for k in near),near_max_r=max((objects[k][0] for k in near),default=0.),
        near_birth_labels=sum(k[0]==t for k in near),near_labels_from_first_three_frames=sum(k[0]<=3 for k in near),
        near_label_age_sum=sum(t-k[0] for k in near),**statistics(near))
    return statistics(origins),target


def equal_row(actual,expected):
    assert actual.keys()==expected.keys(),(actual.keys(),expected.keys())
    for key,value in expected.items():
        if isinstance(value,float):assert abs(actual[key]-value)<1e-10,(key,actual[key],value)
        else:assert actual[key]==value,(key,actual[key],value)


def check_cell(data,stored,mat,cell):
    run=data['runs'];inc=grouped(run['localIncrementRecords'],12)
    gauss=grouped(run['localGaussianRecords'],32);quality=grouped(run['qualityRecords'],10)
    fused=grouped(run['fusionOutputRecords'],19);sources=grouped(run['fusionSourceRecords'],8)
    saved_pools={(r['frame'],r['robot'],r['phase']):r for r in stored['pools']}
    assert len(saved_pools)==960
    roots={};births_by_time={};positions={};root_list=[]
    for t in range(2,241):
        for n in [1,2]:
            z=np.asarray(mat['measurements'][n-1,t-2],float).reshape(2,-1)
            for j in range(z.shape[1]):
                key=(t,n*100000+j+1);mean=np.array([z[0,j],z[1,j],0.,0.])
                positions[key]=len(positions);roots[key]=mean;births_by_time.setdefault((t,n),set()).add(key)
                root_list.append(dict(birth_frame=t,sensor=n,previous_detection_index=j+1,label=list(key),mean=mean.tolist()))
    assert stored['roots']==root_list
    births={(x['frame'],x['robot'],*x['label']):x for x in stored['births']};assert len(births)==len(roots)
    previous={1:{},2:{}};old_origins={1:{},2:{}};population=[];targets=[];predictions=0;covariances=0;links=0
    first={'local':None,'posterior':None}
    f=np.eye(4);f[0,2]=f[1,3]=.1
    q=np.array([[25*.1**3/3,0,25*.1**2/2,0],[0,25*.1**3/3,0,25*.1**2/2],
                [25*.1**2/2,0,2.5,0],[0,25*.1**2/2,0,2.5]])
    for t in range(1,241):
        gt_ids=np.asarray(data['truthIds'][t-1]).ravel();ix=np.flatnonzero(gt_ids==5).item()
        xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,ix]
        local={};local_origins={};local_counts={}
        for n in [1,2]:
            increments=inc.get((t,n),{});gs=gauss.get((t,n),{});qs=quality.get((t,n),{})
            born=births_by_time.get((t,n),set());assert set(increments)==set(previous[n])|born==set(qs)
            local[n]={};local_origins[n]={};new_kept=old_kept=0
            for key,row in increments.items():
                if key in born:
                    r=.01;mean=roots[key];p=np.diag([16.,16.,225.,225.]);ancestry=frozenset([key])
                    expected=dict(frame=t,robot=n,label=list(key),prior_r=float(row[4]),prior_mean=qs[key][4:8].tolist(),posterior_r=float(row[5]),retained=row[5]>.001)
                    assert births[(t,n,*key)]==expected
                else:
                    old=previous[n][key];r=old[0]*.99
                    mean=np.array([old[1][0]+.1*old[1][2],old[1][1]+.1*old[1][3],old[1][2],old[1][3]])
                    p=f@old[2]@f.T+q;ancestry=old_origins[n][key]
                assert abs(row[4]-r)<1e-12 and np.allclose(qs[key][4:8],mean,atol=1e-7,rtol=0)
                predictions+=1
                if row[5]>.001:
                    g=gs[key];assert np.allclose(g[4:8],mean,atol=1e-7,rtol=0)
                    assert np.allclose(symmetric(g[8:18]),p,atol=1e-7,rtol=0);covariances+=1
                    local[n][key]=(float(row[5]),g[18:22],symmetric(g[22:32]));local_origins[n][key]=ancestry
                    if key in born:new_kept+=1
                    else:old_kept+=1
                else:assert key not in gs
            assert set(gs)==set(local[n])
            stats,target=check_pool(saved_pools[t,n,'local'],local[n],local_origins[n],positions,xy)
            targets.append(target)
            local_counts[n]=dict(previous_count=len(previous[n]),birth_count=len(born),predicted_count=len(increments),
                local_count=len(local[n]),birth_local_retained=new_kept,inherited_local_retained=old_kept,
                local_pruned=len(increments)-len(local[n]),birth_local_pruned=len(born)-new_kept,
                inherited_local_pruned=len(previous[n])-old_kept,**{'local_'+k:v for k,v in stats.items()})
            if first['local'] is None and stats['pairs_with_shared_roots']:
                first['local']=(t,n)
        next_objects={};next_origins={}
        for n in [1,2]:
            receive=bool(np.asarray(data['delivered'])[n-1,2-n,t-1]);maps=sources.get((t,n),{})
            after={};origins={};self_only=matched=remote_only=0
            if receive:
                values=fused.get((t,n),{});assert set(values)==set(maps)
                used=[]
                for key,mapping in maps.items():
                    parents=[]
                    for s,col in [(n,4),(3-n,6)]:
                        parent=tuple(map(int,mapping[col:col+2]))
                        if parent[0]>0:
                            assert parent in local[s] and (s,parent) not in used;used.append((s,parent));parents.append((s,parent));links+=1
                            if s==n:assert key==parent
                    assert len(parents) in [1,2]
                    if len(parents)==2:matched+=1
                    elif parents[0][0]==n:self_only+=1
                    else:remote_only+=1
                    ancestry=set()
                    for s,parent in parents:ancestry.update(local_origins[s][parent])
                    row=values[key]
                    after[key]=(float(row[4]),row[5:9],symmetric(row[9:]));origins[key]=frozenset(ancestry)
            else:
                assert not maps and not fused.get((t,n),{})
                after=local[n].copy();origins=local_origins[n].copy();self_only=len(after)
            next_objects[n]={k:v for k,v in after.items() if v[0]>.001}
            next_origins[n]={k:origins[k] for k in next_objects[n]}
            lost=set(local[n])-set(next_objects[n]);imported=set(next_objects[n])-set(local[n])
            delta=len(next_objects[n])-len(previous[n]);counts=local_counts[n]
            assert delta==counts['birth_count']-counts['local_pruned']-len(lost)+len(imported)
            row=dict(condition=cell['condition'],backend=cell['backend'],frame=t,robot=n,**counts,
                received=receive,fusion_pre_count=len(after),local_only_outputs=self_only,matched_outputs=matched,
                remote_only_outputs=remote_only,local_lost_at_fusion=len(lost),remote_only_retained=len(imported),
                fusion_pruned=len(after)-len(next_objects[n]),retained_count=len(next_objects[n]),population_delta=delta,
                posterior_birth_labels=sum(k[0]==t for k in next_objects[n]),posterior_label_age_sum=sum(t-k[0] for k in next_objects[n]))
            stats,target=check_pool(saved_pools[t,n,'posterior'],next_objects[n],next_origins[n],positions,xy)
            row.update({'posterior_'+k:v for k,v in stats.items()});population.append(row);targets.append(target)
            if first['posterior'] is None and stats['pairs_with_shared_roots']:first['posterior']=(t,n)
        previous=next_objects;old_origins=next_origins
    for row in targets:row.update(condition=cell['condition'],backend=cell['backend'])
    assert len(population)==len(stored['population'])==480 and len(targets)==len(stored['targets'])==960
    for a,b in zip(stored['population'],population):equal_row(a,b)
    for a,b in zip(stored['targets'],targets):equal_row(a,b)
    for phase,at in first.items():
        expected=stored['first'][phase]
        if at is None:assert expected is None
        else:
            assert (expected['frame'],expected['robot'])==at and expected['phase']==phase
            chosen=expected['labels'];assert len(chosen)==2
            pool=saved_pools[*at,phase];by_label={tuple(x['label']):x for x in pool['labels']}
            shared=int(chosen[0]['roots'],16)&int(chosen[1]['roots'],16)
            assert shared==int(expected['shared_roots'],16) and shared>0
            for v in chosen:
                base=by_label[tuple(v['label'])]
                for key in ['label','r','mean','roots']:assert v[key]==base[key]
                if phase=='posterior':assert v['source_labels']==sources[at][tuple(v['label'])][4:].astype(int).tolist()
    assert stored['checked_predictions']==predictions and stored['checked_predicted_covariances']==covariances and stored['source_links']==links
    return population,targets,dict(checked_predictions=predictions,checked_predicted_covariances=covariances,source_links=links)


def main():
    destination=OUT/'CENSUS_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'CENSUS_FREEZE.json').read_text());report=json.loads((OUT/'CENSUS_RESULTS.json').read_text())
    execution=json.loads((OUT/'CENSUS_EXECUTION.json').read_text())
    assert report['passed'] and execution['completed'] and execution['returncode']==0
    assert report['freeze_sha256']==execution['freeze_sha256']==sha(OUT/'CENSUS_FREEZE.json')
    assert execution['log_sha256']==sha(ROOT/execution['log'])
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    for name,digest in report['artifacts'].items():assert sha(ROOT/name)==digest,name
    mat=loadmat(ROOT/cfg['unit']['data_path']);population=[];targets=[];checks=[]
    for cell in cfg['cells']:
        with gzip.open(ROOT/cell['path'],'rt') as stream:data=json.load(stream)
        path=OUT/'results'/f"{cell['condition']}_{cell['backend'].replace(' ','_')}.json.gz"
        with gzip.open(path,'rt') as stream:stored=json.load(stream)
        assert stored['cell']==cell
        p,t,c=check_cell(data,stored,mat,cell);population.extend(p);targets.extend(t);checks.append(dict(cell=cell,**c))
        print('INDEPENDENT GENEALOGY VERIFIED',cell['condition'],cell['backend'],flush=True)
    for name,rows in [('POPULATION.csv',population),('TARGET_ANCESTRY.csv',targets)]:
        with (OUT/name).open(newline='') as stream:table=list(csv.DictReader(stream))
        assert len(table)==len(rows)
        for a,b in zip(table,rows):
            for key,value in b.items():
                if isinstance(value,bool):assert a[key]==str(value)
                elif isinstance(value,(int,float)):assert abs(float(a[key])-value)<1e-10
                else:assert a[key]==value
    assert len(population)==report['robot_frames']==2880 and len(targets)==report['target_phase_rows']==5760
    assert sha(OUT/'POPULATION.csv')==report['population_sha256'] and sha(OUT/'TARGET_ANCESTRY.csv')==report['target_sha256']
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    result=dict(passed=True,robot_frames=len(population),target_phase_rows=len(targets),checks=checks,
        method='Independent raw-native reconstruction with frozenset birth IDs, manual CV prediction, population balances, every saved ancestry set, all CSV rows',
        report_sha256=sha(OUT/'CENSUS_RESULTS.json'),freeze_sha256=sha(OUT/'CENSUS_FREEZE.json'),
        execution_sha256=sha(OUT/'CENSUS_EXECUTION.json'),verifier_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('INDEPENDENT CENSUS VERIFICATION PASSED',len(population),'robot frames',flush=True)


if __name__=='__main__':main()
