"""Independently count native hypotheses and verify the first divergence."""
from pathlib import Path
from collections import defaultdict
import ast
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def load(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)


def grouping(rows,width):
    result=defaultdict(list)
    for row in np.asarray(rows,float).reshape(-1,width):result[int(row[0]),int(row[1])].append(row)
    return {k:np.asarray(v) for k,v in result.items()}


def get(group,t,n,width):return group.get((t,n),np.empty((0,width)))


def native(data):
    run=data['runs'];inc=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    gaussian=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    quality=np.asarray(run['qualityRecords'],float).reshape(-1,10)
    local_map={tuple(row[:4].astype(int)):row for row in inc}
    local=np.array([[*row[:4],local_map[tuple(row[:4].astype(int))][5],*row[18:32]] for row in gaussian])
    predicted=np.array([[*row[:4],local_map[tuple(row[:4].astype(int))][4],*row[4:8]] for row in quality])
    return dict(data=data,predicted=grouping(predicted,9),local=grouping(local,19),
        fusion=grouping(run['fusionOutputRecords'],19),increments=grouping(inc,12))


def pool(source,t,n):
    delivered=source['data']['delivered'][n-1][2-n][t-1]
    return get(source['fusion' if delivered else 'local'],t,n,19)


def equality(a,b,prob_tol,spatial_tol):
    left={tuple(v[2:4].astype(int)):v[4:] for v in a};right={tuple(v[2:4].astype(int)):v[4:] for v in b}
    if left.keys()!=right.keys():return False
    for key in left:
        difference=abs(left[key]-right[key])
        if difference[0]>prob_tol or np.any(difference[1:]>spatial_tol):return False
    return True


def main():
    destination=OUT/'TRACE_VERIFICATION.json';assert not destination.exists()
    report_path=OUT/'ORIGIN_TRACE.json';report=json.loads(report_path.read_text());assert report['passed']
    cfg=json.loads((OUT/'TRACE_FREEZE.json').read_text())
    repair=json.loads((OUT/'TRACE_SERIALIZATION_REPAIR.json').read_text())
    original=(OUT/'trace.py').read_text()
    for change in repair['changes']:
        assert original.count(change['before'])==1;original=original.replace(change['before'],change['after'])
    assert original==(OUT/'trace_v2.py').read_text()
    assert sha(OUT/'trace.py')==repair['original_source_sha256']
    assert sha(OUT/'trace_v2.py')==repair['repaired_source_sha256']==report['source_sha256']
    for name,digest in repair['preserved_csv'].items():assert sha(OUT/name)==digest
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    with (OUT/'TARGET_TIMELINE.csv').open(newline='') as stream:rows=list(csv.DictReader(stream))
    lookup={(r['mode'],r['condition'],r['backend'],int(r['frame']),int(r['robot'])):r for r in rows}
    assert len(lookup)==len(rows)==report['target_robot_frames']==5760
    mat=loadmat(ROOT/cfg['unit']['data_path']);checked=0;association_rows=0;firsts=[]
    for mode in ['nominal','range']:
        for condition in ['reliable','intermittent']:
            sources={}
            for cell in cfg['cells']:
                if (cell['mode'],cell['condition'])!=(mode,condition):continue
                path=ROOT/cell['path'];assert sha(path)==cell['sha256'];source=native(load(path));sources[cell['backend']]=source
                data=source['data']
                for t in range(1,241):
                    truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T
                    target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[target,:2]
                    for n in [1,2]:
                        row=lookup[mode,condition,cell['backend'],t,n]
                        estimate=np.asarray(data['runs']['estimates'][n-1+2*(t-1)],float).reshape(-1,4)
                        distance=np.sqrt(((truth[:,None,:2]-estimate[None,:,:2])**2).sum(2))
                        ti,ei=linear_sum_assignment(np.where(distance<=2,distance,1e6))
                        detected=any(i==target and distance[i,j]<=2 for i,j in zip(ti,ei))
                        assert detected==(row['detected']=='True') and len(estimate)==int(row['output_count'])
                        for phase,values in [('predicted',get(source['predicted'],t,n,9)),('local',get(source['local'],t,n,19)),('posterior',pool(source,t,n))]:
                            near=np.asarray([v for v in values if math.hypot(v[5]-xy[0],v[6]-xy[1])<=2]).reshape(-1,values.shape[1])
                            active=near[near[:,4]>.001]
                            assert len(values)==int(row[phase+'_total_components'])
                            assert sum(v[4]>.001 for v in values)==int(row[phase+'_active_components'])
                            assert len(near)==int(row[phase+'_near_components'])
                            assert len(active)==int(row[phase+'_near_active_components'])
                            total=math.fsum(v[4] for v in active);assert abs(total-float(row[phase+'_near_total_r']))<1e-11
                            if len(active):
                                maximum=max(active,key=lambda v:v[4]);assert maximum[4]==float(row[phase+'_near_max_r'])
                                assert list(maximum[2:4].astype(int))==ast.literal_eval(row[phase+'_strongest_label'])
                                assert abs(maximum[4]/total-float(row[phase+'_near_largest_share']))<1e-12
                            else:assert not row[phase+'_near_max_r'] and not row[phase+'_strongest_label']
                        increments=get(source['increments'],t,n,12);z=np.asarray(mat['measurements'][n-1,t-1],float).reshape(2,-1)
                        if row['association_has_column']=='True':
                            nearest=int(row['association_column_1based'])-1
                            distances=[math.hypot(point[0]-xy[0],point[1]-xy[1]) for point in z.T]
                            assert nearest==int(np.argmin(distances)) and abs(distances[nearest]-float(row['association_distance_m']))<1e-12
                            W=np.asarray(data['runs']['localAssociationWeights'][n-1+2*(t-1)],float).reshape(len(increments),z.shape[1]+1)
                            joint=[float(v[5]*w[nearest+1]) for v,w in zip(increments,W)];total=math.fsum(joint)
                            normalized=[v/total for v in joint] if total else [0.]*len(joint)
                            entropy=-math.fsum(v*math.log(v) for v in normalized if v>0)
                            for key,value in [('joint_mass',total),('entropy',entropy),('effective_labels',math.exp(entropy)),('largest_share',max(normalized))]:
                                assert abs(value-float(row['association_'+key]))<1e-10,(cell,t,n,key)
                            association_rows+=1
                        else:assert not len(increments) or z.shape[1]==0
                        checked+=1
            # Find each earliest difference directly from the native states.
            origin=next(x for x in report['origins'] if (x['mode'],x['condition'])==(mode,condition))
            for phase in ['predicted','local','posterior','fused','target_local','target_posterior']:
                first=None
                for t in range(1,241):
                    data=sources['GCE']['data'];target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item()
                    xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,target]
                    for n in [1,2]:
                        if phase=='fused' and not data['delivered'][n-1][2-n][t-1]:continue
                        pair=[]
                        for backend in ['GCE','Guarded Scalar']:
                            source=sources[backend]
                            if phase in ['posterior','target_posterior']:values=pool(source,t,n);values=values[values[:,4]>.001]
                            else:values=get(source['fusion' if phase=='fused' else 'local' if phase=='target_local' else phase],t,n,9 if phase=='predicted' else 19)
                            if phase.startswith('target_'):values=values[np.sum((values[:,5:7]-xy)**2,axis=1)<=4]
                            pair.append(values)
                        if not equality(*pair,cfg['probability_tolerance'],cfg['spatial_tolerance']):first=(t,n);break
                    if first is not None:break
                expected=origin['first'][phase];assert first==(expected['frame'],expected['robot'])
                firsts.append(dict(mode=mode,condition=condition,phase=phase,frame=first[0],robot=first[1]))
            details=origin['first_fusion_details']
            assert all(x['distribution']['equal'] for x in details['local_sources']) and details['source_matching_equal']
            for record in details['rows']:
                g=record['GCE'];s=record['Guarded Scalar']
                assert abs(g['inherited_log_odds']-s['inherited_log_odds'])<1e-10
                assert abs(g['admitted_scalar_increment']-s['admitted_scalar_increment'])<1e-10
                assert abs(g['r_with_old_integral']-s['r'])<2e-10
                for native_record in [record['gce_record'],record['scalar_record']]:
                    for column,value in enumerate(native_record):
                        if value is None:assert column in [17,18] and native_record[13+column-17]==0
            print('ORIGIN VERIFIED',mode,condition,flush=True)
    old=OUT.parent/'icra_range_detection/MECHANISM_FRAMES.csv'
    with old.open(newline='') as stream:prior=list(csv.DictReader(stream))
    common=0
    for row in prior:
        arm=row['arm'];mode='range' if arm.endswith('_range') else 'nominal'
        base=arm[:-6] if mode=='range' else arm
        backend={'marked_gaussian_evidence':'GCE','marked_gaussian_evidence_guarded_scalar':'Guarded Scalar','marked_lineage':'No-age'}.get(base)
        if backend is None:continue
        new=lookup[mode,row['condition'],backend,int(row['frame']),int(row['robot'])]
        assert new['detected']==row['output_match'] and new['posterior_near_active_components']==row['near_active_components']
        common+=1
    assert checked==5760 and common==1680
    result=dict(passed=True,target_robot_frames=checked,association_columns=association_rows,
        first_events=firsts,prior_case_rows_identical=common,
        report_sha256=sha(report_path),freeze_sha256=sha(OUT/'TRACE_FREEZE.json'),
        source_sha256=sha(Path(__file__)),repair_sha256=sha(OUT/'TRACE_SERIALIZATION_REPAIR.json'),
        timeline_sha256=sha(OUT/'TARGET_TIMELINE.csv'),differences_sha256=sha(OUT/'PAIR_DIFFERENCES.csv'))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ORIGIN TRACE VERIFIED',checked,'robot frames;',common,'old case rows match',flush=True)


if __name__=='__main__':main()
