"""Locate first state divergence and trace the already selected target."""
from pathlib import Path
from collections import defaultdict
import csv
import gzip
import hashlib
import io
import json
import sys
import numpy as np
from scipy.io import loadmat
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for name in ['icra_v2x_gce_diagnosis','icra_reviewer_revision']:
    sys.path.insert(0,str(OUT.parent/name))
from diagnose_gap import matched_truth
from review_gaussian_audit import natural,integrate,unpack
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def by_frame(rows):
    result=defaultdict(list)
    for row in rows:result[tuple(row[:2].astype(int))].append(row)
    return {key:np.asarray(value) for key,value in result.items()}


def sorted_rows(rows):
    return rows[np.lexsort((rows[:,3],rows[:,2]))]


def prepare(data):
    run=data['runs'];inc=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    gaussian=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    quality=np.asarray(run['qualityRecords'],float).reshape(-1,10)
    fusion=np.asarray(run['fusionOutputRecords'],float).reshape(-1,19)
    ids={tuple(row[:4].astype(int)):row for row in inc};assert len(ids)==len(inc)
    predicted=np.c_[quality[:,:4],inc[:,4],quality[:,4:8]]
    assert np.array_equal(quality[:,:4],inc[:,:4])
    local=np.c_[gaussian[:,:4],np.asarray([ids[tuple(row[:4].astype(int))][5] for row in gaussian]),gaussian[:,18:32]]
    return dict(data=data,inc=by_frame(inc),gaussian=by_frame(gaussian),
        predicted=by_frame(predicted),local=by_frame(local),fusion=by_frame(fusion),
        records=by_frame(np.asarray(run['iterationRecords'],float).reshape(-1,60)),
        source_labels=by_frame(np.asarray(run['fusionSourceRecords'],float).reshape(-1,8)),
        gaussian_lookup={tuple(row[:4].astype(int)):row for row in gaussian})


def rows_at(prepared,phase,t,n):
    width=9 if phase=='predicted' else 12 if phase=='inc' else 32 if phase=='gaussian' else 60 if phase=='records' else 8 if phase=='source_labels' else 19
    return prepared[phase].get((t,n),np.empty((0,width)))


def pool_at(prepared,t,n):
    data=prepared['data'];delivered=bool(np.asarray(data['delivered'])[n-1,2-n,t-1])
    return rows_at(prepared,'fusion' if delivered else 'local',t,n)


def compare(left,right,cfg):
    left=sorted_rows(left);right=sorted_rows(right)
    lk={tuple(r[2:4].astype(int)):r for r in left};rk={tuple(r[2:4].astype(int)):r for r in right}
    assert len(lk)==len(left) and len(rk)==len(right)
    common=sorted(lk.keys()&rk.keys());same_labels=lk.keys()==rk.keys()
    a=np.asarray([lk[k] for k in common]).reshape(-1,left.shape[1]);b=np.asarray([rk[k] for k in common]).reshape(-1,right.shape[1])
    difference=np.abs(a[:,4:]-b[:,4:])
    rdiff=float(difference[:,0].max(initial=0));mdiff=float(difference[:,1:5].max(initial=0))
    pdiff=float(difference[:,5:].max(initial=0))
    return dict(left_count=len(left),right_count=len(right),common_labels=len(common),same_labels=same_labels,
        exact=same_labels and np.array_equal(left,right),maximum_existence_difference=rdiff,
        maximum_mean_difference=mdiff,maximum_covariance_difference=pdiff,
        equal=same_labels and rdiff<=cfg['probability_tolerance'] and max(mdiff,pdiff)<=cfg['spatial_tolerance'])


def near_summary(pool,xy,cfg):
    near=pool[np.sum((pool[:,5:7]-xy)**2,axis=1)<=cfg['range_m']**2]
    active=near[near[:,4]>cfg['active_threshold']]
    strongest=active[np.argmax(active[:,4])] if len(active) else None
    total=float(active[:,4].sum())
    return dict(total_components=len(pool),active_components=int((pool[:,4]>cfg['active_threshold']).sum()),
        near_components=len(near),near_active_components=len(active),near_total_r=total,
        near_max_r=None if strongest is None else float(strongest[4]),
        near_largest_share=None if not total else float(strongest[4]/total),
        strongest_label=None if strongest is None else [int(x) for x in strongest[2:4]])


def association(prepared,t,n,xy,mat):
    rows=rows_at(prepared,'inc',t,n);run=prepared['data']['runs']
    z=np.asarray(mat['measurements'][n-1,t-1],float).reshape(2,-1)
    if not len(rows) or z.shape[1]==0:return dict(current_detections=z.shape[1],has_column=False)
    column=int(np.argmin(np.sum((z-xy[:,None])**2,axis=0)))
    distance=float(np.linalg.norm(z[:,column]-xy))
    raw=np.asarray(run['localAssociationWeights'][n-1+2*(t-1)],float)
    W=raw.reshape(len(rows),z.shape[1]+1)
    assert np.isfinite(W).all() and (W>=0).all() and np.allclose(W.sum(1),1,atol=1e-12,rtol=0)
    values=rows[:,5]*W[:,column+1];total=float(values.sum());normalized=values/total if total else np.zeros_like(values)
    nonzero=normalized>0;entropy=float(-np.sum(normalized[nonzero]*np.log(normalized[nonzero])))
    strongest=int(np.argmax(values));ratio=float(np.asarray(mat['likelihoodRatios'][n-1,t-1]).ravel()[column])
    return dict(current_detections=z.shape[1],has_column=True,column_1based=column+1,distance_m=distance,
        within_2m=distance<=2.,likelihood_ratio=ratio,positive_mark=float(max(0,np.tanh(.5*np.log(ratio)))),
        conditional_mass=float(W[:,column+1].sum()),joint_mass=total,entropy=entropy,
        effective_labels=float(np.exp(entropy)),largest_joint=float(values[strongest]),
        largest_share=float(normalized[strongest]),strongest_label=[int(x) for x in rows[strongest,2:4]])


def record_json(row):
    missing=np.flatnonzero(~np.isfinite(row))
    assert not np.isinf(row).any() and np.isin(missing,[17,18]).all()
    assert all(row[13+int(column)-17]==0 for column in missing)
    return [None if np.isnan(value) else float(value) for value in row]


def first_event_details(left,right,t,n,cfg):
    local_equal=[]
    for source in [1,2]:
        a=rows_at(left,'local',t,source);b=rows_at(right,'local',t,source)
        la=sorted_rows(rows_at(left,'inc',t,source));lb=sorted_rows(rows_at(right,'inc',t,source))
        ga=sorted_rows(rows_at(left,'gaussian',t,source));gb=sorted_rows(rows_at(right,'gaussian',t,source))
        local_equal.append(dict(source=source,distribution=compare(a,b,cfg),
            exact_increment_records=np.array_equal(la,lb),exact_local_gaussian_records=np.array_equal(ga,gb)))
    lrec=sorted_rows(rows_at(left,'records',t,n));rrec=sorted_rows(rows_at(right,'records',t,n))
    assert np.array_equal(lrec[:,:4],rrec[:,:4]),'First fused labels unexpectedly differ'
    sources_equal=np.array_equal(sorted_rows(rows_at(left,'source_labels',t,n)),sorted_rows(rows_at(right,'source_labels',t,n)))
    decompositions=[]
    for a,b in zip(lrec,rrec):
        record=dict(label=[int(x) for x in a[2:4]],source_labels_equal=bool(np.array_equal(a[31:35],b[31:35])),
            kept_exponents_equal=bool(np.array_equal(a[52:54],b[52:54])),gce_record=record_json(a),scalar_record=record_json(b))
        for backend,row,prepared in [('GCE',a,left),('Guarded Scalar',b,right)]:
            local_ids=row[31:35].reshape(2,2).astype(int);present=local_ids[:,0]>0
            jp=[];hp=[];cp=[];ju=[];hu=[];cu=[]
            for side in range(2):
                if not present[side]:
                    jp.append(np.zeros((4,4)));hp.append(np.zeros(4));cp.append(0.)
                    ju.append(np.zeros((4,4)));hu.append(np.zeros(4));cu.append(0.);continue
                key=(t,n if side==0 else 3-n,*local_ids[side]);local=prepared['gaussian_lookup'][key]
                prior=natural(local[4:8],unpack(local[8:18]));post=natural(local[18:22],unpack(local[22:32]))
                jp.append(prior[0]);hp.append(prior[1]);cp.append(prior[2])
                ju.append(post[0]);hu.append(post[1]);cu.append(post[2])
            jp,hp,cp,ju,hu,cu=map(np.asarray,[jp,hp,cp,ju,hu,cu])
            alpha=row[54:56];kept=row[52:54]
            J0=np.sum(alpha[:,None,None]*ju,axis=0);H0=np.sum(alpha[:,None]*hu,axis=0);C0=float(np.sum(alpha*cu))
            base_mean,base_cov,base_log=integrate(J0,H0,C0)
            J=J0+np.sum(kept[:,None,None]*(ju-jp),axis=0)
            H=H0+np.sum(kept[:,None]*(hu-hp),axis=0);C=C0+float(np.sum(kept*(cu-cp)))
            full_mean,full_cov,full_log=integrate(J,H,C)
            if not kept.any():full_log=row[10]
            if backend=='Guarded Scalar':mean,cov,log_i=base_mean,base_cov,row[10]
            else:mean,cov,log_i=full_mean,full_cov,full_log
            active=row[13:15]>0;prob=np.clip(row[17:19][active],1e-9,1-1e-9)
            inherited=float(np.sum(row[28:30][active]*(np.log(prob)-np.log1p(-prob))))
            scalar=float(np.sum(kept*row[19:21]));expected_r=float(expit(inherited+scalar+log_i))
            assert abs(base_log-row[10])<1e-8 and abs(log_i-row[56])<1e-8
            assert abs(expected_r-row[9])<2e-10
            assert np.allclose(mean,np.r_[row[4:6],row[40:42]],atol=1e-7,rtol=0)
            assert np.allclose(cov,unpack(row[42:52]),atol=1e-7,rtol=0)
            record[backend]=dict(inherited_log_odds=inherited,admitted_scalar_increment=scalar,
                old_spatial_log_integral=float(base_log),new_spatial_log_integral=float(log_i),
                r=float(row[9]),r_with_old_integral=float(expit(inherited+scalar+row[10])),
                mean=mean.tolist(),covariance=cov.tolist())
        decompositions.append(record)
    return dict(frame=t,robot=n,local_sources=local_equal,source_matching_equal=sources_equal,rows=decompositions)


def main():
    destination=OUT/'ORIGIN_TRACE.json';assert not destination.exists()
    freeze=OUT/'TRACE_FREEZE.json';cfg=json.loads(freeze.read_text())
    for key,digest in cfg['source_sha256'].items():assert sha(ROOT/key)==digest,key
    mat=loadmat(ROOT/cfg['unit']['data_path']);timelines=[];differences=[];origins=[];input_shas={}
    for mode in ['nominal','range']:
        for condition in ['reliable','intermittent']:
            group={};headers=None
            for cell in cfg['cells']:
                if (cell['mode'],cell['condition'])!=(mode,condition):continue
                path=ROOT/cell['path'];assert sha(path)==cell['sha256'];input_shas[cell['path']]=sha(path)
                with gzip.open(path,'rt') as stream:data=json.load(stream)
                current={k:data[k] for k in ['truth','truthIds','positions','delivered','time']}
                if headers is None:headers=current
                else:assert current==headers
                assert data['runs']['arm']==cell['arm']
                group[cell['backend']]=prepare(data)
            T=len(headers['time']);assert T==240
            first={phase:None for phase in ['predicted','local','posterior','fused','target_local','target_posterior']}
            for t in range(1,T+1):
                ids=np.asarray(headers['truthIds'][t-1]).ravel();indices=np.flatnonzero(ids==cfg['truth_id'])
                assert len(indices)<=1
                truth=np.asarray(headers['truth'][t-1]).reshape(4,-1)
                xy=truth[:2,int(indices[0])] if len(indices) else None
                for n in [1,2]:
                    for phase in ['predicted','local','posterior','fused','target_local','target_posterior']:
                        if phase=='fused' and not np.asarray(headers['delivered'])[n-1,2-n,t-1]:continue
                        if phase.startswith('target_') and xy is None:continue
                        rows=[]
                        for backend in ['GCE','Guarded Scalar']:
                            prepared=group[backend]
                            if phase in ['posterior','target_posterior']:pool=pool_at(prepared,t,n);pool=pool[pool[:,4]>.001]
                            else:pool=rows_at(prepared,'fusion' if phase=='fused' else 'local' if phase=='target_local' else phase,t,n)
                            if phase.startswith('target_'):pool=pool[np.sum((pool[:,5:7]-xy)**2,axis=1)<=4]
                            rows.append(pool)
                        comparison=compare(*rows,cfg)
                        differences.append(dict(mode=mode,condition=condition,frame=t,robot=n,phase=phase,**comparison))
                        if not comparison['equal'] and first[phase] is None:first[phase]=dict(frame=t,robot=n,**comparison)
                    if xy is None:continue
                    for backend,prepared in group.items():
                        actual=prepared['data']['runs']['estimates'][n-1+2*(t-1)]
                        row=dict(mode=mode,condition=condition,backend=backend,frame=t,robot=n,
                            detected=bool(matched_truth(truth,actual,2.)[int(indices[0])]),output_count=len(actual),
                            delivered=bool(np.asarray(headers['delivered'])[n-1,2-n,t-1]))
                        for phase,pool in [('predicted',rows_at(prepared,'predicted',t,n)),('local',rows_at(prepared,'local',t,n)),('posterior',pool_at(prepared,t,n))]:
                            row.update({phase+'_'+k:v for k,v in near_summary(pool,xy,cfg).items()})
                        row.update({'association_'+k:v for k,v in association(prepared,t,n,xy,mat).items()})
                        timelines.append(row)
            origin=dict(mode=mode,condition=condition,first=first)
            if first['fused'] is not None:
                event=first['fused'];origin['first_fusion_details']=first_event_details(group['GCE'],group['Guarded Scalar'],event['frame'],event['robot'],cfg)
            origins.append(origin)
            print('ORIGIN TRACED',mode,condition,json.dumps(first),flush=True)
    expected=len(cfg['cells'])*2*sum(cfg['truth_id'] in np.asarray(v).ravel() for v in headers['truthIds'])
    assert len(timelines)==expected
    for name,rows in [('TARGET_TIMELINE.csv',timelines),('PAIR_DIFFERENCES.csv',differences)]:
        path=OUT/name;fields=list(dict.fromkeys(k for r in rows for k in r))
        stream=io.StringIO(newline='')
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(rows)
        value=stream.getvalue()
        if path.exists():assert path.read_text()==value,'Existing trace CSV changed'
        else:path.write_text(value)
    for key,digest in cfg['source_sha256'].items():assert sha(ROOT/key)==digest,key
    report=dict(passed=True,scope=cfg['scope'],sequence=cfg['sequence'],truth_id=cfg['truth_id'],
        source_runs=len(cfg['cells']),target_robot_frames=len(timelines),pair_comparisons=len(differences),
        origins=origins,inputs=input_shas,freeze_sha256=sha(freeze),source_sha256=sha(Path(__file__)),
        timeline_sha256=sha(OUT/'TARGET_TIMELINE.csv'),differences_sha256=sha(OUT/'PAIR_DIFFERENCES.csv'))
    destination.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('ORIGIN TRACE COMPLETE',len(timelines),'target robot frames',flush=True)


if __name__=='__main__':main()
