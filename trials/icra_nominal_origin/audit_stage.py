"""Audit full nominal recursion and the single registered negative-scalar event."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import re
import sys
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for folder in ['icra_gaussian_evidence','icra_marked_control','icra_fusion_holdout','icra_reviewer_revision',
               'icra_temporal_association','icra_miss_history','icra_range_detection','icra_recursion_origin']:
    sys.path.insert(0,str(OUT.parent/folder))
sys.path.insert(0,str(OUT))
from analyze_control import read,score_run
from analyze_holdout import radio_draws
import probability_audit
from initial_gaussian_audit import check_gaussians
probability_audit.check_gaussians=check_gaussians
from history_audit_v3 import audit_positive_marks,audit_matching
from audit_observation_run import current_records
from audit_screen_assessment_v2 import packets
from range_audit import source_view,audit_actual_pd,audit_noage
from native_summary import summarize as previous_summary
sys.path.insert(0,str(OUT))

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence';NOAGE='marked_lineage';ARM=GCE+'_once_initial_negative'

def summary(data,mat):
    result,frames=previous_summary(data,mat)
    for name,left,right in [('before_event',1,1),('after_event',2,240)]:
        selected=[r for r in frames if left<=r['frame']<=right]
        result['windows'][name]=dict(detected=sum(r['detected'] for r in selected),robot_frames=len(selected),
            by_robot=[sum(r['detected'] for r in selected if r['robot']==n) for n in [1,2]],
            maximum_near_components=max(r['near_active_components'] for r in selected),
            maximum_components=max(r['active_components'] for r in selected))
    return result,frames

def event_scope(run,data):
    assert run['initialNegativeFrame']==2
    expected=np.zeros((2,len(data['time'])),bool)
    if run['arm']==ARM:
        assert run['initialNegativeEnabled'];expected[:,1]=[data['delivered'][0][1][1],data['delivered'][1][0][1]]
    else:assert run['arm']==GCE and not run['initialNegativeEnabled'] and not run['initialNegativeRecords']
    assert np.array_equal(expected,run['initialNegativeMask'])
    return int(expected.sum())

def exact_prefix(run,control):
    fields=[]
    for name in ['localIncrementRecords','localGaussianRecords','qualityRecords','packetGaussianRecords',
                 'localDirectRecords','packetDirectRecords','fusionSourceRecords']:
        assert [r for r in run[name] if r[0]<=2]==[r for r in control[name] if r[0]<=2],name;fields.append(name)
    for name,omit in [('iterationRecords',{6,9}),('fusionOutputRecords',{4})]:
        a=[[v for i,v in enumerate(r) if i not in omit] for r in run[name] if r[0]<=2]
        b=[[v for i,v in enumerate(r) if i not in omit] for r in control[name] if r[0]<=2]
        assert a==b,name;fields.append(name+'_except_actual_existence')
    for name in ['estimates','rawEstimates','labels','localAssociationWeights']:
        stop=4 if name=='localAssociationWeights' else 2;assert run[name][:stop]==control[name][:stop],name;fields.append(name)
    for name in ['packetBytes','matchedLabels']:
        assert [r[:2] for r in run[name]]==[r[:2] for r in control[name]],name;fields.append(name)
    for name in ['rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages','deliveredMessages']:
        assert run[name][:2]==control[name][:2],name;fields.append(name)
    return fields

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['nominal_origin_controls','nominal_origin_event']);args=p.parse_args()
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    runtimepath=OUT/('runtime_'+args.stage+'.json');runtime=json.loads(runtimepath.read_text())
    freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text())
    for name,h in freeze['configurations'].items():assert sha(ROOT/name)==h,name
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    mapped={re.sub('[^A-Za-z0-9_]','_',name)[:63]:value for name,value in cfg['source_sha256'].items()}
    assert len(mapped)==len(cfg['source_sha256']) and len(runtime)==len(cfg['units'])==2
    rows=[];diagnostics=[];parity=[];timeline=[];hashes={str(cfgpath.relative_to(ROOT)):sha(cfgpath),str(runtimepath.relative_to(ROOT)):sha(runtimepath)}
    for unit,native in zip(cfg['units'],runtime):
        seq=unit['sequence'];eid=unit['execution_id'];assert unit['conditions']==[eid];condition=eid
        assert (native['sequence'],native['execution_id'])==(seq,eid)
        assert native['returncode']==0 and native['completion_line'] and native['files']==native['expected_files']==1
        log=ROOT/'RUN/ICRA_NOMINAL_ORIGIN'/args.stage/(eid+'.log');content=log.read_text()
        for marker in ['RANGE DETECTION CHECK PASSED','INITIAL NEGATIVE CHECK PASSED',f'COMPLETED REVIEW {args.stage} {seq}']:assert marker in content
        hashes[str(log.relative_to(ROOT))]=sha(log)
        assert sha(ROOT/unit['data_path'])==unit['input_sha256'] and sha(ROOT/unit['pose_path'])==unit['pose_sha256']
        mat=loadmat(ROOT/unit['data_path']);assert int(mat['T'].item())==240
        delivered=radio_draws(unit['radio_seed']-8301,240,condition)
        paths=[(a,OUT/'results'/args.stage/f'{seq}_{condition}_{a}.json.gz',False) for a in unit['arms']]
        if args.stage=='nominal_origin_controls':paths.append((NOAGE,ROOT/unit['references'][NOAGE]['path'],True))
        for arm,path,reused in paths:
            data=read(path);run=data['runs'];assert run['arm']==arm and data['condition']==condition and data['sequence']==seq
            assert data['inputSha256']==unit['input_sha256'] and data['pd']==.9
            if not reused:assert data['protocol']==cfg['protocol'] and data['stage']==args.stage and data['cohort']==cfg['cohort'] and data['sourceSha256']==mapped
            assert np.array_equal(data['time'],mat['time'].ravel()) and np.array_equal(data['positions'],mat['positions'])
            assert np.array_equal(data['delivered'],delivered)
            for t in range(240):
                assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
            fit=dict(unit['range_detection_model']);fit['probabilities_by_range']={'x'+k:v for k,v in fit['probabilities_by_range'].items()}
            assert run['rangeDetectionModel']==fit and run['rangeDetectionMode']=='nominal'
            events=event_scope(run,data) if not reused else 0
            quality=audit_actual_pd(run,data);view=source_view(packets(run,delivered,240,''))
            if arm==ARM:view['arm']=GCE
            density=audit_noage(view,data) if reused else probability_audit.audit_probability(view,data)
            frame_data,direct=current_records(run,mat,False)
            marks=audit_positive_marks(run,mat,mat['likelihoodRatios']);matching=audit_matching(view,data,frame_data)
            value,_=score_run(data,run);target,frames=summary(data,mat)
            rows.append(dict(sequence=seq,condition=condition,arm=arm,frames=240,**value,reused=reused))
            timeline.extend([dict(condition=condition,arm=arm,**r) for r in frames])
            event_rows=np.asarray(run.get('initialNegativeRecords',[]),float).reshape(-1,18)
            diagnostics.append(dict(condition=condition,arm=arm,quality=quality,density=density,direct=direct,
                positive_mark_rows=marks,globally_checked_pairs=matching,intervened_robot_frames=events,
                event_records=len(event_rows),altered_event_records=int((event_rows[:,6]<0).sum()),target=target))
            if not reused and args.stage=='nominal_origin_controls':
                ref=unit['references'][GCE];reference=ROOT/ref['path'];assert sha(reference)==ref['sha256']
                original=read(reference)['runs'];keys=[k for k in original if k!='runtimeSeconds'];assert set(keys)<=set(run)
                for key in keys:assert run[key]==original[key],(condition,key,'exact original GCE')
                parity.append(dict(condition=condition,arm=arm,exact_fields=keys));hashes[str(reference.relative_to(ROOT))]=sha(reference)
            elif not reused:
                reference=OUT/'results/nominal_origin_controls'/f'{seq}_{condition}_{GCE}.json.gz'
                prior=json.loads((OUT/'audit_nominal_origin_controls.json').read_text());assert sha(reference)==prior['inputs'][str(reference.relative_to(ROOT))]
                parity.append(dict(condition=condition,arm=arm,exact_prefix_fields=exact_prefix(run,read(reference)['runs'])))
                hashes[str(reference.relative_to(ROOT))]=sha(reference)
            else:assert sha(path)==unit['references'][NOAGE]['sha256']
            hashes[str(path.relative_to(ROOT))]=sha(path)
            print('AUDITED NOMINAL ORIGIN',args.stage,condition,arm,flush=True)
    result=dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,inputs=hashes,
        audited_robot_frames=sum(480 for r in rows if not r['reused']),reused_robot_frames=sum(480 for r in rows if r['reused']),
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.py')})
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for prefix,values in [('scores_',rows),('target_frames_',timeline)]:
        path=OUT/(prefix+args.stage+'.csv');assert not path.exists()
        with path.open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(values[0]),lineterminator='\n');writer.writeheader();writer.writerows(values)
    print('NOMINAL NATIVE AUDIT PASSED',args.stage,result['audited_robot_frames'],'new robot frames',flush=True)

if __name__=='__main__':main()
