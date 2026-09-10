"""Audit every new native trajectory and its actual refined scalar operands."""
from pathlib import Path
import argparse
import csv
import gzip
import hashlib
import json
import re
import sys
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for folder in ['icra_gaussian_evidence','icra_marked_control','icra_fusion_holdout','icra_reviewer_revision',
               'icra_temporal_association','icra_miss_history','icra_range_detection','icra_recursion_origin','icra_known_censor']:
    sys.path.insert(0,str(OUT.parent/folder))
sys.path.insert(0,str(OUT))
from analyze_control import read,score_run
from analyze_holdout import radio_draws
import probability_audit
from censor_gaussian_audit import check_gaussians
probability_audit.check_gaussians=check_gaussians
from history_audit_v3 import audit_positive_marks,audit_matching
from audit_observation_run import current_records
from prune_event_audit import packets,eligible_rows
from event_audit import eligible_rows as local_eligible_rows
from censor_range_audit import source_view,audit_actual_pd,audit_noage
from native_summary import summarize
from population_audit import audit_population
sys.path.insert(0,str(OUT))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SUFFIX='_prune_info'

def storage_value(value):
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

def exact_prefix(run,control):
    events=np.asarray(run['pruneInformationRecords'],float).reshape(-1,24)
    changed=events[(events[:,6]==2)&(events[:,4]!=events[:,5])]
    assert len(changed);first=int(changed[:,0].min());checked=[]
    for name in ['localIncrementRecords','localGaussianRecords','qualityRecords','packetGaussianRecords',
                 'localDirectRecords','packetDirectRecords','fusionSourceRecords']:
        assert [r for r in run[name] if r[0]<=first]==[r for r in control[name] if r[0]<=first],name;checked.append(name)
    for name,omit in [('iterationRecords',{6,9}),('fusionOutputRecords',{4})]:
        assert [r for r in run[name] if r[0]<first]==[r for r in control[name] if r[0]<first],name
        a=[[x for i,x in enumerate(r) if i not in omit] for r in run[name] if r[0]==first]
        b=[[x for i,x in enumerate(r) if i not in omit] for r in control[name] if r[0]==first]
        assert a==b,name;checked.append(name+'_except_actual_r_at_first_peer_event')
    for name in ['estimates','rawEstimates','labels']:
        assert run[name][:2*(first-1)]==control[name][:2*(first-1)],name;checked.append(name)
    assert run['localAssociationWeights'][:2*first]==control['localAssociationWeights'][:2*first];checked.append('localAssociationWeights')
    for name in ['basePacketBytes','matchedLabels']:
        assert [r[:first] for r in run[name]]==[r[:first] for r in control[name]],name;checked.append(name)
    for name in ['controlBytes','attemptedMessages','deliveredMessages']:
        assert run[name]==control[name],name;checked.append(name)
    return dict(first_changed_frame=first,exact_prefix_fields=checked)


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prune_info_controls','prune_info_shared']);args=p.parse_args()
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    runtimepath=OUT/('runtime_'+args.stage+'.json');runtime=json.loads(runtimepath.read_text());freeze=json.loads((OUT/'FREEZE.json').read_text())
    for name,h in freeze['configurations'].items():assert sha(ROOT/name)==h,name
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    mapped={re.sub('[^A-Za-z0-9_]','_',name)[:63]:h for name,h in cfg['source_sha256'].items()}
    assert len(mapped)==len(cfg['source_sha256']) and len(runtime)==len(cfg['units'])==2
    hashes={str(cfgpath.relative_to(ROOT)):sha(cfgpath),str(runtimepath.relative_to(ROOT)):sha(runtimepath)}
    rows=[];diagnostics=[];parity=[];timeline=[];population_rows=[]
    for u,native in zip(cfg['units'],runtime):
        seq=u['sequence'];condition=u['execution_id'];assert u['conditions']==[condition]
        assert native['sequence']==seq and native['execution_id']==condition
        assert native['returncode']==0 and native['completion_line'] and native['files']==native['expected_files']==3
        log=ROOT/'RUN/ICRA_PRUNE_INFORMATION'/args.stage/(condition+'.log');content=log.read_text()
        for marker in ['PRUNE INFORMATION CHECK PASSED','KNOWN CENSOR CHECK PASSED','RANGE DETECTION CHECK PASSED',f'COMPLETED REVIEW {args.stage} {seq}']:assert marker in content
        hashes[str(log.relative_to(ROOT))]=sha(log)
        assert sha(ROOT/u['data_path'])==u['input_sha256'] and sha(ROOT/u['pose_path'])==u['pose_sha256']
        mat=loadmat(ROOT/u['data_path']);assert int(mat['T'].item())==240;delivered=radio_draws(u['radio_seed']-8301,240,condition)
        for arm in u['arms']:
            path=OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz';data=read(path);run=data['runs'];base=arm.removesuffix(SUFFIX).removesuffix('_known_censor')
            assert run['arm']==arm and data['condition']==condition and data['sequence']==seq and data['inputSha256']==u['input_sha256']
            assert data['protocol']==cfg['protocol'] and data['stage']==args.stage and data['cohort']==cfg['cohort'] and data['sourceSha256']==mapped
            assert data['pd']==.9 and np.array_equal(data['time'],mat['time'].ravel()) and np.array_equal(data['positions'],mat['positions'])
            assert np.array_equal(data['delivered'],delivered)
            for t in range(240):
                assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
            fit=dict(u['range_detection_model']);fit['probabilities_by_range']={'x'+k:v for k,v in fit['probabilities_by_range'].items()}
            assert run['rangeDetectionModel']==fit and run['rangeDetectionMode']=='nominal' and run['knownCensorEnabled']==arm.endswith('_known_censor') and run['pruneInformationEnabled']==arm.endswith(SUFFIX)
            quality=audit_actual_pd(run,data);packet_view,communication=packets(run,delivered,240);view=source_view(packet_view);view['arm']=base
            density=audit_noage(view,data) if base=='marked_lineage' else probability_audit.audit_probability(view,data)
            frame_data,direct=current_records(run,mat,False);marks=audit_positive_marks(run,mat,mat['likelihoodRatios'])
            matching=audit_matching(view,data,frame_data);value,_=score_run(data,run);target,frames=summarize(data,mat)
            target['windows']={k:v for k,v in target['windows'].items() if k in ['full','original_window']}
            pop=audit_population(data,mat)
            if run['pruneInformationEnabled']:
                ix,_=eligible_rows(view,np.asarray(view['iterationRecords']).reshape(-1,60))
                events=np.asarray(run['pruneInformationRecords'],float).reshape(-1,24)
            else:
                ix,_=local_eligible_rows(view,np.asarray(view['iterationRecords']).reshape(-1,60))
                events=np.asarray(run['knownCensorRecords'],float).reshape(-1,25)
            assert len(events)==len(ix)
            rows.append(dict(sequence=seq,condition=condition,arm=arm,frames=240,**value))
            timeline.extend(dict(condition=condition,arm=arm,**r) for r in frames)
            population_rows.extend(dict(condition=condition,arm=arm,**r) for r in pop.pop('population_rows'))
            eventpath=OUT/'results'/args.stage/f'{seq}_{condition}_{arm}_reimport.csv.gz';assert not eventpath.exists()
            with gzip.open(eventpath,'wt',newline='') as stream:
                w=csv.DictWriter(stream,fieldnames=list(pop['reimport_events'][0]),lineterminator='\n');w.writeheader();w.writerows(pop.pop('reimport_events'))
            hashes[str(eventpath.relative_to(ROOT))]=sha(eventpath)
            diagnostics.append(dict(condition=condition,arm=arm,quality=quality,density=density,communication=communication,direct=direct,positive_mark_rows=marks,
                globally_checked_pairs=matching,eligible_labels=len(ix),event_records=len(events),changed_event_records=int((events[:,4]!=events[:,5]).sum()),
                target=target,population=pop,reimport_event_path=str(eventpath.relative_to(ROOT))))
            if args.stage=='prune_info_controls':
                ref=u['references'][arm];reference=ROOT/ref['path'];assert sha(reference)==ref['sha256'];original=read(reference)['runs']
                keys=[k for k in original if k!='runtimeSeconds'];assert set(keys)<=set(run)
                for key in keys:assert run[key]==original[key],(condition,arm,key,'exact original control')
                parity.append(dict(condition=condition,arm=arm,exact_fields=keys))
            else:
                reference=OUT/'results/prune_info_controls'/f'{seq}_{condition}_{base}_known_censor.json.gz'
                audit=json.loads((OUT/'audit_prune_info_controls.json').read_text());assert sha(reference)==audit['inputs'][str(reference.relative_to(ROOT))]
                parity.append(dict(condition=condition,arm=arm,**exact_prefix(run,read(reference)['runs'])))
            hashes[str(reference.relative_to(ROOT))]=sha(reference);hashes[str(path.relative_to(ROOT))]=sha(path)
            print('AUDITED PRUNE INFORMATION',args.stage,condition,arm,flush=True)
    for prefix,values in [('scores_',rows),('target_frames_',timeline),('population_frames_',population_rows)]:
        path=OUT/(prefix+args.stage+'.csv');assert not path.exists()
        with path.open('w') as stream:
            w=csv.DictWriter(stream,fieldnames=list(values[0]),lineterminator='\n');w.writeheader();w.writerows(values)
        hashes[str(path.relative_to(ROOT))]=sha(path)
    destination.write_text(json.dumps(dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,
        inputs=hashes,audited_robot_frames=2880,source_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.py')}),indent=2,allow_nan=False,default=storage_value)+'\n')
    print('PRUNE INFORMATION NATIVE AUDIT PASSED',args.stage,2880,'robot frames',flush=True)

if __name__=='__main__':main()
