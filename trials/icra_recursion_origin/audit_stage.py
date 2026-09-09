"""Check full native equations, one-time intervention scope, parity, extraction and scores."""
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
               'icra_temporal_association','icra_miss_history','icra_range_detection']:
    sys.path.insert(0,str(OUT.parent/folder))
sys.path.insert(0,str(OUT))
from analyze_control import read,score_run
from analyze_holdout import radio_draws
import probability_audit
from intervention_gaussian_audit import check_gaussians
probability_audit.check_gaussians=check_gaussians
from history_audit_v3 import audit_positive_marks,audit_matching
from audit_observation_run import current_records
from audit_screen_assessment_v2 import packets
from range_audit import audit_actual_pd,source_view,audit_noage
from native_summary import summarize

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GS='marked_gaussian_evidence_guarded_scalar_range'
GCE='marked_gaussian_evidence_range'

def event_scope(run,data):
    mode=run['interventionMode'];assert run['interventionFrame']==3
    expected=np.zeros((2,len(data['time'])),bool)
    if '_once_' in run['arm']:
        assert mode==run['arm'].split('_once_')[1].removesuffix('_range')
        expected[:,2]=[data['delivered'][0][1][2],data['delivered'][1][0][2]]
    else:assert mode=='none' and not run['interventionRecords']
    assert np.array_equal(run['interventionMask'],expected)
    return int(expected.sum())

def prefix_parity(run,control):
    local=['localIncrementRecords','localGaussianRecords','qualityRecords','packetGaussianRecords','localDirectRecords','packetDirectRecords']
    fused=['iterationRecords','fusionSourceRecords','fusionOutputRecords']
    checked=[]
    for name in local+fused:
        limit=3 if name in local else 2
        a=[r for r in run[name] if r[0]<=limit];b=[r for r in control[name] if r[0]<=limit]
        assert a==b,('exact prefix',name);checked.append(name)
    for name in ['estimates','rawEstimates','labels','localAssociationWeights']:
        stop=6 if name=='localAssociationWeights' else 4
        assert run[name][:stop]==control[name][:stop],('exact prefix',name);checked.append(name)
    for name in ['packetBytes','ospa','countError','matchedSquaredError','matchedCount','matchedLabels']:
        stop=3 if name=='packetBytes' else 2
        assert [r[:stop] for r in run[name]]==[r[:stop] for r in control[name]],('exact prefix',name);checked.append(name)
    for name in ['rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages','deliveredMessages']:
        assert run[name][:3]==control[name][:3],('exact prefix',name);checked.append(name)
    return checked

def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage');args=parser.parse_args()
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    runtimepath=OUT/('runtime_'+args.stage+'.json');runtime=json.loads(runtimepath.read_text())
    freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text())
    for name,expected in freeze['configurations'].items():assert sha(ROOT/name)==expected,name
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    mapped={re.sub('[^A-Za-z0-9_]','_',name)[:63]:value for name,value in cfg['source_sha256'].items()}
    assert len(mapped)==len(cfg['source_sha256']) and len(runtime)==len(cfg['units'])
    rows=[];diagnostics=[];parity=[];timelines=[];hashes={str(cfgpath.relative_to(ROOT)):sha(cfgpath),str(runtimepath.relative_to(ROOT)):sha(runtimepath)}
    for unit,native in zip(cfg['units'],runtime):
        seq=unit['sequence'];eid=unit['execution_id']
        assert (native['sequence'],native['execution_id'])==(seq,eid)
        assert native['returncode']==0 and native['completion_line']
        assert native['files']==native['expected_files']==len(cfg['conditions'])*len(unit['arms'])
        log=ROOT/'RUN/ICRA_RECURSION_ORIGIN'/args.stage/(eid+'.log');content=log.read_text()
        for marker in ['RANGE DETECTION CHECK PASSED','RECURSION INTERVENTION CHECK PASSED',f'COMPLETED REVIEW {args.stage} {seq}']:
            assert marker in content,marker
        hashes[str(log.relative_to(ROOT))]=sha(log)
        assert sha(ROOT/unit['data_path'])==unit['input_sha256']
        assert sha(ROOT/unit['pose_path'])==unit['pose_sha256']
        mat=loadmat(ROOT/unit['data_path']);T=int(mat['T'].item());assert T==240
        ratios=mat['likelihoodRatios'];delivered=radio_draws(unit['radio_seed']-8301,T,'intermittent')
        paths=[(arm,OUT/'results'/args.stage/f'{seq}_intermittent_{arm}.json.gz',False) for arm in unit['arms']]
        if args.stage=='recursion_preflight':
            ref=unit['references']['marked_lineage_range'];paths.append(('marked_lineage_range',ROOT/ref['path'],True))
        for arm,path,reused in paths:
            data=read(path);run=data['runs'];assert run['arm']==arm
            assert data['inputSha256']==unit['input_sha256'] and data['sequence']==seq and data['condition']=='intermittent'
            if not reused:
                assert data['protocol']==cfg['protocol'] and data['stage']==args.stage and data['cohort']==cfg['cohort']
                assert data['sourceSha256']==mapped
            assert data['pd']==cfg['pd'] and np.array_equal(data['time'],mat['time'].ravel())
            assert np.array_equal(data['positions'],mat['positions']) and np.array_equal(data['delivered'],delivered)
            for t in range(T):
                assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
            fit=dict(unit['range_detection_model']);fit['probabilities_by_range']={'x'+k:v for k,v in fit['probabilities_by_range'].items()}
            assert run['rangeDetectionModel']==fit and run['rangeDetectionMode']=='range'
            events=event_scope(run,data) if not reused else 0
            quality=audit_actual_pd(run,data);view=source_view(packets(run,delivered,T,''))
            if '_once_' in arm:view['arm']='marked_gaussian_evidence_guarded_scalar'
            density=audit_noage(view,data) if reused else probability_audit.audit_probability(view,data)
            frames,direct=current_records(run,mat,False)
            marks=audit_positive_marks(run,mat,ratios);matching=audit_matching(view,data,frames)
            value,_=score_run(data,run);assert value['wire_bytes']==run['totalWireBytes']
            target,timeline=summarize(data,mat);timelines.extend([dict(arm=arm,**r) for r in timeline])
            rows.append(dict(sequence=seq,condition='intermittent',arm=arm,frames=T,**value,reused=reused))
            diagnostics.append(dict(arm=arm,quality=quality,density=density,direct=direct,positive_mark_rows=marks,
                globally_checked_pairs=matching,intervened_robot_frames=events,target=target))
            if not reused and args.stage=='recursion_preflight':
                ref=unit['references'][arm];reference=ROOT/ref['path'];assert sha(reference)==ref['sha256']
                original=read(reference)['runs'];keys=[k for k in original if k in run and k!='runtimeSeconds']
                assert len(keys)==len(original)-1
                for key in keys:assert run[key]==original[key],(arm,key,'exact original recursion')
                parity.append(dict(arm=arm,exact_fields=keys));hashes[str(reference.relative_to(ROOT))]=sha(reference)
            elif not reused:
                reference=OUT/'results/recursion_preflight'/f'{seq}_intermittent_{GS}.json.gz'
                prior=json.loads((OUT/'audit_recursion_preflight.json').read_text())
                assert sha(reference)==prior['inputs'][str(reference.relative_to(ROOT))]
                parity.append(dict(arm=arm,exact_prefix_fields=prefix_parity(run,read(reference)['runs'])))
                hashes[str(reference.relative_to(ROOT))]=sha(reference)
            else:assert sha(path)==unit['references'][arm]['sha256']
            hashes[str(path.relative_to(ROOT))]=sha(path)
            print('AUDITED RECURSION',args.stage,arm,flush=True)
    result=dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,inputs=hashes,
        audited_robot_frames=sum(2*r['frames'] for r in rows if not r['reused']),
        reused_robot_frames=sum(2*r['frames'] for r in rows if r['reused']),
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.py')})
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for name,values in [('scores_',rows),('target_frames_',timelines)]:
        path=OUT/(name+args.stage+'.csv');assert not path.exists()
        with path.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(values[0]),lineterminator='\n');writer.writeheader();writer.writerows(values)
    print('RECURSION AUDIT PASSED',args.stage,result['audited_robot_frames'],'new robot frames',flush=True)

if __name__=='__main__':main()
