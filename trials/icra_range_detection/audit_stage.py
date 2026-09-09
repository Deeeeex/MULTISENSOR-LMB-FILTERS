"""Verify frozen inputs, executed sensor model, full fusion, recursion parity and scores."""
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
for folder in ['icra_gaussian_evidence','icra_marked_control','icra_fusion_holdout','icra_reviewer_revision','icra_temporal_association','icra_miss_history']:
    sys.path.insert(0,str(OUT.parent/folder))
sys.path.insert(0,str(OUT))
from analyze_control import read,score_run
from analyze_holdout import radio_draws
from probability_audit import audit_probability
from history_audit_v3 import audit_positive_marks,audit_matching
from audit_observation_run import current_records
from audit_screen_assessment_v2 import packets
from range_audit import base_arm,audit_actual_pd,source_view,audit_noage

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence'

def main():
    p=argparse.ArgumentParser();p.add_argument('stage');args=p.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    runtimepath=OUT/('runtime_'+args.stage+'.json');runtime=json.loads(runtimepath.read_text())
    assert len(runtime)==len(cfg['units'])
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    mapped={re.sub('[^A-Za-z0-9_]','_',name)[:63]:value for name,value in cfg['source_sha256'].items()}
    assert len(mapped)==len(cfg['source_sha256'])
    rows=[];diagnostics=[];parity=[];hashes={}
    for unit,native in zip(cfg['units'],runtime):
        seq=unit['sequence'];assert native['sequence']==seq and native['returncode']==0 and native['completion_line']
        assert native['files']==2*len(cfg['arms'])
        log=ROOT/'RUN/ICRA_RANGE_DETECTION'/args.stage/(seq+'.log')
        assert 'RANGE DETECTION CHECK PASSED' in log.read_text() and f'COMPLETED REVIEW {args.stage} {seq}' in log.read_text()
        hashes[str(log.relative_to(ROOT))]=sha(log)
        mat=loadmat(ROOT/unit['data_path']);T=int(mat['T'].item())
        ratios=mat['likelihoodRatios'] if 'likelihoodRatios' in mat else loadmat(ROOT/unit['ratios_path'])['likelihoodRatios']
        for condition in cfg['conditions']:
            oldpath=ROOT/unit['reference_paths'][condition];assert sha(oldpath)==unit['reference_sha256'][condition]
            olddata=read(oldpath);old=olddata['runs']
            if isinstance(old,list):old=next(r for r in old if r['arm']==GCE)
            assert old['arm']==GCE and olddata['inputSha256']==unit['input_sha256']
            delivered=radio_draws(unit['radio_seed']-8301,T,condition)
            assert np.array_equal(olddata['delivered'],delivered)
            if GCE not in cfg['arms']:
                value,_=score_run(olddata,old)
                rows.append(dict(sequence=seq,dataset=unit['dataset'],recording=unit['recording'],condition=condition,
                    arm=GCE,frames=T,**value,reused=True))
            hashes[str(oldpath.relative_to(ROOT))]=sha(oldpath)
            for arm in cfg['arms']:
                path=OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz';data=read(path);run=data['runs']
                assert data['protocol']==cfg['protocol'] and data['stage']==args.stage and data['cohort']==cfg['cohort']
                assert data['sourceSha256']==mapped and data['inputSha256']==unit['input_sha256']
                assert data['sequence']==seq and data['condition']==condition and run['arm']==arm
                assert data['pd']==cfg['pd'] and np.array_equal(data['time'],mat['time'].ravel())
                assert np.array_equal(data['positions'],mat['positions']) and np.array_equal(data['delivered'],delivered)
                for t in range(T):
                    assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                    assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
                assert run['rangeDetectionModel']==unit['range_detection_model']
                expected_mode='range' if arm.endswith('_range') else 'constant' if arm.endswith('_constant') else 'nominal'
                assert run['rangeDetectionMode']==expected_mode
                quality=audit_actual_pd(run,data)
                view=packets(run,delivered,T,'');view=source_view(view)
                density=audit_noage(view,data) if base_arm(arm)=='marked_lineage' else audit_probability(view,data)
                frames,direct=current_records(run,mat,False)
                marks=audit_positive_marks(run,mat,ratios);matching=audit_matching(view,data,frames)
                value,_=score_run(data,run);assert value['wire_bytes']==run['totalWireBytes']
                rows.append(dict(sequence=seq,dataset=unit['dataset'],recording=unit['recording'],condition=condition,
                    arm=arm,frames=T,**value,reused=False))
                diagnostics.append(dict(sequence=seq,condition=condition,arm=arm,quality=quality,density=density,
                    direct=direct,positive_mark_rows=marks,globally_checked_pairs=matching))
                if expected_mode=='nominal':
                    reference=ROOT/unit['parity_paths'][condition][arm]
                    assert sha(reference)==unit['parity_sha256'][condition][arm]
                    reference_data=read(reference);reference_run=reference_data['runs']
                    if isinstance(reference_run,list):reference_run=next(r for r in reference_run if r['arm']==arm)
                    assert reference_data['inputSha256']==unit['input_sha256']
                    keys=[key for key in reference_run if key in run and key not in ['runtimeSeconds']]
                    for key in keys:assert run[key]==reference_run[key],(seq,condition,arm,key,'exact nominal recursion')
                    parity.append(dict(sequence=seq,condition=condition,arm=arm,exact_fields=keys))
                    hashes[str(reference.relative_to(ROOT))]=sha(reference)
                hashes[str(path.relative_to(ROOT))]=sha(path)
                print('AUDITED RANGE DETECTION',args.stage,seq,condition,arm,flush=True)
    result=dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,inputs=hashes,
        audited_robot_frames=sum(2*r['frames'] for r in rows if not r['reused']),
        config_sha256=sha(cfgpath),runtime_sha256=sha(runtimepath),
        auditor_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.py')})
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/('scores_'+args.stage+'.csv')).open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    print('RANGE DETECTION AUDIT PASSED',args.stage,result['audited_robot_frames'],'robot-frames',flush=True)

if __name__=='__main__':main()
