"""Reconstruct history, moments, fusion, assignments, outputs and bytes."""
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
for folder in ['icra_gaussian_evidence','icra_marked_control','icra_fusion_holdout','icra_reviewer_revision','icra_temporal_association']:
    sys.path.insert(0,str(OUT.parent/folder))
sys.path.insert(0,str(OUT))
from analyze_control import read,score_run
from analyze_holdout import radio_draws
from probability_audit import audit_probability
from history_audit_v2 import audit_positive_marks,audit_matching
from audit_observation_run import current_records
from audit_observation_domain import audit_domain
from audit_screen_assessment_v2 import packets

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence'


def main():
    p=argparse.ArgumentParser();p.add_argument('stage');args=p.parse_args()
    config_path=OUT/'stages'/(args.stage+'.json');cfg=json.loads(config_path.read_text())
    runtime_path=OUT/('runtime_'+args.stage+'.json');runtime=json.loads(runtime_path.read_text())
    assert len(runtime)==len(cfg['units'])
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    mapped={re.sub('[^A-Za-z0-9_]','_',name)[:63]:value for name,value in cfg['source_sha256'].items()}
    assert len(mapped)==len(cfg['source_sha256'])
    rows=[];diagnostics=[];parity=[];hashes={}
    for unit,native in zip(cfg['units'],runtime):
        seq=unit['sequence']
        assert native['sequence']==seq and native['returncode']==0 and native['completion_line']
        assert native['files']==2*len(cfg['arms'])
        log=ROOT/'RUN/ICRA_MISS_HISTORY'/args.stage/(seq+'.log')
        assert 'MISS HISTORY CHECK PASSED' in log.read_text()
        assert f'COMPLETED REVIEW {args.stage} {seq}' in log.read_text()
        hashes[str(log.relative_to(ROOT))]=sha(log)
        mat=loadmat(ROOT/unit['data_path']);T=int(mat['T'].item())
        ratios=mat['likelihoodRatios'] if 'likelihoodRatios' in mat else loadmat(ROOT/unit['ratios_path'])['likelihoodRatios']
        for condition in cfg['conditions']:
            old_path=ROOT/unit['reference_paths'][condition]
            assert sha(old_path)==unit['reference_sha256'][condition]
            for label,name in unit['noage_reference_paths'].items():
                assert sha(ROOT/name)==unit['noage_reference_sha256'][label]
                hashes[name]=sha(ROOT/name)
            old_data=read(old_path);old=old_data['runs']
            if isinstance(old,list):old=next(r for r in old if r['arm']==GCE)
            assert old['arm']==GCE and old_data['inputSha256']==unit['input_sha256']
            delivered=radio_draws(unit['radio_seed']-8301,T,condition)
            assert np.array_equal(old_data['delivered'],delivered)
            if GCE not in cfg['arms']:
                value,_=score_run(old_data,old)
                assert value['wire_bytes']==old['totalWireBytes']
                rows.append(dict(sequence=seq,dataset=unit['dataset'],recording=unit['recording'],condition=condition,
                    arm=GCE,frames=T,**value,reused=True))
            hashes[str(old_path.relative_to(ROOT))]=sha(old_path)
            for arm in cfg['arms']:
                path=OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz';data=read(path);run=data['runs']
                assert data['protocol']=='icra-miss-history-v1' and data['stage']==args.stage and data['cohort']==cfg['cohort']
                assert data['sourceSha256']==mapped and data['inputSha256']==unit['input_sha256']
                assert data['sequence']==seq and data['condition']==condition and run['arm']==arm
                assert data['pd']==cfg['pd'] and np.array_equal(data['time'],mat['time'].ravel())
                assert np.array_equal(data['positions'],mat['positions']) and np.array_equal(data['delivered'],delivered)
                for t in range(T):
                    assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                    assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
                expected_mode='history' if arm.endswith('_miss_history') else 'half' if arm.endswith('_miss_half') else 'original'
                assert run['negativeHistoryMode']==expected_mode
                view=packets(run,delivered,T,'');view['arm']=GCE
                density=audit_probability(view,data)
                frames,direct=current_records(run,mat,False)
                marks=audit_positive_marks(run,mat,ratios)
                matching=audit_matching(run,data,frames)
                domain=audit_domain(run,data)
                value,_=score_run(data,run)
                assert value['wire_bytes']==run['totalWireBytes']
                rows.append(dict(sequence=seq,dataset=unit['dataset'],recording=unit['recording'],condition=condition,
                    arm=arm,frames=T,**value,reused=False))
                diagnostics.append(dict(sequence=seq,condition=condition,arm=arm,density=density,direct=direct,
                    positive_mark_rows=marks,globally_checked_pairs=matching,domain=domain))
                if arm==GCE:
                    keys=[key for key in old if key in run and key not in ['runtimeSeconds']]
                    for key in keys:
                        assert run[key]==old[key],(seq,condition,key,'exact GCE recursion')
                    parity.append(dict(sequence=seq,condition=condition,exact_fields=keys))
                hashes[str(path.relative_to(ROOT))]=sha(path)
                print('AUDITED MISS HISTORY',args.stage,seq,condition,arm,flush=True)
    result=dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,inputs=hashes,
                audited_robot_frames=sum(2*r['frames'] for r in rows if not r['reused']),
                config_sha256=sha(config_path),runtime_sha256=sha(runtime_path),
                auditor_sha256={str(path.relative_to(ROOT)):sha(path) for path in OUT.glob('*.py')})
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/('scores_'+args.stage+'.csv')).open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    print('MISS HISTORY AUDIT PASSED',args.stage,result['audited_robot_frames'],'robot-frames',flush=True)


if __name__=='__main__':main()
