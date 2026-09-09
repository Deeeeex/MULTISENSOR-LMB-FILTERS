"""Verify exits, source/input hashes, full distributions, extraction and scores."""
from pathlib import Path
import argparse
import csv
import json
import re
import sys

import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for directory in ['icra_gaussian_evidence','icra_marked_control','icra_fusion_holdout','icra_reviewer_revision','icra_admission_revision','icra_projected_admission']:
    sys.path.insert(0,str(OUT.parent/directory))
sys.path.insert(0,str(OUT))
from analyze_control import sha,read,score_run
from analyze_full import baseline
from analyze_holdout import radio_draws
from review_probability_audit import audit_probability as audit_original_probability
from admission_probability_audit import audit_probability as audit_admission_probability
from projected_probability_audit_v2 import audit_probability as audit_projected_probability
from compatible_probability_audit import audit_probability as audit_new_probability

def audit_probability(run, data):
    if '_compatible_' in run['arm']:
        return audit_new_probability(run, data)
    if '_projected_' in run['arm']:
        return audit_projected_probability(run, data)
    if '_decoupled_' in run['arm'] or '_fixedx_' in run['arm']:
        return audit_admission_probability(run, data)
    return audit_original_probability(run, data)


def audit_packets(run,delivery,T):
    packets=np.asarray(run['packetBytes'])
    width=352 if 'gaussian_evidence' in run['arm'] else (232 if run['arm']=='marked_asymmetric' else 216)
    assert packets.shape==(2,T) and np.isfinite(packets).all()
    assert np.all(packets>=32) and np.all((packets-32)%width==0)
    assert np.all((packets-32)/width<=run['maximumBernoulliCount'])
    assert run['attemptedMessages']==[2]*T and run['controlBytes']==[256]*T
    assert run['deliveredMessages']==delivery.sum((0,1)).tolist()
    assert run['rawPayloadBytes']==packets.sum(0).tolist()
    assert run['deliveredRawBytes']==(delivery.sum(0)*packets).sum(0).tolist()
    assert run['wireBytes']==(np.ceil(packets/16384).sum(0)*16384+256).tolist()
    assert run['totalWireBytes']==sum(run['wireBytes'])


def main():
    p=argparse.ArgumentParser();p.add_argument('stage');args=p.parse_args()
    cfgpath=OUT/'stages'/f'{args.stage}.json';cfg=json.loads(cfgpath.read_text())
    runtime=json.loads((OUT/f'runtime_{args.stage}.json').read_text())
    assert len(runtime)==len(cfg['units'])
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    source={re.sub('[^A-Za-z0-9_]','_',name)[:63]:value for name,value in cfg['source_sha256'].items()}
    assert len(source)==len(cfg['source_sha256'])
    hashes={};rows=[];diagnostics=[];parity=[]
    for unit,native in zip(cfg['units'],runtime):
        name=unit['sequence'];assert native['sequence']==name
        assert native['returncode']==0 and native['completion_line'] and native['files']==2*len(cfg['arms'])
        log=ROOT/'RUN/ICRA_REVIEWER_REVISION'/args.stage/f'{name}.log'
        assert f'COMPLETED REVIEW {args.stage} {name}' in log.read_text()
        assert 'REVIEW CONTROL CHECK PASSED' in log.read_text()
        assert 'ADMISSION CHECK PASSED' in log.read_text()
        assert 'PROJECTED CHECK PASSED' in log.read_text()
        assert 'COMPATIBLE CHECK PASSED' in log.read_text()
        mat=loadmat(ROOT/unit['data_path']);T=int(mat['T'].item())
        for condition in cfg['conditions']:
            for arm in cfg['arms']:
                path=OUT/'results'/args.stage/f'{name}_{condition}_{arm}.json.gz'
                data=read(path);run=data['runs'];hashes[str(path.relative_to(ROOT))]=sha(path)
                assert data['protocol']=='icra-reviewer-revision-v1' and data['stage']==args.stage
                assert data['sourceSha256']==source and data['inputSha256']==unit['input_sha256']
                assert data['cohort']==cfg['cohort'] and data['sequence']==name and data['condition']==condition
                assert data['pd']==cfg['pd'] and run['arm']==arm
                assert np.array_equal(data['time'],mat['time'].ravel())
                assert np.array_equal(data['positions'],mat['positions'])
                for t in range(T):
                    assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),mat['truth'][0,t])
                    assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),mat['truthIds'][0,t].ravel())
                delivery=radio_draws(unit['radio_seed']-8301,T,condition)
                assert np.array_equal(delivery,data['delivered'])
                audit_packets(run,delivery,T)
                row,_=score_run(data,run)
                rows.append(dict(sequence=name,condition=condition,arm=arm,frames=T,**row))
                if arm not in ['marked_lineage','marked_er']:
                    diagnostics.append(dict(sequence=name,condition=condition,**audit_probability(run,data)))
                if cfg['preflight'] and arm in ['marked_lineage','marked_asymmetric','marked_gaussian_evidence']:
                    if arm=='marked_lineage':
                        original,_,_,oldpath=baseline('development',name,condition,arm)
                        old=next(r for r in original['runs'] if r['arm']==arm)
                        keys=['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount',
                              'rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages',
                              'deliveredMessages','maximumBernoulliCount']
                    else:
                        folder='icra_gaussian_evidence' if arm=='marked_gaussian_evidence' else 'icra_asymmetric_evidence'
                        oldpath=OUT.parent/folder/'results_development'/f'{name}_{condition}_{arm}.json.gz'
                        old=read(oldpath)['runs'];keys=[k for k in old if k!='runtimeSeconds']
                    for key in keys:
                        value=run[key]
                        if arm=='marked_asymmetric' and key=='iterationRecords':
                            value=np.asarray(value)[:,:37].tolist()
                        assert value==old[key],(arm,condition,'exact recursive parity',key)
                    hashes[str(oldpath.relative_to(ROOT))]=sha(oldpath)
                    parity.append(dict(arm=arm,condition=condition,node_frames=2*T,exact_fields=keys))
                print('AUDITED',args.stage,name,condition,arm,flush=True)
    result=dict(passed=True,stage=args.stage,cohort=cfg['cohort'],sequences=len(cfg['units']),
                arms=cfg['arms'],pd=cfg['pd'],rows=rows,diagnostics=diagnostics,parity=parity,
                audited_node_frames=sum(2*r['frames'] for r in rows),inputs=hashes,
                config_sha256=sha(cfgpath),runtime_sha256=sha(OUT/f'runtime_{args.stage}.json'),
                auditor_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),OUT/'compatible_probability_audit.py',OUT/'compatible_gaussian_audit.py',OUT/'compatibility_math.py',OUT.parent/'icra_projected_admission/projected_probability_audit_v2.py',OUT.parent/'icra_projected_admission/projected_gaussian_audit.py',OUT.parent/'icra_projected_admission/projected_ratio.py',OUT.parent/'icra_admission_revision/admission_probability_audit.py',OUT.parent/'icra_admission_revision/admission_gaussian_audit.py',OUT.parent/'icra_reviewer_revision/review_probability_audit.py',OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py']})
    dest=OUT/f'audit_{args.stage}.json';assert not dest.exists()
    dest.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/f'scores_{args.stage}.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    print('REVIEW STAGE AUDIT PASSED',args.stage,result['audited_node_frames'],'node-frames',flush=True)


if __name__=='__main__':main()
