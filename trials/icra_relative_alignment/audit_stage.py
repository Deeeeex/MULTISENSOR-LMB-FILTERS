"""Independently validate input correction, distributions, crop, scores and costs."""
from pathlib import Path
import argparse
import csv
import json
import sys
import numpy as np
from scipy.io import loadmat
from scipy.special import expit
from common import OUT,ROOT,ARMS,sha,write_new,frozen

for folder in ['icra_fusion_holdout','icra_marked_control','icra_reviewer_revision']:
    sys.path.insert(0,str(OUT.parent/folder))
from analyze_control import read,score_run
from review_probability_audit import audit_probability
# Load only the existing packet auditor, avoiding this file's same basename.
import importlib.util
spec=importlib.util.spec_from_file_location('alignment_original_packet_auditor',OUT.parent/'icra_reviewer_revision/audit_stage.py')
packet_auditor=importlib.util.module_from_spec(spec);spec.loader.exec_module(packet_auditor)

def selected_run(data,arm):
    runs=data['runs']
    return next(r for r in runs if r['arm']==arm) if isinstance(runs,list) else runs

def geometry_check(run,positions,pd):
    local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    increments=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    lookup={tuple(row[:4].astype(int)):row for row in increments}
    assert len(lookup)==len(increments) and np.isfinite(local).all()
    for row in local:
        frame,source,birth,label=row[:4].astype(int);xy=row[4:6];centers=positions[:,:,frame-1]
        distances=np.sum((xy[:,None]-centers)**2,axis=0)
        inside=bool(abs(xy[0])<=70.4 and abs(xy[1])<=40 and np.sqrt(distances[source-1])<=40 and (distances>9).all())
        before=lookup[frame,source,birth,label]
        assert before[10]==(pd if inside else 0),(frame,source,birth,label,'corrected model pD')
    return len(local)

def audit_noage(run):
    rows=np.asarray(run['iterationRecords'],float).reshape(-1,60)
    b=rows[:,13:15];r=rows[:,17:19];active=b>0
    assert np.isfinite(rows[:,:17]).all() and np.isfinite(r[active]).all()
    assert np.isin(b,[0,.5,1]).all() and np.allclose(b.sum(1),1,rtol=0,atol=2e-14)
    logits=np.zeros_like(r);values=np.clip(r[active],1e-9,1-1e-9)
    logits[active]=np.log(values)-np.log1p(-values)
    expected=expit((b*logits).sum(1)+rows[:,10])
    for column in [6,7,9]:assert np.allclose(rows[:,column],expected,rtol=0,atol=2e-12)
    return dict(noage_fusion_existence_records=len(rows))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=['parity','corrected']);args=parser.parse_args()
    method=frozen();stage='alignment_'+args.kind;cfgpath=OUT/'stages'/f'{stage}.json';cfg=json.loads(cfgpath.read_text())
    ledger=OUT/f'runtime_{stage}.json';runtime=json.loads(ledger.read_text());assert len(runtime)==len(cfg['units'])
    assert cfg['source_sha256']['method_freeze']==sha(OUT/'FREEZE.json')
    zero=cfg['alignment_mode']=='zero';assert zero==(args.kind=='parity')
    artifacts={str(cfgpath.relative_to(ROOT)):sha(cfgpath),str(ledger.relative_to(ROOT)):sha(ledger)}
    rows=[];references=[];diagnostics=[];parity=[];geometry_rows=0
    for unit,native in zip(cfg['units'],runtime):
        name=unit['sequence'];T=unit['frames']
        assert native['sequence']==name and native['returncode']==0 and native['completion_line'] and native['files']==2
        log=ROOT/native['log'];assert sha(log)==native['log_sha256'];artifacts[native['log']]=sha(log)
        assert f'COMPLETED REVIEW {stage} {name}' in log.read_text()
        raw=loadmat(ROOT/unit['data_path']);assert int(raw['T'].item())==T
        alignment=np.zeros((2,T)) if zero else loadmat(ROOT/unit['alignment_path'])['translations']
        model_positions=raw['positions'].copy();model_positions[:,1,:]+=alignment
        for arm in ARMS:
            path=OUT/'results'/stage/f'{name}_reliable_{arm}.json.gz';data=read(path);run=data['runs']
            artifacts[str(path.relative_to(ROOT))]=sha(path)
            assert data['protocol']==method['protocol'] and data['stage']==stage and data['cohort']==stage
            assert data['sourceSha256']==cfg['source_sha256'] and data['inputSha256']==unit['input_sha256']
            assert data['sequence']==name and data['condition']=='reliable' and data['pd']==.9 and run['arm']==arm
            assert data['alignmentMode']==cfg['alignment_mode']
            assert np.array_equal(data['alignmentTranslations'],alignment)
            assert np.array_equal(data['positions'],raw['positions']) and np.array_equal(data['modelPositions'],model_positions)
            assert np.array_equal(data['time'],raw['time'].ravel())
            for t in range(T):
                assert np.array_equal(np.asarray(data['truth'][t]).reshape(4,-1),raw['truth'][0,t])
                assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(),raw['truthIds'][0,t].ravel())
                for source in [0,1]:
                    expected=raw['measurements'][source,t]+(alignment[:,t,None] if source==1 else 0.)
                    assert np.array_equal(np.asarray(data['measurements'][source+2*t]).reshape(2,-1),expected)
            delivery=np.broadcast_to(np.array([[False,True],[True,False]])[:,:,None],(2,2,T))
            assert np.array_equal(data['delivered'],delivery)
            packet_auditor.audit_packets(run,delivery,T)
            extra=0 if zero else 1
            assert run['alignmentPayloadBytes']==[extra*8064]*T and run['alignmentWireBytes']==[extra*33024]*T
            assert run['totalWithAlignmentWireBytes']==run['totalWireBytes']+extra*33024*T
            if zero:assert run['alignmentComputeSeconds']==0
            else:
                prepared=loadmat(ROOT/unit['alignment_path']);seconds=prepared['gridSeconds'].sum()+prepared['solveSeconds'].sum()
                assert abs(run['alignmentComputeSeconds']-seconds)<=1e-9
            geometry_rows+=geometry_check(run,model_positions,.9)
            diagnostic=audit_probability(run,data) if arm==ARMS[0] else audit_noage(run)
            diagnostics.append(dict(sequence=name,**diagnostic))
            score,_=score_run(data,run)
            rows.append(dict(dataset=unit['dataset'],sequence=name,arm=arm,mode=cfg['alignment_mode'],frames=T,
                             **score,alignment_payload_bytes=sum(run['alignmentPayloadBytes']),
                             alignment_wire_bytes=sum(run['alignmentWireBytes']),total_wire_bytes=run['totalWithAlignmentWireBytes'],
                             alignment_compute_seconds=run['alignmentComputeSeconds']))
            ref=unit['references'][arm];olddata=read(ROOT/ref['path']);old=selected_run(olddata,arm)
            assert sha(ROOT/ref['path'])==ref['sha256'];artifacts[ref['path']]=ref['sha256']
            for key in ['time','positions','truth','truthIds','delivered']:assert olddata[key]==data[key],(name,arm,'same benchmark',key)
            old_score,_=score_run(olddata,old)
            references.append(dict(dataset=unit['dataset'],sequence=name,arm=arm,mode='original',frames=T,**old_score,
                                   alignment_payload_bytes=0,alignment_wire_bytes=0,total_wire_bytes=old_score['wire_bytes'],alignment_compute_seconds=0.))
            if zero:
                keys=['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount']
                if arm==ARMS[0]:keys += ['localIncrementRecords','localGaussianRecords','packetGaussianRecords','iterationRecords','packetBytes']
                for key in keys:assert run[key]==old[key],(name,arm,'exact zero-correction parity',key)
                parity.append(dict(sequence=name,arm=arm,robot_frames=2*T,exact_fields=keys))
            print('NATIVE ALIGNMENT INDEPENDENTLY VERIFIED',stage,name,arm,flush=True)
    expected=4 if zero else 28
    assert len(rows)==len(references)==expected
    csvpath=OUT/f'SCORES_{stage}.csv';assert not csvpath.exists()
    with csvpath.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows+references)
    artifacts[str(csvpath.relative_to(ROOT))]=sha(csvpath)
    write_new(OUT/f'audit_{stage}.json',dict(passed=True,stage=stage,native_runs=len(rows),robot_frames=sum(r['frames']*2 for r in rows),
        rows=rows,references=references,diagnostics=diagnostics,parity=parity,retained_prior_geometry_checks=geometry_rows,
        freeze_sha256=sha(OUT/'FREEZE.json'),config_sha256=sha(cfgpath),runtime_sha256=sha(ledger),artifacts=artifacts,
        source_sha256=sha(Path(__file__))))
    print('ALL NATIVE ALIGNMENT AUDITS PASSED',stage,expected,'complete runs',flush=True)

if __name__=='__main__':main()
