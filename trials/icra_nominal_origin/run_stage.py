"""Execute one native job per declared link condition, with complete exit receipts."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import threading

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['nominal_origin_controls','nominal_origin_event']);p.add_argument('--workers',type=int,default=2);args=p.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text());freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text())
    for name,h in freeze['configurations'].items():assert sha(ROOT/name)==h,name
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    for unit in cfg['units']:
        assert sha(ROOT/unit['data_path'])==unit['input_sha256'] and sha(ROOT/unit['pose_path'])==unit['pose_sha256']
        for ref in unit['references'].values():assert sha(ROOT/ref['path'])==ref['sha256']
    if args.stage=='nominal_origin_event':
        audit=json.loads((OUT/'audit_nominal_origin_controls.json').read_text());assert audit['passed'] and len(audit['parity'])==2
        for name,h in audit['inputs'].items():assert sha(ROOT/name)==h,name
    logdir=ROOT/'RUN/ICRA_NOMINAL_ORIGIN'/args.stage;ledger=OUT/('runtime_'+args.stage+'.json')
    assert not logdir.exists() and not ledger.exists() and not (OUT/'results'/args.stage).exists()
    logdir.mkdir(parents=True);completed=[];lock=threading.Lock()
    def worker(index,unit):
        seq=unit['sequence'];eid=unit['execution_id']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
            f"addpath('trials/icra_nominal_origin');runNominalOriginReplay('{cfgpath.relative_to(ROOT)}',{index+1});"]
        with lock:print('START',args.stage,eid,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1);completion=False
        with (logdir/(eid+'.log')).open('w') as stream:
            for line in process.stdout:
                stream.write(line);stream.flush()
                if f'COMPLETED REVIEW {args.stage} {seq}' in line:completion=True
        code=process.wait();paths=[OUT/'results'/args.stage/f'{seq}_{c}_{a}.json.gz' for c in unit['conditions'] for a in unit['arms']]
        row=dict(sequence=seq,execution_id=eid,index=index,returncode=code,completion_line=completion,
            expected_files=len(paths),files=sum(p.is_file() for p in paths),command=command)
        with lock:
            completed.append(row);temp=ledger.with_suffix('.next.json');temp.write_text(json.dumps(sorted(completed,key=lambda x:x['index']),indent=2)+'\n');temp.replace(ledger)
            print('END',args.stage,eid,'return',code,'complete',completion,'files',row['files'],flush=True)
        return row
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units'])]):
            r=future.result();assert r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files'],r
    assert len(completed)==len(cfg['units']);print('ALL NOMINAL STAGE RUNS COMPLETE',args.stage,flush=True)

if __name__=='__main__':main()
