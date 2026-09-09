"""Run each declared native process once, with per-execution logs and exit receipts."""
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
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['recursion_preflight','recursion_interventions']);p.add_argument('--workers',type=int,default=2);args=p.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text())
    for name,expected in freeze['configurations'].items():assert sha(ROOT/name)==expected,name
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    for unit in cfg['units']:
        assert sha(ROOT/unit['data_path'])==unit['input_sha256']
        assert sha(ROOT/unit['pose_path'])==unit['pose_sha256']
        for ref in unit['references'].values():assert sha(ROOT/ref['path'])==ref['sha256']
    if args.stage=='recursion_interventions':
        auditpath=OUT/'audit_recursion_preflight.json';audit=json.loads(auditpath.read_text())
        assert audit['passed'] and len(audit['parity'])==2
        for name,expected in audit['inputs'].items():assert sha(ROOT/name)==expected,name
    logdir=ROOT/'RUN/ICRA_RECURSION_ORIGIN'/args.stage;ledger=OUT/('runtime_'+args.stage+'.json')
    assert not logdir.exists() and not ledger.exists() and not (OUT/'results'/args.stage).exists()
    logdir.mkdir(parents=True);completed=[];lock=threading.Lock()
    def worker(index,unit):
        seq=unit['sequence'];eid=unit['execution_id']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
            f"addpath('trials/icra_recursion_origin');runRecursionIntervention('{cfgpath.relative_to(ROOT)}',{index+1});"]
        print('START',args.stage,eid,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completion=False
        with (logdir/(eid+'.log')).open('w') as log:
            for line in process.stdout:
                log.write(line);log.flush()
                if f'COMPLETED REVIEW {args.stage} {seq}' in line:completion=True
        code=process.wait()
        paths=[OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz' for condition in cfg['conditions'] for arm in unit['arms']]
        row=dict(sequence=seq,execution_id=eid,index=index,returncode=code,completion_line=completion,
            expected_files=len(paths),files=sum(path.is_file() for path in paths),command=command)
        with lock:
            completed.append(row);temp=ledger.with_suffix('.next.json')
            temp.write_text(json.dumps(sorted(completed,key=lambda r:r['index']),indent=2)+'\n');temp.replace(ledger)
        print('END',args.stage,eid,'return',code,'complete',completion,'files',row['files'],flush=True)
        return row
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units'])]):
            row=future.result()
            assert row['returncode']==0 and row['completion_line'] and row['files']==row['expected_files'],row
    assert len(completed)==len(cfg['units'])
    print('ALL RECURSION NATIVE RUNS COMPLETE',args.stage,len(completed),'processes',flush=True)

if __name__=='__main__':main()
