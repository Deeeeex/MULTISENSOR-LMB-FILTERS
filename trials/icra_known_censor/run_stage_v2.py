"""Native execution with one job per link and immutable exit receipts."""
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
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['known_censor_controls','known_censor_refined']);args=p.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text());freeze=json.loads((OUT/'FREEZE.json').read_text())
    for name,h in freeze['configurations'].items():assert sha(ROOT/name)==h,name
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    v2=json.loads((OUT/'AUDIT_FREEZE_V2.json').read_text())
    for name,h in v2['source_sha256'].items():assert sha(ROOT/name)==h,name
    for u in cfg['units']:
        assert sha(ROOT/u['data_path'])==u['input_sha256'] and sha(ROOT/u['pose_path'])==u['pose_sha256']
        for ref in u['references'].values():assert sha(ROOT/ref['path'])==ref['sha256']
    if args.stage=='known_censor_refined':
        audit=json.loads((OUT/'audit_v2_known_censor_controls.json').read_text());assert audit['passed'] and len(audit['parity'])==6
        for name,h in audit['inputs'].items():assert sha(ROOT/name)==h,name
    logdir=ROOT/'RUN/ICRA_KNOWN_CENSOR'/args.stage;ledger=OUT/('runtime_'+args.stage+'.json')
    assert not logdir.exists() and not ledger.exists() and not (OUT/'results'/args.stage).exists()
    logdir.mkdir(parents=True);completed=[];lock=threading.Lock()
    def worker(index,u):
        seq=u['sequence'];eid=u['execution_id']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
            f"addpath('trials/icra_known_censor');runKnownCensorReplay('{cfgpath.relative_to(ROOT)}',{index+1});"]
        with lock:print('START',args.stage,eid,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1);completion=False
        with (logdir/(eid+'.log')).open('w') as stream:
            for line in process.stdout:
                stream.write(line);stream.flush()
                if f'COMPLETED REVIEW {args.stage} {seq}' in line:completion=True
        code=process.wait();paths=[OUT/'results'/args.stage/f'{seq}_{c}_{a}.json.gz' for c in u['conditions'] for a in u['arms']]
        row=dict(sequence=seq,execution_id=eid,index=index,returncode=code,completion_line=completion,
            expected_files=len(paths),files=sum(x.is_file() for x in paths),command=command)
        with lock:
            completed.append(row);temp=ledger.with_suffix('.next.json');temp.write_text(json.dumps(sorted(completed,key=lambda x:x['index']),indent=2)+'\n');temp.replace(ledger)
            print('END',args.stage,eid,'return',code,'complete',completion,'files',row['files'],flush=True)
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units'])]):
            r=future.result();assert r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files'],r
    assert len(completed)==2;print('ALL KNOWN CENSOR STAGE RUNS COMPLETE',args.stage,flush=True)

if __name__=='__main__':main()
