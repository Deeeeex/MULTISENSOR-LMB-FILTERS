"""Run an immutable registered stage, retaining native exits and completion."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import threading

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('stage')
    p.add_argument('--workers',type=int,default=3)
    args=p.parse_args()
    cfgpath=OUT/'stages'/f'{args.stage}.json'
    cfg=json.loads(cfgpath.read_text())
    for name,expected in cfg['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    logdir=ROOT/'RUN/ICRA_TEMPORAL_ASSOCIATION'/args.stage
    assert not logdir.exists(), 'Never overwrite a prior execution.'
    logdir.mkdir(parents=True)
    ledger=OUT/f'runtime_{args.stage}.json'
    assert not ledger.exists()
    completed=[];lock=threading.Lock()

    def worker(i,unit):
        seq=unit['sequence']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
                 f"addpath('trials/icra_temporal_association');runScreenedAssociation('{cfgpath.relative_to(ROOT)}',{i+1});"]
        print('START',args.stage,seq,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completion=False
        with (logdir/f'{seq}.log').open('w') as log:
            for line in process.stdout:
                log.write(line);log.flush()
                if f'COMPLETED REVIEW {args.stage} {seq}' in line:completion=True
        code=process.wait()
        paths=[OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz'
               for condition in cfg['conditions'] for arm in cfg['arms']]
        row=dict(sequence=seq,index=i,returncode=code,completion_line=completion,
                 files=sum(x.exists() for x in paths),command=command)
        with lock:
            completed.append(row)
            temp=ledger.with_suffix('.next.json')
            temp.write_text(json.dumps(sorted(completed,key=lambda r:r['index']),indent=2)+'\n');temp.replace(ledger)
        print('END',args.stage,seq,'return',code,'complete',completion,'files',row['files'],flush=True)
        return row

    # Validate the complete first unit before dispatching the remaining list.
    first=worker(0,cfg['units'][0])
    assert first['returncode']==0 and first['completion_line'] and first['files']==2*len(cfg['arms']), first
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units']) if i>0]):
            result=future.result()
            if result['returncode'] != 0:
                # Remaining units keep their own logs; all failures remain visible.
                print('FAILED UNIT',result['sequence'],flush=True)
    assert len(completed)==len(cfg['units'])
    assert all(r['returncode']==0 and r['completion_line'] and r['files']==2*len(cfg['arms']) for r in completed)
    print('ALL REVIEW STAGE RUNS COMPLETE',args.stage,len(completed),'sequences',flush=True)


if __name__=='__main__':main()
