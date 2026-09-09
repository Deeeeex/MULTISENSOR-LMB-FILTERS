"""Execute a frozen native stage and preserve every process outcome."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import threading

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('stage');p.add_argument('--workers',type=int,default=2);args=p.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');cfg=json.loads(cfgpath.read_text())
    for name,expected in cfg['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    for unit in cfg['units']:
        for prefix in ['reference','noage_reference']:
            for condition,name in unit[prefix+'_paths'].items():
                assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==unit[prefix+'_sha256'][condition],name
        for condition,paths in unit['parity_paths'].items():
            for arm,name in paths.items():
                assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==unit['parity_sha256'][condition][arm],name
    logdir=ROOT/'RUN/ICRA_RANGE_DETECTION'/args.stage
    ledger=OUT/('runtime_'+args.stage+'.json')
    assert not logdir.exists() and not ledger.exists()
    logdir.mkdir(parents=True);completed=[];lock=threading.Lock()

    def worker(index,unit):
        seq=unit['sequence']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
                 f"addpath('trials/icra_range_detection');runRangeDetectionReplay('{cfgpath.relative_to(ROOT)}',{index+1});"]
        print('START',args.stage,seq,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completion=False
        with (logdir/(seq+'.log')).open('w') as log:
            for line in process.stdout:
                log.write(line);log.flush()
                if f'COMPLETED REVIEW {args.stage} {seq}' in line:completion=True
        code=process.wait()
        paths=[OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz' for condition in cfg['conditions'] for arm in cfg['arms']]
        row=dict(sequence=seq,index=index,returncode=code,completion_line=completion,
                 files=sum(path.is_file() for path in paths),command=command)
        with lock:
            completed.append(row)
            temp=ledger.with_suffix('.next.json')
            temp.write_text(json.dumps(sorted(completed,key=lambda r:r['index']),indent=2)+'\n');temp.replace(ledger)
        print('END',args.stage,seq,'return',code,'complete',completion,'files',row['files'],flush=True)
        return row

    first=worker(0,cfg['units'][0])
    assert first['returncode']==0 and first['completion_line'] and first['files']==2*len(cfg['arms']),first
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units']) if i>0]):
            row=future.result()
            if row['returncode']!=0:print('FAILED UNIT',row['sequence'],flush=True)
    assert len(completed)==len(cfg['units'])
    assert all(r['returncode']==0 and r['completion_line'] and r['files']==2*len(cfg['arms']) for r in completed)
    print('ALL RANGE DETECTION STAGE RUNS COMPLETE',args.stage,len(completed),'units',flush=True)


if __name__=='__main__':main()
