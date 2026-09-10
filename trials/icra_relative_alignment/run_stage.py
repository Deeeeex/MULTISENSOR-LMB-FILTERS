"""Execute each registered whole trajectory and preserve actual process exits."""
from concurrent.futures import ThreadPoolExecutor,as_completed
import argparse
import json
import subprocess
import threading
from common import OUT,ROOT,sha,frozen

def main():
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=['parity','corrected']);args=parser.parse_args()
    frozen();stage='alignment_'+args.kind;cfgpath=OUT/'stages'/f'{stage}.json';cfg=json.loads(cfgpath.read_text())
    assert cfg['source_sha256']['method_freeze']==sha(OUT/'FREEZE.json')
    if args.kind=='corrected':
        assert cfg['source_sha256']['parity_audit']==sha(OUT/'audit_alignment_parity.json')
        assert cfg['source_sha256']['alignment_estimation']==sha(OUT/'ESTIMATION.json')
    for unit in cfg['units']:
        if args.kind=='corrected':assert sha(ROOT/unit['alignment_path'])==unit['alignment_sha256']
    logdir=ROOT/'RUN/ICRA_RELATIVE_ALIGNMENT'/stage;ledger=OUT/f'runtime_{stage}.json'
    assert not logdir.exists() and not ledger.exists() and not (OUT/'results'/stage).exists()
    logdir.mkdir(parents=True);complete=[];lock=threading.Lock()
    def worker(index,unit):
        name=unit['sequence']
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
                 f"addpath('trials/icra_relative_alignment');runAlignmentReplay('{cfgpath.relative_to(ROOT)}',{index+1});"]
        print('START',stage,name,flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completion=False;logpath=logdir/f'{name}.log'
        with logpath.open('w') as stream:
            for line in process.stdout:
                stream.write(line);stream.flush()
                if f'COMPLETED REVIEW {stage} {name}' in line:completion=True
        code=process.wait()
        paths=[OUT/'results'/stage/f'{name}_reliable_{arm}.json.gz' for arm in cfg['arms']]
        row=dict(index=index,sequence=name,returncode=code,completion_line=completion,files=sum(p.is_file() for p in paths),
                 command=command,log=str(logpath.relative_to(ROOT)),log_sha256=sha(logpath))
        with lock:
            complete.append(row);temp=ledger.with_suffix('.next.json')
            temp.write_text(json.dumps(sorted(complete,key=lambda r:r['index']),indent=2)+'\n');temp.replace(ledger)
            print('END',stage,name,'exit',code,'complete',completion,'files',row['files'],flush=True)
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(worker,i,u) for i,u in enumerate(cfg['units'])]):future.result()
    assert len(complete)==len(cfg['units']) and all(r['returncode']==0 and r['completion_line'] and r['files']==2 for r in complete)
    print('ALL NATIVE ALIGNMENT RUNS EXITED',stage,len(complete)*2,flush=True)

if __name__=='__main__':main()
