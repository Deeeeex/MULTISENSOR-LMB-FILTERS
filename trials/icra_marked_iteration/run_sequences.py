"""Retry a verified terminated process without changing frozen methods."""
from pathlib import Path
import gzip,hashlib,json,shutil,subprocess
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
LOG=ROOT/'RUN/ICRA_MARKED_ITERATION'


def read(path):
    with gzip.open(path,'rt') as f:return json.load(f)


def main():
    frozen=json.loads((OUT/'source_sha256.json').read_text())
    for name,h in frozen.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
    before=OUT/'diagnostics_before_process_retry';before.mkdir(exist_ok=True)
    assert not list(before.glob('*.json.gz')),'Do not overwrite the first-attempt evidence.'
    original=list((OUT/'results_development').glob('*.json.gz'))
    assert {p.name for p in original}=={'0000_reliable.json.gz','0000_intermittent.json.gz','0001_reliable.json.gz'}
    hashes={}
    for path in original:
        shutil.copy2(path,before/path.name);hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    (before/'sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    commands=[];parity_files=[]
    for seq in range(9):
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
                 f"addpath('trials/icra_marked_iteration'); runMarkedEvidenceReplay({seq},{seq},false);"]
        print('FRESH PROCESS SEQUENCE',f'{seq:04d}',flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completed=False
        with (LOG/f'sequence_{seq:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line);print(line,end='',flush=True)
                if f'COMPLETED iteration sequences {seq:04d}--{seq:04d} smoke=0' in line:completed=True
        code=process.wait();commands.append(dict(sequence=f'{seq:04d}',command=command,returncode=code,completion_line=completed))
        (OUT/'runtime_attempts.json').write_text(json.dumps(dict(commands=commands,overlap_parity_files=parity_files),indent=2)+'\n')
        assert code==0 and completed,(seq,code,completed)
        for condition in ['reliable','intermittent']:
            name=f'{seq:04d}_{condition}.json.gz';old=before/name
            if not old.exists():continue
            assert hashlib.sha256(old.read_bytes()).hexdigest()==hashes[name]
            a=read(old);b=read(OUT/'results_development'/name)
            for item in [a,b]:
                for run in item['runs']:run.pop('runtimeSeconds')
            assert a==b,(seq,condition,'Exact frozen-code retry changed outputs')
            parity_files.append(name);print('RETRY EXACT OUTPUT PARITY',name,flush=True)
    (OUT/'runtime_attempts.json').write_text(json.dumps(dict(commands=commands,overlap_parity_files=parity_files),indent=2)+'\n')
    print('ALL 9 FRESH PROCESSES COMPLETE; original completed files preserve exact non-runtime output parity.',flush=True)


if __name__=='__main__':main()
