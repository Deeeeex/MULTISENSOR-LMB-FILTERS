"""One process per registered sequence, with exact source and completion gates."""
from pathlib import Path
import hashlib,json,subprocess
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
LOG=ROOT/'RUN/ICRA_FUSION_HOLDOUT'


def main():
    frozen=json.loads((OUT/'METHOD_FREEZE.json').read_text())
    source=json.loads((OUT/'source_sha256_port.json').read_text())
    for p,h in {**source,**frozen['source_and_selection_evidence_sha256']}.items():
        assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h,p
    assert not list((OUT/'results_holdout').glob('*.json.gz')),'Do not overwrite a partial run; inspect and explicitly resume it.'
    attempts=[]
    for index,seq in enumerate(frozen['units']):
        command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
                 f"addpath('trials/icra_fusion_holdout'); runFusionSelectionReplay({index},{index},'holdout');"]
        print('HOLDOUT SEQUENCE',f'{seq:04d}',f'{index+1}/25',flush=True)
        process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        completed=False
        with (LOG/f'holdout_{seq:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line);print(line,end='',flush=True)
                if f'COMPLETED PORT indices {index}--{index} cohort=holdout' in line:completed=True
        code=process.wait()
        paths=[OUT/'results_holdout'/f'{seq:04d}_{c}_{a}.json.gz' for c in frozen['conditions'] for a in frozen['arms']]
        count=sum(p.exists() for p in paths)
        attempts.append(dict(sequence=seq,index=index,returncode=code,completion_line=completed,files=count,command=command))
        (OUT/'holdout_runtime.json').write_text(json.dumps(attempts,indent=2)+'\n')
        assert code==0 and completed and count==32,(seq,code,completed,count)
    print('ALL 25 HOLDOUT SEQUENCES COMPLETE; all 800 registered arm files exist.',flush=True)


if __name__=='__main__':main()
