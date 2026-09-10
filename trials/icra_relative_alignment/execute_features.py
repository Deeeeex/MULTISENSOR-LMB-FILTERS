"""Keep the actual raw-preparation process exit and complete log."""
from pathlib import Path
import subprocess
import sys
from common import OUT,ROOT,sha,write_new,frozen

def main():
    frozen();receipt=OUT/'FEATURE_EXECUTION.json';assert not receipt.exists()
    log=ROOT/'RUN/ICRA_RELATIVE_ALIGNMENT/features.log';assert not log.exists();log.parent.mkdir(parents=True,exist_ok=True)
    command=[sys.executable,'-u',str((OUT/'prepare_features.py').relative_to(ROOT))]
    process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    with log.open('w') as stream:
        for line in process.stdout:stream.write(line);stream.flush();print(line,end='',flush=True)
    code=process.wait()
    write_new(receipt,dict(returncode=code,completed=code==0,command=command,freeze_sha256=sha(OUT/'FREEZE.json'),
                           log=str(log.relative_to(ROOT)),log_sha256=sha(log),source_sha256=sha(Path(__file__))))
    print('RAW PREPARATION PROCESS EXITED',code,flush=True)
    assert code==0

if __name__=='__main__':main()
