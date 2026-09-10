"""Record the actual process lifecycle of the immutable ancestry producer."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import subprocess
import sys

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    receipt=OUT/'CENSUS_EXECUTION.json';log=ROOT/'RUN/ICRA_LABEL_GENEALOGY/census.log'
    assert not receipt.exists() and not log.exists();log.parent.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((OUT/'CENSUS_FREEZE.json').read_text())
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    state=dict(started_utc=datetime.now(timezone.utc).isoformat(),completed=False,
        command=[sys.executable,'-u',str(OUT/'census.py')],source_sha256=sha(Path(__file__)),
        freeze_sha256=sha(OUT/'CENSUS_FREEZE.json'),log=str(log.relative_to(ROOT)))
    receipt.write_text(json.dumps(state,indent=2)+'\n')
    with log.open('w') as stream:
        process=subprocess.Popen(state['command'],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        state['pid']=process.pid;receipt.write_text(json.dumps(state,indent=2)+'\n')
        for line in process.stdout:stream.write(line);stream.flush();print(line,end='',flush=True)
        state['returncode']=process.wait()
    state.update(completed=True,ended_utc=datetime.now(timezone.utc).isoformat(),log_sha256=sha(log))
    receipt.write_text(json.dumps(state,indent=2)+'\n');assert state['returncode']==0,state
    print('CENSUS PROCESS EXITED 0',flush=True)


if __name__=='__main__':main()
