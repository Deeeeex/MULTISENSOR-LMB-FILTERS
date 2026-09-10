"""Run the bounded native fixtures before freezing any trajectory execution."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import subprocess

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    log=ROOT/'RUN/ICRA_PRUNE_INFORMATION/fixture.log';receipt=OUT/'FIXTURE.json'
    assert not log.exists() and not receipt.exists();log.parent.mkdir(parents=True,exist_ok=True)
    command=['/Applications/MATLAB_R2024a.app/bin/matlab','-singleCompThread','-batch',
        "addpath('common','lmb','multisensorLmb','trials/icra_known_censor','trials/icra_prune_information');checkKnownCensor();checkPruneInformation();"]
    with log.open('w') as stream:code=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT).returncode
    content=log.read_text();passed=code==0 and 'KNOWN CENSOR CHECK PASSED' in content and 'PRUNE INFORMATION CHECK PASSED' in content
    receipt.write_text(json.dumps(dict(passed=passed,returncode=code,command=command,completed_utc=datetime.now(timezone.utc).isoformat(),
        log=str(log.relative_to(ROOT)),log_sha256=sha(log),source_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.m')}),indent=2)+'\n')
    print(content,flush=True);assert passed

if __name__=='__main__':main()
