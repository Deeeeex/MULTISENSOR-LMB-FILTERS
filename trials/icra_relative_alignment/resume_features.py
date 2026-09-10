"""Preserve failed attempt receipts and publish the latest actual input-process exit."""
from pathlib import Path
import json
import subprocess
import sys
from common import OUT,ROOT,sha,write_new,frozen

def main():
    frozen();receipt=OUT/'FEATURE_EXECUTION.json';old_bytes=receipt.read_bytes();previous=json.loads(old_bytes)
    assert not previous['completed'] and previous['returncode']!=0 and not (OUT/'FEATURES.json').exists()
    attempt=len(list(OUT.glob('FEATURE_EXECUTION_ATTEMPT*.json')))+2
    archived=OUT/f'FEATURE_EXECUTION_ATTEMPT{attempt-1}.json';assert not archived.exists();archived.write_bytes(old_bytes)
    assert sha(archived)==sha(receipt)
    log=ROOT/'RUN/ICRA_RELATIVE_ALIGNMENT'/f'features_attempt{attempt}.log';assert not log.exists()
    command=[sys.executable,'-u',str((OUT/'prepare_features_retry.py').relative_to(ROOT))]
    process=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    with log.open('w') as stream:
        for line in process.stdout:stream.write(line);stream.flush();print(line,end='',flush=True)
    code=process.wait()
    current=dict(returncode=code,completed=code==0,command=command,freeze_sha256=sha(OUT/'FREEZE.json'),
                 log=str(log.relative_to(ROOT)),log_sha256=sha(log),source_sha256=sha(Path(__file__)),attempt=attempt,
                 previous_receipt=str(archived.relative_to(ROOT)),previous_receipt_sha256=sha(archived),
                 original_preparer_sha256=sha(OUT/'prepare_features.py'),transport_retry_sha256=sha(OUT/'prepare_features_retry.py'),
                 unchanged_estimator_sha256=sha(OUT/'alignment_math.py'),unchanged_grid_verifier_sha256=sha(OUT/'independent_math.py'),
                 note='Input execution journal updated after archiving the failed receipt byte-for-byte. Only transient transport setup retries were added.')
    write_new(OUT/f'FEATURE_RETRY_{attempt}.json',current)
    assert receipt.read_bytes()==old_bytes
    temporary=receipt.with_suffix('.next.json');temporary.write_text(json.dumps(current,indent=2)+'\n');temporary.replace(receipt)
    print('RAW INPUT RETRY PROCESS EXITED',attempt,code,flush=True)
    assert code==0

if __name__=='__main__':main()
