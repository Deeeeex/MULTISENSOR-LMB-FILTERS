"""Audit the remaining registered stages as their native unit processes exit."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import subprocess
import sys
import time

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def finish(stage, auditor):
    configuration = json.loads((OUT / 'stages' / (stage + '.json')).read_text())
    ledger = OUT / ('runtime_' + stage + '.json')
    destination = OUT / ('audit_' + stage + '.json')
    while True:
        if ledger.exists():
            records = json.loads(ledger.read_text())
            if len(records) == len(configuration['units']):
                assert all(row['returncode'] == 0 and row['completion_line'] for row in records), stage
                break
        time.sleep(20)
    assert not destination.exists(), 'Do not overwrite a completed audit'
    path = ROOT / 'RUN/ICRA_REVIEWER_REVISION' / ('audit_' + stage + '.log')
    assert not path.exists()
    print('AUDIT START', stage, flush=True)
    with path.open('w') as stream:
        subprocess.run([sys.executable, str(OUT / auditor), stage], cwd=ROOT,
                       stdout=stream, stderr=subprocess.STDOUT, check=True)
    assert json.loads(destination.read_text())['passed']
    print('AUDIT COMPLETE', stage, flush=True)


def main():
    jobs = [('motion_seen_transfer', 'audit_motion_stage.py'),
            ('pd080_development', 'audit_stage.py'), ('pd095_development', 'audit_stage.py')]
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = [pool.submit(finish, *job) for job in jobs]
        for future in futures:
            future.result()
    assert json.loads((OUT / 'audit_motion_development.json').read_text())['passed']
    for script in ['summarize_model_sensitivity.py', 'write_revision_report.py']:
        subprocess.run([sys.executable, str(OUT / script)], cwd=ROOT, check=True)
    print('ALL REGISTERED MODEL CHECKS AND SUMMARIES COMPLETE', flush=True)


if __name__ == '__main__':
    main()
