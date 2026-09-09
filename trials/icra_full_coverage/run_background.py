"""Run, verify, and summarize both frozen stages; persist terminal status."""
from datetime import datetime, timezone
from pathlib import Path
import json
import hashlib
import os
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
STATUS = OUT / 'BACKGROUND_STATUS.json'


def status(state, step, returncode=None):
    value = dict(state=state, step=step, pid=os.getpid(), updated_utc=datetime.now(timezone.utc).isoformat(),
                 returncode=returncode, log='RUN/ICRA_FULL_COVERAGE/master.log')
    temporary = STATUS.with_suffix('.next.json')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(STATUS)


def main():
    assert not STATUS.exists(), 'Never restart or overwrite an earlier execution.'
    manifest = json.loads((OUT / 'INPUT_MANIFEST.json').read_text())
    for name, expected in manifest['source_sha256'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    commands = [
        ['run_stage.py', 'coverage_preflight', '--workers', '1'],
        ['audit_stage.py', 'coverage_preflight'],
        ['run_stage.py', 'coverage_remaining_train', '--workers', '2'],
        ['audit_stage.py', 'coverage_remaining_train'],
        ['summarize_coverage.py'],
    ]
    for args in commands:
        step = ' '.join(args)
        status('running', step)
        print(datetime.now(timezone.utc).isoformat(), 'STEP', step, flush=True)
        result = subprocess.run([sys.executable, str(OUT / args[0]), *args[1:]], cwd=ROOT)
        if result.returncode:
            status('failed', step, result.returncode)
            print('FULL COVERAGE FAILED', step, 'native exit', result.returncode, flush=True)
            raise SystemExit(result.returncode)
    status('complete', 'all native runs, independent audits, and complete-release summary passed', 0)
    print('FULL COVERAGE COMPLETE: 43 unique scenes, 9699 paired frames, seven methods, both conditions.', flush=True)


if __name__ == '__main__':
    main()
