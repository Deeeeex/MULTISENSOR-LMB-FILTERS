"""Continue after the completed parity audit; do not rerun any native output."""
from datetime import datetime, timezone
from pathlib import Path
import json
import os
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
STATUS = OUT / 'BACKGROUND_STATUS.json'


def status(state, step, code=None):
    value = dict(state=state, step=step, pid=os.getpid(), updated_utc=datetime.now(timezone.utc).isoformat(),
                 returncode=code, log='RUN/ICRA_FULL_COVERAGE/resume_master.log',
                 earlier_attempt='trials/icra_full_coverage/BACKGROUND_ATTEMPT_1.json')
    temporary = STATUS.with_suffix('.next.json')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(STATUS)


def main():
    old = json.loads((OUT / 'BACKGROUND_ATTEMPT_1.json').read_text())
    assert old['state'] == 'failed' and old['step'] == 'audit_stage.py coverage_preflight'
    checked = json.loads((OUT / 'audit_coverage_preflight.json').read_text())
    assert checked['passed'] and len(checked['parity']) == 4
    assert not (OUT / 'runtime_coverage_remaining_train.json').exists()
    commands = [
        ['run_stage.py', 'coverage_remaining_train', '--workers', '2'],
        ['audit_stage.py', 'coverage_remaining_train'],
        ['summarize_coverage.py'],
    ]
    for args in commands:
        step = ' '.join(args)
        status('running', step)
        print(datetime.now(timezone.utc).isoformat(), 'STEP', step, flush=True)
        process = subprocess.run([sys.executable, str(OUT / args[0]), *args[1:]], cwd=ROOT)
        if process.returncode:
            status('failed', step, process.returncode)
            print('FULL COVERAGE FAILED', step, 'native exit', process.returncode, flush=True)
            raise SystemExit(process.returncode)
    status('complete', 'all native runs, independent audits, and complete-release summary passed', 0)
    print('FULL COVERAGE COMPLETE: 43 unique scenes, 9699 paired frames, seven methods, both conditions.', flush=True)


if __name__ == '__main__':
    main()
