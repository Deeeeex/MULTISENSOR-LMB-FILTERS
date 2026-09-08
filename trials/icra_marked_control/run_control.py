"""Execute only the additional marked conservative control, two jobs at once."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import hashlib
import json
import subprocess
import threading

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_MARKED_CONTROL'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    frozen = json.loads((OUT / 'CONTROL_FREEZE.json').read_text())
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in {**source, **frozen['source_and_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    for cohort in ['development', 'holdout']:
        assert not list((OUT / f'results_{cohort}').glob('*.json.gz')), 'Never overwrite an existing full control run.'
    assert not (OUT / 'runtime.json').exists()
    LOG.mkdir(parents=True, exist_ok=True)
    completed = []
    lock = threading.Lock()

    def worker(unit):
        cohort, seq, index = unit['cohort'], unit['sequence'], unit['index']
        command = ['/Applications/MATLAB_R2024a.app/bin/matlab', '-singleCompThread', '-batch',
                   f"addpath('trials/icra_marked_control'); runMarkedControlReplay({index},{index},'{cohort}');"]
        print('CONTROL START', cohort, f'{seq:04d}', flush=True)
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        completion = False
        with (LOG / f'{cohort}_{seq:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if f'COMPLETED MARKED CONTROL indices {index}--{index} cohort={cohort}' in line:
                    completion = True
        code = process.wait()
        paths = [OUT / f'results_{cohort}' / f'{seq:04d}_{condition}_marked_conservative.json.gz'
                 for condition in frozen['conditions']]
        entry = dict(**unit, returncode=code, completion_line=completion,
                     files=sum(p.exists() for p in paths), command=command)
        with lock:
            completed.append(entry)
            tmp = OUT / 'runtime.next.json'
            tmp.write_text(json.dumps(sorted(completed, key=lambda r: (r['cohort'], r['index'])), indent=2) + '\n')
            tmp.replace(OUT / 'runtime.json')
        print('CONTROL END', cohort, f'{seq:04d}', 'return', code, 'complete', completion, 'files', entry['files'], flush=True)
        return entry

    with ThreadPoolExecutor(max_workers=2) as executor:
        for future in as_completed([executor.submit(worker, unit) for unit in frozen['units']]):
            future.result()
    assert len(completed) == 34 and all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2 for r in completed)
    print('ALL EXTRA CONTROL RUNS COMPLETE: 34 sequences; 68 files.', flush=True)


if __name__ == '__main__':
    main()
