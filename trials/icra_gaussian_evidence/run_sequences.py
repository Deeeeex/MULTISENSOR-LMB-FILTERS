"""Execute a registered Gaussian evidence stage in three independent MATLAB processes."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import threading

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_GAUSSIAN_EVIDENCE'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    args = parser.parse_args()
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    arms = frozen['arms_by_cohort'][args.cohort]
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in {**source, **frozen['source_and_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    if args.cohort == 'seen_transfer':
        development = json.loads((OUT / 'summary_development.json').read_text())
        assert development['round_freeze_sha256'] == sha(OUT / 'ROUND_FREEZE.json')
        assert development['primary'] == frozen['primary'] and development['sequences'] == 9
        assert development['continuation_gate_passed'], 'The primary failed the registered continuation rule.'
        for name, expected in development['inputs'].items():
            assert sha(ROOT / name) == expected, name
    assert not list((OUT / f'results_{args.cohort}').glob('*.json.gz')), 'Never overwrite an existing stage.'
    ledger = OUT / f'runtime_{args.cohort}.json'
    assert not ledger.exists()
    units = [u for u in frozen['units'] if u['cohort'] == args.cohort]
    LOG.mkdir(parents=True, exist_ok=True)
    completed, lock = [], threading.Lock()

    def worker(unit):
        cohort, seq, index = unit['cohort'], unit['sequence'], unit['index']
        command = ['/Applications/MATLAB_R2024a.app/bin/matlab', '-singleCompThread', '-batch',
                   f"addpath('trials/icra_gaussian_evidence'); runGaussianEvidenceReplay({index},{index},'{cohort}');"]
        print('GAUSSIAN EVIDENCE START', cohort, f'{seq:04d}', flush=True)
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        completion = False
        with (LOG / f'{cohort}_{seq:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if f'COMPLETED GAUSSIAN EVIDENCE indices {index}--{index} cohort={cohort}' in line:
                    completion = True
        code = process.wait()
        paths = [OUT / f'results_{cohort}' / f'{seq:04d}_{condition}_{arm}.json.gz'
                 for condition in frozen['conditions'] for arm in arms]
        entry = dict(**unit, returncode=code, completion_line=completion,
                     files=sum(p.exists() for p in paths), command=command)
        with lock:
            completed.append(entry)
            temporary = ledger.with_suffix('.next.json')
            temporary.write_text(json.dumps(sorted(completed, key=lambda r: r['index']), indent=2) + '\n')
            temporary.replace(ledger)
        print('GAUSSIAN EVIDENCE END', cohort, f'{seq:04d}', 'return', code, 'complete', completion, 'files', entry['files'], flush=True)
        return entry

    with ThreadPoolExecutor(max_workers=3) as executor:
        for future in as_completed([executor.submit(worker, unit) for unit in units]):
            future.result()
    assert len(completed) == len(units) and all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2*len(arms) for r in completed)
    print('ALL GAUSSIAN EVIDENCE STAGE RUNS COMPLETE:', args.cohort, len(units), 'sequences;', 2 * len(arms) * len(units), 'files.', flush=True)


if __name__ == '__main__':
    main()
