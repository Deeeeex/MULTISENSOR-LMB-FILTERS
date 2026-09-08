"""Replay only the fixed primary with the bit-preserving zero codec."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import json
import subprocess
import threading
from analyze_codec import sha, source_check

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_GAUSSIAN_ZERO_CODEC'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    frozen = json.loads((OUT / f'CODEC_FREEZE_{cohort}.json').read_text())
    source_check()
    for name, expected in frozen['source_and_evidence_sha256'].items():
        assert sha(ROOT / name) == expected, name
    assert not list((OUT / f'results_{cohort}').glob('*.json.gz'))
    ledger = OUT / f'runtime_{cohort}.json'
    assert not ledger.exists()
    completed, lock = [], threading.Lock()

    def worker(unit):
        index, sequence = unit['index'], unit['sequence']
        command = ['/Applications/MATLAB_R2024a.app/bin/matlab', '-singleCompThread', '-batch',
                   f"addpath('trials/icra_gaussian_zero_codec'); runGaussianZeroReplay({index},{index},'{cohort}');"]
        print('ZERO CODEC START', cohort, sequence, flush=True)
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        done = False
        with (LOG / f'{cohort}_{sequence:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if f'COMPLETED GAUSSIAN ZERO CODEC indices {index}--{index} cohort={cohort}' in line:
                    done = True
        code = process.wait()
        paths = [OUT / f'results_{cohort}' / f"{sequence:04d}_{condition}_{frozen['primary']}.json.gz"
                 for condition in ['reliable', 'intermittent']]
        entry = dict(**unit, returncode=code, completion_line=done, files=sum(p.exists() for p in paths), command=command)
        with lock:
            completed.append(entry)
            temporary = ledger.with_suffix('.next.json')
            temporary.write_text(json.dumps(sorted(completed, key=lambda r: r['index']), indent=2)+'\n')
            temporary.replace(ledger)
        print('ZERO CODEC END', cohort, sequence, 'return', code, 'complete', done, 'files', entry['files'], flush=True)

    with ThreadPoolExecutor(max_workers=frozen['max_workers']) as executor:
        for future in as_completed([executor.submit(worker, u) for u in frozen['units']]):
            future.result()
    assert len(completed) == len(frozen['units'])
    assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2 for r in completed)
    print('ALL ZERO CODEC STAGE RUNS COMPLETE:', cohort, len(completed), flush=True)


if __name__ == '__main__':
    main()
