"""Resume the unchanged registered cohort with three isolated sequence jobs."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
import gzip
import hashlib
import json
import shutil
import subprocess
import threading

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_FUSION_HOLDOUT'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    source = json.loads((OUT / 'source_sha256_port.json').read_text())
    for path, expected in {**source, **freeze['source_and_selection_evidence_sha256']}.items():
        assert sha(ROOT / path) == expected, path
    previous = json.loads((OUT / 'holdout_runtime.json').read_text())
    assert [r['sequence'] for r in previous] == [1, 2, 3, 4, 6, 7, 8]
    assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 32 for r in previous[:6])
    assert previous[-1]['returncode'] == -15 and not previous[-1]['completion_line'] and previous[-1]['files'] == 3
    backup = OUT / 'diagnostics_before_parallel'
    assert not backup.exists(), 'Do not overwrite an earlier execution-amendment snapshot.'
    backup.mkdir()
    shutil.copyfile(OUT / 'holdout_runtime.json', backup / 'runtime_interrupted.json')
    partial = list((OUT / 'results_holdout').glob('0008_*.json.gz'))
    assert len(partial) == 3
    hashes = {}
    for path in partial:
        with gzip.open(path, 'rt') as stream:
            data = json.load(stream)
        assert data['sequence'] == '0008' and data['condition'] == 'reliable'
        hashes[path.name] = sha(path)
        shutil.copyfile(path, backup / path.name)
        assert sha(backup / path.name) == hashes[path.name]
    (backup / 'sha256.json').write_text(json.dumps(hashes, indent=2) + '\n')
    completed = previous[:6]
    # Save hashes of every complete earlier result before any parallel job.
    earlier = {}
    for entry in completed:
        for condition in freeze['conditions']:
            for arm in freeze['arms']:
                path = OUT / 'results_holdout' / f"{entry['sequence']:04d}_{condition}_{arm}.json.gz"
                earlier[path.name] = sha(path)
    amendment = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), concurrent_sequences=3,
                     method_freeze_sha256=sha(OUT / 'METHOD_FREEZE.json'),
                     driver_sha256=sha(Path(__file__)),
                     protocol_sha256=sha(OUT / 'PARALLEL_EXECUTION_AMENDMENT.md'),
                     interrupted_ledger_sha256=sha(backup / 'runtime_interrupted.json'),
                     previous_complete_files=earlier, partial_completed_files=hashes)
    (OUT / 'EXECUTION_AMENDMENT.json').write_text(json.dumps(amendment, indent=2) + '\n')
    ledger_lock = threading.Lock()

    def save_ledger():
        temporary = OUT / 'holdout_runtime.next.json'
        temporary.write_text(json.dumps(sorted(completed, key=lambda r: r['index']), indent=2) + '\n')
        temporary.replace(OUT / 'holdout_runtime.json')

    save_ledger()

    def worker(index):
        seq = freeze['units'][index]
        command = ['/Applications/MATLAB_R2024a.app/bin/matlab', '-singleCompThread', '-batch',
                   f"addpath('trials/icra_fusion_holdout'); runFusionSelectionReplay({index},{index},'holdout');"]
        print('PARALLEL SEQUENCE START', f'{seq:04d}', f'index {index}', flush=True)
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        complete_line = False
        with (LOG / f'parallel_{seq:04d}.log').open('w') as log:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if f'COMPLETED PORT indices {index}--{index} cohort=holdout' in line:
                    complete_line = True
        code = process.wait()
        paths = [OUT / 'results_holdout' / f'{seq:04d}_{condition}_{arm}.json.gz'
                 for condition in freeze['conditions'] for arm in freeze['arms']]
        count = sum(path.exists() for path in paths)
        entry = dict(sequence=seq, index=index, returncode=code, completion_line=complete_line,
                     files=count, command=command, execution='parallel-three-sequences')
        with ledger_lock:
            completed.append(entry)
            save_ledger()
        print('PARALLEL SEQUENCE END', f'{seq:04d}', 'return', code, 'complete', complete_line, 'files', count, flush=True)
        return entry

    with ThreadPoolExecutor(max_workers=3) as executor:
        for future in as_completed([executor.submit(worker, index) for index in range(6, 25)]):
            future.result()
    for name, expected in earlier.items():
        assert sha(OUT / 'results_holdout' / name) == expected, (name, 'earlier output changed')
    assert len(completed) == 25 and all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 32 for r in completed)
    print('ALL 25 REGISTERED SEQUENCES COMPLETE; 800 files; original 192 complete files unchanged.', flush=True)


if __name__ == '__main__':
    main()
