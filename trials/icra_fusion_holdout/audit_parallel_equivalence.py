"""Check the scheduling-only restart against preserved complete outputs."""
from pathlib import Path
import gzip
import hashlib
import json

OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def main():
    amendment = json.loads((OUT / 'EXECUTION_AMENDMENT.json').read_text())
    assert amendment['method_freeze_sha256'] == sha(OUT / 'METHOD_FREEZE.json')
    assert amendment['driver_sha256'] == sha(OUT / 'run_holdout_parallel.py')
    assert amendment['protocol_sha256'] == sha(OUT / 'PARALLEL_EXECUTION_AMENDMENT.md')
    earlier = amendment['previous_complete_files']
    for name, expected in earlier.items():
        assert sha(OUT / 'results_holdout' / name) == expected
    rows = []
    for name, expected in amendment['partial_completed_files'].items():
        saved_path = OUT / 'diagnostics_before_parallel' / name
        assert sha(saved_path) == expected
        saved = read(saved_path)
        replay = read(OUT / 'results_holdout' / name)
        for data in [saved, replay]:
            data['runs'].pop('runtimeSeconds')
        assert saved == replay, name
        rows.append(dict(file=name, exact_values_except_runtime=True, node_frames=2 * len(saved['time'])))
    report = dict(previous_complete_files_byte_identical=len(earlier), restarted_complete_arms=rows,
                  exact_restarted_output_node_frames=sum(r['node_frames'] for r in rows),
                  execution_amendment_sha256=sha(OUT / 'EXECUTION_AMENDMENT.json'))
    assert len(earlier) == 192 and len(rows) == 3 and report['exact_restarted_output_node_frames'] == 2460
    (OUT / 'parallel_equivalence.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PARALLEL EQUIVALENCE PASSED: 192 earlier files unchanged; 2460 restarted node-frames exactly equal.')


if __name__ == '__main__':
    main()
