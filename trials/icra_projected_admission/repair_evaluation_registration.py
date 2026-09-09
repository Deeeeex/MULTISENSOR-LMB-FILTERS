"""Preserve the failed mixed-schema registration and normalize its metadata."""
from datetime import datetime, timezone
from pathlib import Path
import copy
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    oldpath = OUT / 'stages/final_v2v_evaluation.json'
    cfg = json.loads(oldpath.read_text())
    runtime_path = OUT / 'runtime_final_v2v_evaluation.json'
    runtime = json.loads(runtime_path.read_text())
    assert len(runtime) == 34 and all(r['returncode'] == 1 and r['files'] == 0 and not r['completion_line'] for r in runtime)
    assert not list((OUT / 'results/final_v2v_evaluation').glob('*.gz'))
    for key, expected in cfg['source_sha256'].items():
        assert sha(ROOT / key) == expected, key
    repaired = copy.deepcopy(cfg)
    repaired['stage'] = 'final_v2v_evaluation_v2'
    repaired['created_utc'] = datetime.now(timezone.utc).isoformat()
    fields = ['sequence', 'original_sequence', 'split', 'scene', 'recording',
              'data_path', 'input_sha256', 'radio_seed']
    units = []
    for row in cfg['units']:
        one = {key: row[key] for key in fields}
        # New-data MATs embed all mark/ratio fields. These same-file fallbacks
        # are valid inputs, although the native embedded-field path skips them.
        one['marks_path'] = row.get('marks_path', row['data_path'])
        one['ratios_path'] = row.get('ratios_path', row['data_path'])
        for key in ['data_path', 'marks_path', 'ratios_path']:
            assert (ROOT / one[key]).exists()
            assert sha(ROOT / one[key]) == cfg['source_sha256'][one[key]]
        units.append(one)
    assert all(list(u) == list(units[0]) for u in units)
    repaired['units'] = units
    repaired['source_sha256'][str(Path(__file__).relative_to(ROOT))] = sha(Path(__file__))
    receipt_path = OUT / 'EVALUATION_REGISTRATION_REPAIR.json'; assert not receipt_path.exists()
    receipt = dict(created_utc=repaired['created_utc'], failed_stage=cfg['stage'], corrected_stage=repaired['stage'],
                   reason='MATLAB jsondecode returned a cell array for units with different field sets; unit.data_path failed before data loading or any tracking.',
                   failed_unit_count=34, failed_result_count=0, original_config_sha256=sha(oldpath),
                   failed_runtime_sha256=sha(runtime_path), algorithm_changed=False, prepared_inputs_changed=False,
                   arms_changed=False, radio_seeds_changed=False, repair_sha256=sha(Path(__file__)),
                   change='Use one common unit field set. Embedded-mark inputs point optional mark/ratio paths to the same complete MAT.')
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    repaired['source_sha256'][str(receipt_path.relative_to(ROOT))] = sha(receipt_path)
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in repaired['source_sha256']]
    assert len(set(mapped)) == len(mapped)
    destination = OUT / 'stages' / (repaired['stage'] + '.json'); assert not destination.exists()
    destination.write_text(json.dumps(repaired, indent=2) + '\n')
    print('REPAIRED REGISTRATION', repaired['stage'], len(units), 'uniform units; algorithms and inputs unchanged.', flush=True)


if __name__ == '__main__':
    main()
