"""Register the observation-array repair without changing the selected method."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous_stage = 'association_selected_v2v_loaded'
    previous = OUT / 'stages' / (previous_stage + '.json')
    cfg = json.loads(previous.read_text())
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    ledger = OUT / ('runtime_' + previous_stage + '.json')
    runtime = json.loads(ledger.read_text())
    assert len(runtime) == 1 and runtime[0]['sequence'] == 'train_0001'
    assert runtime[0]['returncode'] == 1 and runtime[0]['files'] == 0
    failed_log = ROOT / 'RUN/ICRA_TEMPORAL_ASSOCIATION' / previous_stage / 'train_0001.log'
    assert 'currentCost(qualified)=sumDistance(qualified)' in failed_log.read_text()
    unit_log = ROOT / 'RUN/ICRA_TEMPORAL_ASSOCIATION/column_shape_unit.log'
    assert 'COLUMN ASSOCIATION CHECK PASSED' in unit_log.read_text()
    patch_path = OUT / 'COLUMN_SHAPE_PATCH.json'
    patch = json.loads(patch_path.read_text())
    assert not patch['equation_change']
    for row in patch['files']:
        for key in ['source', 'destination']:
            assert sha(ROOT / row[key]) == row[key + '_sha256']
    receipt_path = OUT / 'COLUMN_FAILURE.json'
    assert not receipt_path.exists()
    receipt = dict(created_utc=datetime.now(timezone.utc).isoformat(), failed_stage=previous_stage,
        failed_units=1, completed_trajectories=0, equation_change=False,
        repair='Columnize both indexed operands before division; retain scalar, column, matrix, and empty-case exact parity.',
        native_boundary_check_returncode=0,
        inputs={str(p.relative_to(ROOT)): sha(p) for p in [previous, ledger, failed_log, unit_log, patch_path]})
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    stage = 'association_selected_v2v_column'
    destination = OUT / 'stages' / (stage + '.json')
    assert not destination.exists() and not (OUT / 'results' / stage).exists()
    protected = list(OUT.glob('*.py')) + list(OUT.glob('*.m')) + [receipt_path, patch_path]
    cfg['source_sha256'].update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in cfg['source_sha256']]
    assert len(mapped) == len(set(mapped))
    cfg.update(stage=stage, created_utc=datetime.now(timezone.utc).isoformat(), retry_of=previous_stage,
               previous_stage_sha256=sha(previous), metadata_loader='trials/icra_temporal_association/runColumnAssociation.m',
               array_shape_repair=str(patch_path.relative_to(ROOT)))
    destination.write_text(json.dumps(cfg, indent=2, allow_nan=False) + '\n')
    print('REGISTERED COLUMN RETRY', stage, len(cfg['units']), 'unchanged units', flush=True)


if __name__ == '__main__':
    main()
