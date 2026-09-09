"""Retry the failed heterogeneous unit list with an otherwise identical runner."""
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
    repair_path = OUT / 'ASSESSMENT_LOADING_REPAIR.json'
    repair = json.loads(repair_path.read_text())
    assert repair['failed_units'] == 34 and repair['completed_trajectories'] == 0
    previous = OUT / 'stages' / (repair['failed_stage'] + '.json')
    cfg = json.loads(previous.read_text())
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    for name, expected in repair['sources'].items():
        assert sha(ROOT / name) == expected, name
    for field in ['original_matlab', 'new_matlab', 'original_launcher', 'new_launcher']:
        assert sha(ROOT / repair[field]['path']) == repair[field]['sha256']
    stage = 'association_selected_v2v_loaded'
    destination = OUT / 'stages' / (stage + '.json')
    assert not destination.exists() and not (OUT / 'results' / stage).exists()
    protected = list(OUT.glob('*.py')) + list(OUT.glob('*.m')) + [repair_path, previous]
    cfg['source_sha256'].update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in cfg['source_sha256']]
    assert len(mapped) == len(set(mapped))
    cfg.update(stage=stage, created_utc=datetime.now(timezone.utc).isoformat(), retry_of=repair['failed_stage'],
               metadata_loader=repair['new_matlab']['path'])
    destination.write_text(json.dumps(cfg, indent=2, allow_nan=False) + '\n')
    print('REGISTERED METADATA RETRY', stage, len(cfg['units']), 'unchanged units;',
          len(cfg['units']) * len(cfg['arms']) * 2, 'outputs', flush=True)


if __name__ == '__main__':
    main()
