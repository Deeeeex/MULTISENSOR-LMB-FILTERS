"""Freeze matched-fusion controls and exposed-release tests after development."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', choices=['controls_development', 'v2v', 'v2x'])
    args = parser.parse_args()
    stage = 'association_selected_' + args.dataset
    destination = OUT / 'stages' / (stage + '.json')
    assert not destination.exists() and not (OUT / 'results' / stage).exists()
    selection_path = OUT / 'RESTORED_DEVELOPMENT_SELECTION.json'
    selection = json.loads(selection_path.read_text())
    assert selection['passed'] and selection['advance'] and selection['selected']['eligible']
    for group in [selection['source_sha256'], selection['inputs']]:
        for key, expected in group.items():
            assert sha(ROOT / key) == expected, key
    selected = selection['selected']['arm']
    mode = selected.rsplit('_assoc_', 1)[1]
    assert mode in ['direct', 'temporal', 'reopen', 'split']
    control = 'marked_gaussian_evidence_guarded_scalar_assoc_' + mode
    sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
    for name, expected in sources.items():
        assert sha(ROOT / name) == expected, name
    protected = [selection_path, OUT / 'PATH_RESTORATION.md', OUT / 'CANDIDATES_V1.md', OUT / 'CANDIDATES_V2.md']
    units = []
    if args.dataset == 'controls_development':
        previous = REVIEW / 'stages/controls_development.json'
        units = json.loads(previous.read_text())['units']
        protected.append(previous)
        assert len(units) == 9
        arms = [control]
    elif args.dataset == 'v2v':
        inventory_path = OUT.parent / 'icra_full_coverage/DATA_INVENTORY.json'
        inventory = json.loads(inventory_path.read_text())
        mapping = {(r['split'], r['sequence']): r for r in inventory['rows']}
        protected.append(inventory_path)
        for previous, split in [(REVIEW / 'stages/gs_seen_transfer.json', 'train'),
                (OUT.parent / 'icra_full_coverage/stages/coverage_remaining_train.json', 'train'),
                (REVIEW / 'stages/new_validation_primary.json', 'val')]:
            protected.append(previous)
            for row in json.loads(previous.read_text())['units']:
                scene = mapping[split, row['sequence']]
                units.append(dict(row, sequence=split + '_' + row['sequence'], original_sequence=row['sequence'],
                    split=split, scene=scene['scene'], recording=scene['recording']))
        assert len(units) == len({r['scene'] for r in units}) == 34
        arms = [selected, control]
    else:
        folder = OUT.parent / 'icra_v2x_transfer'
        manifest_path, audit_path = folder / 'NEW_INPUT_MANIFEST.json', folder / 'NEW_INPUT_AUDIT.json'
        manifest, audit = json.loads(manifest_path.read_text()), json.loads(audit_path.read_text())
        assert manifest['passed'] and audit['passed'] and audit['manifest_sha256'] == sha(manifest_path)
        for key, expected in manifest['source_sha256'].items():
            assert sha(ROOT / key) == expected, key
        freeze_path = folder / 'INFERENCE_FREEZE.json'
        freeze = json.loads(freeze_path.read_text())
        for group in [freeze['protected_sha256'], freeze['raw_file_sha256']]:
            for key, expected in group.items():
                assert sha(ROOT / key) == expected, key
        protected += [manifest_path, audit_path, freeze_path, folder / 'DETECTION_MANIFEST.json']
        for row in manifest['sequences']:
            units.append(dict(row, sequence='v2x_' + row['sequence'], original_sequence=row['sequence'],
                split='v2x_val', recording=row['collection_date']))
        assert len(units) == 5 and sum(r['frames'] for r in units) == 619
        arms = [selected, control]
    for unit in units:
        assert sha(ROOT / unit['data_path']) == unit['input_sha256']
        for field in ['data_path', 'marks_path', 'ratios_path']:
            if field in unit:
                protected.append(ROOT / unit[field])
    protected += list(OUT.glob('*.py')) + list(OUT.glob('*.m'))
    protected += [REVIEW / name for name in ['runReviewerReplay.m', 'checkReviewerEvidence.m',
        'fuseReviewerEvidence.m', 'review_probability_audit.py', 'review_gaussian_audit.py']]
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
    assert len(set(mapped)) == len(mapped), 'MATLAB field-name collision'
    result = dict(protocol='icra-restored-association-v1', stage=stage,
        cohort='association_assessment_' + args.dataset, created_utc=datetime.now(timezone.utc).isoformat(),
        arms=arms, pd=.9, preflight=False, conditions=['reliable', 'intermittent'], units=units,
        source_sha256=sources, selection_sha256=sha(selection_path), selected_primary=selected,
        matched_frontend_control=control, exposure='Previously exposed development or release data; not the additional test cohort.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('REGISTERED ASSOCIATION ASSESSMENT', stage, len(units), 'segments', len(arms), 'arms',
        len(units) * len(arms) * 2, 'native outputs', flush=True)


if __name__ == '__main__':
    main()
