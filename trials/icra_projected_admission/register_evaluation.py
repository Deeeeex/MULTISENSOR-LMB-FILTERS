"""Freeze selected-arm evaluation only after the complete development choice."""
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
    parser.add_argument('dataset', choices=['v2v', 'v2x'])
    args = parser.parse_args()
    stage = 'final_' + args.dataset + '_evaluation'
    path = OUT / 'stages' / f'{stage}.json'
    assert not path.exists() and not (OUT / 'results' / stage).exists()
    selection_path = OUT / 'FINAL_SELECTION.json'
    selection = json.loads(selection_path.read_text()); assert selection['passed']
    for key, expected in selection['source_sha256'].items():
        assert sha(ROOT / key) == expected, key
    primary = selection['selected']['arm']; fixed = selection['selected_fixed']['arm']
    sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
    for key, expected in sources.items():
        assert sha(ROOT / key) == expected, key
    protected = [selection_path, OUT / 'PROTOCOL.md', OUT / 'PROJECTION_DIAGNOSTIC.json']
    units = []
    if args.dataset == 'v2v':
        inventory_path = OUT.parent / 'icra_full_coverage/DATA_INVENTORY.json'
        inventory = json.loads(inventory_path.read_text())
        mapping = {(r['split'], r['sequence']): r for r in inventory['rows']}
        protected.append(inventory_path)
        for previous, split in [(REVIEW / 'stages/gs_seen_transfer.json', 'train'),
                                 (OUT.parent / 'icra_full_coverage/stages/coverage_remaining_train.json', 'train'),
                                 (REVIEW / 'stages/new_validation_primary.json', 'val')]:
            old = json.loads(previous.read_text()); protected.append(previous)
            for row in old['units']:
                item = mapping[split, row['sequence']]
                new = dict(row, sequence=split + '_' + row['sequence'], original_sequence=row['sequence'],
                           split=split, scene=item['scene'], recording=item['recording'])
                units.append(new)
        assert len(units) == 34 and len({r['scene'] for r in units}) == 34
        # Reuse complete old trajectories whenever an unchanged arm wins.
        arms = [a for a in [primary, fixed] if a not in ['marked_gaussian_evidence', 'marked_gaussian_evidence_fixed_025']]
        assert arms and len(arms) == len(set(arms))
    else:
        folder = OUT.parent / 'icra_v2x_transfer'
        manifest_path = folder / 'NEW_INPUT_MANIFEST.json'
        manifest = json.loads(manifest_path.read_text()); assert manifest['passed']
        audit_path = folder / 'NEW_INPUT_AUDIT.json'
        audit = json.loads(audit_path.read_text())
        assert audit['passed'] and audit['manifest_sha256'] == sha(manifest_path)
        for key, expected in manifest['source_sha256'].items():
            assert sha(ROOT / key) == expected, key
        freeze_path = folder / 'INFERENCE_FREEZE.json'
        freeze = json.loads(freeze_path.read_text())
        for key, expected in {**freeze['protected_sha256'], **freeze['raw_file_sha256']}.items():
            assert sha(ROOT / key) == expected, key
        protected += [manifest_path, audit_path, freeze_path, folder / 'DETECTION_MANIFEST.json']
        protected += list(folder.glob('*.py')) + [folder / 'PROTOCOL.md', folder / 'ADAPTER_NOTES.md']
        for row in manifest['sequences']:
            units.append(dict(row, sequence='v2x_' + row['sequence'], original_sequence=row['sequence'],
                              split='v2x_val', recording=row['collection_date']))
        assert len(units) == 5 and sum(u['frames'] for u in units) == 619
        arms = ['marked_lineage', 'marked_er', 'marked_asymmetric', 'marked_gaussian_evidence_guarded_scalar',
                'marked_gaussian_evidence_no_curvature', fixed, 'marked_gaussian_evidence']
        if primary not in arms:
            arms.append(primary)
    for unit in units:
        assert sha(ROOT / unit['data_path']) == unit['input_sha256']
        for field in ['data_path', 'marks_path', 'ratios_path']:
            if field in unit:
                protected.append(ROOT / unit[field])
    for folder in [OUT, OUT.parent / 'icra_admission_revision']:
        protected += list(folder.glob('*.m')) + list(folder.glob('*.py'))
    protected += [REVIEW / n for n in ['runReviewerReplay.m', 'checkReviewerEvidence.m', 'fuseReviewerEvidence.m',
                                      'review_probability_audit.py', 'review_gaussian_audit.py']]
    protected += [OUT.parent / 'icra_ceiling_iteration/calibration.json',
                  OUT.parent / 'icra_marked_iteration/likelihood_manifest.json']
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
    assert len(set(mapped)) == len(mapped)
    report = dict(protocol='icra-projected-admission-v1', stage=stage, cohort='evaluation_' + args.dataset,
                  created_utc=datetime.now(timezone.utc).isoformat(), arms=arms, pd=.9, preflight=False,
                  conditions=['reliable', 'intermittent'], units=units, source_sha256=sources,
                  selection_sha256=sha(selection_path), selected_primary=primary, selected_fixed=fixed,
                  selection_utc=selection['selected_utc'])
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print('REGISTERED FROZEN EVALUATION', stage, len(units), 'scenes', len(arms), 'arms', len(units) * len(arms) * 2, 'outputs', flush=True)


if __name__ == '__main__':
    main()
