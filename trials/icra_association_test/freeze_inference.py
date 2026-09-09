"""Freeze all selected raw inputs and unchanged detection dependencies."""
from datetime import datetime, timezone
from pathlib import Path
import json

from input_adapter import OUT, ROOT, AUTHOR, old, sha


def main():
    path = OUT / 'INFERENCE_FREEZE.json'; assert not path.exists()
    cohort = json.loads((OUT / 'COHORT_FREEZE.json').read_text())
    raw = json.loads((OUT / 'RAW_INPUT_MANIFEST.json').read_text())
    assert raw['passed'] and raw['cohort_freeze_sha256'] == sha(OUT / 'COHORT_FREEZE.json')
    check = json.loads((OUT / 'ADAPTER_CHECK.json').read_text())
    assert check['passed'] and check['adapter_sha256'] == sha(OUT / 'input_adapter.py')
    hashes = {}
    for row in raw['files']:
        assert sha(ROOT / row['path']) == row['sha256']
        hashes[row['path']] = row['sha256']
    paths = [OUT / n for n in ['PROTOCOL.md', 'COHORT_FREEZE.json', 'RAW_INPUT_MANIFEST.json',
                              'input_adapter.py', 'freeze_inference.py', 'infer_transfer.py',
                              'check_adapter.py', 'ADAPTER_CHECK.json', 'INPUT_PIPELINE_PATCH.json', 'fetch_inputs.py',
                              'prepare_inputs.py', 'audit_inputs.py']]
    paths += [old.OUT / 'official_input_adapter.py', OUT.parent / 'icra_v2x_transfer/ADAPTER_NOTES.md',
              OUT.parent / 'icra_v2x_transfer/SHARED_ANNOTATION_DIAGNOSTIC.json']
    paths += [old.AUTHOR / 'official_models/no_fusion_keep_all' / n for n in ['config.yaml', 'net_epoch60.pth']]
    paths += [AUTHOR / 'opencood' / n for n in ['utils/transformation_utils.py', 'utils/pcd_utils.py',
                                             'utils/box_utils.py', 'data_utils/__init__.py', 'data_utils/datasets/basedataset.py',
                                             'data_utils/post_processor/base_postprocessor.py']]
    paths += [OUT.parent / 'icra_ceiling_iteration/calibration.json',
              OUT.parent / 'icra_marked_iteration/likelihood_manifest.json']
    protected = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), sequences=cohort['sequences'],
                  paired_frames=2172, raw_file_sha256=hashes, protected_sha256=protected,
                  detector='Unchanged DMSTrack released no_fusion_keep_all/net_epoch60.pth',
                  inference='MPS float32, batch one, fixed shuffle seed 20 + 100000*sequence + 2*frame + sensor',
                  training=False, truth_used_for_detections=False)
    path.write_text(json.dumps(report, indent=2) + '\n')
    print('V2X INFERENCE FROZEN', len(hashes), 'raw files', flush=True)


if __name__ == '__main__':
    main()
