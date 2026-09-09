"""Freeze all available official validation source frames before inference."""
from datetime import datetime, timezone
import json

from official_input_adapter import *


def main():
    destination = OUT / 'NEW_DATA_FREEZE.json'; assert not destination.exists()
    check = json.loads((OUT / 'OFFICIAL_ADAPTER_PREFLIGHT.json').read_text())
    assert check['passed'] and check['adapter_sha256'] == sha(OUT / 'official_input_adapter.py')
    old_names = set(); old_crc = {'.yaml': set(), '.pcd': set()}; directory_hashes = {}
    for directory in sorted(CACHE.glob('*/directory.json')):
        content = json.loads(directory.read_text()); directory_hashes[str(directory.relative_to(ROOT))] = sha(directory)
        if directory.parent.name == 'val':
            continue
        for entry in content['entries']:
            path = Path(entry['name'])
            if path.suffix in old_crc:
                old_names.add(path.parts[-3]); old_crc[path.suffix].add((entry['raw'], entry['crc']))
    metadata = json.loads((CACHE / 'val/directory.json').read_text())
    overlap_counts = {suffix: sum((entry['raw'], entry['crc']) in signatures for entry in metadata['entries']
                                  if Path(entry['name']).suffix == suffix) for suffix, signatures in old_crc.items()}
    assert not any(overlap_counts.values())
    manifests = [CACHE / f'val/{kind}_manifest.json' for kind in ['yaml', 'pcd']]
    sources = {}; raw_records = {}
    for path in manifests:
        manifest = json.loads(path.read_text()); sources[str(path.relative_to(ROOT))] = sha(path)
        for row in manifest['files']:
            actual = ROOT / row['path']; assert sha(actual) == row['sha256']
            raw_records[str(actual.relative_to(ROOT))] = row['sha256']
    sequences = []
    for index, (name, scene) in enumerate(scene_directories('val').items()):
        assert name not in old_names
        stems = [set(p.stem for p in (scene / str(n)).glob('*.yaml')) for n in range(2)]
        common = sorted(stems[0] & stems[1]); assert common == [f'{t:06d}' for t in range(len(common))]
        for source in range(2):
            for stem in common:
                for suffix in ['.yaml', '.pcd']:
                    assert str((scene / str(source) / (stem + suffix)).relative_to(ROOT)) in raw_records
        recording = name.rsplit('_', 1)[0]
        sequences.append(dict(sequence=f'{index:04d}', scene=name, scene_path=str(scene.relative_to(ROOT)),
                              original_recording=recording, frames=len(common), paired_stems=common,
                              unpaired_source_frames=[sorted(s - set(common)) for s in stems],
                              original_recording_present_in_old_data=any(n.rsplit('_', 1)[0] == recording for n in old_names),
                              inference_seed_rule='20 + 100000*sequence_index + 2*frame_index + sensor_index',
                              radio_seed=8301 + index))
    assert len(sequences) == 3 and sum(row['frames'] for row in sequences) == 748
    protected = [OUT / n for n in ['official_input_adapter.py', 'check_official_adapter.py',
                 'OFFICIAL_ADAPTER_PREFLIGHT.json', 'freeze_new_data.py', 'infer_new_data.py',
                 'FIXED_SELECTION.json', 'PROTOCOL.md', 'PRE_REVISION_FREEZE.json']]
    protected += [AUTHOR / 'official_models/no_fusion_keep_all' / name for name in ['config.yaml', 'net_epoch60.pth']]
    protected += [OUT.parent / 'icra_ceiling_iteration/calibration.json',
                  OUT.parent / 'icra_marked_iteration/likelihood_manifest.json']
    for path in protected:
        sources[str(path.relative_to(ROOT))] = sha(path)
    result = dict(created_utc=datetime.now(timezone.utc).isoformat(), source='Official UCLA public validation archive',
                  source_url='https://ucla.app.box.com/v/UCLA-MobilityLab-V2V4REAL/file/1619910191481',
                  sequences=sequences, frames=748, archive_directory_hashes=directory_hashes,
                  raw_file_sha256=raw_records, protected_sha256=sources,
                  byte_identical_overlap_candidates_by_crc_size=overlap_counts,
                  detector_training_relationship='The unchanged author checkpoint config uses ./data/train for training and ./data/test for validation. This ./data/val archive is distinct. This config is evidence of intended split use, not a forensic guarantee about all checkpoint history.',
                  exposure_relationship='No identical scene names or YAML/PCD size+CRC candidates with old train/test archives. New sequence outcomes were never available for fusion development. Recording/date/route independence must not be inferred; shared recording identities are recorded per sequence.',
                  shuffle_seed='Fixed per source frame, not selected from outcomes',
                  inference_device='MPS float32, CPU decode/NMS; batch size one; model.eval; no augmentation or training',
                  primary_and_controls='Frozen GCE, No-age, Scalar, Guarded Scalar, no-curvature GCE, and fixed lambda=0.25 selected solely on nine old development sequences')
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print('NEW OFFICIAL COHORT FROZEN', [(r['scene'], r['frames'], r['original_recording_present_in_old_data']) for r in sequences], flush=True)


if __name__ == '__main__':
    main()
