"""Metadata-only registration of all paired mobile inputs in V2X-Real test."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
directory_path = ROOT / 'tmp/external_baselines/v2x_real/test/directory.json'
directory = json.loads(directory_path.read_text())
assert directory['file_id'] == '1668588258866' and directory['archive_bytes'] == 5772980847
groups = {}
for entry in directory['entries']:
    p = Path(entry['name'])
    if len(p.parts) == 4 and p.suffix == '.yaml':
        groups.setdefault(p.parts[1], {}).setdefault(p.parts[2], set()).add(p.stem)
sequences = []
for scene, agents in sorted(groups.items()):
    assert all(a in agents for a in ['1', '2']), scene
    stems = sorted(agents['1'] & agents['2'])
    assert set(stems) == agents['1'] == agents['2']
    assert stems == [f'{i:06d}' for i in range(len(stems))]
    sequences.append(dict(sequence=f'{len(sequences):04d}', scene=scene, frames=len(stems),
        paired_stems=stems, collection_date=scene[:10], timestamp_group=scene[:19],
        radio_seed=8301 + len(sequences)))
assert len(sequences) == 14 and sum(r['frames'] for r in sequences) == 2172
names = {f"test/{r['scene']}/{agent}/{stem}.{kind}" for r in sequences
         for agent in ['1', '2'] for stem in r['paired_stems'] for kind in ['yaml', 'bin']}
entries = [e for e in directory['entries'] if e['name'] in names]
assert len(entries) == len(names) == 8688
validation_path = OUT.parent / 'icra_v2x_transfer/COHORT_FREEZE.json'
validation = json.loads(validation_path.read_text())
assert not {r['scene'] for r in sequences} & {r['scene'] for r in validation['sequences']}
old_clouds = {(r['crc'], r['raw']) for r in validation['files'] if r['name'].endswith('.bin')}
repeated = [r['name'] for r in entries if r['name'].endswith('.bin') and (r['crc'], r['raw']) in old_clouds]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
result = dict(created_utc=datetime.now(timezone.utc).isoformat(),
    source=directory['source'], file_id=directory['file_id'], archive_bytes=directory['archive_bytes'],
    selection='All official 64-line test segments with paired mobile agents 1 and 2; no tracking-score selection.',
    sequences=sequences, paired_frames=2172, files=entries,
    raw_bytes=sum(e['raw'] for e in entries), compressed_bytes=sum(e['compressed'] for e in entries),
    collection_dates=sorted({r['collection_date'] for r in sequences}),
    annotation='Collection dates and timestamp groups describe related segments; 14 segments are not 14 independently sampled recordings.',
    overlap_screen=dict(shared_scene_names=[], shared_cloud_crc_size_keys=repeated,
        rule='CRC and size are a preliminary input-overlap screen; confirm exact hashes after acquisition.'),
    sources={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), directory_path, validation_path]},
    exposure='Central-directory metadata only. No test point cloud, annotation, detector output or tracking outcome has been inspected in this study.',
    execution='Acquire and evaluate after an association candidate passes its development gate; keep model and calibration fixed.')
destination = OUT / 'PROSPECTIVE_TEST_COHORT.json'
assert not destination.exists()
destination.write_text(json.dumps(result, indent=2) + '\n')
print('PROSPECTIVE TEST FROZEN', len(sequences), 'segments', result['paired_frames'], 'paired frames; raw cloud CRC-size overlap', len(repeated))
