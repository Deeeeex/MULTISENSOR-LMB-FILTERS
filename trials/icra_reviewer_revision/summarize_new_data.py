"""Report every newly acquired segment, retaining recording-level exposure."""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PRIMARY = 'marked_gaussian_evidence'
CONDITIONS = ['reliable', 'intermittent']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError',
           'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_audit(name):
    value = json.loads((OUT / name).read_text())
    assert value['passed']
    for relative, digest in value['inputs'].items():
        assert sha(ROOT / relative) == digest, relative
    assert sha(OUT / 'stages' / (value['stage'] + '.json')) == value['config_sha256']
    assert sha(OUT / ('runtime_' + value['stage'] + '.json')) == value['runtime_sha256']
    return value


def main():
    destination = OUT / 'NEW_DATA_ANALYSIS.json'
    assert not destination.exists()
    audits = [checked_audit(name) for name in
              ['audit_new_validation_primary.json', 'audit_new_validation_recency.json']]
    freeze = json.loads((OUT / 'NEW_DATA_FREEZE.json').read_text())
    assert json.loads((OUT / 'NEW_INPUT_AUDIT.json').read_text())['passed']
    sequences = [{k: value[k] for k in ['sequence', 'scene', 'original_recording',
                 'frames', 'original_recording_present_in_old_data']} for value in freeze['sequences']]
    rows = [row for report in audits for row in report['rows']]
    arms = sorted({row['arm'] for row in rows})
    lookup = {(row['sequence'], row['condition'], row['arm']): row for row in rows}
    assert len(rows) == len(lookup) == 3 * 2 * 7
    assert sum(row['frames'] for row in sequences) == 748
    groups = {}
    for key, names in [
        ('all_new_segments', [row['sequence'] for row in sequences]),
        ('new_recording', [row['sequence'] for row in sequences if not row['original_recording_present_in_old_data']]),
        ('related_recording_segments', [row['sequence'] for row in sequences if row['original_recording_present_in_old_data']]),
    ]:
        assert names
        aggregates, differences = [], []
        for condition in CONDITIONS:
            for arm in arms:
                selected = [lookup[name, condition, arm] for name in names]
                aggregates.append(dict(condition=condition, arm=arm,
                    **{metric: float(np.mean([row[metric] for row in selected])) for metric in METRICS}))
                if arm != PRIMARY:
                    values = [lookup[name, condition, PRIMARY]['ospa'] - lookup[name, condition, arm]['ospa'] for name in names]
                    differences.append(dict(condition=condition, candidate=PRIMARY, reference=arm,
                        mean=float(np.mean(values)), differences=values,
                        lower_ospa_segments=sum(value < -1e-10 for value in values)))
        groups[key] = dict(sequences=names, frames=sum(row['frames'] for row in sequences if row['sequence'] in names),
                           aggregate=aggregates, paired=differences)
    sources = ['audit_new_validation_primary.json', 'audit_new_validation_recency.json',
               'NEW_DATA_FREEZE.json', 'NEW_INPUT_AUDIT.json', 'NEW_INPUT_MANIFEST.json',
               'NEW_DETECTION_MANIFEST.json', 'FIXED_SELECTION.json', 'OFFICIAL_ADAPTER_PREFLIGHT.json']
    result = dict(passed=True, sequences=sequences, arms=arms, rows=rows, groups=groups,
        source_reports={name: sha(OUT / name) for name in sources}, analyzer_sha256=sha(Path(__file__)),
        uncertainty='Three segments from two original recordings; only one recording was absent from old train/test archives. Report all segment means and differences without an inferential confidence interval.',
        primary_selection='The same primary GCE, detector checkpoint, calibration and settings were fixed before new detection inference and tracking. Fixed Ratio lambda=0.25 was selected only on the nine old development sequences.',
        detector_parity='Identical checkpoint and author model/postprocessor; deterministic CPU voxelization and MPS inference are not bitwise identical to released GPU detections. The old-frame preflight quantifies the difference.',
        communication='No-age packets in this extension carry the 216-byte legacy development schema. GS, no-curvature, Fixed and GCE use full 352-byte Gaussian packets; codec savings are evaluated separately on the original corpus.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / 'new_data_sequence_scores.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    for name, group in groups.items():
        print(name, group['sequences'], group['frames'], flush=True)
        for row in group['aggregate']:
            print(row['condition'], row['arm'], round(row['ospa'], 6), flush=True)


if __name__ == '__main__':
    main()
