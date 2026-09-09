"""Compare all audited first-revision arms on the original nine sequences."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
CONDITIONS = ['reliable', 'intermittent']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT / 'ADMISSION_V1_DEVELOPMENT.json'
    assert not destination.exists()
    provenance = {}
    rows = []
    for stage in ['admission_v1_preflight', 'admission_v1_development_rest']:
        path = OUT / f'audit_{stage}.json'
        audit = json.loads(path.read_text())
        assert audit['passed'] and audit['stage'] == stage
        assert sha(OUT / f'stages/{stage}.json') == audit['config_sha256']
        assert sha(OUT / f'runtime_{stage}.json') == audit['runtime_sha256']
        for key, expected in audit['inputs'].items():
            assert sha(ROOT / key) == expected, key
            provenance[key] = expected
        provenance[str(path.relative_to(ROOT))] = sha(path)
        rows += [r for r in audit['rows'] if '_decoupled_' in r['arm'] or '_fixedx_' in r['arm']]
    fullpath = OUT.parent / 'icra_full_coverage/COVERAGE_ANALYSIS.json'
    full = json.loads(fullpath.read_text()); assert full['passed']
    provenance[str(fullpath.relative_to(ROOT))] = sha(fullpath)
    for key, expected in full['inputs'].items():
        assert sha(ROOT / key) == expected, key
    original = [r for r in full['rows'] if r['cohort'] == 'development']
    mapping = {r['sequence']: r for r in original}
    rows += original
    controlpath = OUT.parent / 'icra_reviewer_revision/audit_controls_development.json'
    control = json.loads(controlpath.read_text()); assert control['passed']
    provenance[str(controlpath.relative_to(ROOT))] = sha(controlpath)
    for key, expected in control['inputs'].items():
        assert sha(ROOT / key) == expected, key
    rows += [r for r in control['rows'] if r['arm'] in ['marked_gaussian_evidence_fixed_050', 'marked_gaussian_evidence_fixed_100']]
    rows = [dict(sequence=r['sequence'], recording=mapping[r['sequence']]['recording'],
                 condition=r['condition'], arm=r['arm'], frames=r['frames'],
                 **{key: r[key] for key in METRICS}) for r in rows]
    arms = sorted({r['arm'] for r in rows})
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    assert len(rows) == len(lookup) == 9 * 2 * len(arms)
    assert len(arms) == 17
    aggregate = []
    for arm in arms:
        means = {}
        details = {}
        for condition in CONDITIONS:
            group = [r for r in rows if r['arm'] == arm and r['condition'] == condition]
            assert len(group) == 9
            means[condition] = float(np.mean([r['ospa'] for r in group]))
            recs = defaultdict(list)
            for r in group:
                recs[r['recording']].append(r['ospa'])
            details[condition] = dict(sequence_macro={k: float(np.mean([r[k] for r in group])) for k in METRICS},
                                      recording_macro_ospa=float(np.mean([np.mean(v) for v in recs.values()])),
                                      recording_count=len(recs))
        aggregate.append(dict(arm=arm, sequence_macro_ospa_by_condition=means,
                              selection_mean_ospa=float(np.mean(list(means.values()))), metrics=details))
    fixed = [dict(row, eta=(int(row['arm'].rsplit('_', 1)[1]) / (1000 if '_fixedx_' in row['arm'] else 100)))
             for row in aggregate if '_fixed' in row['arm']]
    fixed.sort(key=lambda r: (r['selection_mean_ospa'], r['eta']))
    proposed = [r for r in aggregate if '_decoupled_' in r['arm'] or r['arm'] == 'marked_gaussian_evidence']
    proposed.sort(key=lambda r: (r['selection_mean_ospa'], r['arm'] != 'marked_gaussian_evidence', r['arm']))
    output = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(),
                  first_revision_candidates=4, new_fixed_strengths=4, sequences=9,
                  rows=rows, aggregate=aggregate, fixed_candidates=fixed,
                  selected_fixed=fixed[0], admission_ranking=proposed,
                  best_first_revision=proposed[0],
                  rule='Compare mean reliable/intermittent sequence-macro OSPA on all nine original development sequences. Fixed-grid ties prefer smaller eta. An exact admission tie retains original GCE.',
                  inputs=provenance, analyzer_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    with (OUT / 'admission_v1_development_scores.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)
    for row in sorted(aggregate, key=lambda r: r['selection_mean_ospa']):
        print(row['arm'], row['sequence_macro_ospa_by_condition'], 'mean', row['selection_mean_ospa'], flush=True)
    print('FIXED SELECTED', fixed[0]['eta'], fixed[0]['arm'], flush=True)
    print('FIRST REVISION BEST', proposed[0]['arm'], flush=True)


if __name__ == '__main__':
    main()
