"""Release the complete external summary after the development choice."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def main():
    destination = OUT / 'EXTERNAL_ANALYSIS.json'
    assert not destination.exists()
    selection_path = TRIALS / 'icra_compatible_admission/FINAL_SELECTION.json'
    selection = json.loads(selection_path.read_text())
    assert selection['passed'] and not selection['external_tracking_scores_inspected']
    primary = selection['selected']['arm']
    assert primary == 'marked_gaussian_evidence_projected_space'
    folder = TRIALS / 'icra_projected_admission'
    stage = 'final_v2x_evaluation'
    auditpath = folder / ('audit_' + stage + '.json')
    audit = json.loads(auditpath.read_text()); assert audit['passed']
    cfgpath = folder / 'stages' / (stage + '.json')
    runtimepath = folder / ('runtime_' + stage + '.json')
    assert audit['config_sha256'] == sha(cfgpath)
    assert audit['runtime_sha256'] == sha(runtimepath)
    cfg = json.loads(cfgpath.read_text())
    native = json.loads(runtimepath.read_text())
    assert len(native) == 5 and all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 16 for r in native)
    inputs = {str(p.relative_to(ROOT)): sha(p) for p in [selection_path, auditpath, cfgpath, runtimepath, Path(__file__), OUT / 'PROTOCOL.md']}
    for source in [cfg['source_sha256'], audit['auditor_sha256'], audit['inputs'], selection['source_sha256']]:
        for key, expected in source.items():
            if key not in inputs:
                inputs[key] = sha(ROOT / key)
            assert inputs[key] == expected, key
    units = {r['sequence']: r for r in cfg['units']}
    rows = [dict(r, scene=units[r['sequence']]['scene'], collection_date=units[r['sequence']]['recording'],
                 geometry='overlapping_supports' if units[r['sequence']]['relative_vehicle_range_max'] < 80 else 'disjoint_supports')
            for r in audit['rows']]
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    assert len(rows) == len(lookup) == 80
    aggregate = []; dates = []; paired = []
    for scope in ['all', 'overlapping_supports', 'disjoint_supports']:
        part = rows if scope == 'all' else [r for r in rows if r['geometry'] == scope]
        scenes = sorted({r['sequence'] for r in part})
        assert len(scenes) == {'all': 5, 'overlapping_supports': 3, 'disjoint_supports': 2}[scope]
        for condition in cfg['conditions']:
            for arm in cfg['arms']:
                selected = [r for r in part if r['condition'] == condition and r['arm'] == arm]
                by_date = defaultdict(list)
                for r in selected:
                    by_date[r['collection_date']].append(r)
                for date, group in by_date.items():
                    dates.append(dict(scope=scope, collection_date=date, condition=condition, arm=arm,
                                      **{k: float(np.mean([r[k] for r in group])) for k in METRICS}))
                aggregate.append(dict(scope=scope, condition=condition, arm=arm, sequences=len(selected),
                    frames=sum(r['frames'] for r in selected), dates=len(by_date),
                    sequence_macro={k: float(np.mean([r[k] for r in selected])) for k in METRICS},
                    date_macro_ospa=float(np.mean([np.mean([r['ospa'] for r in g]) for g in by_date.values()]))))
            for candidate in [primary, 'marked_gaussian_evidence']:
                for reference in cfg['arms']:
                    if candidate == reference:
                        continue
                    values = np.array([lookup[s, condition, candidate]['ospa'] - lookup[s, condition, reference]['ospa'] for s in scenes])
                    paired.append(dict(scope=scope, condition=condition, candidate=candidate, reference=reference,
                        sequence_macro_difference=float(values.mean()), differences=values.tolist(), sequences=scenes,
                        wins=int(np.sum(values < -1e-9)), ties=int(np.sum(np.abs(values) <= 1e-9)), losses=int(np.sum(values > 1e-9))))
    report = dict(passed=True, first_external_scores_viewed_utc=datetime.now(timezone.utc).isoformat(),
        candidate_selection_utc=selection['selected_utc'], selected_candidate=primary, selected_fixed=selection['selected_fixed']['arm'],
        all_five_scenes_retained=True, paired_frames=619, collection_dates=3, aggregate=aggregate, paired=paired,
        date_means=dates, rows=rows, inputs=inputs,
        inference='Descriptive external vehicle-pair transfer with frozen detector and calibration; no inferential interval for three dates.')
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for scope in ['all', 'overlapping_supports', 'disjoint_supports']:
        print('SCOPE', scope, flush=True)
        for arm in cfg['arms']:
            values = [next(r['sequence_macro']['ospa'] for r in aggregate if r['scope'] == scope and r['arm'] == arm and r['condition'] == c) for c in cfg['conditions']]
            print(arm, *values, flush=True)


if __name__ == '__main__':
    main()
