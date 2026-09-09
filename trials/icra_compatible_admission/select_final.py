"""Choose once from complete, audited development recursions."""
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
ORIGINAL = 'marked_gaussian_evidence'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT / 'FINAL_SELECTION.json'; assert not destination.exists()
    first_path = OUT.parent / 'icra_projected_admission/FINAL_SELECTION.json'
    first = json.loads(first_path.read_text()); assert first['passed']
    rows = first['development_rows'].copy(); inputs = {str(first_path.relative_to(ROOT)): sha(first_path)}
    mapping = {r['sequence']: r['recording'] for r in rows}
    for name, expected in first['inputs'].items():
        assert sha(ROOT / name) == expected, name
        inputs[name] = expected
    for stage in ['compatible_v1_preflight', 'compatible_v1_development_rest']:
        path = OUT / f'audit_{stage}.json'; report = json.loads(path.read_text()); assert report['passed']
        assert sha(OUT / f'stages/{stage}.json') == report['config_sha256']
        assert sha(OUT / f'runtime_{stage}.json') == report['runtime_sha256']
        inputs[str(path.relative_to(ROOT))] = sha(path)
        for name, expected in report['inputs'].items():
            assert sha(ROOT / name) == expected, name
            inputs[name] = expected
        rows += [dict(sequence=r['sequence'], recording=mapping[r['sequence']], condition=r['condition'],
                      arm=r['arm'], frames=r['frames'], **{k: r[k] for k in METRICS})
                 for r in report['rows'] if '_compatible_' in r['arm']]
    arms = sorted({r['arm'] for r in rows})
    assert len(arms) == 22
    assert len({(r['sequence'], r['condition'], r['arm']) for r in rows}) == len(rows) == 22 * 18
    aggregate = []
    for arm in arms:
        means = {}; details = {}
        for condition in CONDITIONS:
            selected = [r for r in rows if r['arm'] == arm and r['condition'] == condition]
            assert len(selected) == 9
            by_recording = defaultdict(list)
            for r in selected:
                by_recording[r['recording']].append(r['ospa'])
            means[condition] = float(np.mean([r['ospa'] for r in selected]))
            details[condition] = dict(sequence_macro={k: float(np.mean([r[k] for r in selected])) for k in METRICS},
                                      recording_macro_ospa=float(np.mean([np.mean(v) for v in by_recording.values()])),
                                      recording_count=len(by_recording))
        aggregate.append(dict(arm=arm, sequence_macro_ospa_by_condition=means,
                              selection_mean_ospa=float(np.mean(list(means.values()))), metrics=details))
    candidates = [r for r in aggregate if r['arm'] == ORIGINAL or '_decoupled_' in r['arm'] or '_projected_' in r['arm'] or '_compatible_' in r['arm']]
    candidates.sort(key=lambda r: (r['selection_mean_ospa'], r['arm'] != ORIGINAL, '_compatible_' in r['arm'], '_projected_' in r['arm'], r['arm']))
    assert len(candidates) == 10
    selected = candidates[0]
    original = next(r for r in candidates if r['arm'] == ORIGINAL)
    selected_fixed = first['selected_fixed']
    report = dict(passed=True, selected_utc=datetime.now(timezone.utc).isoformat(),
                  selected=selected, original=original, selected_fixed=selected_fixed,
                  candidate_ranking=candidates, aggregate=aggregate, development_rows=rows,
                  development_sequences=9, nonoriginal_admission_candidates=9,
                  fixed_strength_grid=[0, .05, .1, .125, .25, .5, 1],
                  rule='Lowest mean reliable/intermittent sequence-macro OSPA on all nine development sequences; exact ties retain the earlier version. Fixed-grid ties prefer lower eta.',
                  external_tracking_scores_inspected=False, remaining_release_candidate_scores_inspected=True,
                  earlier_projection_external_results_available=True,
                  exposure_note='No V2X tracking scores have been inspected. A routine native-log tail exposed reliable train_0014 projected-space and fixed-zero OSPA after all third-family native development runs completed and before this selection report. Candidate definitions and the already-written development-only selection rule remain unchanged; see DATA_EXPOSURE.json.',
                  selector_sha256=sha(Path(__file__)), inputs=inputs)
    report['source_sha256'] = {str(p.relative_to(ROOT)): sha(p) for folder in [OUT, OUT.parent / 'icra_admission_revision', OUT.parent / 'icra_projected_admission']
                              for p in folder.glob('*.m')}
    exposure_path = OUT / 'DATA_EXPOSURE.json'
    report['data_exposure'] = json.loads(exposure_path.read_text())
    report['inputs'][str(exposure_path.relative_to(ROOT))] = sha(exposure_path)
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (OUT / 'development_comparison.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)
    print('FINAL DEVELOPMENT RANKING', flush=True)
    for r in candidates:
        print(r['arm'], r['sequence_macro_ospa_by_condition'], r['selection_mean_ospa'], flush=True)
    print('SELECTED', selected['arm'], 'FIXED', selected_fixed['arm'], flush=True)


if __name__ == '__main__':
    main()
