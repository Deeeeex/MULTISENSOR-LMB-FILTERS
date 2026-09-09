"""Assemble only frozen choices and complete, independently audited runs."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent
CONDITIONS = ['reliable', 'intermittent']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
ORIGINAL = 'marked_gaussian_evidence'
PROJECTED = 'marked_gaussian_evidence_projected_space'
FIXED025 = 'marked_gaussian_evidence_fixed_025'


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def main():
    destination = OUT / 'EVALUATION_ANALYSIS.json'
    assert not destination.exists()
    # This must precede the first read of any evaluation report.
    selection_path = TRIALS / 'icra_compatible_admission/FINAL_SELECTION.json'
    selection = json.loads(selection_path.read_text())
    assert selection['passed'] and not selection['external_tracking_scores_inspected']
    primary = selection['selected']['arm']
    fixed = selection['selected_fixed']['arm']
    inputs = {}

    def remember(path, expected=None):
        name = str(path.relative_to(ROOT))
        if name not in inputs:
            inputs[name] = sha(path)
        if expected is not None:
            assert inputs[name] == expected, name

    remember(selection_path)
    for name, expected in selection['source_sha256'].items():
        remember(ROOT / name, expected)
    for name, expected in selection['inputs'].items():
        remember(ROOT / name, expected)
    coverage_path = TRIALS / 'icra_full_coverage/COVERAGE_ANALYSIS.json'
    coverage = json.loads(coverage_path.read_text())
    assert coverage['passed'] and coverage['unique_scenes'] == 43
    remember(coverage_path)
    for name, expected in coverage['inputs'].items():
        remember(ROOT / name, expected)
    rows = [dict(dataset='v2v', geometry='all', **r) for r in coverage['rows']]
    devmap = {r['sequence']: r for r in rows if r['cohort'] == 'development'}
    for r in selection['development_rows']:
        if r['arm'] not in {primary, PROJECTED, fixed} - set(coverage['methods']):
            continue
        original = devmap[r['sequence']]
        rows.append(dict(dataset='v2v', geometry='all', cohort='development',
                         split=original['split'], scene=original['scene'], **r))
    external_manifest = TRIALS / 'icra_v2x_transfer/NEW_INPUT_MANIFEST.json'
    external = json.loads(external_manifest.read_text())
    assert external['passed'] and external['frames'] == 619
    remember(external_manifest)
    external_map = {r['sequence']: r for r in external['sequences']}
    for r in external_map.values():
        assert r['relative_vehicle_range_max'] < 80 or r['relative_vehicle_range_min'] >= 80

    def collect(folder, stage, dataset):
        cfgpath = folder / 'stages' / (stage + '.json')
        runtimepath = folder / ('runtime_' + stage + '.json')
        auditpath = folder / ('audit_' + stage + '.json')
        audit = json.loads(auditpath.read_text())
        assert audit['passed']
        remember(auditpath)
        remember(cfgpath, audit['config_sha256'])
        remember(runtimepath, audit['runtime_sha256'])
        cfg = json.loads(cfgpath.read_text())
        native = json.loads(runtimepath.read_text())
        assert len(native) == len(cfg['units']) == audit['sequences']
        assert all(r['returncode'] == 0 and r['completion_line']
                   and r['files'] == 2 * len(cfg['arms']) for r in native)
        for name, expected in cfg['source_sha256'].items():
            remember(ROOT / name, expected)
        for field in ['inputs', 'auditor_sha256']:
            for name, expected in audit[field].items():
                remember(ROOT / name, expected)
        units = {r['sequence']: r for r in cfg['units']}
        for r in audit['rows']:
            unit = units[r['sequence']]
            seq = unit['original_sequence']
            geometry = 'all'
            if dataset == 'v2x':
                meta = external_map[seq]
                geometry = ('overlapping_supports' if meta['relative_vehicle_range_max'] < 80
                            else 'disjoint_supports')
                assert meta['frames'] == r['frames']
            record = dict(sequence=seq, condition=r['condition'], arm=r['arm'], frames=r['frames'],
                          **{k: r[k] for k in METRICS}, dataset=dataset, geometry=geometry,
                          cohort='external' if dataset == 'v2x' else 'remaining_release',
                          split=unit['split'], scene=unit['scene'], recording=unit['recording'])
            if dataset == 'v2v':
                old = next(x for x in coverage['rows'] if x['scene'] == unit['scene'])
                record['cohort'] = old['cohort']
            rows.append(record)
        print('VERIFIED', dataset, stage, len(audit['rows']), 'rows', flush=True)

    projected = TRIALS / 'icra_projected_admission'
    collect(projected, 'final_v2v_evaluation_v2', 'v2v')
    collect(projected, 'final_v2x_evaluation', 'v2x')
    if '_compatible_' in primary:
        for dataset in ['v2v', 'v2x']:
            collect(TRIALS / 'icra_compatible_admission', 'final_' + dataset + '_evaluation', dataset)
    lookup = {(r['dataset'], r['scene'], r['condition'], r['arm']): r for r in rows}
    assert len(lookup) == len(rows), 'Duplicated scene/condition/method'
    datasets = {}
    for dataset, count, frames in [('v2v', 43, 9699), ('v2x', 5, 619)]:
        part = [r for r in rows if r['dataset'] == dataset]
        scenes = sorted({r['scene'] for r in part})
        arms = sorted({r['arm'] for r in part})
        assert len(scenes) == count and len(part) == count * 2 * len(arms)
        assert all((dataset, s, c, a) in lookup for s in scenes for c in CONDITIONS for a in arms)
        assert sum(lookup[dataset, s, 'reliable', ORIGINAL]['frames'] for s in scenes) == frames
        assert {primary, fixed, ORIGINAL, PROJECTED}.issubset(arms)
        datasets[dataset] = dict(scenes=count, paired_frames=frames,
                                 groups=len({r['recording'] for r in part}), arms=arms)
    aggregate = []; paired = []; group_rows = []
    scopes = [('v2v', 'all', lambda r: True),
              ('v2v', 'development', lambda r: r['cohort'] == 'development'),
              ('v2v', 'remaining_release', lambda r: r['cohort'] != 'development'),
              ('v2x', 'all', lambda r: True),
              ('v2x', 'overlapping_supports', lambda r: r['geometry'] == 'overlapping_supports'),
              ('v2x', 'disjoint_supports', lambda r: r['geometry'] == 'disjoint_supports')]
    for dataset, scope, predicate in scopes:
        part = [r for r in rows if r['dataset'] == dataset and predicate(r)]
        scenes = sorted({r['scene'] for r in part})
        arms = datasets[dataset]['arms']
        for condition in CONDITIONS:
            for arm in arms:
                group = [r for r in part if r['condition'] == condition and r['arm'] == arm]
                by_recording = defaultdict(list)
                for r in group:
                    by_recording[r['recording']].append(r)
                for recording, members in sorted(by_recording.items()):
                    group_rows.append(dict(dataset=dataset, scope=scope, condition=condition, arm=arm,
                        recording=recording, sequences=len(members), frames=sum(r['frames'] for r in members),
                        **{k: float(np.mean([r[k] for r in members])) for k in METRICS}))
                aggregate.append(dict(dataset=dataset, scope=scope, condition=condition, arm=arm,
                    sequences=len(group), groups=len(by_recording), frames=sum(r['frames'] for r in group),
                    sequence_macro={k: float(np.mean([r[k] for r in group])) for k in METRICS},
                    frame_weighted_ospa=float(np.average([r['ospa'] for r in group], weights=[r['frames'] for r in group])),
                    group_macro_ospa=float(np.mean([np.mean([r['ospa'] for r in v]) for v in by_recording.values()]))))
            for candidate in sorted({primary, ORIGINAL, PROJECTED}):
                for reference in arms:
                    if reference == candidate:
                        continue
                    deltas = []; by_recording = defaultdict(list)
                    for scene in scenes:
                        left = lookup[dataset, scene, condition, candidate]
                        right = lookup[dataset, scene, condition, reference]
                        delta = left['ospa'] - right['ospa']
                        deltas.append(delta); by_recording[left['recording']].append(delta)
                    group_deltas = np.array([np.mean(v) for _, v in sorted(by_recording.items())])
                    record = dict(dataset=dataset, scope=scope, condition=condition,
                        candidate=candidate, reference=reference, sequences=len(deltas), groups=len(group_deltas),
                        sequence_macro_difference=float(np.mean(deltas)), group_macro_difference=float(group_deltas.mean()),
                        wins=int(np.sum(np.array(deltas) < -1e-9)), ties=int(np.sum(np.abs(deltas) <= 1e-9)),
                        losses=int(np.sum(np.array(deltas) > 1e-9)))
                    if dataset == 'v2v':
                        indices = np.random.default_rng(8301).integers(0, len(group_deltas), (10000, len(group_deltas)))
                        low, high = np.quantile(group_deltas[indices].mean(1), [.025, .975])
                        record.update(low=float(low), high=float(high))
                    paired.append(record)
    rows.sort(key=lambda r: (r['dataset'], r['scene'], r['condition'], r['arm']))
    remember(OUT / 'PROTOCOL.md'); remember(Path(__file__))
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(),
        candidate_selected_before_evaluation=primary, selected_fixed=fixed,
        development_selection_utc=selection['selected_utc'], datasets=datasets,
        aggregates=aggregate, paired=paired, group_rows=group_rows, rows=rows, inputs=inputs,
        protocol='Complete frozen evaluation; external collection-date summaries are descriptive.')
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    fields = ['dataset', 'cohort', 'split', 'sequence', 'scene', 'recording', 'geometry', 'condition', 'arm', 'frames'] + METRICS
    with (OUT / 'all_evaluation_scores.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)
    print('COMPLETE FINAL EVALUATION', len(rows), 'audited comparison rows', flush=True)
    for r in aggregate:
        if r['scope'] == 'all':
            print(r['dataset'], r['condition'], r['arm'], r['sequence_macro']['ospa'], flush=True)


if __name__ == '__main__':
    main()
