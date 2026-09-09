"""Compare completed S and matched GS controls with all exposed-release baselines."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json

import numpy as np

from identity_metrics import evaluate_identities, pooled

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
BASE = 'marked_gaussian_evidence'
GS = BASE + '_guarded_scalar'
SELECTED = BASE + '_assoc_split'
CONTROL = GS + '_assoc_split'
FOCUS = [BASE, GS, SELECTED, CONTROL]
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
CONDITIONS = ['reliable', 'intermittent']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = OUT / 'RESTORED_ASSESSMENT.json'
    assert not destination.exists()
    inputs, rows, paths, identities = {}, [], {}, []

    def verify(group):
        for name, expected in group.items():
            if name in inputs:
                assert inputs[name] == expected, name
            else:
                assert sha(ROOT / name) == expected, name
                inputs[name] = expected

    def report(path):
        value = json.loads(path.read_text())
        assert value['passed']
        inputs[str(path.relative_to(ROOT))] = sha(path)
        verify(value['inputs'])
        if 'auditor_sha256' in value:
            verify(value['auditor_sha256'])
        if path.name.startswith('audit_'):
            stage = value['stage']
            cfgpath = path.parent / 'stages' / (stage + '.json')
            runtimepath = path.parent / ('runtime_' + stage + '.json')
            assert sha(cfgpath) == value['config_sha256'] and sha(runtimepath) == value['runtime_sha256']
            cfg, runtime = json.loads(cfgpath.read_text()), json.loads(runtimepath.read_text())
            assert len(cfg['units']) == len(runtime)
            assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2*len(cfg['arms']) for r in runtime)
            verify(cfg['source_sha256'])
            for p in [cfgpath, runtimepath]:
                inputs[str(p.relative_to(ROOT))] = sha(p)
            return value, cfg
        return value, None

    def add(row, dataset, cohort, sequence, scene, recording, path=None):
        result = dict(dataset=dataset, cohort=cohort, sequence=sequence, scene=scene, recording=recording,
            condition=row['condition'], arm=row['arm'], frames=row['frames'], **{k: row[k] for k in METRICS})
        rows.append(result)
        if path is not None:
            name = str(path.relative_to(ROOT))
            assert name in inputs and sha(path) == inputs[name]
            paths[dataset, sequence, row['condition'], row['arm']] = path

    coverage, _ = report(OUT.parent / 'icra_full_coverage/COVERAGE_ANALYSIS.json')
    assert coverage['unique_scenes'] == 43 and coverage['paired_frames'] == 9699
    mapping = {(r['split'], r['sequence']): r for r in coverage['rows']}
    old_folders = {
        ('development', BASE): 'icra_gaussian_evidence/results_development',
        ('development', GS): 'icra_reviewer_revision/results/controls_development',
        ('seen_transfer', BASE): 'icra_gaussian_evidence/results_seen_transfer',
        ('seen_transfer', GS): 'icra_reviewer_revision/results/gs_seen_transfer',
        ('previous_train_probes', BASE): 'icra_full_coverage/results/coverage_remaining_train',
        ('previous_train_probes', GS): 'icra_full_coverage/results/coverage_remaining_train',
        ('new_validation', BASE): 'icra_reviewer_revision/results/new_validation_primary',
        ('new_validation', GS): 'icra_reviewer_revision/results/new_validation_primary'}
    for row in coverage['rows']:
        seq = row['split'] + '_' + row['sequence']
        path = None
        if row['arm'] in [BASE, GS]:
            path = OUT.parent / old_folders[row['cohort'], row['arm']] / f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
        add(row, 'v2v', row['cohort'], seq, row['scene'], row['recording'], path)
    for stage, allowed in [
            ('association_restored_preflight', [SELECTED]),
            ('association_restored_development_rest', [SELECTED]),
            ('association_selected_controls_development', [CONTROL]),
            ('association_selected_v2v_column', [SELECTED, CONTROL])]:
        audit, cfg = report(OUT / ('audit_' + stage + '.json'))
        units = {u['sequence']: u for u in cfg['units']}
        for row in audit['rows']:
            if row['arm'] not in allowed:
                continue
            unit = units[row['sequence']]
            split, original = unit.get('split', 'test'), unit.get('original_sequence', row['sequence'])
            old = mapping[split, original]
            assert row['frames'] == old['frames']
            path = OUT / 'results' / stage / f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
            add(row, 'v2v', old['cohort'], split + '_' + original, old['scene'], old['recording'], path)
    for folder, stage in [(OUT.parent / 'icra_projected_admission', 'final_v2x_evaluation'),
                          (OUT, 'association_selected_v2x')]:
        audit, cfg = report(folder / ('audit_' + stage + '.json'))
        units = {u['sequence']: u for u in cfg['units']}
        for row in audit['rows']:
            unit = units[row['sequence']]
            scene, recording = unit['scene'], unit.get('collection_date', unit.get('recording'))
            assert recording
            path = folder / 'results' / stage / f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz" if row['arm'] in FOCUS else None
            add(row, 'v2x_val', 'exposed_external', row['sequence'], scene, recording, path)
    assert len(rows) == 43*2*9 + 5*2*10
    assert len({(r['dataset'],r['sequence'],r['condition'],r['arm']) for r in rows}) == len(rows)
    assert len(paths) == (43+5)*2*4
    row_lookup = {(r['dataset'],r['sequence'],r['condition'],r['arm']): r for r in rows}
    for key, path in sorted(paths.items()):
        with gzip.open(path, 'rt') as handle:
            data = json.load(handle)
        run, row = data['runs'], row_lookup[key]
        assert row['arm'] == run['arm'] and len(data['time']) == row['frames']
        identity = evaluate_identities(data)
        identities.append(dict(dataset=row['dataset'], sequence=row['sequence'], condition=row['condition'], arm=row['arm'], **identity))
        row['wire_bytes'] = int(run['totalWireBytes'])
        row['raw_payload_bytes'] = int(sum(run['rawPayloadBytes']))
        row['delivered_raw_bytes'] = int(sum(run['deliveredRawBytes']))
        row['split_branches'] = len(run.get('associationSplits', []))
        print('ASSESSMENT IDENTITY CHECKED', *key, flush=True)
    aggregates = []
    scopes = [('v2v_all', lambda r:r['dataset']=='v2v'),
              ('v2v_development', lambda r:r['dataset']=='v2v' and r['cohort']=='development'),
              ('v2v_remaining', lambda r:r['dataset']=='v2v' and r['cohort']!='development'),
              ('v2x_val', lambda r:r['dataset']=='v2x_val')]
    for scope, predicate in scopes:
        chosen = [r for r in rows if predicate(r)]
        for condition in CONDITIONS:
            for arm in sorted({r['arm'] for r in chosen}):
                part = [r for r in chosen if r['arm']==arm and r['condition']==condition]
                recordings = defaultdict(list)
                for r in part:
                    recordings[r['recording']].append(r['ospa'])
                value = dict(scope=scope, condition=condition, arm=arm, sequences=len(part),
                    frames=sum(r['frames'] for r in part), recording_groups=len(recordings),
                    sequence_macro={k:float(np.mean([r[k] for r in part])) for k in METRICS},
                    frame_weighted_ospa=float(sum(r['frames']*r['ospa'] for r in part)/sum(r['frames'] for r in part)),
                    recording_macro_ospa=float(np.mean([np.mean(v) for v in recordings.values()])))
                if arm in FOCUS:
                    ids = {(r['dataset'],r['sequence']) for r in part}
                    value['identity'] = pooled([r for r in identities if (r['dataset'],r['sequence']) in ids and r['arm']==arm and r['condition']==condition])
                    value['communication'] = {k:sum(r[k] for r in part) for k in ['wire_bytes','raw_payload_bytes','delivered_raw_bytes','split_branches']}
                aggregates.append(value)
    paired = []
    for scope, predicate in scopes:
        chosen = [r for r in rows if predicate(r)]
        scenes = sorted({r['scene'] for r in chosen})
        recordings = sorted({r['recording'] for r in chosen})
        lookup = {(r['scene'],r['condition'],r['arm']):r for r in chosen}
        indices = np.random.default_rng(8301).integers(0,len(recordings),size=(10000,len(recordings)))
        for candidate,reference in [(SELECTED,BASE),(CONTROL,GS),(SELECTED,CONTROL)]:
            for condition in CONDITIONS:
                delta = {s:lookup[s,condition,candidate]['ospa']-lookup[s,condition,reference]['ospa'] for s in scenes}
                by_rec = [np.mean([delta[s] for s in scenes if lookup[s,condition,candidate]['recording']==rec]) for rec in recordings]
                by_rec = np.asarray(by_rec)
                low,high = np.quantile(by_rec[indices].mean(1),[.025,.975])
                paired.append(dict(scope=scope, condition=condition, candidate=candidate, reference=reference,
                    sequence_macro_difference=float(np.mean(list(delta.values()))),
                    recording_macro_difference=float(by_rec.mean()), low=float(low), high=float(high),
                    recording_groups=len(recordings), improved=sum(v < -1e-10 for v in delta.values()),
                    worsened=sum(v > 1e-10 for v in delta.values()), unchanged=sum(abs(v)<=1e-10 for v in delta.values()),
                    sequence_differences=delta))
    result = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(),
        selected=SELECTED, matched_control=CONTROL, rows=rows, aggregate=aggregates,
        identity_rows=identities, paired_recording=paired, inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),OUT/'identity_metrics.py',OUT/'diagnose_association_v2.py']},
        interval_definition='10000 paired recording-macro bootstrap samples with seed 8301; fixed settings and calibration; shared collection dates grouped for V2X.',
        exposure='All 43 V2V and five V2X validation segments were previously exposed. Additional frozen test data remain unread.')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'restored_assessment_scores.csv').open('w') as handle:
        names=list(dict.fromkeys(k for row in rows for k in row))
        writer=csv.DictWriter(handle,fieldnames=names,lineterminator='\n')
        writer.writeheader();writer.writerows(rows)
    print('RESTORED ASSESSMENT COMPLETE',len(rows),'rows',flush=True)
    for r in aggregates:
        if r['scope'] in ['v2v_all','v2x_val'] and r['arm'] in FOCUS:
            print(r['scope'],r['condition'],r['arm'],r['sequence_macro']['ospa'],r['recording_macro_ospa'],flush=True)


if __name__ == '__main__':
    main()
