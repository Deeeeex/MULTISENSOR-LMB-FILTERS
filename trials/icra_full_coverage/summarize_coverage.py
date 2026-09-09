"""Combine completed audited outputs for every distinct public scene."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'
ARMS = ['marked_lineage', 'marked_er', 'marked_asymmetric',
        'marked_gaussian_evidence_guarded_scalar', 'marked_gaussian_evidence_no_curvature',
        'marked_gaussian_evidence_fixed_025', 'marked_gaussian_evidence']
LABELS = ['No-age KLA', 'Recency', 'Scalar', 'Guarded Scalar',
          'Joint w/o curvature', 'Fixed Ratio 0.25', 'GCE']
PRIMARY = ARMS[-1]
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def main():
    destination = OUT / 'COVERAGE_ANALYSIS.json'
    assert not destination.exists()
    cfgpath = OUT / 'stages/coverage_remaining_train.json'
    cfg = json.loads(cfgpath.read_text())
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    input_manifest = json.loads((OUT / 'INPUT_MANIFEST.json').read_text())
    for name, expected in input_manifest['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    accepted = json.loads((REVIEW / 'REPLAY_ACCEPTANCE.json').read_text())
    assert accepted['passed'] and accepted['all_native_unit_exit_codes_zero']
    assert accepted['all_expected_results_present_and_independently_audited']
    catalog = json.loads((OUT / 'DATA_INVENTORY.json').read_text())
    mapping = {(r['split'], r['sequence']): r for r in catalog['rows']}
    rows = []
    provenance = {}

    def collect(path, cohort, split, allowed):
        report = json.loads(path.read_text())
        assert report.get('passed', True)
        provenance[str(path.relative_to(ROOT))] = sha(path)
        for name, expected in report['inputs'].items():
            if name in provenance:
                assert provenance[name] == expected, name
            else:
                assert sha(ROOT / name) == expected, name
                provenance[name] = expected
        if path.name.startswith('audit_'):
            stage = report['stage']
            stage_root = path.parent
            stage_cfg = stage_root / 'stages' / f'{stage}.json'
            runtime_path = stage_root / f'runtime_{stage}.json'
            assert sha(stage_cfg) == report['config_sha256']
            assert sha(runtime_path) == report['runtime_sha256']
            runtime = json.loads(runtime_path.read_text())
            assert all(r['returncode'] == 0 and r['completion_line'] for r in runtime)
            provenance[str(runtime_path.relative_to(ROOT))] = sha(runtime_path)
        else:
            runtime_path = path.parent / f'runtime_{cohort}.json'
            native = json.loads(runtime_path.read_text())
            assert len(native) == report['sequences']
            assert all(r['returncode'] == 0 and r['completion_line'] for r in native)
            provenance[str(runtime_path.relative_to(ROOT))] = sha(runtime_path)
        for row in report.get('rows', report.get('runs', [])):
            if row['arm'] not in allowed:
                continue
            original = mapping[split, row['sequence']]
            assert row['frames'] == original['frames']
            rows.append(dict(cohort=cohort, split=split, sequence=row['sequence'], scene=original['scene'],
                             recording=original['recording'], condition=row['condition'], arm=row['arm'],
                             frames=row['frames'], **{key: row[key] for key in METRICS}))
        print('VERIFIED REUSED EVIDENCE', path.name, flush=True)

    original_arms = [a for a in ARMS if a not in [ARMS[3], ARMS[5]]]
    for cohort, split in [('development', 'test'), ('seen_transfer', 'train')]:
        collect(OUT.parent / f'icra_gaussian_evidence/summary_{cohort}.json', cohort, split, original_arms)
    collect(REVIEW / 'audit_controls_development.json', 'development', 'test', [ARMS[3], ARMS[5]])
    collect(REVIEW / 'audit_gs_seen_transfer.json', 'seen_transfer', 'train', [ARMS[3]])
    collect(REVIEW / 'audit_fixed025_seen_transfer.json', 'seen_transfer', 'train', [ARMS[5]])
    collect(REVIEW / 'audit_new_validation_primary.json', 'new_validation', 'val', ARMS)
    collect(REVIEW / 'audit_new_validation_recency.json', 'new_validation', 'val', [ARMS[1]])
    collect(OUT / 'audit_coverage_remaining_train.json', 'previous_train_probes', 'train', ARMS)
    preflight = json.loads((OUT / 'audit_coverage_preflight.json').read_text())
    assert preflight['passed'] and len(preflight['parity']) == 4
    rows.sort(key=lambda r: (r['split'], r['sequence'], r['condition'], ARMS.index(r['arm'])))
    lookup = {(r['scene'], r['condition'], r['arm']): r for r in rows}
    assert len(lookup) == len(rows) == 43 * 2 * 7
    scenes = sorted({r['scene'] for r in rows})
    assert len(scenes) == 43
    assert sum(lookup[s, 'reliable', PRIMARY]['frames'] for s in scenes) == 9699
    recordings = sorted({r['recording'] for r in rows})
    assert all((s, c, a) in lookup for s in scenes for c in ['reliable', 'intermittent'] for a in ARMS)

    aggregates = []
    scopes = ['all_unique', 'development', 'seen_transfer', 'previous_train_probes', 'new_validation']
    for scope in scopes:
        selected = rows if scope == 'all_unique' else [r for r in rows if r['cohort'] == scope]
        for condition in ['reliable', 'intermittent']:
            for arm in ARMS:
                group = [r for r in selected if r['condition'] == condition and r['arm'] == arm]
                frames = sum(r['frames'] for r in group)
                by_recording = defaultdict(list)
                for row in group:
                    by_recording[row['recording']].append(row['ospa'])
                aggregates.append(dict(scope=scope, condition=condition, arm=arm, sequences=len(group),
                    recordings=len(by_recording), frames=frames,
                    sequence_macro={key: float(np.mean([r[key] for r in group])) for key in METRICS},
                    frame_weighted_ospa=float(sum(r['ospa'] * r['frames'] for r in group) / frames),
                    recording_macro_ospa=float(np.mean([np.mean(v) for v in by_recording.values()]))))

    # Pair methods within recording, then resample whole recording groups.
    indices = np.random.default_rng(8301).integers(0, len(recordings), size=(10000, len(recordings)))
    paired = []
    for condition in ['reliable', 'intermittent']:
        for reference in ARMS[:-1]:
            differences = []
            for recording in recordings:
                members = [s for s in scenes if lookup[s, condition, PRIMARY]['recording'] == recording]
                differences.append(np.mean([lookup[s, condition, PRIMARY]['ospa'] - lookup[s, condition, reference]['ospa'] for s in members]))
            differences = np.asarray(differences)
            lo, hi = np.quantile(differences[indices].mean(1), [.025, .975])
            paired.append(dict(condition=condition, candidate=PRIMARY, reference=reference,
                               recording_macro_difference=float(differences.mean()), low=float(lo), high=float(hi),
                               recording_groups=len(recordings)))

    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(),
                  unique_scenes=43, paired_frames=9699, recording_groups=len(recordings), methods=ARMS,
                  newly_run_scenes=6, newly_run_paired_frames=1357, newly_run_outputs=84,
                  preflight_outputs=4, total_compared_sequence_method_condition_rows=len(rows),
                  cohorts=scopes, aggregate=aggregates, paired_recording=paired, rows=rows,
                  inputs=provenance, config_sha256=sha(cfgpath),
                  counting='Deduplicate train/test 0000 by scene; use the existing test-backed development result.',
                  interval_definition='10000 paired recording-macro bootstrap samples, seed 8301; existing calibration fits held fixed.',
                  model='Frozen primary current-ego CV, pD=0.9; original development uses leave-one-sequence-out calibration and other scenes use the full-nine fit.')
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (OUT / 'all_sequence_scores.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    text = ['# V2V4Real 全发布覆盖结果', '',
            '已完成 43 个去重片段、9,699 对双车帧的七方法双通信比较。新增六段、1,357 对帧、84 份轨迹；四份预检轨迹通过原输出逐字段比较。', '',
            f'按 {len(recordings)} 个原始记录另列分组均值和配对重采样区间。训练/测试的 0000 只计一次。', '',
            '| 方法 | 可靠 OSPA（片段等权） | 间歇 OSPA（片段等权） |', '| --- | --- | --- |']
    for arm, label in zip(ARMS, LABELS):
        means = [next(r for r in aggregates if r['scope'] == 'all_unique' and r['condition'] == c and r['arm'] == arm)['sequence_macro']['ospa'] for c in ['reliable', 'intermittent']]
        text.append(f'| {label} | {means[0]:.6f} | {means[1]:.6f} |')
    text += ['', '完整分组、逐帧加权结果和记录级配对结果见 `COVERAGE_ANALYSIS.json`；逐片段数据见 `all_sequence_scores.csv`。', '',
             '数据口径：32 个 train 条目、9 个 test 条目、3 个 validation 条目，扣除一个重复条目。官方论文的 67 个场景与这些发布目录的映射未提供。', '',
             '配置：沿用主实验的 current-ego CV、pD=0.9；九段开发数据沿用逐序列留出校准，其余片段沿用九段完整拟合。', '']
    (OUT / 'RESULTS_CN.md').write_text('\n'.join(text))
    print('COVERAGE SUMMARY VERIFIED:', len(rows), 'comparison rows,', len(recordings), 'recording groups.', flush=True)


if __name__ == '__main__':
    main()
