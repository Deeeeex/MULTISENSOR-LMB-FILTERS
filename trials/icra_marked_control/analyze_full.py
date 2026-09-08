"""Audit both complete extra-control cohorts using saved, unchanged baselines."""
from pathlib import Path
import argparse
import csv
import json
import time

import numpy as np
from analyze_control import (OUT, ROOT, PORT, ARM, REFERENCES, METRICS, sha, read,
                             source_check, audit_file, score_run, interval)


def ready_report(path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def baseline(cohort, name, condition, arm):
    if cohort == 'development':
        parent = OUT.parent / 'icra_marked_iteration'
        summary = json.loads((parent / 'summary_development.json').read_text())
        entry = next(r for r in summary['inputs'] if r['sequence'] == name and r['condition'] == condition)
        path = parent / 'results_stable' / f'{name}_{condition}.json.gz'
        expected_row = next(r for r in summary['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == arm)
        assert sha(path) == entry['sha256']
        data = read(path)
        run = next(r for r in data['runs'] if r['arm'] == arm)
    else:
        cache = ready_report(PORT / 'audit_sequences' / f'{name}.json')
        assert cache is not None
        assert cache['auditor_sha256'] == sha(PORT / 'analyze_holdout.py')
        assert cache['method_freeze_sha256'] == sha(PORT / 'METHOD_FREEZE.json')
        entry = next(r for r in cache['inputs'] if r['condition'] == condition and r['arm'] == arm)
        expected_row = next(r for r in cache['runs'] if r['condition'] == condition and r['arm'] == arm)
        path = PORT / 'results_holdout' / f'{name}_{condition}_{arm}.json.gz'
        assert sha(path) == entry['sha256']
        data = read(path)
        run = data['runs']
    row, matched = score_run(data, run)
    for key in METRICS[:6]:
        assert np.isclose(row[key], expected_row[key], atol=1e-10, rtol=0), (cohort, name, arm, key)
    row.update(sequence=name, condition=condition, arm=arm, frames=len(data['time']))
    return data, row, matched, path


def audit_unit(unit, source):
    cohort, name = unit['cohort'], f"{unit['sequence']:04d}"
    folder = PORT / 'data' if cohort == 'holdout' else OUT.parent / 'icra_external_fusion/data'
    mat = folder / f'v2v4real_{name}.mat'
    rows, common, provenance, diagnostics = [], [], {}, []
    for condition in ['reliable', 'intermittent']:
        path = OUT / f'results_{cohort}' / f'{name}_{condition}_{ARM}.json.gz'
        data, row, control_matches, diagnostic = audit_file(path, source, mat, cohort, name, condition, ARM)
        rows.append(dict(cohort=cohort, **row))
        diagnostics.append(dict(condition=condition, **diagnostic))
        provenance[str(path.relative_to(ROOT))] = sha(path)
        for arm in REFERENCES:
            reference, row, matches, reference_path = baseline(cohort, name, condition, arm)
            for key in ['time', 'positions', 'truth', 'truthIds', 'delivered']:
                assert data[key] == reference[key], (cohort, name, condition, 'shared input', key)
            provenance[str(reference_path.relative_to(ROOT))] = sha(reference_path)
            rows.append(dict(cohort=cohort, **row))
            both = np.isfinite(control_matches) & np.isfinite(matches)
            common.append(dict(cohort=cohort, sequence=name, condition=condition, candidate=arm,
                               reference=ARM, support=int(both.sum()),
                               candidate_sse=float(matches[both].sum()), reference_sse=float(control_matches[both].sum())))
    T = rows[0]['frames']
    assert len(rows) == 8
    return dict(cohort=cohort, sequence=name, runs=rows, common=common, diagnostics=diagnostics,
                input_sha256=provenance, new_control_node_frames=4 * T, rescored_baseline_node_frames=12 * T,
                analyzer_sha256=sha(Path(__file__)), helper_sha256=sha(OUT / 'analyze_control.py'),
                control_freeze_sha256=sha(OUT / 'CONTROL_FREEZE.json'))


def summarize(reports, frozen):
    result = dict(protocol=frozen['protocol'], additional_control=ARM, primary=frozen['primary'],
                  additional_ablation_after_partial_primary_outputs=True, no_primary_reselection=True,
                  original_primary_comparisons_unchanged=True, cohorts={},
                  inputs={name: value for report in reports for name, value in report['input_sha256'].items()},
                  new_control_node_frames=sum(r['new_control_node_frames'] for r in reports),
                  rescored_baseline_node_frames=sum(r['rescored_baseline_node_frames'] for r in reports),
                  control_freeze_sha256=sha(OUT / 'CONTROL_FREEZE.json'), analyzer_sha256=sha(Path(__file__)))
    for cohort in ['development', 'holdout']:
        selected = [r for r in reports if r['cohort'] == cohort]
        rows = [r for report in selected for r in report['runs']]
        names = [f"{u['sequence']:04d}" for u in frozen['units'] if u['cohort'] == cohort]
        lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
        assert len(names) == len(selected) == (9 if cohort == 'development' else 25)
        samples = np.random.default_rng(8301).integers(0, len(names), (10000, len(names)))
        aggregate, paired, common = [], [], []
        for condition in frozen['conditions']:
            for arm in REFERENCES + [ARM]:
                group = [lookup[name, condition, arm] for name in names]
                aggregate.append(dict(condition=condition, arm=arm,
                                      **{key: interval([r[key] for r in group], samples) for key in METRICS}))
            for candidate, reference in [(frozen['primary'], ARM), (ARM, 'marked_lineage'), (ARM, 'marked_er')]:
                a = [lookup[name, condition, candidate] for name in names]
                b = [lookup[name, condition, reference] for name in names]
                paired.append(dict(condition=condition, candidate=candidate, reference=reference,
                                   **{key: interval([x[key] - y[key] for x, y in zip(a, b)], samples) for key in METRICS},
                                   ospa_wins=sum(x['ospa'] < y['ospa'] - 1e-10 for x, y in zip(a, b))))
            for arm in REFERENCES:
                group = [r for report in selected for r in report['common'] if r['condition'] == condition and r['candidate'] == arm]
                support = sum(r['support'] for r in group)
                common.append(dict(condition=condition, candidate=arm, reference=ARM, support=support,
                                   candidate_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group) / support)),
                                   reference_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group) / support))))
        result['cohorts'][cohort] = dict(sequences=len(names), frames=sum(r['runs'][0]['frames'] for r in selected),
                                         aggregate=aggregate, paired=paired, common_aggregate=common, runs=rows)
    assert result['new_control_node_frames'] == 30376 and result['rescored_baseline_node_frames'] == 91128
    result['packet_note'] = 'Reserved-cohort controls use native 208 B/Bernoulli, ECR 216 B. Older development marked controls retained an unused scalar; do not treat their recorded bytes as native same-packet comparisons.'
    result['interval_note'] = 'Sequence-macro descriptive percentile intervals; 10000 resamples, seed 8301; development was used for method work, and the reserved split was used to train the released detector. This extra control was specified after partial primary outputs existed.'
    (OUT / 'summary_control.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for cohort, content in result['cohorts'].items():
        for row in content['paired']:
            print(cohort, row['condition'], row['candidate'], 'minus', row['reference'], row['ospa'], flush=True)
    print('ALL EXTRA CONTROL COHORTS AUDITED:', result['new_control_node_frames'], 'new node-frames;',
          result['rescored_baseline_node_frames'], 'saved baseline node-frames rescored.', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    frozen = json.loads((OUT / 'CONTROL_FREEZE.json').read_text())
    for name, expected in frozen['source_and_evidence_sha256'].items():
        assert sha(ROOT / name) == expected, name
    source = source_check()
    cache = OUT / 'audited_sequences'
    cache.mkdir(exist_ok=True)
    reports = []
    for unit in frozen['units']:
        while True:
            attempts = ready_report(OUT / 'runtime.json') or []
            entry = next((r for r in attempts if r['cohort'] == unit['cohort'] and r['sequence'] == unit['sequence']), None)
            if entry is not None:
                assert entry['returncode'] == 0 and entry['completion_line'] and entry['files'] == 2, entry
                original = unit['cohort'] == 'development' or ready_report(PORT / 'audit_sequences' / f"{unit['sequence']:04d}.json") is not None
                if original:
                    break
            assert args.watch, ('Control or matching baseline not complete', unit)
            time.sleep(10)
        path = cache / f"{unit['cohort']}_{unit['sequence']:04d}.json"
        if path.exists():
            report = json.loads(path.read_text())
            assert report['analyzer_sha256'] == sha(Path(__file__)) and report['helper_sha256'] == sha(OUT / 'analyze_control.py')
            assert report['control_freeze_sha256'] == sha(OUT / 'CONTROL_FREEZE.json')
            for name, expected in report['input_sha256'].items():
                assert sha(ROOT / name) == expected, name
        else:
            report = audit_unit(unit, source)
            path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        reports.append(report)
        print('CONTROL AUDITED', unit['cohort'], f"{unit['sequence']:04d}", len(reports), '/34', flush=True)
    summarize(reports, frozen)


if __name__ == '__main__':
    main()
