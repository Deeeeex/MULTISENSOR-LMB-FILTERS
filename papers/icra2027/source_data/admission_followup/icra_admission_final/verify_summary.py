"""Recompute portable tables and paired statistics independently of the native traces."""
from collections import defaultdict
from pathlib import Path
import hashlib
import json

import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def equal(left, right):
    assert np.isclose(left, right, rtol=1e-12, atol=1e-11), (left, right)


def main():
    source = HERE / 'EVALUATION_ANALYSIS.json'
    data = json.loads(source.read_text()); assert data['passed']
    rows = data['rows']
    assert len(rows) == 854
    index = {(r['dataset'], r['scene'], r['condition'], r['arm']): r for r in rows}
    assert len(index) == len(rows)

    def subset(dataset, scope, condition, arm):
        values = [r for r in rows if r['dataset'] == dataset and r['condition'] == condition and r['arm'] == arm]
        if scope in ['overlapping_supports', 'disjoint_supports']:
            values = [r for r in values if r['geometry'] == scope]
        elif scope == 'development':
            values = [r for r in values if r['cohort'] == 'development']
        elif scope == 'remaining_release':
            values = [r for r in values if r['cohort'] != 'development']
        else:
            assert scope == 'all'
        return values

    for row in data['aggregates']:
        selected = subset(row['dataset'], row['scope'], row['condition'], row['arm'])
        assert len(selected) == row['sequences']
        assert sum(r['frames'] for r in selected) == row['frames']
        groups = defaultdict(list)
        for r in selected:
            groups[r['recording']].append(r['ospa'])
        assert len(groups) == row['groups']
        for metric, mean in row['sequence_macro'].items():
            equal(np.mean([r[metric] for r in selected]), mean)
        equal(np.average([r['ospa'] for r in selected], weights=[r['frames'] for r in selected]), row['frame_weighted_ospa'])
        equal(np.mean([np.mean(v) for v in groups.values()]), row['group_macro_ospa'])
    for row in data['paired']:
        selected = subset(row['dataset'], row['scope'], row['condition'], row['candidate'])
        differences = []
        groups = defaultdict(list)
        for left in selected:
            right = index[row['dataset'], left['scene'], row['condition'], row['reference']]
            value = left['ospa'] - right['ospa']
            differences.append(value); groups[left['recording']].append(value)
        differences = np.asarray(differences)
        group_means = np.asarray([np.mean(v) for _, v in sorted(groups.items())])
        assert len(differences) == row['sequences'] and len(group_means) == row['groups']
        equal(differences.mean(), row['sequence_macro_difference']); equal(group_means.mean(), row['group_macro_difference'])
        assert row['wins'] == np.sum(differences < -1e-9)
        assert row['ties'] == np.sum(np.abs(differences) <= 1e-9)
        assert row['losses'] == np.sum(differences > 1e-9)
        if row['dataset'] == 'v2v':
            draws = np.random.default_rng(8301).integers(0, len(group_means), (10000, len(group_means)))
            bounds = np.percentile(np.mean(group_means[draws], axis=1), [2.5, 97.5])
            equal(bounds[0], row['low']); equal(bounds[1], row['high'])
        else:
            assert 'low' not in row and 'high' not in row
    external_path = HERE / 'EXTERNAL_ANALYSIS.json'
    external = json.loads(external_path.read_text()); assert external['passed']
    assert external['candidate_selection_utc'] < external['first_external_scores_viewed_utc']
    for row in external['aggregate']:
        actual = next(r for r in data['aggregates'] if r['dataset'] == 'v2x' and r['scope'] == row['scope']
                      and r['condition'] == row['condition'] and r['arm'] == row['arm'])
        assert actual['sequence_macro'] == row['sequence_macro']
        equal(actual['group_macro_ospa'], row['date_macro_ospa'])
    selection_path = HERE.parent / 'icra_compatible_admission/FINAL_SELECTION.json'
    selection = json.loads(selection_path.read_text()); assert selection['passed']
    assert len(selection['development_rows']) == 396
    for summary in selection['aggregate']:
        means = []
        for condition in ['reliable', 'intermittent']:
            values = [r for r in selection['development_rows'] if r['arm'] == summary['arm'] and r['condition'] == condition]
            assert len(values) == 9
            mean = float(np.mean([r['ospa'] for r in values])); means.append(mean)
            equal(mean, summary['sequence_macro_ospa_by_condition'][condition])
        equal(np.mean(means), summary['selection_mean_ospa'])
    assert selection['selected'] == selection['candidate_ranking'][0]
    assert selection['selected']['arm'] == data['candidate_selected_before_evaluation']
    assert selection['selected_fixed']['arm'] == data['selected_fixed']
    assert not selection['external_tracking_scores_inspected']
    assert selection['remaining_release_candidate_scores_inspected']
    fixed = [r for r in selection['aggregate'] if '_fixedx_' in r['arm'] or '_fixed_' in r['arm']]
    assert len(fixed) == 7
    assert min(fixed, key=lambda r: r['selection_mean_ospa'])['arm'] == data['selected_fixed']
    result = dict(passed=True, evaluation_rows=len(rows), development_rows=396,
                  recomputed_aggregates=len(data['aggregates']), recomputed_paired_comparisons=len(data['paired']),
                  sources={str(p.relative_to(HERE.parent)): sha(p) for p in [source, external_path, selection_path]},
                  verifier_sha256=sha(Path(__file__)), external_dates_descriptive_only=True,
                  single_log_exposure_recorded=True)
    destination = HERE / 'SUMMARY_AUDIT.json'
    if destination.exists():
        assert json.loads(destination.read_text()) == result
    else:
        destination.write_text(json.dumps(result, indent=2) + '\n')
    print('PORTABLE SUMMARY VERIFIED:', len(rows), 'evaluation rows;', len(data['paired']), 'paired comparisons.', flush=True)


if __name__ == '__main__':
    main()
