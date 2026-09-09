"""Independently check portable reviewer rows, statistics, and rendered values."""
from pathlib import Path
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / 'source_data'
SNAPSHOTS = DATA / 'reviewer_revision'
PRIMARY = 'marked_gaussian_evidence'
GS, FIXED = PRIMARY + '_guarded_scalar', PRIMARY + '_fixed_025'
CONDITIONS = ['reliable', 'intermittent']
ARMS = ['marked_lineage', 'marked_asymmetric', GS, PRIMARY]
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError',
           'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def same(x, y):
    assert math.isclose(x, y, abs_tol=1e-10, rel_tol=1e-12), (x, y)


def read(name):
    return json.loads((SNAPSHOTS / name).read_text())


def verify_statistic(values, stat, draws):
    values = np.asarray(values, float)
    assert np.isfinite(values).all()
    same(float(values.mean()), stat['mean'])
    if 'values' in stat:
        assert np.allclose(values, stat['values'], rtol=0, atol=1e-12)
    if draws is None:
        assert 'low' not in stat and 'high' not in stat
    else:
        bounds = np.percentile(values[draws].mean(1), [2.5, 97.5])
        same(bounds[0], stat['low']); same(bounds[1], stat['high'])


def verify_group(group, count):
    names = sorted({row['sequence'] for row in group['rows']})
    assert len(names) == count
    lookup = {(row['sequence'], row['condition'], row['arm']): row for row in group['rows']}
    assert len(lookup) == len(group['rows'])
    arms = sorted({row['arm'] for row in group['rows']})
    assert len(lookup) == count * len(arms) * 2
    assert group['frames'] == sum(lookup[n, 'reliable', PRIMARY]['frames'] for n in names)
    draws = np.random.default_rng(8301).integers(0, count, (10000, count)) if count >= 9 else None
    assert len(group['aggregate']) == 2 * len(arms)
    for row in group['aggregate']:
        for metric in METRICS:
            verify_statistic([lookup[n, row['condition'], row['arm']][metric] for n in names], row[metric], draws)
    for row in group['paired']:
        values = [lookup[n, row['condition'], row['candidate']]['ospa'] -
                  lookup[n, row['condition'], row['reference']]['ospa'] for n in names]
        verify_statistic(values, row['ospa'], draws)
    for row in group.get('interaction', []):
        condition = row['condition']
        values = [lookup[n, condition, PRIMARY]['ospa'] - lookup[n, condition, GS]['ospa'] -
                  lookup[n, condition, PRIMARY + '_no_curvature']['ospa'] + lookup[n, condition, 'marked_asymmetric']['ospa'] for n in names]
        verify_statistic(values, row['ospa'], draws)
    return lookup, names, draws


def check_reviewer_evidence():
    manifest = json.loads((DATA / 'reviewer_source_manifest.json').read_text())
    evidence = json.loads((DATA / 'reviewer_evidence.json').read_text())
    assert evidence['source_manifest_sha256'] == sha(DATA / 'reviewer_source_manifest.json')
    assert evidence['generator_sha256'] == sha(HERE / 'prepare_reviewer_evidence.py')
    for name, record in manifest.items():
        assert sha(SNAPSHOTS / name) == record['sha256'], name
        original = HERE.parents[1] / record['source']
        if original.exists():
            assert sha(original) == record['sha256'], str(original)
    acceptance = read('REPLAY_ACCEPTANCE.json')
    result_files = read('RESULT_FILES_MANIFEST.json')
    assert acceptance['passed'] and acceptance['all_native_unit_exit_codes_zero']
    assert acceptance['result_manifest_sha256'] == sha(SNAPSHOTS / 'RESULT_FILES_MANIFEST.json')
    assert acceptance['retained_native_result_files'] == len(result_files) == 750
    assert acceptance['retained_native_result_bytes'] == sum(row['bytes'] for row in result_files.values())
    for key, name in [('controls', 'CONTROLS_ANALYSIS.json'), ('new_data', 'NEW_DATA_ANALYSIS.json'),
                      ('sensitivity', 'MODEL_SENSITIVITY_ANALYSIS.json'), ('correlation', 'correlation_control.json')]:
        assert evidence[key] == read(name)
    for name, field in [('CONTROLS_ANALYSIS.json', 'sources'), ('NEW_DATA_ANALYSIS.json', 'source_reports'),
                        ('MODEL_SENSITIVITY_ANALYSIS.json', 'source_reports')]:
        value = read(name); assert value['passed']
        for source, digest in value[field].items():
            if source in manifest:
                assert manifest[source]['sha256'] == digest
    completed_files = node_frames = 0
    for name in manifest:
        if name.startswith('audit_'):
            report = read(name); assert report['passed']
            completed_files += len(report['rows'])
            node_frames += report['audited_node_frames']
            native = HERE.parents[1] / 'trials/icra_reviewer_revision'
            if native.is_dir():
                assert sha(native / 'stages' / (report['stage'] + '.json')) == report['config_sha256']
                runtime = native / ('runtime_' + report['stage'] + '.json')
                assert sha(runtime) == report['runtime_sha256']
                records = json.loads(runtime.read_text())
                assert len(records) == report['sequences']
                assert all(row['returncode'] == 0 and row['completion_line'] for row in records)
    controls = evidence['controls']['cohorts']
    for cohort, count in [('development', 9), ('seen_transfer', 25)]:
        verify_group(controls[cohort], count)
        old = [row for row in read('LEGACY_RESCORING.json')['rows'] if row['cohort'] == cohort]
        old = [{k: v for k, v in row.items() if k != 'cohort'} for row in old]
        files = ['audit_controls_development.json'] if cohort == 'development' else ['audit_gs_seen_transfer.json', 'audit_fixed025_seen_transfer.json']
        combined = old + [row for name in files for row in read(name)['rows']]
        keys = lambda row: (row['sequence'], row['condition'], row['arm'])
        actual = [{k: v for k, v in row.items() if k != 'cohort'} for row in controls[cohort]['rows']]
        assert sorted(actual, key=keys) == sorted(combined, key=keys)
    selected = read('FIXED_SELECTION.json')
    assert selected['selected']['arm'] == FIXED
    new = evidence['new_data']
    assert len(new['sequences']) == 3 and sum(row['frames'] for row in new['sequences']) == 748
    assert len({row['original_recording'] for row in new['sequences']}) == 2
    absent = [row for row in new['sequences'] if not row['original_recording_present_in_old_data']]
    assert len(absent) == 1 and absent[0]['sequence'] == '0000' and absent[0]['frames'] == 409
    assert new['rows'] == read('audit_new_validation_primary.json')['rows'] + read('audit_new_validation_recency.json')['rows']
    new_lookup = {(row['sequence'], row['condition'], row['arm']): row for row in new['rows']}
    for key, group in new['groups'].items():
        names = group['sequences']
        assert group['frames'] == sum(new_lookup[n, 'reliable', PRIMARY]['frames'] for n in names)
        for row in group['aggregate']:
            for metric in METRICS:
                same(row[metric], np.mean([new_lookup[n, row['condition'], row['arm']][metric] for n in names]))
        for row in group['paired']:
            differences = [new_lookup[n, row['condition'], PRIMARY]['ospa'] - new_lookup[n, row['condition'], row['reference']]['ospa'] for n in names]
            assert differences == row['differences']; same(np.mean(differences), row['mean'])
    for condition in CONDITIONS:
        assert new_lookup['0000', condition, PRIMARY]['ospa'] > new_lookup['0000', condition, 'marked_lineage']['ospa']
    sensitivity = evidence['sensitivity']
    for cohort, count in [('development', 9), ('seen_transfer', 25), ('new_validation', 3)]:
        group = sensitivity['motion'][cohort]
        assert group['rows'] == read('audit_motion_' + cohort + '.json')['rows']
        after, names, draws = verify_group(group, count)
        rows = new['rows'] if cohort == 'new_validation' else controls[cohort]['rows']
        before = {(row['sequence'], row['condition'], row['arm']): row for row in rows}
        for row in [r for r in sensitivity['motion_effect'] if r['cohort'] == cohort]:
            differences = [after[n, row['condition'], row['arm']]['ospa'] - before[n, row['condition'], row['arm']]['ospa'] for n in names]
            verify_statistic(differences, row['ospa'], draws)
    for probability, stage in [('0.7', 'pd070_development'), ('0.8', 'pd080_development'),
                               ('0.9', None), ('0.95', 'pd095_development')]:
        group = sensitivity['detection_probability'][probability]
        expected = read('audit_' + stage + '.json')['rows'] if stage else [row for row in controls['development']['rows'] if row['arm'] in ARMS]
        assert group['rows'] == expected
        verify_group(group, 9)
    correlation = evidence['correlation']
    control_audit = read('CORRELATION_AUDIT_V2.json')
    assert control_audit['passed'] and control_audit['source_report_sha256'] == sha(SNAPSHOTS / 'correlation_control.json')
    assert correlation['zero_correlation_gce_oracle_exact']
    assert len(correlation['rows']) == 15
    assert all(row['admitted_source_fraction'] == 1 for row in correlation['rows'] if row['arm'] == 'Unit-admission GCE')
    check_facts(evidence)
    check_new_table(evidence)
    figure = json.loads((DATA / 'gaussian_robustness.json').read_text())
    assert figure['reviewer_evidence_sha256'] == sha(DATA / 'reviewer_evidence.json')
    assert figure['point_count'] == 47 and figure['pd_point_count'] == 32 and figure['correlation_point_count'] == 15
    assert figure['correlation_rows'] == correlation['rows']
    for row in figure['pd_rows']:
        group = sensitivity['detection_probability'][str(row['pd'])]
        expected = next(r for r in group['aggregate'] if r['condition'] == row['condition'] and r['arm'] == row['arm'])
        same(expected['ospa']['mean'], row['ospa'])
        assert row['sequence_values'] == expected['ospa']['values']
        assert figure['pd_y_limits'][0] < row['ospa'] < figure['pd_y_limits'][1]
    svg = ET.parse(HERE / 'figures/gaussian_robustness.svg')
    groups = {node.get('id'): node for node in svg.iter() if node.get('id')}
    plotted = 0
    for prefix, count in [('pd_', 4), ('correlation_', 5)]:
        selected = [node for key, node in groups.items() if key.startswith(prefix)]
        assert len(selected) == (8 if prefix == 'pd_' else 3)
        for node in selected:
            markers = [child for child in node.iter() if child.tag.endswith('}use')]
            assert len(markers) == count
            plotted += len(markers)
    assert plotted == 47
    bounds = json.loads((HERE / 'figures/gaussian_robustness_text_bounds.json').read_text())
    assert bounds['passed'] and bounds['text_collisions_checked']
    assert min(row['font_size_pt'] for row in bounds['text_bounds']) >= 7
    return dict(portable_source_snapshots=len(manifest), checked_trajectory_files=completed_files,
                native_node_frames_independently_rescored=node_frames, main_comparison_methods=13,
                new_segments=3, new_paired_frames=748, original_recordings=2,
                previously_absent_recordings=1, new_recording_reversal_retained=True,
                recursive_factor_cells=4, all_motion_cohorts_complete=True,
                modeled_detection_probabilities=[.7, .8, .9, .95], correlation_samples=50000,
                plotted_sensitivity_points=47, intervals_from_whole_sequences=True,
                statistical_generalization_or_covariance_guarantee=False)


def check_new_table(evidence):
    groups = evidence['new_data']['groups']
    methods = ['marked_lineage', 'marked_er', 'marked_asymmetric', GS, PRIMARY + '_no_curvature', FIXED, PRIMARY]
    labels = ['No-age KLA', 'Recency', 'Scalar', 'Guarded Scalar', 'w/o curvature', 'Fixed Ratio (0.25)', r'\textbf{GCE}']
    values = []
    for arm in methods:
        values.append([next(row['ospa'] for row in groups[key]['aggregate'] if row['condition'] == condition and row['arm'] == arm)
                       for key in ['all_new_segments', 'new_recording'] for condition in CONDITIONS])
    minima = np.min(values, axis=0)
    table = (HERE / 'generated/new_data_table.tex').read_text()
    positions = []
    for label, row in zip(labels, values):
        cells = [label]
        for index, value in enumerate(row):
            cell = f'{value:.3f}'
            cells.append(r'\textbf{' + cell + '}' if abs(value - minima[index]) < 1e-12 else cell)
        line = ' & '.join(cells) + r' \\'
        assert line in table, line
        positions.append(table.index(line))
    assert positions == sorted(positions)


def check_facts(evidence):
    record = json.loads((HERE / 'generated/reviewer_facts.json').read_text())
    values, digits = record['values'], record['decimal_places']
    expected = {}
    for cohort, group in evidence['controls']['cohorts'].items():
        prefix = 'Rev' + ('Seen' if cohort == 'seen_transfer' else 'Dev')
        for row in group['paired']:
            if row['candidate'] != PRIMARY or row['reference'] not in [GS, FIXED]:
                continue
            key = prefix + 'Delta' + ('GS' if row['reference'] == GS else 'Fixed') + row['condition'].title()
            for statistic in ['mean', 'low', 'high']:
                expected[key + statistic.title()] = row['ospa'][statistic]
        for row in group['interaction']:
            for statistic in ['mean', 'low', 'high']:
                expected[prefix + 'Interaction' + row['condition'].title() + statistic.title()] = row['ospa'][statistic]
    short = dict(zip(ARMS, ['NoAge', 'Scalar', 'GS', 'GCE']))
    for key, group in evidence['new_data']['groups'].items():
        prefix = 'Rev' + {'all_new_segments': 'NewAll', 'new_recording': 'NewRecording', 'related_recording_segments': 'Related'}[key]
        for row in group['aggregate']:
            if row['arm'] in ARMS:
                expected[prefix + short[row['arm']] + row['condition'].title()] = row['ospa']
    for cohort, group in evidence['sensitivity']['motion'].items():
        prefix = 'RevMotion' + {'development': 'Dev', 'seen_transfer': 'Seen', 'new_validation': 'New'}[cohort]
        for row in group['aggregate']:
            expected[prefix + short[row['arm']] + row['condition'].title()] = row['ospa']['mean']
        for row in group['paired']:
            for statistic in ['mean', 'low', 'high']:
                if statistic in row['ospa']:
                    expected[prefix + 'Delta' + short[row['reference']] + row['condition'].title() + statistic.title()] = row['ospa'][statistic]
        if cohort == 'new_validation':
            for row in group['rows']:
                if row['sequence'] == '0000':
                    expected['RevMotionNewRecording' + short[row['arm']] + row['condition'].title()] = row['ospa']
    for row in evidence['correlation']['rows']:
        if row['rho'] == .9 and row['arm'] in ['Unit-admission GCE', 'Known-correlation oracle']:
            prefix = 'RevCorrelation' + ('GCE' if row['arm'].startswith('Unit') else 'Oracle')
            expected[prefix + 'Coverage'] = row['coverage_95'] * 100
            expected[prefix + 'NEES'] = row['mean_nees_per_dimension']
    assert values == expected and set(values) == set(digits)
    rendered = dict(re.findall(r'\\newcommand\{\\(\w+)\}\{([^}]+)\}', (HERE / 'generated/reviewer_numbers.tex').read_text()))
    assert rendered == {name: f'{value:.{digits[name]}f}' for name, value in expected.items()}


if __name__ == '__main__':
    print(json.dumps(check_reviewer_evidence(), indent=2))
