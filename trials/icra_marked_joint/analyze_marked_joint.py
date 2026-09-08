"""Independently check marked joint trajectories and reuse complete saved controls."""
from pathlib import Path
import argparse
import json
import re
import sys
import time

import numpy as np
from scipy.io import loadmat
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PORT = OUT.parent / 'icra_fusion_holdout'
CONTROL = OUT.parent / 'icra_marked_control'
sys.path.insert(0, str(PORT))
from analyze_holdout import counterfactual, radio_draws, score
sys.path.insert(0, str(CONTROL))
from analyze_control import sha, read, score_run, interval, METRICS
from analyze_full import baseline as saved_baseline

PRIMARY = 'marked_joint_evidence'
ARMS = [PRIMARY, 'marked_joint_evidence_recency']
REFERENCES = ['marked_lineage', 'marked_er', 'marked_conservative',
              'marked_ceiling_calibrated', 'marked_ceiling_score']


def source_check():
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    mapped = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in source.items()}
    assert len(mapped) == len(source)
    return mapped


def audit_probability(run, data):
    arm = run['arm']
    assert arm in ARMS + ['marked_er']
    local = np.asarray(run['localIncrementRecords'], float).reshape(-1, 8)
    assert len(local) and np.isfinite(local).all()
    assert np.all(local[:, 0] >= 1) and np.all(local[:, 0] <= len(data['time']))
    assert np.isin(local[:, 1], [1, 2]).all() and np.isin(local[:, 7], [0, 1]).all()
    assert np.all((local[:, 4:6] >= 0) & (local[:, 4:6] <= 1))
    assert len(np.unique(local[:, :4], axis=0)) == len(local)
    before, after = np.clip(local[:, 4], 1e-9, 1-1e-9), np.clip(local[:, 5], 1e-9, 1-1e-9)
    expected_delta = np.log(after)-np.log1p(-after)-np.log(before)+np.log1p(-before)
    assert np.allclose(local[:, 6], expected_delta, atol=2e-12, rtol=0)
    assert np.all(np.abs(local[local[:, 7] == 0, 6]) < 1e-10)
    records = np.asarray(run['iterationRecords'], float).reshape(-1, 26)
    assert len(records) and np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, delta = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    active = b > 0
    assert np.isfinite(r[active]).all() and np.all((r[active] >= 0) & (r[active] <= 1))
    assert np.allclose(b.sum(1), 1, atol=1e-14) and np.isin(b, [0, .5, 1]).all()
    expected_factors = np.where(stamps > 0, .25+.75*np.exp(-(records[:, 0, None]-stamps)*.1/5), .25)
    assert np.all(stamps <= records[:, 0, None])
    assert np.allclose(factors[stamps > 0], expected_factors[stamps > 0], atol=2e-14, rtol=0)
    assert np.isin(factors[stamps == 0], [.25, 1]).all()
    expected_q = b*factors
    expected_q /= expected_q.sum(1, keepdims=True)
    assert np.allclose(q, expected_q, atol=2e-14, rtol=0)
    logits = np.zeros_like(b)
    rr = np.clip(r[active], 1e-9, 1-1e-9)
    logits[active] = np.log(rr)-np.log1p(-rr)
    base = (b*logits).sum(1)+records[:, 10]
    age = ((q-b)*logits).sum(1)
    joint = (active.sum(1) >= 2) & ((~active) | (r > .001)).all(1)
    recent = arm in ['marked_er', 'marked_joint_evidence_recency']
    correction = age.copy() if recent else np.zeros(len(records))
    if arm != 'marked_er':
        w = q if recent else b
        correction += joint*((active-w)*delta).sum(1)
    expected, r0, er = expit(base+correction), expit(base), expit(base+age)
    for column, value in [(6, expected), (7, r0), (8, er), (9, expected)]:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'joint scalar', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 24], correction, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23]-records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    diagnostic = dict(arm=arm, local_update_records=len(local), labels=len(records), joint_labels=int(joint.sum()),
                      positive_shift_from_er=int((correction-age > 1e-8).sum()),
                      negative_shift_from_er=int((correction-age < -1e-8).sum()))
    if arm == PRIMARY:
        delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in ['no_age', 'ER', 'candidate']}
        for t in range(len(data['time'])):
            for n in range(2):
                subset = records[(records[:, 0] == t+1) & (records[:, 1] == n+1)]
                if not delivery[n, 1-n, t]:
                    assert not len(subset)
                    continue
                for rule, column in [('no_age', 7), ('ER', 8), ('candidate', 9)]:
                    value = score(data['truth'][t], counterfactual(subset, column, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(v)) for key, v in values.items()}
                                    for rule, values in effects.items()}
    return diagnostic

def audit_file(path, source, mat, cohort, name, condition, arm):
    data = read(path)
    assert data['protocol'] == 'marked-current-innovation-v1' and data['implementation'] == 'marked-joint-v1'
    assert data['sourceSha256'] == source and data['cohort'] == cohort
    assert data['sequence'] == name and data['condition'] == condition and not data['smoke']
    assert data['inputSha256'] == sha(mat) and data['runs']['arm'] == arm
    original = loadmat(mat)
    T = int(original['T'].item())
    assert np.array_equal(data['time'], original['time'].ravel())
    assert np.array_equal(data['positions'], original['positions'])
    for t in range(T):
        assert np.array_equal(np.asarray(data['truth'][t]).reshape(4, -1), original['truth'][0, t])
        assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(), original['truthIds'][0, t].ravel())
    delivery = radio_draws(int(name), T, condition)
    assert np.array_equal(data['delivered'], delivery)
    run = data['runs']
    packets = np.asarray(run['packetBytes'])
    width = 208 if arm == 'marked_er' else 216
    assert packets.shape == (2, T) and np.isfinite(packets).all()
    assert np.all(packets >= 32) and np.all((packets - 32) % width == 0)
    assert np.all((packets - 32) / width <= run['maximumBernoulliCount'])
    assert run['attemptedMessages'] == [2] * T and run['controlBytes'] == [256] * T
    assert run['deliveredMessages'] == delivery.sum((0, 1)).tolist()
    assert run['rawPayloadBytes'] == packets.sum(0).tolist()
    assert run['deliveredRawBytes'] == (delivery.sum(0) * packets).sum(0).tolist()
    assert run['wireBytes'] == (np.ceil(packets / 16384).sum(0) * 16384 + 256).tolist()
    assert run['totalWireBytes'] == sum(run['wireBytes'])
    diagnostic = audit_probability(run, data)
    row, matched = score_run(data, run)
    return data, dict(sequence=name, condition=condition, arm=arm, frames=T, **row), matched, diagnostic


def baseline(cohort, name, condition, arm):
    old_cohort = 'development' if cohort == 'development' else 'holdout'
    if arm != 'marked_conservative':
        return saved_baseline(old_cohort, name, condition, arm)
    summary = json.loads((CONTROL / 'summary_control.json').read_text())
    path = CONTROL / f'results_{old_cohort}' / f'{name}_{condition}_{arm}.json.gz'
    assert sha(path) == summary['inputs'][str(path.relative_to(ROOT))]
    data = read(path)
    row, matched = score_run(data, data['runs'])
    expected = next(r for r in summary['cohorts'][old_cohort]['runs']
                    if r['sequence'] == name and r['condition'] == condition and r['arm'] == arm)
    for key in METRICS:
        assert np.isclose(row[key], expected[key], atol=1e-10, rtol=0)
    row.update(sequence=name, condition=condition, arm=arm, frames=len(data['time']))
    return data, row, matched, path


def audit_unit(unit, source):
    cohort, name = unit['cohort'], f"{unit['sequence']:04d}"
    folder = PORT / 'data' if cohort == 'seen_transfer' else OUT.parent / 'icra_external_fusion/data'
    mat = folder / f'v2v4real_{name}.mat'
    rows, common, provenance, diagnostics, backend_rows, backend_common = [], [], {}, [], [], []
    for condition in ['reliable', 'intermittent']:
        candidate_matches = {}
        for arm in ARMS:
            path = OUT / f'results_{cohort}' / f'{name}_{condition}_{arm}.json.gz'
            data, row, matched, diagnostic = audit_file(path, source, mat, cohort, name, condition, arm)
            rows.append(row)
            candidate_matches[arm] = matched
            diagnostics.append(dict(sequence=name, condition=condition, **diagnostic))
            provenance[str(path.relative_to(ROOT))] = sha(path)
        for reference in REFERENCES:
            ref_data, row, matches, path = baseline(cohort, name, condition, reference)
            for key in ['time', 'positions', 'truth', 'truthIds', 'delivered']:
                assert data[key] == ref_data[key], (cohort, name, condition, 'same inputs', key)
            rows.append(row)
            provenance[str(path.relative_to(ROOT))] = sha(path)
            for candidate, candidate_match in candidate_matches.items():
                both = np.isfinite(candidate_match) & np.isfinite(matches)
                common.append(dict(sequence=name, condition=condition, candidate=candidate,
                                   reference=reference, support=int(both.sum()),
                                   candidate_sse=float(candidate_match[both].sum()), reference_sse=float(matches[both].sum())))
        if cohort == 'development':
            earlier = OUT.parent / 'icra_evidence_iteration'
            summary = json.loads((earlier / 'summary_development.json').read_text())
            old_path = earlier / 'results_development' / f'{name}_{condition}.json.gz'
            entry = next(r for r in summary['inputs'] if r['sequence'] == name and r['condition'] == condition)
            assert sha(old_path) == entry['sha256']
            old_data = read(old_path)
            for key in ['time', 'positions', 'truth', 'truthIds', 'delivered']:
                assert data[key] == old_data[key], (name, condition, 'unmarked interaction inputs', key)
            provenance[str(old_path.relative_to(ROOT))] = sha(old_path)
            for candidate in ARMS:
                old_arm = candidate.removeprefix('marked_')
                old_run = next(r for r in old_data['runs'] if r['arm'] == old_arm)
                old_row, old_matches = score_run(old_data, old_run)
                expected = next(r for r in summary['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == old_arm)
                for key in METRICS[:6]:
                    assert np.isclose(old_row[key], expected[key], atol=1e-10, rtol=0)
                backend_rows.append(dict(sequence=name, condition=condition, arm=old_arm, frames=len(data['time']), **old_row))
                matches = candidate_matches[candidate]
                both = np.isfinite(matches) & np.isfinite(old_matches)
                backend_common.append(dict(sequence=name, condition=condition, candidate=candidate, reference=old_arm,
                                           support=int(both.sum()), candidate_sse=float(matches[both].sum()),
                                           reference_sse=float(old_matches[both].sum())))
    T = rows[0]['frames']
    assert len(rows) == 2 * (len(ARMS) + len(REFERENCES))
    return dict(cohort=cohort, sequence=name, frames=T, runs=rows, common=common,
                diagnostics=diagnostics, backend_runs=backend_rows, backend_common=backend_common, input_sha256=provenance,
                new_node_frames=4 * len(ARMS) * T, rescored_baseline_node_frames=4 * len(REFERENCES) * T,
                analyzer_sha256=sha(Path(__file__)), round_freeze_sha256=sha(OUT / 'ROUND_FREEZE.json'))


def summarize(reports, frozen, cohort):
    names = [f"{u['sequence']:04d}" for u in frozen['units'] if u['cohort'] == cohort]
    assert len(names) == len(reports) == (9 if cohort == 'development' else 25)
    rows = [r for report in reports for r in report['runs']]
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    samples = np.random.default_rng(8301).integers(0, len(names), (10000, len(names)))
    aggregate, paired, common = [], [], []
    for condition in frozen['conditions']:
        for arm in ARMS + REFERENCES:
            group = [lookup[name, condition, arm] for name in names]
            aggregate.append(dict(condition=condition, arm=arm,
                                  **{key: interval([r[key] for r in group], samples) for key in METRICS}))
        for candidate in ARMS:
            for reference in REFERENCES:
                a = [lookup[name, condition, candidate] for name in names]
                b = [lookup[name, condition, reference] for name in names]
                paired.append(dict(condition=condition, candidate=candidate, reference=reference,
                                   **{key: interval([x[key] - y[key] for x, y in zip(a, b)], samples) for key in METRICS},
                                   ospa_wins=sum(x['ospa'] < y['ospa'] - 1e-10 for x, y in zip(a, b))))
                group = [r for report in reports for r in report['common']
                         if r['condition'] == condition and r['candidate'] == candidate and r['reference'] == reference]
                support = sum(r['support'] for r in group)
                common.append(dict(condition=condition, candidate=candidate, reference=reference, support=support,
                                   candidate_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group) / support)),
                                   reference_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group) / support))))
    primary = [r for r in paired if r['candidate'] == PRIMARY]
    internal = [r for r in primary if r['reference'] in ['marked_lineage', 'marked_er']]
    conservative = [r for r in primary if r['reference'] == 'marked_conservative']
    continuation = all(r['ospa']['mean'] < 0 for r in internal) and all(r['ospa']['mean'] <= 0 for r in conservative)
    result = dict(protocol=frozen['protocol'], cohort=cohort, primary=PRIMARY, sequences=len(names),
                  frames=sum(r['frames'] for r in reports), all_outcomes_previously_seen=True,
                  continuation_gate_passed=continuation if cohort == 'development' else None,
                  all_four_primary_intervals_below_zero=all(r['ospa']['high'] < 0 for r in internal),
                  aggregate=aggregate, paired=paired, common_aggregate=common, runs=rows,
                  diagnostics=[d for report in reports for d in report['diagnostics']],
                  inputs={name: value for report in reports for name, value in report['input_sha256'].items()},
                  new_node_frames=sum(r['new_node_frames'] for r in reports),
                  rescored_baseline_node_frames=sum(r['rescored_baseline_node_frames'] for r in reports),
                  round_freeze_sha256=sha(OUT / 'ROUND_FREEZE.json'), analyzer_sha256=sha(Path(__file__)),
                  interval_note='Sequence-macro descriptive percentile intervals, 10000 resamples, seed 8301; all source trajectories have previously informed development. No independent-validation claim.',
                  packet_note='Marked joint arms 216 B/object. Native No-age/ER/CR 208 B/object in seen-transfer; older development No-age/ER retained an unused extra scalar, so their recorded bytes are not native 208 B costs.')
    result['unmarked_backend_runs'] = [r for report in reports for r in report['backend_runs']]
    result['rescored_unmarked_backend_node_frames'] = 4 * len(ARMS) * result['frames'] if cohort == 'development' else 0
    result['backend_interaction'] = []
    if cohort == 'development':
        old_lookup = {(r['sequence'], r['condition'], r['arm']): r for r in result['unmarked_backend_runs']}
        for condition in frozen['conditions']:
            for candidate in ARMS:
                old_arm = candidate.removeprefix('marked_')
                a = [lookup[name, condition, candidate] for name in names]
                b = [old_lookup[name, condition, old_arm] for name in names]
                group = [r for report in reports for r in report['backend_common'] if r['condition'] == condition and r['candidate'] == candidate]
                support = sum(r['support'] for r in group)
                result['backend_interaction'].append(dict(condition=condition, candidate=candidate, reference=old_arm,
                    **{key: interval([x[key]-y[key] for x,y in zip(a,b)], samples) for key in METRICS},
                    common_support=support, candidate_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group)/support)),
                    reference_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group)/support))))
    (OUT / f'summary_{cohort}.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for row in primary:
        print(cohort, row['condition'], PRIMARY, 'minus', row['reference'], row['ospa'], flush=True)
    print('ALL MARKED JOINT OUTPUTS AUDITED', cohort, result['new_node_frames'], 'new node-frames;',
          result['rescored_baseline_node_frames'], 'baseline node-frames;', 'continuation', result['continuation_gate_passed'], flush=True)


def preflight(source):
    rows, hashes = [], {}
    for condition in ['reliable', 'intermittent']:
        mat = OUT.parent / 'icra_external_fusion/data/v2v4real_0000.mat'
        for arm in ['marked_er'] + ARMS:
            path = OUT / 'results_development_check' / f'0000_{condition}_{arm}.json.gz'
            data, row, _, diagnostic = audit_file(path, source, mat, 'development_check', '0000', condition, arm)
            rows.append(dict(**row, diagnostics=diagnostic))
            hashes[str(path.relative_to(ROOT))] = sha(path)
            if arm == 'marked_er':
                original_path = PORT / 'results_development_check' / f'0000_{condition}_marked_er.json.gz'
                original = read(original_path)
                hashes[str(original_path.relative_to(ROOT))] = sha(original_path)
                for key, value in data['runs'].items():
                    if key not in ['runtimeSeconds', 'iterationRecords', 'localIncrementRecords']:
                        assert value == original['runs'][key], (condition, 'ER port parity', key)
                a = np.asarray(data['runs']['iterationRecords'], float)
                b = np.asarray(original['runs']['iterationRecords'], float)
                assert a.shape == b.shape and np.allclose(a[:, :19], b[:, :19], atol=2e-12, rtol=0, equal_nan=True)
                assert np.allclose(a[:, 21:24], b[:, 21:24], atol=1e-10, rtol=0)
    result = dict(passed=True, independently_scored_node_frames=sum(2 * r['frames'] for r in rows),
                  exact_er_node_frames=sum(2 * r['frames'] for r in rows if r['arm'] == 'marked_er'),
                  er_trajectory_metric_and_packet_fields_exact=True, er_common_scalar_diagnostics_match=True, scalar_radio_packet_and_extraction_checked=True,
                  source_files=len(source), rows=rows, input_sha256=hashes, auditor_sha256=sha(Path(__file__)))
    assert result['independently_scored_node_frames'] == 1764 and result['exact_er_node_frames'] == 588
    (OUT / 'preflight_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('MARKED JOINT PREFLIGHT AUDIT PASSED: 1764 node-frames, 588 exact ER node-frames.', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    args = parser.parse_args()
    source = source_check()
    if args.preflight:
        preflight(source)
        return
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    for name, expected in frozen['source_and_evidence_sha256'].items():
        assert sha(ROOT / name) == expected, name
    cache = OUT / 'audited_sequences'
    cache.mkdir(exist_ok=True)
    reports = []
    units = [u for u in frozen['units'] if u['cohort'] == args.cohort]
    for unit in units:
        while True:
            path = OUT / f'runtime_{args.cohort}.json'
            attempts = json.loads(path.read_text()) if path.exists() else []
            entry = next((r for r in attempts if r['sequence'] == unit['sequence']), None)
            if entry is not None:
                assert entry['returncode'] == 0 and entry['completion_line'] and entry['files'] == 4, entry
                break
            assert args.watch, ('Marked joint unit is not complete', unit)
            time.sleep(10)
        path = cache / f"{unit['cohort']}_{unit['sequence']:04d}.json"
        if path.exists():
            report = json.loads(path.read_text())
            assert report['analyzer_sha256'] == sha(Path(__file__))
            assert report['round_freeze_sha256'] == sha(OUT / 'ROUND_FREEZE.json')
            for name, expected in report['input_sha256'].items():
                assert sha(ROOT / name) == expected, name
        else:
            report = audit_unit(unit, source)
            path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        reports.append(report)
        print('MARKED JOINT AUDITED', unit['cohort'], f"{unit['sequence']:04d}", len(reports), '/', len(units), flush=True)
    summarize(reports, frozen, args.cohort)


if __name__ == '__main__':
    main()
