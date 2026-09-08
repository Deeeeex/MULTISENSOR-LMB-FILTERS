"""Independent audit of the complete frozen fusion-selection cohort.

--watch audits completed sequence files while the frozen tracker continues.
It emits no cohort comparison until all 25 registered sequences are complete.
"""
from pathlib import Path
import argparse
import csv
import gzip
import hashlib
import json
import re
import sys
import time

import numpy as np
from scipy.io import loadmat
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_external_fusion'))
from analyze_case_studies import score
from analyze_v2v4real import domain, interval
sys.path.insert(0, str(OUT.parent / 'icra_method_iteration'))
from analyze_development import counterfactual

METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError',
           'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def radio_draws(seq, T, condition):
    delivered = np.repeat(np.array([[0, 1], [1, 0]], dtype=bool)[:, :, None], T, axis=2)
    if condition == 'intermittent':
        draws = np.random.RandomState(8301 + seq).random_sample(4 * T).reshape(2, 2, T, order='F')
        delivered &= draws >= .1
        delivered[:, :, int(.4 * T):int(.6 * T)] = False
    return delivered


def audit_packets(run, delivery, T, local_counts):
    arm = run['arm']
    packets = np.asarray(run['packetBytes'])
    assert packets.shape == (2, T) and np.isfinite(packets).all()
    assert np.all(packets >= 0) and np.all(packets == np.floor(packets))
    if arm.endswith('local'):
        for key in ['packetBytes', 'attemptedMessages', 'deliveredMessages', 'controlBytes',
                    'rawPayloadBytes', 'deliveredRawBytes', 'wireBytes']:
            assert not np.asarray(run[key]).any(), (arm, key)
    else:
        assert run['attemptedMessages'] == [2] * T
        assert run['controlBytes'] == [256] * T
        assert run['deliveredMessages'] == delivery.sum((0, 1)).tolist()
        assert run['rawPayloadBytes'] == packets.sum(0).tolist()
        assert run['deliveredRawBytes'] == (delivery.sum(0) * packets).sum(0).tolist()
        assert run['wireBytes'] == (np.ceil(packets / 16384).sum(0) * 16384 + 256).tolist()
        if 'tc_ospa2' in arm:
            window = 5 if arm.endswith('_w5') else 10
            counts = local_counts['marked_local' if arm.startswith('marked_') else 'local']
            expected = np.zeros_like(packets)
            for t in range(T):
                first = max(0, t - window + 1)
                expected[:, t] = 8 * (5 + 2 * (t - first + 1) + 6 * counts[:, first:t + 1].sum(1))
            assert np.array_equal(packets, expected), (arm, 'actual corresponding local history bytes')
        else:
            width = 216 if 'ceiling_' in arm else 208
            assert np.all(packets >= 32) and np.all((packets - 32) % width == 0)
            assert np.all((packets - 32) / width <= run['maximumBernoulliCount'])
    assert run['totalWireBytes'] == sum(run['wireBytes'])


def audit_existence(run, data, primary):
    arm = run['arm']
    records = np.asarray(run['iterationRecords'], dtype=float).reshape(-1, 26)
    if arm.endswith('local') or 'tc_ospa2' in arm or 'mil_support' in arm:
        assert not len(records)
        return None
    if not len(records):
        return dict(arm=arm, labels=0)
    assert np.isfinite(records[:, :17]).all() and np.isfinite(records[:, 19:]).all()
    b, q = records[:, 13:15], records[:, 11:13]
    r, evidence = records[:, 17:19], records[:, 19:21]
    factors, stamps = records[:, 15:17], records[:, 21:23]
    active = b > 0
    assert np.isfinite(r[active]).all()
    assert np.all((r[active] >= 0) & (r[active] <= 1))
    assert np.allclose(b.sum(1), 1, atol=1e-14)
    assert np.isin(b, [0, .5, 1]).all()
    expected_factors = np.where(stamps > 0, .25 + .75 * np.exp(-(records[:, 0, None] - stamps) * .1 / 5), .25)
    assert np.all(stamps <= records[:, 0, None])
    # A missing label has no object stamp and retains the neutral factor 1;
    # a represented object without direct history uses .25. The compact
    # record has no separate represented-label mask, so distinguish only
    # the two permitted zero-stamp cases, not an invented age for absence.
    assert np.allclose(factors[stamps > 0], expected_factors[stamps > 0], rtol=0, atol=2e-14)
    assert np.isin(factors[stamps == 0], [.25, 1]).all()
    expected_q = b * factors
    expected_q /= expected_q.sum(1, keepdims=True)
    assert np.allclose(q, expected_q, rtol=0, atol=2e-14)
    logits = np.zeros_like(b)
    rr = np.clip(r[active], 1e-9, 1 - 1e-9)
    logits[active] = np.log(rr) - np.log1p(-rr)
    base = (b * logits).sum(1) + records[:, 10]
    age = ((q - b) * logits).sum(1)
    assert np.all((evidence >= 0) & (evidence <= 1))
    assert np.all(evidence[stamps != records[:, 0, None]] == 0)
    eligible = active & (q > b + 1e-12) & (r >= .5)
    cap = np.where(eligible, evidence, 0).max(1)
    r0, rER = expit(base), expit(base + age)
    if arm.endswith('lineage'):
        expected = r0
    elif arm in ['qualified_exist', 'marked_er']:
        expected = rER
    else:
        expected = np.minimum(rER, np.maximum(r0, cap))
    for column, value in [(6, expected), (7, r0), (8, rER), (9, expected)]:
        assert np.allclose(records[:, column], value, atol=2e-12, rtol=0), (arm, 'existence equation', column)
    assert np.allclose(records[:, 23], age, atol=1e-10, rtol=0)
    assert np.allclose(records[:, 23] - records[:, 24], records[:, 25], atol=1e-10, rtol=0)
    if not arm.endswith('lineage'):
        assert np.all(expected >= np.minimum(r0, rER) - 2e-12)
        assert np.all(expected <= rER + 2e-12)
        assert np.allclose(expected[age < 0], rER[age < 0], atol=2e-12, rtol=0)
    diagnostic = dict(arm=arm, labels=len(records),
                      positive_age_labels=int((age > 1e-8).sum()),
                      support_above_noage=int((cap > r0 + 1e-8).sum()),
                      constrained_below_er=int((expected < rER - 1e-8).sum()),
                      preserved_above_cr=int((expected > np.minimum(r0, rER) + 1e-8).sum()))
    if arm == primary:
        delivery = np.asarray(data['delivered'])
        poses = np.asarray(data['positions'])
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in ['no_age', 'ER', 'candidate']}
        for t in range(len(data['time'])):
            for n in range(2):
                subset = records[(records[:, 0] == t + 1) & (records[:, 1] == n + 1)]
                if not delivery[n, 1 - n, t]:
                    assert not len(subset)
                    continue
                for rule, column in [('no_age', 7), ('ER', 8), ('candidate', 9)]:
                    value = score(data['truth'][t], counterfactual(subset, column, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(values)) for key, values in metrics.items()}
                                    for rule, metrics in effects.items()}
    return diagnostic


def audit_sequence(entry, freeze, matlab_source):
    name, T = entry['sequence'], entry['frames']
    mat = loadmat(OUT / 'data' / f'v2v4real_{name}.mat')
    assert digest(OUT / 'data' / f'v2v4real_{name}.mat') == entry['input_sha256']
    rows, common, diagnostics, inputs = [], [], [], []
    audited = 0
    local_outputs = {}
    for condition in freeze['conditions']:
        matches, local_counts = {}, {}
        for arm in freeze['arms']:
            path = OUT / 'results_holdout' / f'{name}_{condition}_{arm}.json.gz'
            data = read(path)
            run = data['runs']
            assert data['implementation'] == 'common-port-v1' and data['cohort'] == 'holdout'
            assert data['protocol'] == freeze['protocol'] and data['sourceSha256'] == matlab_source
            assert not data['smoke'] and data['sequence'] == name and data['condition'] == condition
            assert data['inputSha256'] == entry['input_sha256'] and run['arm'] == arm
            assert np.array_equal(data['time'], mat['time'].ravel())
            poses = np.asarray(data['positions'])
            assert np.array_equal(poses, mat['positions'])
            delivery = radio_draws(int(name), T, condition)
            assert np.array_equal(data['delivered'], delivery), (name, condition, 'independent radio RNG')
            for t in range(T):
                assert np.array_equal(np.asarray(data['truth'][t]).reshape(4, -1), mat['truth'][0, t])
                assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(), mat['truthIds'][0, t].ravel())
            if arm.endswith('local'):
                local_counts[arm] = np.array([len(x) for x in run['rawEstimates']]).reshape(2, T, order='F')
                if arm in local_outputs:
                    for key in ['estimates', 'rawEstimates', 'labels']:
                        assert run[key] == local_outputs[arm][key]
                else:
                    local_outputs[arm] = {key: run[key] for key in ['estimates', 'rawEstimates', 'labels']}
            audit_packets(run, delivery, T, local_counts)
            values = {key: [] for key in METRICS[:6]}
            matched = []
            sse, support = 0., 0
            assert len(run['estimates']) == len(run['rawEstimates']) == len(run['labels']) == 2 * T
            for t in range(T):
                for n in range(2):
                    index = n + 2 * t
                    raw = np.asarray(run['rawEstimates'][index], dtype=float).reshape(-1, 4)
                    output = np.asarray(run['estimates'][index], dtype=float).reshape(-1, 4)
                    labels = np.asarray(run['labels'][index]).reshape(2, -1)
                    assert np.isfinite(raw).all() and labels.shape[1] == len(raw)
                    assert np.unique(labels, axis=1).shape[1] == len(raw)
                    assert np.array_equal(output, raw[domain(raw, poses[:, :, t])])
                    value = score(data['truth'][t], output)
                    audited += 1
                    for key in ['ospa', 'countError', 'matchedSquaredError', 'matchedCount']:
                        assert np.isclose(value[key], run[key][n][t], atol=1e-8, rtol=1e-9), (name, condition, arm, key, t, n)
                    for key in values:
                        values[key].append(value[key])
                    sse += value['matchedSquaredError']
                    support += value['matchedCount']
                    matched.append(value['match_d2'])
            matches[arm] = np.concatenate(matched)
            rows.append(dict(sequence=name, condition=condition, arm=arm, frames=T,
                             **{key: float(np.mean(v)) for key, v in values.items()},
                             matched_sse=sse, matched_support=support,
                             raw_bytes=sum(run['rawPayloadBytes']), delivered_raw_bytes=sum(run['deliveredRawBytes']),
                             wire_bytes=sum(run['wireBytes']), runtime_s=run['runtimeSeconds'],
                             maximum_bernoulli_count=run['maximumBernoulliCount']))
            diagnostic = audit_existence(run, data, freeze['primary'])
            if diagnostic is not None:
                diagnostics.append(dict(sequence=name, condition=condition, **diagnostic))
            inputs.append(dict(sequence=name, condition=condition, arm=arm, sha256=digest(path)))
        for reference in freeze['arms']:
            if reference == freeze['primary']:
                continue
            a, b = matches[freeze['primary']], matches[reference]
            both = np.isfinite(a) & np.isfinite(b)
            common.append(dict(sequence=name, condition=condition, candidate=freeze['primary'], reference=reference,
                               support=int(both.sum()), candidate_sse=float(a[both].sum()), reference_sse=float(b[both].sum())))
    assert len(rows) == 32 and audited == 64 * T
    return dict(sequence=name, audited_node_frames=audited, runs=rows, inputs=inputs,
                common_target=common, diagnostics=diagnostics, auditor_sha256=digest(Path(__file__)),
                method_freeze_sha256=digest(OUT / 'METHOD_FREEZE.json'))


def summarize(reports, freeze, destination, source_count):
    rows = [row for report in reports for row in report['runs']]
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    sequences = [f'{s:04d}' for s in freeze['units']]
    samples = np.random.default_rng(freeze['bootstrap']['seed']).integers(0, len(sequences), (10000, len(sequences)))
    aggregate, paired = [], []
    contrast_pairs = [(freeze['primary'], arm) for arm in freeze['arms'] if arm != freeze['primary']]
    contrast_pairs += [(arm, arm.removeprefix('marked_')) for arm in
                       ['marked_local', 'marked_lineage', 'marked_mil_support', 'marked_tc_ospa2_w5', 'marked_tc_ospa2_w10']]
    contrast_pairs += [('marked_er', 'qualified_exist')]
    for condition in freeze['conditions']:
        for arm in freeze['arms']:
            group = [lookup[name, condition, arm] for name in sequences]
            aggregate.append(dict(condition=condition, arm=arm,
                                  **{key: interval([r[key] for r in group], samples) for key in METRICS},
                                  frame_weighted_ospa=float(np.average([r['ospa'] for r in group], weights=[r['frames'] for r in group]))))
        for candidate, reference in contrast_pairs:
            a = [lookup[name, condition, candidate] for name in sequences]
            b = [lookup[name, condition, reference] for name in sequences]
            delta = np.array([x['ospa'] - y['ospa'] for x, y in zip(a, b)])
            paired.append(dict(condition=condition, candidate=candidate, reference=reference,
                               **{key: interval([x[key] - y[key] for x, y in zip(a, b)], samples) for key in METRICS},
                               ospa_wins=int((delta < -1e-10).sum()),
                               leave_one_sequence_out_mean_range=[float(((delta.sum() - delta) / (len(delta) - 1)).min()),
                                                                  float(((delta.sum() - delta) / (len(delta) - 1)).max())]))
    common = [row for report in reports for row in report['common_target']]
    common_aggregate = []
    for condition in freeze['conditions']:
        for reference in freeze['arms']:
            if reference == freeze['primary']:
                continue
            group = [r for r in common if r['condition'] == condition and r['reference'] == reference]
            support = sum(r['support'] for r in group)
            common_aggregate.append(dict(condition=condition, candidate=freeze['primary'], reference=reference,
                                         support=support, candidate_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group) / support)),
                                         reference_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group) / support))))
    primary = [r for r in paired if r['candidate'] == freeze['primary'] and r['reference'] in freeze['primary_references']]
    result = dict(protocol=freeze['protocol'], primary=freeze['primary'], scope=freeze['limitations'],
                  sequences=len(sequences), frames=sum(r['frames'] for r in rows if r['condition'] == 'reliable' and r['arm'] == freeze['primary']),
                  source_hashes_verified=source_count, audited_node_frames=sum(r['audited_node_frames'] for r in reports),
                  aggregate=aggregate, paired=paired, runs=rows, common_target=common, common_aggregate=common_aggregate,
                  diagnostics=[r for report in reports for r in report['diagnostics']],
                  inputs=[r for report in reports for r in report['inputs']],
                  registered_primary_intervals_all_below_zero=all(r['ospa']['high'] < 0 for r in primary),
                  interval_note='Descriptive sequence bootstrap; 10000 resamples, seed 8301; no multiplicity adjustment; related routes possible.',
                  radio_rng_independently_reconstructed=True,
                  per_source_actual_packet_and_tc_history_bytes_verified=True,
                  native_baseline_packets='208 B/Bernoulli + 32 B header; ceiling 216 B/Bernoulli + 32 B header; TC actual 5/10-frame histories.',
                  runtime_note='Density runtime includes local update and fusion. TC runtime covers history serialization and fusion only; its corresponding local runtime must be added for an end-to-end comparison.',
                  method_freeze_sha256=digest(OUT / 'METHOD_FREEZE.json'),
                  input_audit_sha256=digest(OUT / 'input_audit.json'), auditor_sha256=digest(Path(__file__)))
    assert len(rows) == len(result['inputs']) == 800
    assert result['frames'] == 5601 and result['audited_node_frames'] == 358464
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'summary_holdout.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (destination / 'holdout_runs.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    for row in aggregate:
        print(row['condition'], row['arm'], 'OSPA', round(row['ospa']['mean'], 6), flush=True)
    for row in primary:
        print('FROZEN PRIMARY CONTRAST', row['condition'], row['reference'], row['ospa'], 'wins', row['ospa_wins'], flush=True)
    print('ALL HOLDOUT OUTPUTS INDEPENDENTLY AUDITED:', result['audited_node_frames'], 'node-frames; 800/800 files.', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=OUT)
    args = parser.parse_args()
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    source = json.loads((OUT / 'source_sha256_port.json').read_text())
    for name, expected in {**source, **freeze['source_and_selection_evidence_sha256']}.items():
        assert digest(ROOT / name) == expected, name
    input_audit = json.loads((OUT / 'input_audit.json').read_text())
    assert input_audit['method_freeze_sha256'] == digest(OUT / 'METHOD_FREEZE.json')
    assert input_audit['exact_pose_measurement_truth_score_and_likelihood_reconstruction']
    assert input_audit['auditor_sha256'] == digest(OUT / 'audit_holdout_inputs.py')
    matlab_source = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: h for name, h in source.items()}
    assert len(matlab_source) == len(source)
    manifest = json.loads((OUT / 'input_manifest.json').read_text())
    cache = args.output_dir / 'audit_sequences'
    cache.mkdir(parents=True, exist_ok=True)
    reports = []
    for entry in manifest['sequences']:
        name = entry['sequence']
        while True:
            runtime_path = OUT / 'holdout_runtime.json'
            attempts = json.loads(runtime_path.read_text()) if runtime_path.exists() else []
            attempt = next((r for r in attempts if r['sequence'] == int(name)), None)
            if attempt is not None:
                assert attempt['returncode'] == 0 and attempt['completion_line'] and attempt['files'] == 32, attempt
                break
            assert args.watch, (name, 'frozen tracking sequence not complete')
            time.sleep(10)
        path = cache / f'{name}.json'
        if path.exists():
            report = json.loads(path.read_text())
            assert report['auditor_sha256'] == digest(Path(__file__)), 'Use a new output directory after an auditor change.'
            assert report['method_freeze_sha256'] == digest(OUT / 'METHOD_FREEZE.json')
            for row in report['inputs']:
                result_path = OUT / 'results_holdout' / f"{name}_{row['condition']}_{row['arm']}.json.gz"
                assert digest(result_path) == row['sha256'], result_path
        else:
            report = audit_sequence(entry, freeze, matlab_source)
            path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        reports.append(report)
        print('HOLDOUT SEQUENCE AUDITED', name, report['audited_node_frames'], 'node-frames;', len(reports), '/25', flush=True)
    summarize(reports, freeze, args.output_dir, len(source))


if __name__ == '__main__':
    main()
