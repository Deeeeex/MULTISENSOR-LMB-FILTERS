"""Rescore saved trajectories, then build portable phase and fixed-input analyses.

Use --extract once in the research checkout. Ordinary regeneration uses the
portable per-receiver-scan scores, without requiring native tracking outputs.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import json

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DATA = HERE / 'source_data'
PRIMARY = 'marked_gaussian_evidence'
ARMS = ['marked_lineage', 'marked_asymmetric', PRIMARY]
CONDITIONS = ['reliable', 'intermittent']
PHASES = ['before', 'outage', 'after']
CELLS = ['base_space_base_integral', 'new_space_base_integral',
         'base_space_new_integral', 'new_space_new_integral']
SNAPSHOT = DATA / 'mechanism_diagnostic_snapshot.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def ospa(truth, estimates):
    from scipy.optimize import linear_sum_assignment
    truth = np.asarray(truth, float).reshape(4, -1).T[:, :2]
    estimates = np.asarray(estimates, float).reshape(-1, 4)[:, :2]
    n, m = len(truth), len(estimates)
    if max(n, m) == 0:
        return 0.
    costs = np.minimum(((truth[:, None]-estimates[None])**2).sum(2), 144.)
    rows, cols = linear_sum_assignment(costs)
    return float(np.sqrt((costs[rows, cols].sum()+144.*abs(n-m))/max(n, m)))


def extract_set(rows, probabilities, positions, new_space):
    keep = probabilities > .001
    rows, probabilities = rows[keep], probabilities[keep]-1e-6
    mass = np.array([1.])
    for p in probabilities:
        mass = np.convolve(mass, [1-p, p])
    count = int(mass.argmax())
    selected = rows[np.argsort(-probabilities, kind='stable')[:count]]
    means = selected[:, 4:6] if new_space else selected[:, 38:40]
    distance = ((means[:, None]-positions.T[None])**2).sum(2).min(1)
    valid = ((np.abs(means[:, 0]) <= 70.4) & (np.abs(means[:, 1]) <= 40)
             & (distance <= 1600) & (distance > 9))
    return np.c_[means[valid], np.zeros((int(valid.sum()), 2))]


def extract(evidence):
    from scipy.special import expit
    runs, fixed, inputs = [], [], {}
    native_scores = {(r['sequence'], r['condition'], r['arm']): r['ospa']
                     for r in evidence['runs']}
    nodes, fixed_nodes, largest_parity_error = 0, 0, 0.
    for name in evidence['sequences']:
        for condition in CONDITIONS:
            common = None
            for arm in ARMS:
                suffix = f'/{name}_{condition}_{arm}.json.gz'
                candidates = [p for p in evidence['source_inputs_sha256']
                              if p.endswith(suffix) and 'zero_codec' not in p]
                assert len(candidates) == 1, candidates
                relative = candidates[0]
                path = ROOT / relative
                assert sha(path) == evidence['source_inputs_sha256'][relative]
                inputs[relative] = sha(path)
                with gzip.open(path, 'rt') as stream:
                    data = json.load(stream)
                assert data['sequence'] == name and data['condition'] == condition and not data['smoke']
                if common is None:
                    common = {k: data[k] for k in ['truth', 'truthIds', 'time', 'positions', 'delivered', 'inputSha256']}
                else:
                    assert all(data[k] == v for k, v in common.items())
                run = data['runs']
                assert run['arm'] == arm
                length = len(data['time'])
                boundaries = [0, int(.4*length), int(.6*length), length]
                delivered = np.asarray(data['delivered'], bool)
                assert delivered.shape == (2, 2, length)
                if condition == 'intermittent':
                    assert not delivered[:, :, boundaries[1]:boundaries[2]].any()
                else:
                    assert delivered.sum() == 2*length
                values = np.zeros((2, length))
                for t in range(length):
                    for receiver in range(2):
                        values[receiver, t] = ospa(data['truth'][t], run['estimates'][receiver+2*t])
                original = np.asarray(run['ospa'], float)
                assert original.shape == values.shape
                error = float(np.max(np.abs(original-values)))
                assert error < 1e-8
                largest_parity_error = max(largest_parity_error, error)
                assert np.isclose(values.mean(), native_scores[name, condition, arm], atol=1e-9, rtol=0)
                nodes += values.size
                runs.append(dict(sequence=name, condition=condition, arm=arm,
                                 frames=length, phase_boundaries=boundaries, ospa=values.tolist(),
                                 native_mean_ospa=native_scores[name, condition, arm]))
                if arm != PRIMARY:
                    continue
                records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)
                active = records[:, 28:30] > 0
                logits = np.zeros_like(records[:, 28:30])
                r = np.clip(records[:, 17:19][active], 1e-9, 1-1e-9)
                logits[active] = np.log(r)-np.log1p(-r)
                z = (records[:, 28:30]*logits).sum(1)+(records[:, 52:54]*records[:, 19:21]).sum(1)
                p0, p1 = expit(z+records[:, 10]), expit(z+records[:, 56])
                assert np.allclose(p1, records[:, 9], atol=2e-12, rtol=0)
                # All four cells share these exact admitted increments, including rejection.
                assert np.all(records[:, 52:54] >= 0)
                cell_values = {cell: [] for cell in CELLS}
                sites = []
                poses = np.asarray(data['positions'], float)
                for t in range(length):
                    for receiver in range(2):
                        mask = (records[:, 0] == t+1) & (records[:, 1] == receiver+1)
                        if not delivered[receiver, 1-receiver, t]:
                            assert not mask.any()
                            continue
                        sites.append([t+1, receiver+1])
                        for cell in CELLS:
                            prob = p1 if cell.endswith('new_integral') else p0
                            output = extract_set(records[mask], prob[mask], poses[:, :, t], cell.startswith('new_space'))
                            value = ospa(data['truth'][t], output)
                            cell_values[cell].append(value)
                            if cell == CELLS[-1]:
                                error = abs(value-values[receiver, t])
                                assert error < 1e-8, (name, condition, t, receiver, error)
                                largest_parity_error = max(largest_parity_error, error)
                assert len(sites) == int(delivered.sum())
                fixed_nodes += len(sites)
                fixed.append(dict(sequence=name, condition=condition, receiver_scans=sites, ospa=cell_values))
            print('MECHANISM SCORES VERIFIED', name, condition, flush=True)
    assert len(runs) == 150 and len(fixed) == 50 and nodes == 67212
    snapshot = dict(protocol='all-sequence-phase-and-guarded-fixed-input-v1',
                    sequences=evidence['sequences'], runs=runs, fixed_input=fixed,
                    source_inputs_sha256=inputs, source_evidence_sha256=sha(DATA/'gaussian_paper_evidence.json'),
                    extractor_sha256=sha(Path(__file__)), protocol_sha256=sha(HERE/'ANALYSIS_PROTOCOL.md'),
                    rescored_native_receiver_scans=nodes, verified_joint_receiver_scans=fixed_nodes,
                    maximum_native_ospa_absolute_error=largest_parity_error,
                    all_four_cells_share_admitted_kappa=True,
                    alternate_outputs_fed_back=False)
    dump(SNAPSHOT, snapshot)


def interval(values, draws):
    values = np.asarray(values, float)
    low, high = np.quantile(values[draws].mean(1), [.025, .975])
    return dict(mean=float(values.mean()), low=float(low), high=float(high), n=len(values))


def aggregate(snapshot):
    names = snapshot['sequences']
    assert len(names) == 25 and len(snapshot['runs']) == 150 and len(snapshot['fixed_input']) == 50
    assert snapshot['protocol_sha256'] == sha(HERE/'ANALYSIS_PROTOCOL.md')
    assert snapshot['all_four_cells_share_admitted_kappa'] and not snapshot['alternate_outputs_fed_back']
    assert snapshot['maximum_native_ospa_absolute_error'] < 1e-8
    for path, expected in snapshot['source_inputs_sha256'].items():
        if (ROOT/path).exists():
            assert sha(ROOT/path) == expected
    draws = np.random.default_rng(8301).integers(0, 25, (10000, 25))
    phase_rows = []
    for row in snapshot['runs']:
        values = np.asarray(row['ospa'])
        assert values.shape == (2, row['frames'])
        assert np.isclose(values.mean(), row['native_mean_ospa'], atol=1e-9, rtol=0)
        for index, phase in enumerate(PHASES):
            start, end = row['phase_boundaries'][index:index+2]
            assert end > start
            phase_rows.append(dict(sequence=row['sequence'], condition=row['condition'], arm=row['arm'], phase=phase,
                                   receiver_scans=2*(end-start), ospa=float(values[:, start:end].mean())))
    look = {(r['sequence'], r['condition'], r['arm'], r['phase']): r['ospa'] for r in phase_rows}
    phase_aggregate, phase_paired = [], []
    for condition in CONDITIONS:
        for phase in PHASES:
            for arm in ARMS:
                values = [look[n, condition, arm, phase] for n in names]
                phase_aggregate.append(dict(condition=condition, phase=phase, arm=arm, ospa=interval(values, draws)))
                if arm != PRIMARY:
                    diffs = [look[n, condition, PRIMARY, phase]-look[n, condition, arm, phase] for n in names]
                    phase_paired.append(dict(condition=condition, phase=phase, reference=arm,
                                             ospa=interval(diffs, draws), differences=diffs))
    fixed_rows, fixed_aggregate, fixed_pairs = [], [], []
    for row in snapshot['fixed_input']:
        assert len(set(map(tuple, row['receiver_scans']))) == len(row['receiver_scans'])
        for cell in CELLS:
            values = row['ospa'][cell]
            assert len(values) == len(row['receiver_scans']) > 0
            fixed_rows.append(dict(sequence=row['sequence'], condition=row['condition'], cell=cell,
                                   receiver_scans=len(values), ospa=float(np.mean(values))))
    look = {(r['sequence'], r['condition'], r['cell']): r['ospa'] for r in fixed_rows}
    for condition in CONDITIONS:
        for cell in CELLS:
            fixed_aggregate.append(dict(condition=condition, cell=cell,
                                       ospa=interval([look[n, condition, cell] for n in names], draws)))
            if cell != CELLS[-1]:
                diffs = [look[n, condition, CELLS[-1]]-look[n, condition, cell] for n in names]
                fixed_pairs.append(dict(condition=condition, reference=cell,
                                        ospa=interval(diffs, draws), differences=diffs))
    result = dict(sequences=names, phase_rows=phase_rows, phase_aggregate=phase_aggregate,
                  phase_paired=phase_paired, fixed_input_rows=fixed_rows,
                  fixed_input_aggregate=fixed_aggregate, fixed_input_paired=fixed_pairs,
                  snapshot_sha256=sha(SNAPSHOT), generator_sha256=sha(Path(__file__)),
                  interval='10000 paired complete-sequence percentile resamples; descriptive, unadjusted',
                  method_reselected=False, new_tracking_trajectories=False)
    dump(DATA/'mechanism_analysis.json', result)
    for label, rows in [('PHASE', phase_paired), ('FIXED INPUT', fixed_pairs)]:
        for row in rows:
            print(label, row['condition'], row.get('phase', ''), row['reference'], row['ospa'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract', action='store_true')
    args = parser.parse_args()
    evidence = json.loads((DATA/'gaussian_paper_evidence.json').read_text())
    if args.extract:
        assert not SNAPSHOT.exists(), 'Preserve the saved diagnostic snapshot.'
        extract(evidence)
    aggregate(json.loads(SNAPSHOT.read_text()))


if __name__ == '__main__':
    main()
