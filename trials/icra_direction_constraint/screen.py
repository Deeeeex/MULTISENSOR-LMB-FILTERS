"""Score all frozen source runs without feeding substitutions into recursion."""
from datetime import datetime, timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys
import numpy as np
from scipy.io import loadmat

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
for name in ['icra_method_iteration', 'icra_external_fusion']:
    sys.path.insert(0, str(OUT.parent / name))
from analyze_development import counterfactual
from analyze_case_studies import score
from direction_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights

METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError', 'outputCount']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def sorted_xy(estimate):
    points = np.asarray(estimate, float).reshape(-1,4)[:,:2]
    return points[np.lexsort((points[:,1], points[:,0]))]


def main():
    destination = OUT / 'SCREEN_RESULTS.json'; assert not destination.exists()
    freeze = OUT / 'SCREEN_FREEZE.json'; cfg = json.loads(freeze.read_text())
    assert cfg['rules'] == RULES and cfg['primary'] == 'nonreversal' and len(cfg['cells']) == 56
    for name, digest in cfg['source_sha256'].items(): assert sha(ROOT/name) == digest, name
    fixtures = check_fixtures()
    resultdir = OUT / 'results'; resultdir.mkdir(exist_ok=True)
    rows = []; diagnostics = []; artifacts = {}; total_frames = 0; original_frames = 0
    for index, cell in enumerate(cfg['cells']):
        native = ROOT / cell['path']; assert sha(native) == cell['sha256']
        with gzip.open(native, 'rt') as stream: data = json.load(stream)
        run = data['runs']; assert data['pd'] == .9
        assert data['sequence'] == cell['sequence'] and data['condition'] == cell['condition']
        assert run['arm'] == ('marked_gaussian_evidence' if cell['backend'] == 'GCE' else
                             'marked_gaussian_evidence_guarded_scalar')
        state = source_state(run); records = state['records']
        ratio = loadmat(ROOT / cell['ratios_path'])['likelihoodRatios']
        raw = audit_raw_weights(data, state, ratio)
        T = len(data['time']); poses = np.asarray(data['positions']); delivery = np.asarray(data['delivered'], bool)
        masks = {(t,n): (records[:,0] == t+1) & (records[:,1] == n+1) for t in range(T) for n in range(2)}
        tensors = {}; outputs = {}; cell_diags = []
        for rule in RULES:
            alternate, values, diag = calculate(state, rule, cell['backend'] == 'Guarded Scalar')
            for name, value in values.items(): tensors[rule+'__'+name] = value
            scores = {k: [] for k in METRICS}; estimates = []
            for t in range(T):
                for n in range(2):
                    mask = masks[t,n]
                    if not delivery[n,1-n,t]: assert not mask.any()
                    estimate = (counterfactual(alternate[mask], 9, poses[:,:,t]) if delivery[n,1-n,t]
                                else run['estimates'][n+2*t])
                    value = score(data['truth'][t], estimate); value['outputCount'] = len(estimate)
                    if rule == 'original':
                        actual = run['estimates'][n+2*t]
                        assert len(actual) == len(estimate)
                        assert np.allclose(sorted_xy(estimate), sorted_xy(actual), atol=1e-12, rtol=0)
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                        assert value['countError'] == run['countError'][n][t]
                        original_frames += 1
                    for key in METRICS: scores[key].append(value[key])
                    estimates.append(estimate); total_frames += 1
            outputs[rule] = dict(estimates=estimates, scores=scores)
            row = {k:cell[k] for k in ['dataset','sequence','condition','backend','recording']}
            row.update(rule=rule, frames=T, **{k:float(np.mean(v)) for k,v in scores.items()})
            rows.append(row); cell_diags.append(dict(rule=rule, **diag))
        prefix = resultdir / f"{index:02d}_{cell['sequence']}_{cell['condition']}_{cell['backend'].replace(' ', '_')}"
        arrays_path = prefix.with_suffix('.npz'); output_path = prefix.with_suffix('.json.gz')
        assert not arrays_path.exists() and not output_path.exists()
        np.savez_compressed(arrays_path, **tensors)
        with gzip.open(output_path, 'wt') as stream:
            json.dump(dict(cell=cell, outputs=outputs, diagnostics=cell_diags, raw_weights=raw), stream, allow_nan=False)
        for path in [arrays_path, output_path]: artifacts[str(path.relative_to(ROOT))] = sha(path)
        diagnostics.append(dict(cell=cell, original=state['original_audit'], raw_weights=raw, rules=cell_diags))
        print('SUBSTITUTION COMPLETE', index+1, '/', len(cfg['cells']), cell['sequence'], cell['condition'], cell['backend'], flush=True)
    aggregate = []
    for dataset in ['v2v_development', 'v2x_val']:
        for backend in ['GCE', 'Guarded Scalar']:
            for rule in RULES:
                group = [r for r in rows if (r['dataset'],r['backend'],r['rule']) == (dataset,backend,rule)]
                conditions = {}
                for condition in cfg['conditions']:
                    chosen = [r for r in group if r['condition'] == condition]
                    assert len(chosen) == (9 if dataset == 'v2v_development' else 5)
                    conditions[condition] = {k:float(np.mean([r[k] for r in chosen])) for k in METRICS}
                aggregate.append(dict(dataset=dataset, backend=backend, rule=rule, conditions=conditions,
                    selection_mean_ospa=float(np.mean([r['ospa'] for r in conditions.values()]))))
    lookup = {(r['dataset'],r['backend'],r['rule']):r for r in aggregate}
    gates = []
    for dataset in ['v2v_development','v2x_val']:
        for condition in cfg['conditions']:
            for metric in ['ospa','gospa']:
                primary=lookup[dataset,'GCE','nonreversal']['conditions'][condition][metric]
                ref=lookup[dataset,'GCE','original']['conditions'][condition][metric]
                gates.append(dict(dataset=dataset,condition=condition,metric=metric,reference='original',
                    candidate_mean=primary,reference_mean=ref,difference=primary-ref,
                    passed=primary<ref if metric=='ospa' else primary<=ref))
    for name, digest in cfg['source_sha256'].items(): assert sha(ROOT/name) == digest, name
    report = dict(passed=True, protocol=cfg['protocol'], completed_utc=datetime.now(timezone.utc).isoformat(),
        fixed_input_only=True, native_runs=0, source_runs=len(cfg['cells']), robot_frames=total_frames,
        original_parity_robot_frames=original_frames, rows=rows, aggregate=aggregate, gates=gates,
        advance_to_recursion=all(r['passed'] for r in gates), diagnostics=diagnostics,
        fixture_summaries=fixtures, freeze_sha256=sha(freeze), source_sha256=cfg['source_sha256'], artifacts=artifacts)
    destination.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    table = OUT / 'ALL_SCREEN_SCORES.csv'; assert not table.exists()
    with table.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n'); writer.writeheader(); writer.writerows(rows)
    for gate in gates: print('FIXED GATE', json.dumps(gate), flush=True)
    print('SCREEN COMPLETE', 'advance', report['advance_to_recursion'], 'robot frames', total_frames, flush=True)


if __name__ == '__main__': main()
