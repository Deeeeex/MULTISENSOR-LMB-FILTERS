"""Apply the registered two-condition and association-quality development gate."""
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
CANDIDATES = [BASE + '_assoc_direct', BASE + '_assoc_temporal']
CONDITIONS = ['reliable', 'intermittent']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT / 'DEVELOPMENT_SELECTION.json'
    assert not destination.exists()
    baseline_path = OUT.parent / 'icra_gaussian_evidence/summary_development.json'
    original = json.loads(baseline_path.read_text())
    rows = [r for r in original['runs'] if r['arm'] == BASE]
    assert len(rows) == 18
    inputs = {str(baseline_path.relative_to(ROOT)): sha(baseline_path)}
    paths = {(r['sequence'], r['condition'], BASE): OUT.parent / 'icra_gaussian_evidence/results_development' /
             f"{r['sequence']}_{r['condition']}_{BASE}.json.gz" for r in rows}
    for path in paths.values():
        name = str(path.relative_to(ROOT))
        assert sha(path) == original['inputs'][name]
        inputs[name] = original['inputs'][name]
    for stage, expected in [('association_v1_preflight', 6), ('association_v1_development_rest', 32)]:
        audit_path = OUT / ('audit_' + stage + '.json')
        audit = json.loads(audit_path.read_text())
        assert audit['passed'] and len(audit['rows']) == expected
        cfg_path = OUT / 'stages' / (stage + '.json')
        runtime_path = OUT / ('runtime_' + stage + '.json')
        assert sha(cfg_path) == audit['config_sha256'] and sha(runtime_path) == audit['runtime_sha256']
        cfg = json.loads(cfg_path.read_text())
        runtime = json.loads(runtime_path.read_text())
        assert len(runtime) == len(cfg['units']) and all(r['returncode'] == 0 and r['completion_line'] for r in runtime)
        for group in [cfg['source_sha256'], audit['auditor_sha256'], audit['inputs']]:
            for name, expected_hash in group.items():
                assert sha(ROOT / name) == expected_hash, name
                inputs[name] = expected_hash
        for path in [audit_path, cfg_path, runtime_path]:
            inputs[str(path.relative_to(ROOT))] = sha(path)
        for row in audit['rows']:
            if row['arm'] in CANDIDATES:
                rows.append(row)
                paths[row['sequence'], row['condition'], row['arm']] = OUT / 'results' / stage / f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
    assert len(rows) == len(paths) == 54
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    assert len(lookup) == 54
    identities = []
    old_diagnostic = json.loads((OUT / 'ASSOCIATION_DIAGNOSTIC.json').read_text())
    old_lookup = {(r['sequence'], r['condition']): r for r in old_diagnostic['rows']
                  if r['dataset'] == 'v2v' and r['arm'] == BASE}
    for (sequence, condition, arm), path in sorted(paths.items()):
        with gzip.open(path, 'rt') as handle:
            data = json.load(handle)
        value = evaluate_identities(data)
        if arm == BASE:
            old = old_lookup[sequence, condition]['identity']
            for cutoff in ['2', '12']:
                actual = value['cutoffs'][cutoff]
                for key in ['known_wrong', 'known_correct', 'assigned_wrong', 'assigned_correct',
                            'common_identity_pairs_missed', 'common_identity_opportunities', 'final_assigned_estimates']:
                    assert actual.get(key, 0) == old[cutoff].get(key, 0), (sequence, condition, cutoff, key)
                assert actual.get('label_truth_switches', 0) == old[cutoff].get('consecutive_identity_switches', 0)
        identities.append(dict(sequence=sequence, condition=condition, arm=arm, **value))
        print('IDENTITY CHECKED', sequence, condition, arm, flush=True)
    aggregate = []
    for arm in [BASE, *CANDIDATES]:
        for condition in CONDITIONS:
            part = [r for r in rows if r['arm'] == arm and r['condition'] == condition]
            assert len(part) == 9 and sum(r['frames'] for r in part) == 1993
            identity = pooled([r for r in identities if r['arm'] == arm and r['condition'] == condition])
            aggregate.append(dict(arm=arm, condition=condition,
                **{k: float(np.mean([r[k] for r in part])) for k in METRICS}, identity=identity))
    summaries = {(r['arm'], r['condition']): r for r in aggregate}
    all_identity = {arm: pooled([r for r in identities if r['arm'] == arm]) for arm in [BASE, *CANDIDATES]}
    decisions = []
    for arm in CANDIDATES:
        delta = {c: summaries[arm, c]['ospa'] - summaries[BASE, c]['ospa'] for c in CONDITIONS}
        quality = all_identity[arm]['2']['wrong_pair_rate'] <= all_identity[BASE]['2']['wrong_pair_rate']
        eligible = all(d < 0 for d in delta.values()) and quality
        decisions.append(dict(arm=arm, mean_ospa=float(np.mean([summaries[arm, c]['ospa'] for c in CONDITIONS])),
            ospa_difference=delta, two_conditions_improve=all(d < 0 for d in delta.values()),
            pooled_identity=all_identity[arm], wrong_pair_rate_nonincreasing=quality, eligible=bool(eligible)))
    ranking = sorted(decisions, key=lambda r: (r['mean_ospa'], CANDIDATES.index(r['arm'])))
    eligible = [r for r in ranking if r['eligible']]
    selected = eligible[0] if eligible else None
    result = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha(OUT / 'CANDIDATES_V1.md'), selection_rule='Among candidates passing both OSPA conditions and the pooled 2 m wrong-pair gate, choose the lowest two-condition sequence mean; exact ties prefer direct.',
        rows=rows, aggregate=aggregate, identity_rows=identities, baseline_pooled_identity=all_identity[BASE],
        ranking=ranking, selected=selected, advance=selected is not None,
        inputs=inputs, source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__),
            OUT / 'identity_metrics.py', OUT / 'diagnose_association_v2.py']},
        exposure='All development data were previously exposed. No additional V2X test tracking outcome has been read.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / 'development_comparison.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print('DEVELOPMENT SELECTION', json.dumps(selected, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
