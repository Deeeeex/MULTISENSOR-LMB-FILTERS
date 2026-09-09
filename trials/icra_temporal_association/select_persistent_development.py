"""Apply the unchanged V2 gate to all nine segments and retain all V1 results."""
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
CANDIDATES = [BASE + '_assoc_reopen', BASE + '_assoc_split']
CONDITIONS = ['reliable', 'intermittent']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT / 'PERSISTENT_DEVELOPMENT_SELECTION.json'
    assert not destination.exists()
    prior_path = OUT / 'DEVELOPMENT_SELECTION.json'
    prior = json.loads(prior_path.read_text())
    assert prior['passed'] and len(prior['rows']) == len(prior['identity_rows']) == 54
    inputs = {str(prior_path.relative_to(ROOT)): sha(prior_path)}
    for group in [prior['inputs'], prior['source_sha256']]:
        for name, expected in group.items():
            assert sha(ROOT / name) == expected, name
            inputs[name] = expected
    rows, identities, aggregate = prior['rows'].copy(), prior['identity_rows'].copy(), prior['aggregate'].copy()
    paths = {}
    for stage, expected in [('association_v2_preflight', 12), ('association_v2_development_rest', 28)]:
        audit_path = OUT / ('audit_' + stage + '.json')
        audit = json.loads(audit_path.read_text())
        assert audit['passed'] and len(audit['rows']) == expected
        cfg_path = OUT / 'stages' / (stage + '.json')
        runtime_path = OUT / ('runtime_' + stage + '.json')
        assert sha(cfg_path) == audit['config_sha256'] and sha(runtime_path) == audit['runtime_sha256']
        cfg, runtime = json.loads(cfg_path.read_text()), json.loads(runtime_path.read_text())
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
    assert len(paths) == 36 and len(rows) == 90
    assert len({(r['sequence'], r['condition'], r['arm']) for r in rows}) == 90
    for (sequence, condition, arm), path in sorted(paths.items()):
        with gzip.open(path, 'rt') as handle:
            data = json.load(handle)
        identities.append(dict(sequence=sequence, condition=condition, arm=arm, **evaluate_identities(data)))
        print('PERSISTENT IDENTITY CHECKED', sequence, condition, arm, flush=True)
    for arm in CANDIDATES:
        for condition in CONDITIONS:
            part = [r for r in rows if r['arm'] == arm and r['condition'] == condition]
            assert len(part) == 9 and sum(r['frames'] for r in part) == 1993
            identity = pooled([r for r in identities if r['arm'] == arm and r['condition'] == condition])
            aggregate.append(dict(arm=arm, condition=condition,
                **{k: float(np.mean([r[k] for r in part])) for k in METRICS}, identity=identity))
    summaries = {(r['arm'], r['condition']): r for r in aggregate}
    all_identity = {arm: pooled([r for r in identities if r['arm'] == arm]) for arm in [BASE, *CANDIDATES]}
    assert all_identity[BASE] == prior['baseline_pooled_identity']
    decisions = []
    for arm in CANDIDATES:
        delta = {c: summaries[arm, c]['ospa'] - summaries[BASE, c]['ospa'] for c in CONDITIONS}
        quality = all_identity[arm]['2']['wrong_pair_rate'] <= all_identity[BASE]['2']['wrong_pair_rate']
        eligible = all(d < 0 for d in delta.values()) and quality
        decisions.append(dict(arm=arm, mean_ospa=float(np.mean([summaries[arm, c]['ospa'] for c in CONDITIONS])),
            ospa_difference=delta, two_conditions_improve=all(d < 0 for d in delta.values()),
            pooled_identity=all_identity[arm], wrong_pair_rate_nonincreasing=quality, eligible=bool(eligible)))
    ranking = sorted(decisions, key=lambda r: (r['mean_ospa'], CANDIDATES.index(r['arm'])))
    selected = next((r for r in ranking if r['eligible']), None)
    result = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha(OUT / 'CANDIDATES_V2.md'),
        selection_rule='Among R and S passing both OSPA conditions and the pooled 2 m wrong-pair gate, choose the lowest two-condition sequence mean; exact ties prefer R.',
        rows=rows, aggregate=aggregate, identity_rows=identities, baseline_pooled_identity=all_identity[BASE],
        ranking=ranking, selected=selected, advance=selected is not None, inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'identity_metrics.py']},
        exposure='All development data were previously exposed. No additional V2X test tracking outcome has been read.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / 'persistent_development_comparison.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print('PERSISTENT DEVELOPMENT SELECTION', json.dumps(selected, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
