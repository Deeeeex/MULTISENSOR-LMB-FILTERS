"""Apply V3's frozen one-percent gate, retaining every restored candidate."""
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
EARLIER = [BASE + '_assoc_' + mode for mode in ['direct', 'temporal', 'reopen', 'split']]
CANDIDATES = [BASE + '_assoc_' + mode for mode in ['quality', 'nis', 'quality_nis']]
CONDITIONS = ['reliable', 'intermittent']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def verify(groups, inputs):
    for group in groups:
        for name, expected in group.items():
            assert sha(ROOT / name) == expected, name
            if name in inputs:
                assert inputs[name] == expected
            inputs[name] = expected


def main():
    destination = OUT / 'SCREENED_DEVELOPMENT_SELECTION.json'
    assert not destination.exists()
    previous_path = OUT / 'RESTORED_DEVELOPMENT_SELECTION.json'
    previous = json.loads(previous_path.read_text())
    assert previous['passed'] and len(previous['rows']) == len(previous['identity_rows']) == 90
    rows, identities = previous['rows'].copy(), previous['identity_rows'].copy()
    inputs = {str(previous_path.relative_to(ROOT)): sha(previous_path)}
    verify([previous['inputs'], previous['source_sha256']], inputs)
    for stage, count in [('association_screen_preflight', 30), ('association_screen_rest', 36)]:
        audit_path = OUT / ('audit_' + stage + '.json')
        audit = json.loads(audit_path.read_text())
        assert audit['passed'] and len(audit['rows']) == count
        cfg_path, runtime_path = OUT / 'stages' / (stage + '.json'), OUT / ('runtime_' + stage + '.json')
        assert sha(cfg_path) == audit['config_sha256'] and sha(runtime_path) == audit['runtime_sha256']
        cfg, runtime = json.loads(cfg_path.read_text()), json.loads(runtime_path.read_text())
        assert len(cfg['units']) == len(runtime)
        assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2*len(cfg['arms']) for r in runtime)
        verify([cfg['source_sha256'], audit['auditor_sha256'], audit['inputs']], inputs)
        for path in [audit_path, cfg_path, runtime_path]:
            inputs[str(path.relative_to(ROOT))] = sha(path)
        for row in audit['rows']:
            if row['arm'] not in CANDIDATES:
                continue
            rows.append(row)
            seq, condition, arm = [row[k] for k in ['sequence', 'condition', 'arm']]
            path = OUT / 'results' / stage / f'{seq}_{condition}_{arm}.json.gz'
            with gzip.open(path, 'rt') as handle:
                value = evaluate_identities(json.load(handle))
            identities.append(dict(sequence=seq, condition=condition, arm=arm, **value))
            print('SCREEN IDENTITY CHECKED', seq, condition, arm, flush=True)
    assert len(rows) == len(identities) == 144
    assert len({(r['sequence'], r['condition'], r['arm']) for r in rows}) == 144
    arms = [BASE, *EARLIER, *CANDIDATES]
    aggregate = []
    for arm in arms:
        for condition in CONDITIONS:
            part = [r for r in rows if r['arm'] == arm and r['condition'] == condition]
            assert len(part) == 9 and sum(r['frames'] for r in part) == 1993
            identity = pooled([r for r in identities if r['arm'] == arm and r['condition'] == condition])
            aggregate.append(dict(arm=arm, condition=condition,
                **{k: float(np.mean([r[k] for r in part])) for k in METRICS}, identity=identity))
    summaries = {(r['arm'], r['condition']): r for r in aggregate}
    all_identity = {a: pooled([r for r in identities if r['arm'] == a]) for a in arms}
    decisions = []
    for arm in CANDIDATES:
        difference = {c: summaries[arm,c]['ospa'] - summaries[BASE,c]['ospa'] for c in CONDITIONS}
        reduction = {c: -difference[c] / summaries[BASE,c]['ospa'] for c in CONDITIONS}
        quality = all_identity[arm]['2']['wrong_pair_rate'] <= all_identity[BASE]['2']['wrong_pair_rate']
        margin = all(r >= .01 for r in reduction.values())
        decisions.append(dict(arm=arm, mean_ospa=float(np.mean([summaries[arm,c]['ospa'] for c in CONDITIONS])),
            ospa_difference=difference, ospa_relative_reduction=reduction,
            at_least_one_percent_both_conditions=margin, pooled_identity=all_identity[arm],
            wrong_pair_rate_nonincreasing=quality, eligible=bool(margin and quality)))
    ranking = sorted(decisions, key=lambda r: (r['mean_ospa'], CANDIDATES.index(r['arm'])))
    eligible = [r for r in ranking if r['eligible']]
    selected = eligible[0] if eligible else None
    result = dict(passed=True, created_utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha(OUT / 'CANDIDATES_V3.md'),
        selection_rule='Require at least 1% sequence-macro OSPA reduction in each condition and nonincreasing pooled 2 m wrong-pair rate; choose lowest two-condition mean; exact ties prefer Q, N, QN.',
        rows=rows, aggregate=aggregate, identity_rows=identities,
        baseline_pooled_identity=all_identity[BASE], ranking=ranking, selected=selected,
        advance=selected is not None, inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), OUT / 'identity_metrics.py', OUT / 'diagnose_association_v2.py']},
        exposure='All development data were already exposed. No additional V2X test outcome has been read.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / 'screened_development_comparison.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)
    print('SCREENED DEVELOPMENT SELECTION', json.dumps(selected, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
