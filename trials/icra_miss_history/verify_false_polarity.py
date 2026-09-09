"""Reproduce the saved post-hoc polarity diagnostic and bind it to its inputs."""
from pathlib import Path
import gzip
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
diagnosis_path = OUT / 'FALSE_SUPPORT_DIAGNOSTIC.json'
diagnosis = json.loads(diagnosis_path.read_text())
assert diagnosis['passed']
assert diagnosis['source_sha256'] == sha(OUT / 'diagnose_false_support.py')
inputs = dict(diagnosis['inputs'])
inputs[str(diagnosis_path.relative_to(ROOT))] = sha(diagnosis_path)
rows = []
for case in diagnosis['cases']:
    sequence, condition = case['sequence'], case['condition']
    path = OUT / 'results/miss_history_screen' / f'{sequence}_{condition}_marked_gaussian_evidence_miss_history.json.gz'
    with gzip.open(path, 'rt') as handle:
        data = json.load(handle)
    index = {tuple(row[:4].astype(int)): row for row in np.asarray(data['runs']['iterationRecords'], float).reshape(-1, 60)}
    ratios = []
    for item in diagnosis['rows']:
        if item['sequence'] != sequence or item['condition'] != condition or not any(item['attenuated_admitted_negative_sources']):
            continue
        row = index[(item['frame'], item['robot'], *item['label'])]
        delta, kappa = row[19:21], row[52:54]
        rho = np.asarray(item['history_discounts'])
        assert np.isfinite(rho).all() and (rho > 0).all() and (rho <= 1).all()
        positive = float(sum(kappa * np.maximum(delta, 0)))
        negative = float(-sum((kappa / rho) * np.minimum(delta, 0)))
        assert negative > 0
        ratios.append(positive / negative)
    rows.append(dict(sequence=sequence, condition=condition, rows=len(ratios),
                     already_positive_extra=sum(value >= 1 for value in ratios),
                     zero_positive_extra=sum(value < 1e-9 for value in ratios),
                     positive_over_nominal_negative_quantiles=np.quantile(ratios, [0, .25, .5, .75, 1]).tolist()))
saved_path = OUT / 'FALSE_EXTRA_POLARITY_DIAGNOSTIC.json'
saved = json.loads(saved_path.read_text())
assert saved['passed'] and saved['rows'] == rows
inputs[str(saved_path.relative_to(ROOT))] = sha(saved_path)
for name, expected in inputs.items():
    assert sha(ROOT / name) == expected, name
answer = dict(passed=True, rows=sum(row['rows'] for row in rows), cases=len(rows),
              exact_saved_result_reproduced=True, inputs=inputs,
              source_sha256=sha(Path(__file__)),
              scope='Post-outcome fixed visited inputs; no candidate recursion or selection change.')
destination = OUT / 'FALSE_POLARITY_VERIFICATION.json'
assert not destination.exists()
destination.write_text(json.dumps(answer, indent=2) + '\n')
print('FALSE POLARITY VERIFIED', answer['rows'], 'rows', flush=True)
