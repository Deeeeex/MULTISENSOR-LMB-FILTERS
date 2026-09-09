"""Freeze the original-nine GCE instrumentation and input identities."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import re

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
previous = REVIEW / 'stages/controls_development.json'
prior = json.loads(previous.read_text())
sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
for name, expected in sources.items():
    assert sha(ROOT / name) == expected, name
units = prior['units']
for unit in units:
    for field in ['data_path', 'marks_path', 'ratios_path']:
        if field in unit:
            name = unit[field]
            assert sha(ROOT / name) == prior['source_sha256'][name], name
            sources[name] = sha(ROOT / name)
paths = [OUT / p for p in ['PROTOCOL.md', 'directObservationSummary.m', 'checkDirectObservationSummary.m',
    'recordAssociationReplay.m', 'make_instrumentation.py', 'INSTRUMENTATION_PATCH.json',
    'register_instrumentation.py', 'run_instrumentation.py', 'audit_instrumentation.py', 'ASSOCIATION_DIAGNOSTIC.json']]
paths += [previous, REVIEW / 'runReviewerReplay.m', REVIEW / 'checkReviewerEvidence.m',
          REVIEW / 'fuseReviewerEvidence.m', REVIEW / 'review_probability_audit.py', REVIEW / 'review_gaussian_audit.py']
sources.update({str(p.relative_to(ROOT)): sha(p) for p in paths})
mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
assert len(set(mapped)) == len(mapped)
cfg = dict(protocol='icra-temporal-instrumentation-v1', stage='association_instrumentation_development',
    cohort='development', created_utc=datetime.now(timezone.utc).isoformat(),
    arms=['marked_gaussian_evidence'], pd=.9, preflight=True,
    conditions=['reliable', 'intermittent'], units=units, source_sha256=sources)
path = OUT / 'stages' / (cfg['stage'] + '.json')
assert not path.exists()
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(cfg, indent=2) + '\n')
print('REGISTERED original-nine instrumentation', len(units), 'sequences', len(units) * 2, 'outputs')
