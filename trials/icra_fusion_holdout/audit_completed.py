"""Audit completed sequences out of order with the unchanged main auditor.

This creates only per-sequence caches. The main auditor still requires all
25 registered sequences before computing any cohort comparison.
"""
import json
import re
from pathlib import Path
import analyze_holdout as audit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def main():
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    source = json.loads((OUT / 'source_sha256_port.json').read_text())
    for name, expected in {**source, **freeze['source_and_selection_evidence_sha256']}.items():
        assert audit.digest(ROOT / name) == expected, name
    input_audit = json.loads((OUT / 'input_audit.json').read_text())
    assert input_audit['method_freeze_sha256'] == audit.digest(OUT / 'METHOD_FREEZE.json')
    assert input_audit['auditor_sha256'] == audit.digest(OUT / 'audit_holdout_inputs.py')
    mapped = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in source.items()}
    assert len(mapped) == len(source)
    attempts = json.loads((OUT / 'holdout_runtime.json').read_text())
    complete = {r['sequence'] for r in attempts if r['returncode'] == 0 and r['completion_line'] and r['files'] == 32}
    entries = json.loads((OUT / 'input_manifest.json').read_text())['sequences']
    cache = OUT / 'audit_sequences'
    cache.mkdir(exist_ok=True)
    count = 0
    for entry in entries:
        name = entry['sequence']
        path = cache / f'{name}.json'
        if int(name) not in complete or path.exists():
            continue
        result = audit.audit_sequence(entry, freeze, mapped)
        temporary = cache / f'{name}.next'
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        temporary.replace(path)
        count += 1
        print('ADDITIONAL COMPLETE SEQUENCE AUDITED', name, result['audited_node_frames'], 'node-frames', flush=True)
    print('Completed', count, 'additional sequence audits; no cohort statistics before all 25.', flush=True)


if __name__ == '__main__':
    main()
