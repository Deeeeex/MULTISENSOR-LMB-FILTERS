"""Preserve the combination and preceding negative evidence before preflight."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (OUT / 'source_sha256.json').exists(), 'Never replace an executed source snapshot.'
    source = json.loads((OUT.parent / 'icra_tempered_iteration/source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    files = [OUT / name for name in ['PROTOCOL.md', 'make_runner.py', 'runMarkedJointReplay.m', 'freeze_source.py']]
    files += [OUT.parent / name for name in ['icra_tempered_iteration/summary_development.json',
                                            'icra_absence_diagnostic/summary_absence.json',
                                            'icra_evidence_iteration/summary_development.json']]
    source.update({str(p.relative_to(ROOT)): sha(p) for p in files})
    (OUT / 'source_sha256.json').write_text(json.dumps(source, indent=2, sort_keys=True) + '\n')
    print('Marked joint source fixed before preflight:', len(source), 'files.')


if __name__ == '__main__':
    main()
