"""Freeze the Gaussian evidence implementation after units and before tracking."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'source_sha256.json'
    assert not target.exists()
    source = json.loads((OUT.parent / 'icra_asymmetric_evidence/source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    files = list(OUT.glob('*.m')) + [OUT / name for name in
             ['PROTOCOL.md', 'make_core.py', 'freeze_source.py', 'UNIT_NUMERICAL_NOTE.md']]
    files += list((OUT / 'initial_unit').iterdir())
    files += [OUT.parent / 'icra_asymmetric_evidence/summary_development.json']
    source.update({str(path.relative_to(ROOT)): sha(path) for path in files})
    target.write_text(json.dumps(source, indent=2, sort_keys=True) + '\n')
    print('Gaussian evidence source fixed before tracking:', len(source), 'files.')


if __name__ == '__main__':
    main()
