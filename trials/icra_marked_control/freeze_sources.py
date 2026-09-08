"""Preserve the already fixed method plus the additional control sources."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PORT = OUT.parent / 'icra_fusion_holdout'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'source_sha256.json'
    assert not target.exists(), 'Preserve the first source snapshot.'
    source = json.loads((PORT / 'source_sha256_port.json').read_text())
    freeze = json.loads((PORT / 'METHOD_FREEZE.json').read_text())
    for name, expected in {**source, **freeze['source_and_selection_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    source.update(freeze['source_and_selection_evidence_sha256'])
    for path in [PORT / 'METHOD_FREEZE.json', *sorted(OUT.glob('*.py')), *sorted(OUT.glob('*.m')), OUT / 'PROTOCOL.md']:
        source[str(path.relative_to(ROOT))] = sha(path)
    target.write_text(json.dumps(source, indent=2) + '\n')
    print('Additional control source snapshot:', len(source), 'unchanged/fixed files.')


if __name__ == '__main__':
    main()
