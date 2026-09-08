"""Freeze bit-preserving codec after unit validation and before any replay."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PARENT = OUT.parent / 'icra_gaussian_evidence'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'source_sha256.json'
    assert not target.exists()
    source = json.loads((PARENT / 'source_sha256.json').read_text())
    parent = json.loads((PARENT / 'ROUND_FREEZE.json').read_text())
    source.update(parent['source_and_evidence_sha256'])
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    files = list(OUT.glob('*.m')) + [OUT / n for n in ['PROTOCOL.md', 'make_replay.py', 'freeze_source.py']]
    files += [PARENT / n for n in ['summary_development.json', 'ROUND_FREEZE.json', 'CURRENT_QA_development.json']]
    files += [OUT.parent / 'icra_gaussian_packet_diagnostic/summary_development.json']
    source.update({str(p.relative_to(ROOT)): sha(p) for p in files})
    target.write_text(json.dumps(source, indent=2, sort_keys=True)+'\n')
    print('Gaussian zero codec source fixed:', len(source), 'files.')


if __name__ == '__main__':
    main()
