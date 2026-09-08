"""Fix branch-supported evidence and preceding complete results before preflight."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (OUT / 'source_sha256.json').exists()
    source = json.loads((OUT.parent / 'icra_selective_innovation/source_sha256.json').read_text())
    for name, value in source.items():
        assert sha(ROOT / name) == value, name
    files = list(OUT.glob('*.m')) + [OUT / n for n in ['PROTOCOL.md', 'make_core.py', 'freeze_source.py']]
    files += [OUT / 'PREFLIGHT_EMPTY_SHAPE_FIX.md'] + list((OUT / 'initial_preflight').iterdir())
    files += [OUT.parent / p for p in ['icra_selective_innovation/summary_development.json',
                                      'icra_signed_diagnostic/summary_negative.json']]
    source.update({str(p.relative_to(ROOT)): sha(p) for p in files})
    (OUT / 'source_sha256.json').write_text(json.dumps(source, indent=2, sort_keys=True) + '\n')
    print('Asymmetric evidence source fixed before preflight:', len(source), 'files.')


if __name__ == '__main__':
    main()
