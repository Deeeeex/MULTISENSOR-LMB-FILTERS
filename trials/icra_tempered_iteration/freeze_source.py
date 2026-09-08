"""Record the isolated development source before any new tracking output."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (OUT / 'source_sha256.json').exists(), 'Never rewrite a source snapshot after execution.'
    source = json.loads((OUT.parent / 'icra_marked_control/source_sha256.json').read_text())
    for name, value in source.items():
        assert sha(ROOT / name) == value, name
    files = [OUT / name for name in ['PROTOCOL.md', 'make_runner.py', 'fuseTemperedRecency.m',
                                    'runTemperedReplay.m', 'checkTemperedRecency.m', 'freeze_source.py']]
    files += [OUT.parent / 'icra_fusion_holdout/summary_holdout.json',
              OUT.parent / 'icra_fusion_holdout/decision.json',
              OUT.parent / 'icra_marked_control/summary_control.json']
    source.update({str(p.relative_to(ROOT)): sha(p) for p in files})
    (OUT / 'source_sha256.json').write_text(json.dumps(source, indent=2, sort_keys=True) + '\n')
    print('Tempered source fixed before preflight:', len(source), 'files.')


if __name__ == '__main__':
    main()
