"""Fetch pinned author sources into an ignored research-use cache."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]
REPOS = {
    'Distributed-limitedFoV-MOT': ('AdelaideAuto-IDLab/Distributed-limitedFoV-MOT',
                                 'b6b20ec30b7854dcee6f4a718237d82d96ac7c2a'),
    'DMSTrack': ('eddyhkchiu/DMSTrack', 'd3b9949499c8e68ea33060873bd1cb95b6d4d323'),
}


def main():
    manifest = {}
    for name, (repo, commit) in REPOS.items():
        target = ROOT/'tmp/external_baselines'/name
        if not target.exists():
            subprocess.run(['git', 'clone', 'https://github.com/'+repo+'.git', str(target)], check=True)
            subprocess.run(['git', '-C', str(target), 'checkout', '--detach', commit], check=True)
        actual = subprocess.check_output(['git', '-C', str(target), 'rev-parse', 'HEAD'], text=True).strip()
        assert actual == commit, (name, actual)
        assert not subprocess.check_output(['git', '-C', str(target), 'status', '--porcelain'], text=True).strip()
        manifest[name] = dict(url='https://github.com/'+repo, commit=commit,
                             cache=str(target.relative_to(ROOT)))
    tc=ROOT/manifest['Distributed-limitedFoV-MOT']['cache']
    manifest['Distributed-limitedFoV-MOT']['source_sha256'] = {
        str(p.relative_to(tc)): hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ['data_fusion', 'track_matching', 'misc'] for p in sorted((tc/folder).rglob('*.m'))
    }
    Path(__file__).with_name('dependency_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Pinned author repositories verified.')


if __name__ == '__main__':
    main()
