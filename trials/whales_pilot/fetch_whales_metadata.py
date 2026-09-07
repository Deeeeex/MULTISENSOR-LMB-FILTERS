"""Download the official metadata archive, then extract only validation info."""
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
CACHE = ROOT / 'tmp/whales'
CACHE.mkdir(parents=True, exist_ok=True)
ARCHIVE = CACHE / 'whales_meta.tar.zst'
SIZE = 207096707
URL = ('https://drive.usercontent.google.com/download?'
       'id=17G5RAihDVfj05IG6Nsq9Kpdu8X205D6e&export=download&confirm=t')

if not ARCHIVE.exists() or ARCHIVE.stat().st_size != SIZE:
    partial = ARCHIVE.with_suffix('.partial')
    with urllib.request.urlopen(URL, timeout=60) as response:
        if 'html' in response.headers.get('Content-Type', ''):
            raise RuntimeError('Drive returned HTML instead of the archive.')
        with partial.open('wb') as out:
            while chunk := response.read(4 * 1024 * 1024):
                out.write(chunk)
    if partial.stat().st_size != SIZE:
        raise RuntimeError('Archive size differs from the inspected release.')
    partial.replace(ARCHIVE)

# Extract the exact top-level member; no archive-provided path is trusted.
subprocess.run(['tar', '-xf', str(ARCHIVE), '-C', str(CACHE),
                'whales_infos_val.pkl'], check=True)
print(CACHE / 'whales_infos_val.pkl')
