"""Package the manuscript and figure source data, without the large simulation corpus."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
files = [p for p in HERE.iterdir() if p.is_file() and p.name != '.gitignore']
for folder in ['sections', 'generated', 'figures', 'source_data', 'literature', 'official_template',
               'main_figure_integrated']:
    files.extend(p for p in (HERE / folder).rglob('*') if p.is_file())
files.extend(p for folder in ['output/pdf', 'output/qa'] for p in (HERE / folder).rglob('*') if p.is_file())
manifest = {str(p.relative_to(HERE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(HERE / 'output').mkdir(exist_ok=True)
manifest_path = HERE / 'output/review_manifest.json'
manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
archive = HERE / 'output/icra2027_review_source.zip'
with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
    for path in sorted(files + [manifest_path]):
        bundle.write(path, Path('icra2027') / path.relative_to(HERE))
with zipfile.ZipFile(archive) as bundle:
    assert bundle.testzip() is None
    for name, digest in manifest.items():
        assert hashlib.sha256(bundle.read('icra2027/' + name)).hexdigest() == digest
print('Verified review archive:', archive)
