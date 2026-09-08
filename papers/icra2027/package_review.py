"""Package the active GCE manuscript and portable evidence, excluding old drafts."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
files = [HERE / name for name in [
    'main.tex', 'references.bib', 'ieeeconf.cls', 'IEEEtran.bst', 'README_CN.md',
    'LITERATURE_SCOPE.md', 'FIGURE_CONTRACT.md', 'build.py', 'qa.py', 'package_review.py',
    'build_bibliography.py', 'prepare_gaussian_evidence.py', 'make_gaussian_tables.py',
    'make_gaussian_figures.py', 'make_main_figure_integrated.py']]
for folder in ['sections', 'literature', 'official_template', 'main_figure_integrated']:
    files.extend(p for p in (HERE / folder).rglob('*') if p.is_file())
files.extend(HERE / 'generated' / name for name in [
    'numbers.tex', 'facts.json', 'main_table.tex', 'ablation_table.tex', 'communication_table.tex'])
for name in ['overview', 'gaussian_paired', 'gaussian_components', 'gaussian_communication']:
    files.extend(HERE / 'figures' / (name+suffix) for suffix in ['.svg', '.pdf', '.png', '_text_bounds.json'])
snapshots = json.loads((HERE / 'source_data/gaussian_source_manifest.json').read_text())
files.extend(HERE / 'source_data' / name for name in [*snapshots,
    'gaussian_source_manifest.json', 'gaussian_paper_evidence.json', 'overview_schematic.json',
    'gaussian_paired.json', 'gaussian_components.json', 'gaussian_communication.json'])
pdf = HERE / 'output/pdf/icra2027_draft.pdf'
files.extend([pdf, HERE / 'output/qa/artifact_qa.json'])
pdf_hash = hashlib.sha256(pdf.read_bytes()).hexdigest()
visual = HERE / 'output/qa/visual_review.md'
if visual.exists() and pdf_hash in visual.read_text():
    files.append(visual)
portable = HERE / 'output/qa/portable_rebuild.json'
if portable.exists():
    report = json.loads(portable.read_text())
    input_files = {str(p.relative_to(HERE)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in files if not str(p.relative_to(HERE)).startswith('output/')}
    input_digest = hashlib.sha256(json.dumps(input_files, sort_keys=True).encode()).hexdigest()
    if report.get('source_pdf_sha256') == pdf_hash and report.get('build_input_manifest_sha256') == input_digest:
        files.append(portable)
assert all(p.is_file() for p in files)
assert len(files) == len(set(files))
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
