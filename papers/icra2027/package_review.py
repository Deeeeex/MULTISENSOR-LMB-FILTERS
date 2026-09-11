"""Package the active GCE manuscript and portable evidence, excluding old drafts."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
files = [HERE / name for name in [
    'main.tex', 'references.bib', 'ieeeconf.cls', 'IEEEtran.bst', 'README_CN.md',
    'LITERATURE_SCOPE.md', 'FIGURE_CONTRACT.md', 'ANALYSIS_PROTOCOL.md', 'build.py', 'qa.py', 'package_review.py',
    'build_bibliography.py', 'prepare_gaussian_evidence.py', 'make_gaussian_tables.py',
    'make_gaussian_figures.py', 'make_main_figure_integrated.py', 'make_intro_figure.py',
    'prepare_mechanism_analysis.py', 'make_mechanism_results.py',
    'prepare_reviewer_evidence.py', 'make_reviewer_results.py', 'reviewer_artifact_qa.py', 'verify_portable_rebuild.py',
    'REVIEW_REVISION_CN.md', 'FOLLOWUP_REVISION_CN.md', 'prepare_followup_evidence.py',
    'make_followup_results.py', 'followup_artifact_qa.py']]
for folder in ['sections', 'literature', 'official_template', 'main_figure_integrated', 'intro_design']:
    files.extend(p for p in (HERE / folder).rglob('*') if p.is_file()
                 and '__pycache__' not in p.parts and p.suffix != '.pyc')
files.extend(HERE / 'generated' / name for name in [
    'numbers.tex', 'facts.json', 'main_table.tex', 'ablation_table.tex', 'communication_table.tex',
    'mechanism_numbers.tex', 'mechanism_facts.json', 'fixed_input_table.tex',
    'reviewer_numbers.tex', 'reviewer_facts.json', 'new_data_table.tex',
    'followup_numbers.tex', 'followup_facts.json', 'followup_table.tex'])
for name in ['intro', 'overview', 'gaussian_paired', 'gaussian_components',
             'gaussian_communication', 'gaussian_sequence_differences', 'gaussian_phases', 'gaussian_robustness']:
    files.extend(HERE / 'figures' / (name+suffix) for suffix in ['.svg', '.pdf', '.png', '_text_bounds.json'])
snapshots = json.loads((HERE / 'source_data/gaussian_source_manifest.json').read_text())
files.extend(HERE / 'source_data' / name for name in [*snapshots,
    'gaussian_source_manifest.json', 'gaussian_paper_evidence.json', 'overview_schematic.json',
    'gaussian_paired.json', 'gaussian_components.json', 'gaussian_communication.json', 'gaussian_sequence_differences.json',
    'intro.json', 'gaussian_phases.json', 'mechanism_diagnostic_snapshot.json', 'mechanism_analysis.json'])
reviewer_snapshots = json.loads((HERE / 'source_data/reviewer_source_manifest.json').read_text())
files.extend(HERE / 'source_data/reviewer_revision' / name for name in reviewer_snapshots)
files.extend(HERE / 'source_data' / name for name in ['reviewer_source_manifest.json', 'reviewer_evidence.json', 'gaussian_robustness.json'])
followup_snapshots = json.loads((HERE / 'source_data/followup_source_manifest.json').read_text())
files.extend(HERE / 'source_data/admission_followup' / name for name in followup_snapshots)
files.extend(HERE / 'source_data' / name for name in ['followup_source_manifest.json', 'followup_evidence.json'])
pdf = HERE / 'output/pdf/icra2027_draft.pdf'
files.extend([pdf, HERE / 'output/qa/artifact_qa.json'])
pdf_hash = hashlib.sha256(pdf.read_bytes()).hexdigest()
visual = HERE / 'output/qa/visual_review.md'
if visual.exists() and pdf_hash in visual.read_text():
    files.append(visual)
for name in ['final_submission_check.md', 'final_submission_check.json']:
    preflight = HERE / 'output/qa' / name
    if preflight.exists() and pdf_hash in preflight.read_text():
        files.append(preflight)
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
