"""Rebuild the verified source ZIP in a fresh directory without native data."""
from pathlib import Path, PurePosixPath
import datetime
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile

import fitz

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    archive = HERE / 'output/icra2027_review_source.zip'
    archive_digest = sha(archive)
    parent = Path(tempfile.mkdtemp(prefix='icra-reviewer-rebuild-'))
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None
        names = bundle.namelist()
        assert len(names) == len(set(names))
        assert all(name.startswith('icra2027/') and '..' not in PurePosixPath(name).parts
                   and '\\' not in name for name in names)
        manifest = json.loads(bundle.read('icra2027/output/review_manifest.json'))
        assert set(names) == {'icra2027/' + name for name in manifest} | {'icra2027/output/review_manifest.json'}
        for name, digest in manifest.items():
            assert hashlib.sha256(bundle.read('icra2027/' + name)).hexdigest() == digest
            assert sha(HERE / name) == digest, ('Source changed after packaging', name)
        bundle.extractall(parent)
    root = parent / 'icra2027'
    assert not (root.parents[1] / 'trials').exists()
    inputs = {name: digest for name, digest in manifest.items() if not name.startswith('output/')}
    input_digest = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    print('PORTABLE REBUILD START', root, len(inputs), 'source inputs', flush=True)
    with (parent / 'rebuild.log').open('w') as stream:
        process = subprocess.run([sys.executable, 'build.py', '--regenerate'], cwd=root,
                                 stdout=stream, stderr=subprocess.STDOUT)
    assert process.returncode == 0, ('Fresh rebuild failed; inspect', parent / 'rebuild.log')
    original_pdf = HERE / 'output/pdf/icra2027_draft.pdf'
    rebuilt_pdf = root / 'output/pdf/icra2027_draft.pdf'
    source_document, rebuilt_document = fitz.open(original_pdf), fitz.open(rebuilt_pdf)
    assert len(source_document) == len(rebuilt_document) == 8
    rasters = []
    for old, new in zip(source_document, rebuilt_document):
        assert old.get_text() == new.get_text()
        old_pix = old.get_pixmap(matrix=fitz.Matrix(1.5, 1.5), alpha=False)
        new_pix = new.get_pixmap(matrix=fitz.Matrix(1.5, 1.5), alpha=False)
        assert old_pix.width == new_pix.width and old_pix.height == new_pix.height
        assert old_pix.samples == new_pix.samples
        rasters.append(hashlib.sha256(old_pix.samples).hexdigest())
    source_qa = json.loads((HERE / 'output/qa/artifact_qa.json').read_text())
    rebuilt_qa = json.loads((root / 'output/qa/artifact_qa.json').read_text())
    assert {key: value for key, value in source_qa.items() if key != 'pdf_sha256'} == {
        key: value for key, value in rebuilt_qa.items() if key != 'pdf_sha256'}
    assert all(sha(root / name) == digest for name, digest in inputs.items())
    assert all(sha(HERE / name) == digest for name, digest in inputs.items()), 'Root inputs changed during verification'
    assert sha(original_pdf) == source_qa['pdf_sha256'] == manifest['output/pdf/icra2027_draft.pdf']
    generated = [name for name in inputs if name.startswith(('generated/', 'figures/'))
                 or name in ['references.bib', 'source_data/gaussian_paper_evidence.json',
                             'source_data/mechanism_analysis.json', 'source_data/reviewer_evidence.json',
                             'source_data/gaussian_robustness.json']]
    result = dict(status='passed', verified_on=datetime.date.today().isoformat(),
        source_pdf_sha256=sha(original_pdf), rebuilt_pdf_sha256=sha(rebuilt_pdf),
        pdf_binary_identical=sha(original_pdf) == sha(rebuilt_pdf), pages=8, body_pages=7,
        acknowledgment_reference_pages=1, pdf_text_identical=True, pixel_identical_page_previews=8,
        page_raw_rgb_sha256_1_5x=rasters, bibliography_entry_count=source_qa['bibliography_entry_count'],
        verified_doi_identifiers_rendered=source_qa['verified_doi_identifiers_rendered'],
        manuscript_figures=source_qa['manuscript_figures'], manuscript_tables=source_qa['manuscript_tables'],
        automated_qa_facts_identical=True, reviewer_experiments=source_qa['reviewer_experiments'],
        regenerated_file_hash_matches=len(generated), regenerated_files=sorted(generated),
        build_input_manifest_sha256=input_digest, tested_archive_sha256_before_attaching_this_report=archive_digest,
        archive_input_files_verified=len(manifest), non_output_files_identical_after_rebuild=len(inputs),
        native_experiment_corpus_available=False, image_generation_or_tracing_required_for_rebuild=False,
        extracted_directory=str(root), python_executable=sys.executable, command=['build.py', '--regenerate'],
        returncode=process.returncode,
        scope='Fresh-directory paper rebuild with saved audited sequence values and vector sources, using the same local runtime and fonts. All page text and RGB pixels agree. This does not rerun detector inference or native tracking.')
    (HERE / 'output/qa/portable_rebuild.json').write_text(json.dumps(result, indent=2) + '\n')
    subprocess.run([sys.executable, str(HERE / 'package_review.py')], cwd=HERE, check=True)
    final_manifest = json.loads((HERE / 'output/review_manifest.json').read_text())
    assert final_manifest['output/qa/portable_rebuild.json'] == sha(HERE / 'output/qa/portable_rebuild.json')
    assert {name: digest for name, digest in final_manifest.items() if not name.startswith('output/')} == inputs
    print('PORTABLE REBUILD PASSED: all eight pages have identical text and RGB pixels;',
          len(inputs), 'unchanged source inputs; final ZIP contains the report.', flush=True)


if __name__ == '__main__':
    main()
