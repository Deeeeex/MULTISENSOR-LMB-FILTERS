"""Artifact checks; these are automated self-checks, not author/third-party review."""
from pathlib import Path
import csv
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
import fitz
from pypdf import PdfReader

HERE = Path(__file__).resolve().parent
PDF = HERE / 'output/pdf/icra2027_draft.pdf'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check():
    reader = PdfReader(PDF)
    assert 1 <= len(reader.pages) <= 8
    assert not reader.is_encrypted
    assert not reader.metadata.author
    fonts, seen = {}, set()

    def resources(ref):
        if not ref:
            return
        obj = ref.get_object()
        key = id(obj)
        if key in seen:
            return
        seen.add(key)
        for font_ref in obj.get('/Font', {}).get_object().values() if '/Font' in obj else []:
            font = font_ref.get_object()
            target = font['/DescendantFonts'][0].get_object() if '/DescendantFonts' in font else font
            descriptor = target.get('/FontDescriptor', {})
            descriptor = descriptor.get_object() if hasattr(descriptor, 'get_object') else descriptor
            entry = {'subtype': str(font.get('/Subtype')),
                     'embedded': any(k in descriptor for k in ['/FontFile', '/FontFile2', '/FontFile3'])}
            assert entry['embedded'] and entry['subtype'] != '/Type3', (font, entry)
            fonts[str(font.get('/BaseFont'))] = entry
        xobjects = obj.get('/XObject', {})
        xobjects = xobjects.get_object() if hasattr(xobjects, 'get_object') else xobjects
        for xobject in xobjects.values():
            resources(xobject.get_object().get('/Resources'))

    for page in reader.pages:
        assert abs(float(page.mediabox.width) - 612) < .1
        assert abs(float(page.mediabox.height) - 792) < .1
        assert not page.get('/Annots')
        resources(page.get('/Resources'))
    assert fonts
    document = fitz.open(PDF)
    texts, span_count = [], 0
    preview = HERE / 'build/preview'
    preview.mkdir(parents=True, exist_ok=True)
    for number, page in enumerate(document, 1):
        texts.append(page.get_text())
        for block in page.get_text('dict')['blocks']:
            for line in block.get('lines', []):
                for span in line['spans']:
                    x0, y0, x1, y1 = span['bbox']
                    assert x0 >= -.5 and y0 >= -.5 and x1 <= 612.5 and y1 <= 792.5, (number, span)
                    span_count += 1
        page.get_pixmap(matrix=fitz.Matrix(1.5, 1.5), alpha=False).save(preview / f'page-{number}.png')
    full_text = '\n'.join(texts)
    assert '??' not in full_text
    assert 'OpenAI Codex' in full_text and 'Acknowledgment' in full_text.replace('ACKNOWLEDGMENT', 'Acknowledgment')
    (HERE / 'build/main.txt').write_text(full_text)
    log = (HERE / 'build/compile.log').read_text() + (HERE / 'build/main.log').read_text()
    bad = ['Overfull', 'undefined', 'Missing character', 'Undefined control sequence',
           'LaTeX Error', 'BibTeX subsystem:', 'internal error']
    assert not any(word.lower() in log.lower() for word in bad), log

    template = json.loads((HERE / 'official_template/download_manifest.json').read_text())
    for filename, item in template.items():
        assert sha(HERE / 'official_template' / filename) == item['sha256']
    for filename in ['ieeeconf.cls', 'IEEEtran.bst']:
        assert (HERE / filename).read_bytes() == (HERE / 'official_template' / filename).read_bytes()

    figures = {}
    for name in ['overview', 'scene', 'mechanism', 'outcomes', 'time']:
        svg = ET.parse(HERE / 'figures' / f'{name}.svg')
        live = [x for x in svg.iter() if x.tag.endswith('}text')]
        assert live and not any(x.tag.endswith('}image') for x in svg.iter())
        bounds = json.loads((HERE / 'figures' / f'{name}_text_bounds.json').read_text())
        assert bounds['passed']
        figures[name] = {'live_svg_text_elements': len(live), 'no_embedded_raster': True,
                         'label_bounds_pass': True, 'svg_sha256': sha(HERE / 'figures' / f'{name}.svg')}

    source = json.loads((HERE / 'source_data/validation_summary.json').read_text())
    corpus = HERE.parents[1] / 'trials/icra_reunion_fusion/summary_validation.json'
    if corpus.exists():
        assert json.loads(corpus.read_text()) == source
    assert source['seeds'] == list(range(2901, 2921)) and len(source['arms']) == 9
    assert len(source['runs']) == 540 and source['audited_node_frames'] == 518400
    with (HERE / 'source_data/validation_runs.csv').open() as stream:
        assert len(list(csv.DictReader(stream))) == 540
    a = {(r['scene'], r['arm']): r for r in source['aggregate']}
    p = {(r['scene'], r['arm'], r['reference']): r for r in source['paired']}
    facts = json.loads((HERE / 'generated/facts.json').read_text())
    for scene, suffix in zip(['split_latebirth', 'churn_departure', 'split_no_new'], ['Split', 'Churn', 'Nonew']):
        er, reference = a[scene, 'qualified_exist']['ospa']['mean'], a[scene, 'lineage']['ospa']['mean']
        expected = {'Er': er, 'Lineage': reference, 'Gain': 100 * (1 - er / reference),
                    'Delta': p[scene, 'qualified_exist', 'lineage']['ospa']['mean'],
                    'DeltaLow': p[scene, 'qualified_exist', 'lineage']['ospa']['low'],
                    'DeltaHigh': p[scene, 'qualified_exist', 'lineage']['ospa']['high']}
        for prefix, value in expected.items():
            assert math.isclose(facts[prefix + suffix], value, rel_tol=1e-12, abs_tol=1e-12)

    tex = '\n'.join(p.read_text() for p in [HERE / 'main.tex', *(HERE / 'sections').glob('*.tex'),
                                         *(HERE / 'main_figure_integrated').glob('*.tex')])
    assert not re.search(r'\b(seed|frozen|hash|audit|v[234])\b', tex, flags=re.I)
    cited = {key.strip() for group in re.findall(r'\\cite\{([^}]+)\}', tex) for key in group.split(',')}
    available = set(re.findall(r'@\w+\{([^,]+),', (HERE / 'references.bib').read_text()))
    assert cited <= available
    records = json.loads((HERE / 'literature/verification.json').read_text())
    assert all(records[key]['verified'] for key in cited - {'lang2026adaptive'})
    assert '\\author{}' in tex
    result = {'status': 'automated_artifact_checks_passed', 'pdf_sha256': sha(PDF),
              'pages': len(reader.pages), 'paper_size': 'US Letter', 'text_spans_in_page_bounds': span_count,
              'embedded_fonts': fonts, 'type3_fonts': 0, 'pdf_annotations': 0, 'blank_author_metadata': True,
              'tex_font_substitution_warnings': 0,
              'official_class_and_bst_unmodified': True, 'figures': figures,
              'validation_seeds_per_family': 20, 'validation_arm_runs': 540,
              'audited_validation_node_frames': 518400, 'scalar_facts_match_audited_summary': True,
              'citation_keys_resolved': sorted(cited),
              'citation_scope': 'DOI metadata plus public SSRN metadata; see LITERATURE_SCOPE.md',
              'limitations': 'Automated artifact self-checks; not independent replication, author approval, or a submission acceptance check.'}
    out = HERE / 'output/qa/artifact_qa.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'PASS artifact QA: {len(reader.pages)} pages, {len(fonts)} embedded fonts, {len(figures)} live-text SVGs, {len(cited)} citation keys.')


if __name__ == '__main__':
    check()
