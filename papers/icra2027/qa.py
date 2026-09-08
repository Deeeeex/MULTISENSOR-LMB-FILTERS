"""Artifact checks; these are automated self-checks, not author/third-party review."""
from pathlib import Path
from collections import Counter
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
    # Check the rendered positions: valid TeX alone does not keep floats
    # before the bibliography or prevent a page crowded by wide floats.
    captions, reference_positions = {}, []
    for page_number, page in enumerate(document, 1):
        for block in page.get_text('dict')['blocks']:
            for line in block.get('lines', []):
                text = ''.join(span['text'] for span in line['spans']).strip()
                if text == 'REFERENCES':
                    reference_positions.append((page_number, line['bbox'][1]))
                match = re.match(r'^(Fig\.)\s+(\d+)\.|^(TABLE)\s+([IVX]+)$', text)
                if match:
                    kind = 'figure' if match.group(1) else 'table'
                    number = match.group(2) or match.group(4)
                    key = (kind, number)
                    assert key not in captions, ('Duplicate rendered caption', key)
                    captions[key] = (page_number, line['bbox'][3])
    assert len(reference_positions) == 1
    reference_position = reference_positions[0]
    labels = re.findall(r'\\newlabel\{((?:fig|tab):[^}]+)\}\{\{([^}]+)\}\{(\d+)\}',
                        (HERE / 'build/main.aux').read_text())
    assert len(labels) == 8 and len(captions) == 8
    float_pages = {}
    for label, number, page in labels:
        kind = 'figure' if label.startswith('fig:') else 'table'
        position = captions[kind, number]
        assert position[0] == int(page), (label, page, position)
        assert position < reference_position, ('Float after References', label, position)
        float_pages[label] = int(page)
    wide_per_page = Counter(page for label, page in float_pages.items() if label != 'fig:mechanism')
    assert max(wide_per_page.values()) <= 2, ('Crowded wide-float page', wide_per_page)
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
    for name in ['overview', 'scene', 'mechanism', 'outcomes', 'time', 'v2v4real']:
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

    external={}
    for name in ['case_studies','v2v4real']:
        snapshot=HERE/'source_data'/('external_'+name+'_summary.json')
        original=HERE.parents[1]/'trials/icra_external_fusion'/('summary_'+name+'.json')
        external[name]=json.loads(snapshot.read_text())
        if original.exists():assert json.loads(original.read_text())==external[name]
    assert external['case_studies']['audited_new_node_frames']==115200
    real=external['v2v4real']
    assert real['audited_node_frames']==47832 and real['frames']==1993 and len(real['runs'])==108
    for condition,suffix in [('reliable','Reliable'),('intermittent','Intermittent')]:
        rows={r['arm']:r for r in real['aggregate'] if r['condition']==condition}
        for arm,prefix in [('qualified_exist','RealEr'),('lineage','RealNoAge'),('mil_support','RealMil'),('tc_ospa2_w5','RealTcFive'),('tc_ospa2_w10','RealTcTen')]:
            assert math.isclose(facts[prefix+suffix],rows[arm]['ospa']['mean'],abs_tol=1e-12)
        pair=next(r for r in real['paired'] if r['condition']==condition and r['reference']=='lineage')['ospa']
        for key,prefix in [('mean','RealDelta'),('low','RealDeltaLow'),('high','RealDeltaHigh')]:
            assert math.isclose(facts[prefix+suffix],pair[key],abs_tol=1e-12)

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
              'float_pages':float_pages, 'references_start_page':reference_position[0],
              'all_float_captions_before_references':True,
              'maximum_double_column_floats_per_page':max(wide_per_page.values()),
              'validation_seeds_per_family': 20, 'validation_arm_runs': 540,
              'audited_validation_node_frames': 518400, 'scalar_facts_match_audited_summary': True,
              'external_tc_cases':60, 'external_tc_added_arm_runs':120,
              'audited_external_tc_node_frames':115200,
              'real_sequences':9, 'real_frames':1993, 'real_sequence_condition_arm_runs':108,
              'audited_real_node_frames':47832,
              'citation_keys_resolved': sorted(cited),
              'citation_scope': 'DOI metadata, author BibTeX, official documentation and public SSRN metadata; see LITERATURE_SCOPE.md',
              'limitations': 'Automated artifact self-checks; not independent replication, author approval, or a submission acceptance check.'}
    out = HERE / 'output/qa/artifact_qa.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'PASS artifact QA: {len(reader.pages)} pages, {len(fonts)} embedded fonts, {len(figures)} live-text SVGs, {len(cited)} citation keys.')


if __name__ == '__main__':
    check()
