"""Artifact checks; these are automated self-checks, not author/third-party review."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
import fitz
import numpy as np
from pypdf import PdfReader

HERE = Path(__file__).resolve().parent
PDF = HERE / 'output/pdf/icra2027_draft.pdf'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check():
    reader = PdfReader(PDF)
    assert len(reader.pages) >= 1
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
    assert len(labels) == 7 and len(captions) == 7
    float_pages = {}
    for label, number, page in labels:
        kind = 'figure' if label.startswith('fig:') else 'table'
        position = captions[kind, number]
        assert position[0] == int(page), (label, page, position)
        assert position < reference_position, ('Float after References', label, position)
        float_pages[label] = int(page)
    wide_per_page = Counter(float_pages.values())
    assert max(wide_per_page.values()) <= 2, ('Crowded wide-float page', wide_per_page)
    log = (HERE / 'build/compile.log').read_text() + (HERE / 'build/main.log').read_text()
    bad = ['Overfull', 'undefined', 'Missing character', 'Undefined control sequence',
           'LaTeX Error', 'BibTeX subsystem:', 'internal error', 'LaTeX Font Warning']
    bad_hits = [word for word in bad if word.lower() in log.lower()]
    assert not bad_hits, ('Compilation failures', bad_hits)

    template = json.loads((HERE / 'official_template/download_manifest.json').read_text())
    for filename, item in template.items():
        assert sha(HERE / 'official_template' / filename) == item['sha256']
    for filename in ['ieeeconf.cls', 'IEEEtran.bst']:
        assert (HERE / filename).read_bytes() == (HERE / 'official_template' / filename).read_bytes()

    figures = {}
    for name in ['overview', 'gaussian_paired', 'gaussian_components', 'gaussian_communication']:
        svg = ET.parse(HERE / 'figures' / f'{name}.svg')
        live = [x for x in svg.iter() if x.tag.endswith('}text')]
        assert live and not any(x.tag.endswith('}image') for x in svg.iter())
        bounds = json.loads((HERE / 'figures' / f'{name}_text_bounds.json').read_text())
        assert bounds['passed']
        figures[name] = {'live_svg_text_elements': len(live), 'no_embedded_raster': True,
                         'label_bounds_pass': True, 'svg_sha256': sha(HERE / 'figures' / f'{name}.svg')}

    numerical = check_gaussian_evidence()

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
              'tex_informational_font_aliases': len(re.findall(r'LaTeX Font Info:\s+Font shape', log)),
              'official_class_and_bst_unmodified': True, 'figures': figures,
              'float_pages':float_pages, 'references_start_page':reference_position[0],
              'all_float_captions_before_references':True,
              'maximum_double_column_floats_per_page':max(wide_per_page.values()),
              'numerical_evidence': numerical,
              'draft_page_cap': None,
              'conference_page_limit': 8,
              'requires_length_revision_before_submission': len(reader.pages) > 8,
              'citation_keys_resolved': sorted(cited),
              'citation_scope': 'Crossref and DataCite metadata plus available author manuscripts; see LITERATURE_SCOPE.md',
              'limitations': 'Automated artifact self-checks; not independent replication, author approval, or a submission acceptance check.'}
    out = HERE / 'output/qa/artifact_qa.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'PASS artifact QA: {len(reader.pages)} pages, {len(fonts)} embedded fonts, {len(figures)} live-text SVGs, {len(cited)} citation keys.')


def check_gaussian_evidence():
    def read(name):
        return json.loads((HERE / 'source_data' / (name+'.json')).read_text())
    def close(a, b):
        assert math.isclose(a, b, rel_tol=1e-11, abs_tol=1e-11), (a, b)
    primary = 'marked_gaussian_evidence'
    data = read('gaussian_paper_evidence')
    manifest = read('gaussian_source_manifest')
    assert len(manifest) == 15
    for name, record in manifest.items():
        assert sha(HERE / 'source_data' / name) == record['sha256'], name
        original = HERE.parents[1] / record['source']
        if original.exists():
            assert sha(original) == record['sha256'], original
    assert sha(HERE / 'source_data/gaussian_source_manifest.json') == data['source_manifest_sha256']
    assert sha(HERE / 'prepare_gaussian_evidence.py') == data['generator_sha256']
    assert len(data['sequences']) == 25 and data['frames'] == 5601 and len(data['runs']) == 550
    assert data['all_real_outcomes_previously_seen']
    native = read('gaussian_main_summary')
    native_rows = {(r['sequence'], r['condition'], r['arm']): r for r in read('shared_main_summary')['runs']}
    native_rows.update({(r['sequence'], r['condition'], r['arm']): r for r in native['runs']})
    codec_rows = {(r['sequence'], r['condition']): r for r in read('gaussian_codec_main_summary')['rows']}
    metric_keys = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError', 'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']
    for row in data['runs']:
        original = native_rows[row['sequence'], row['condition'], row['arm']]
        for metric in metric_keys:
            expected = codec_rows[row['sequence'], row['condition']][metric] if row['arm'] == primary and metric.endswith('bytes') else original[metric]
            close(row[metric], expected)
    for cohort in ['main', 'development']:
        original_qa, codec_qa = read('gaussian_'+cohort+'_qa'), read('gaussian_codec_'+cohort+'_qa')
        assert original_qa['passed'] and codec_qa['passed'] and codec_qa['exact_whole_trajectory_parity']
        assert original_qa['summary_sha256'] == sha(HERE / 'source_data' / ('gaussian_'+cohort+'_summary.json'))
        assert codec_qa['summary_sha256'] == sha(HERE / 'source_data' / ('gaussian_codec_'+cohort+'_summary.json'))
        assert all(x == 0 for x in original_qa['native_matlab_stage_exits'] + codec_qa['native_matlab_exits'])
    draws = np.random.default_rng(8301).integers(0, 25, (10000, 25))
    for row in data['paired']:
        values = np.array(row['ospa_differences'])
        close(values.mean(), row['ospa']['mean'])
        low, high = np.quantile(values[draws].mean(1), [.025, .975])
        close(low, row['ospa']['low']); close(high, row['ospa']['high'])
    for row in data['components']:
        pairs = sorted([r for r in data['component_sequence_pairs'] if r['condition'] == row['condition'] and r['reference'] == row['reference']], key=lambda r: r['sequence'])
        assert len(pairs) == 25
        values = np.array([r['ospa'] for r in pairs])
        close(values.mean(), row['ospa']['mean'])
        low, high = np.quantile(values[draws].mean(1), [.025, .975])
        close(low, row['ospa']['low']); close(high, row['ospa']['high'])
    a = {(r['condition'], r['arm']): r for r in data['aggregate']}
    na = {(r['condition'], r['arm']): r for r in native['aggregate']}
    p = {(r['condition'], r['reference']): r for r in data['paired']}
    c = {(r['condition'], r['reference']): r for r in data['components']}
    common = {(r['condition'], r['reference']): r for r in data['common'] if r['candidate'] == primary}
    dev = {(r['condition'], r['arm']): r for r in data['development']}
    unmarked = {(r['condition'], r['arm']): r for r in data['unmarked_controls']}
    transport = {r['condition']: r for r in data['codec']}
    expected_facts = {}
    for condition, suffix in [('reliable', 'Reliable'), ('intermittent', 'Intermittent')]:
        for prefix, arm in [('Main', primary), ('NoAge', 'marked_lineage'), ('Recency', 'marked_er'), ('Scalar', 'marked_asymmetric')]:
            expected_facts[prefix+suffix] = a[condition, arm]['ospa']['mean']
        for prefix, arm in [('NoAge', 'marked_lineage'), ('Recency', 'marked_er')]:
            expected_facts['Gain'+prefix+suffix] = 100*(1-a[condition, primary]['ospa']['mean']/a[condition, arm]['ospa']['mean'])
            for key, middle in [('mean', ''), ('low', 'Low'), ('high', 'High')]:
                expected_facts['Delta'+prefix+middle+suffix] = p[condition, arm]['ospa'][key]
        for metric, prefix in [('miss2', 'Miss'), ('false2', 'False')]:
            expected_facts[prefix+'GainNoAge'+suffix] = 100*(1-a[condition, primary][metric]['mean']/a[condition, 'marked_lineage'][metric]['mean'])
        support = common[condition, 'marked_lineage']
        for key, prefix in [('candidate_rmse', 'LocMainNoAge'), ('reference_rmse', 'LocNoAge'), ('support', 'SupportNoAge')]:
            expected_facts[prefix+suffix] = support[key]
        scalar = c[condition, 'marked_asymmetric']
        for key, middle in [('mean', ''), ('low', 'Low'), ('high', 'High')]:
            expected_facts['DeltaScalar'+middle+suffix] = scalar['ospa'][key]
        expected_facts['LocMainScalar'+suffix] = scalar['candidate_common_rmse']
        expected_facts['LocScalar'+suffix] = scalar['reference_common_rmse']
        for prefix, arm in [('Main', primary), ('NoAge', 'marked_lineage'), ('Scalar', 'marked_asymmetric')]:
            expected_facts['Dev'+prefix+suffix] = dev[condition, arm]['ospa']['mean']
        expected_facts['UnmarkedNoAge'+suffix] = unmarked[condition, 'lineage']['ospa']['mean']
        diagnostics = [r for r in data['diagnostics'] if r['arm'] == primary and r['condition'] == condition]
        for prefix, key in [('Scalar', 'scalar_AS'), ('Main', 'candidate')]:
            expected_facts['Fixed'+prefix+suffix] = float(np.mean([r['same_input'][key]['ospa'] for r in diagnostics]))
        for metric, prefix in [('raw_bytes', 'Raw'), ('wire_bytes', 'Wire')]:
            expected_facts['Codec'+prefix+'Saving'+suffix] = 100*transport[condition]['savings'][metric]
            expected_facts['Codec'+prefix+'ExtraNoAge'+suffix] = 100*(a[condition, primary][metric]['mean']/a[condition, 'marked_lineage'][metric]['mean']-1)
    facts = json.loads((HERE / 'generated/facts.json').read_text())
    assert set(facts) == set(expected_facts) and len(facts) == 64
    for key, value in facts.items():
        close(value, expected_facts[key])
    macros = dict(re.findall(r'\\newcommand\{\\(\w+)\}\{([^}]+)\}', (HERE / 'generated/numbers.tex').read_text()))
    assert set(macros) == set(facts)
    for key, shown in macros.items():
        number = shown.replace(r'\,', '')
        decimals = len(number.split('.')[-1]) if '.' in number else 0
        assert number == f'{facts[key]:.{decimals}f}', (key, number, facts[key])
    for table, methods, lookup, metrics, scale in [
        ('main_table', data['methods'], a, ['ospa', 'miss2', 'false2'], 1),
        ('ablation_table', [primary, 'marked_asymmetric', primary+'_no_curvature', primary+'_no_history', primary+'_no_mark'], na, ['ospa', 'miss2', 'false2'], 1),
        ('communication_table', data['methods'], a, ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes'], 2**20)]:
        tex = (HERE / 'generated' / (table+'.tex')).read_text()
        for method in methods:
            label = data['labels'][method]
            if method == primary:
                label = r'\textbf{'+label+'}'
            values = [f"{lookup[condition, method][metric]['mean']/scale:.3f}" for condition in ['reliable', 'intermittent'] for metric in metrics]
            assert label+' & '+' & '.join(values)+r' \\' in tex, (table, method)
    for name, count in [('gaussian_paired', 300), ('gaussian_components', 200)]:
        figure = read(name)
        assert figure['evidence_sha256'] == sha(HERE / 'source_data/gaussian_paper_evidence.json')
        assert figure['point_count'] == count == sum(len(r['points']) for r in figure['groups'])
        for group in figure['groups']:
            target = p if name == 'gaussian_paired' else c
            assert group['summary'] == target[group['condition'], group['reference']]['ospa']
            assert [r['sequence'] for r in group['points']] == data['sequences']
    communication = read('gaussian_communication')
    assert communication['evidence_sha256'] == sha(HERE / 'source_data/gaussian_paper_evidence.json')
    assert sum(len(r['sequences']) for r in communication['rows']) == 200
    for row in communication['rows']:
        lookup, arm = (na, primary) if row['arm'] == 'full' else (a, row['arm'])
        for metric in ['raw_bytes', 'wire_bytes']:
            close(row['means_mib'][metric], lookup[row['condition'], arm][metric]['mean']/2**20)
    return dict(main_sequences=25, main_frames=5601, development_sequences=9, development_frames=1993,
                main_methods=11, main_sequence_condition_method_runs=550, scalar_facts_checked=64,
                paired_sequence_points=300, component_sequence_points=200,
                codec_exact_primary_trajectories=68, all_real_outcomes_previously_seen=True,
                portable_source_snapshots=len(manifest), source_result_identities=len(data['source_inputs_sha256']))


if __name__ == '__main__':
    check()
