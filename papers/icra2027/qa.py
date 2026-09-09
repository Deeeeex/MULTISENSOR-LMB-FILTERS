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
    assert len(reader.pages) == 8, ('Seven body pages plus one acknowledgment/reference page', len(reader.pages))
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
    captions, reference_positions, acknowledgment_positions = {}, [], []
    for page_number, page in enumerate(document, 1):
        for block in page.get_text('dict')['blocks']:
            for line in block.get('lines', []):
                text = ''.join(span['text'] for span in line['spans']).strip()
                if text == 'REFERENCES':
                    reference_positions.append((page_number, line['bbox'][1]))
                if text == 'ACKNOWLEDGMENT':
                    acknowledgment_positions.append((page_number, line['bbox'][1]))
                match = re.match(r'^(Fig\.)\s+(\d+)\.|^(TABLE)\s+([IVX]+)$', text)
                if match:
                    kind = 'figure' if match.group(1) else 'table'
                    number = match.group(2) or match.group(4)
                    key = (kind, number)
                    assert key not in captions, ('Duplicate rendered caption', key)
                    captions[key] = (page_number, line['bbox'][3])
    assert len(reference_positions) == 1
    reference_position = reference_positions[0]
    assert len(acknowledgment_positions) == 1
    assert acknowledgment_positions[0][0] == reference_position[0] == 8
    assert acknowledgment_positions[0] < reference_position
    assert 'VI. DISCUSSION AND CONCLUSION' in texts[6]
    assert not any(re.search(r'^ACKNOWLEDGMENT$|^REFERENCES$', t, re.M) for t in texts[:7])
    body_bottoms = []
    for side in [0, 1]:
        boxes = [line['bbox'] for block in document[6].get_text('dict')['blocks']
                 for line in block.get('lines', []) if (line['bbox'][0] < 305) == (side == 0)]
        bottom = max(box[3] for box in boxes)
        assert 700 <= bottom <= 742, ('Incomplete or overflowing seventh-page column', side, bottom)
        body_bottoms.append(bottom)
    labels = re.findall(r'\\newlabel\{((?:fig|tab):[^}]+)\}\{\{([^}]+)\}\{(\d+)\}',
                        (HERE / 'build/main.aux').read_text())
    assert len(labels) == 8 and len(captions) == 8
    float_pages = {}
    for label, number, page in labels:
        kind = 'figure' if label.startswith('fig:') else 'table'
        position = captions[kind, number]
        assert position[0] == int(page), (label, page, position)
        assert position < reference_position, ('Float after References', label, position)
        assert int(page) <= 7
        float_pages[label] = int(page)
    assert float_pages['fig:intro'] == 1
    wide_labels = {'fig:overview', 'fig:robustness', 'tab:main', 'tab:ablation'}
    wide_per_page = Counter(page for label, page in float_pages.items() if label in wide_labels)
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
    for name in ['intro', 'overview', 'gaussian_paired', 'gaussian_components',
                 'gaussian_communication', 'gaussian_sequence_differences', 'gaussian_phases', 'gaussian_robustness']:
        svg = ET.parse(HERE / 'figures' / f'{name}.svg')
        live = [x for x in svg.iter() if x.tag.endswith('}text')]
        assert live and not any(x.tag.endswith('}image') for x in svg.iter())
        bounds = json.loads((HERE / 'figures' / f'{name}_text_bounds.json').read_text())
        assert bounds['passed']
        if name in ['intro', 'gaussian_communication', 'gaussian_phases', 'gaussian_robustness']:
            assert bounds['text_collisions_checked']
        figures[name] = {'live_svg_text_elements': len(live), 'no_embedded_raster': True,
                         'label_bounds_pass': True, 'svg_sha256': sha(HERE / 'figures' / f'{name}.svg')}

    numerical = check_gaussian_evidence()
    mechanism = check_mechanism_analysis()
    from reviewer_artifact_qa import check_reviewer_evidence
    reviewer = check_reviewer_evidence()
    from intro_design.audit_vector import audit as audit_intro_vector
    figures['intro']['master_reconstruction'] = audit_intro_vector()

    tex = '\n'.join(p.read_text() for p in [HERE / 'main.tex', *(HERE / 'sections').glob('*.tex'),
                                         *(HERE / 'main_figure_integrated').glob('*.tex')])
    assert not re.search(r'\b(seed|frozen|hash|audit|v[234])\b', tex, flags=re.I)
    cited = {key.strip() for group in re.findall(r'\\cite\{([^}]+)\}', tex) for key in group.split(',')}
    available = set(re.findall(r'@\w+\{([^,]+),', (HERE / 'references.bib').read_text()))
    assert cited <= available
    rendered_citations = re.findall(r'\\bibitem\{([^}]+)\}', (HERE / 'build/main.bbl').read_text())
    assert len(rendered_citations) == len(set(rendered_citations))
    assert set(rendered_citations) == cited
    records = json.loads((HERE / 'literature/verification.json').read_text())
    assert cited <= records.keys()
    assert all(records[key]['verified'] for key in cited)
    identifiers = {key: records[key]['doi'] for key in cited
                   if records[key].get('doi') and not records[key]['doi'].lower().startswith('10.48550/')}
    bbl = (HERE/'build/main.bbl').read_text()
    compact_text = re.sub(r'\s+', '', full_text).lower()
    for key, identifier in identifiers.items():
        assert '\\url{'+identifier+'}' in bbl, ('Missing verified DOI', key)
        assert identifier.lower() in compact_text, ('DOI not rendered', key, identifier)
    assert '\\author{}' in tex
    result = {'status': 'automated_artifact_checks_passed', 'pdf_sha256': sha(PDF),
              'pages': len(reader.pages), 'paper_size': 'US Letter', 'text_spans_in_page_bounds': span_count,
              'embedded_fonts': fonts, 'type3_fonts': 0, 'pdf_annotations': 0, 'blank_author_metadata': True,
              'tex_font_substitution_warnings': 0,
              'tex_informational_font_aliases': len(re.findall(r'LaTeX Font Info:\s+Font shape', log)),
              'official_class_and_bst_unmodified': True, 'figures': figures,
              'float_pages':float_pages, 'references_start_page':reference_position[0],
              'acknowledgment_start_page':acknowledgment_positions[0][0],
              'body_pages':7, 'acknowledgment_reference_pages':1,
              'seventh_page_column_bottoms_pt':body_bottoms,
              'all_float_captions_before_references':True,
              'maximum_double_column_floats_per_page':max(wide_per_page.values()),
              'numerical_evidence': numerical,
              'additional_mechanism_analysis': mechanism,
              'reviewer_experiments': reviewer,
              'manuscript_figures': 4, 'manuscript_tables':4, 'companion_evidence_figures': 4, 'ablation_complete_method_last': True,
              'draft_page_cap': 8,
              'conference_page_limit': 8,
              'requires_length_revision_before_submission': len(reader.pages) > 8,
              'citation_keys_resolved': sorted(cited),
              'bibliography_entry_count': len(rendered_citations),
              'verified_doi_identifiers_rendered':len(identifiers),
              'all_cited_keys_rendered_once': True,
              'citation_scope': 'Primary Crossref, DataCite, NeurIPS and CVF metadata plus available author texts; see LITERATURE_SCOPE.md',
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
    revision = read('reviewer_evidence')
    controls = {(r['condition'], r['arm']): r for r in revision['controls']['cohorts']['seen_transfer']['aggregate']}
    gs, fixed = primary + '_guarded_scalar', primary + '_fixed_025'
    table_specs = [
        ('main_table', [(arm, data['labels'][arm], a) for arm in data['methods'][:-1]] +
                       [(gs, 'Guarded Scalar', controls), (fixed, 'Fixed Ratio (0.25)', controls), (primary, 'GCE', a)],
                       ['ospa', 'miss2', 'false2'], 1, 3, 'GCE'),
        ('ablation_table', [('marked_asymmetric', 'Scalar reference', na),
                           (gs, 'Guarded Scalar', controls),
                           (primary+'_no_curvature', 'w/o curvature guard', na),
                           (primary+'_no_history', 'w/o history switch', na),
                           (primary+'_no_mark', 'w/o score constraint', na),
                           (primary, 'GCE (complete)', na)], ['ospa', 'miss2', 'false2'], 1, 3, 'GCE (complete)'),
        ('communication_table', [('marked_lineage', 'No-age KLA', a), ('marked_asymmetric', 'Scalar', a),
                                 (primary, 'GCE (full)', na), (primary, 'GCE (encoded)', a)],
         ['raw_bytes', 'wire_bytes'], 2**20, 2, 'GCE (encoded)')]
    for table, methods, metrics, scale, decimals, emphasized in table_specs:
        tex = (HERE / 'generated' / (table+'.tex')).read_text()
        numbers = np.array([[lookup[condition, arm][metric]['mean']/scale
                            for condition in ['reliable', 'intermittent'] for metric in metrics]
                           for arm, label, lookup in methods])
        minima = numbers.min(axis=0)
        positions = []
        for (method, label, lookup), values in zip(methods, numbers):
            if label == emphasized:
                label = r'\textbf{'+label+'}'
            shown = []
            for j, value in enumerate(values):
                cell = f'{value:.{decimals}f}'
                if table != 'communication_table' and abs(value-minima[j]) < 1e-12:
                    cell = r'\textbf{'+cell+'}'
                shown.append(cell)
            expected = label+' & '+' & '.join(shown)+r' \\'
            assert expected in tex, (table, method, expected)
            positions.append(tex.index(expected))
        assert positions == sorted(positions), ('Incorrect table order', table)
    assert (HERE / 'generated/ablation_table.tex').read_text().rfind(r'\textbf{GCE (complete)}') > (HERE / 'generated/ablation_table.tex').read_text().rfind('w/o ')
    for name, count in [('gaussian_sequence_differences', 300), ('gaussian_components', 200)]:
        figure = read(name)
        assert figure['evidence_sha256'] == sha(HERE / 'source_data/gaussian_paper_evidence.json')
        assert figure['point_count'] == count == sum(len(r['points']) for r in figure['groups'])
        for group in figure['groups']:
            target = c if name == 'gaussian_components' else p
            assert group['summary'] == target[group['condition'], group['reference']]['ospa']
            assert [r['sequence'] for r in group['points']] == data['sequences']
    gains = read('gaussian_paired')
    assert gains['evidence_sha256'] == sha(HERE / 'source_data/gaussian_paper_evidence.json')
    assert gains['kind'] == 'paired_cross_condition_ospa_gain'
    assert gains['point_count'] == 50 and gains['number_of_sequence_measurements'] == 100
    assert gains['individual_points_displayed'] and not gains['intervals_shown']
    assert gains['positive_favors'] == 'GCE'
    assert [panel['reference'] for panel in gains['panels']] == ['marked_lineage', 'marked_asymmetric']
    runs = {(r['condition'], r['arm'], r['sequence']): r for r in data['runs']}
    for panel in gains['panels']:
        assert [point['sequence'] for point in panel['points']] == data['sequences']
        assert panel['x_limits'] == panel['y_limits'] and panel['equal_axis_scale']
        assert not panel['coordinate_jitter']
        counts = Counter()
        for point in panel['points']:
            values = []
            for condition in ['reliable', 'intermittent']:
                gain = (runs[condition, panel['reference'], point['sequence']]['ospa']
                        - runs[condition, primary, point['sequence']]['ospa'])
                close(point[condition+'_gain_m'], gain)
                assert panel['x_limits'][0] < gain < panel['x_limits'][1]
                values.append(gain)
            category = ('both' if min(values) > 0 else
                        'neither' if max(values) <= 0 else 'one')
            assert point['outcome'] == category
            counts[category] += 1
        assert dict(counts) == panel['outcome_counts'] and sum(counts.values()) == 25
        assert f'{counts["both"]}/25 improve in both' in (HERE / 'figures/gaussian_paired.svg').read_text()
    plot_svg = ET.parse(HERE / 'figures/gaussian_paired.svg')
    marker_count = sum(sum(node.tag.endswith('}use') for node in group.iter())
                       for group in plot_svg.iter() if group.get('id', '').startswith('PathCollection_'))
    assert marker_count == 50, ('Incomplete scatter markers', marker_count)
    axis_boxes = plot_svg.findall('.//{http://www.w3.org/2000/svg}clipPath/{http://www.w3.org/2000/svg}rect')
    assert len(axis_boxes) == 2
    assert all(abs(float(box.get('width'))-float(box.get('height'))) < 1e-5 for box in axis_boxes)
    communication = read('gaussian_communication')
    assert communication['evidence_sha256'] == sha(HERE / 'source_data/gaussian_paper_evidence.json')
    assert sum(len(r['sequences']) for r in communication['rows']) == 200
    for row in communication['rows']:
        lookup, arm = (na, primary) if row['arm'] == 'full' else (a, row['arm'])
        for metric in ['raw_bytes', 'wire_bytes']:
            close(row['means_mib'][metric], lookup[row['condition'], arm][metric]['mean']/2**20)
        close(row['mean_ospa_m'], a[row['condition'], arm]['ospa']['mean'])
    return dict(main_sequences=25, main_frames=5601, development_sequences=9, development_frames=1993,
                original_comparison_methods=11, original_sequence_condition_method_runs=550, scalar_facts_checked=64,
                paired_sequence_points=300, component_sequence_points=200,
                main_figure_sequence_markers=50, main_figure_paired_gain_values=100,
                codec_exact_primary_trajectories=68, original_corpus_outcomes_previously_seen=True,
                portable_source_snapshots=len(manifest), source_result_identities=len(data['source_inputs_sha256']))


def check_mechanism_analysis():
    data_dir = HERE/'source_data'
    snapshot = json.loads((data_dir/'mechanism_diagnostic_snapshot.json').read_text())
    analysis = json.loads((data_dir/'mechanism_analysis.json').read_text())
    evidence = json.loads((data_dir/'gaussian_paper_evidence.json').read_text())
    primary = 'marked_gaussian_evidence'
    names = evidence['sequences']
    conditions = ['reliable', 'intermittent']
    arms = ['marked_lineage', 'marked_asymmetric', primary]
    phases = ['before', 'outage', 'after']
    cells = ['base_space_base_integral', 'new_space_base_integral',
             'base_space_new_integral', 'new_space_new_integral']
    assert snapshot['sequences'] == analysis['sequences'] == names and len(names) == 25
    assert len(snapshot['runs']) == len(snapshot['source_inputs_sha256']) == 150
    assert len(snapshot['fixed_input']) == 50
    assert snapshot['protocol_sha256'] == sha(HERE/'ANALYSIS_PROTOCOL.md')
    assert snapshot['extractor_sha256'] == sha(HERE/'prepare_mechanism_analysis.py')
    assert snapshot['source_evidence_sha256'] == sha(data_dir/'gaussian_paper_evidence.json')
    assert analysis['snapshot_sha256'] == sha(data_dir/'mechanism_diagnostic_snapshot.json')
    assert analysis['generator_sha256'] == sha(HERE/'prepare_mechanism_analysis.py')
    assert snapshot['all_four_cells_share_admitted_kappa'] and not snapshot['alternate_outputs_fed_back']
    assert not analysis['method_reselected'] and not analysis['new_tracking_trajectories']
    assert snapshot['maximum_native_ospa_absolute_error'] < 1e-8
    for path, digest in snapshot['source_inputs_sha256'].items():
        assert evidence['source_inputs_sha256'][path] == digest
        if (HERE.parents[1]/path).exists():
            assert sha(HERE.parents[1]/path) == digest

    means, traces, phase_rows = {}, {}, {}
    native = {(r['sequence'], r['condition'], r['arm']): r['ospa'] for r in evidence['runs']}
    count = 0
    for row in snapshot['runs']:
        key = row['sequence'], row['condition'], row['arm']
        assert key not in traces
        values = np.array(row['ospa'], float)
        length = row['frames']
        assert values.shape == (2, length) and np.isfinite(values).all()
        assert np.all((values >= 0) & (values <= 12))
        assert math.isclose(values.mean(), native[key], abs_tol=1e-9, rel_tol=0)
        cuts = [0, int(.4*length), int(.6*length), length]
        assert row['phase_boundaries'] == cuts
        total = 0.
        for phase, start, end in zip(phases, cuts, cuts[1:]):
            selected = values[:, start:end]
            phase_rows[key+(phase,)] = float(selected.mean())
            total += selected.size*selected.mean()
        assert math.isclose(total, values.sum(), abs_tol=1e-9, rel_tol=1e-12)
        traces[key] = values
        count += values.size
    assert count == snapshot['rescored_native_receiver_scans'] == 67212
    assert len(analysis['phase_rows']) == 450
    for row in analysis['phase_rows']:
        key = row['sequence'], row['condition'], row['arm'], row['phase']
        assert math.isclose(row['ospa'], phase_rows[key], abs_tol=1e-12)

    draws = np.random.default_rng(8301).integers(0, 25, (10000, 25))
    def verify_summary(values, summary):
        values = np.array(values)
        expected = [values.mean(), *np.percentile(values[draws].mean(axis=1), [2.5, 97.5])]
        assert summary['n'] == 25
        assert np.allclose(expected, [summary[k] for k in ['mean', 'low', 'high']], atol=1e-12, rtol=0)

    assert len(analysis['phase_aggregate']) == 18 and len(analysis['phase_paired']) == 12
    for row in analysis['phase_aggregate']:
        verify_summary([phase_rows[n, row['condition'], row['arm'], row['phase']] for n in names], row['ospa'])
    for row in analysis['phase_paired']:
        values = [phase_rows[n, row['condition'], primary, row['phase']]
                  - phase_rows[n, row['condition'], row['reference'], row['phase']] for n in names]
        assert np.allclose(values, row['differences'], atol=1e-12, rtol=0)
        verify_summary(values, row['ospa'])

    fixed_count, fixed_means = 0, {}
    for row in snapshot['fixed_input']:
        sites = np.array(row['receiver_scans'], int)
        assert len(set(map(tuple, sites))) == len(sites)
        assert np.isin(sites[:, 1], [1, 2]).all() and (sites[:, 0] >= 1).all()
        full = traces[row['sequence'], row['condition'], primary]
        joint = np.array(row['ospa']['new_space_new_integral'])
        assert np.allclose(joint, full[sites[:, 1]-1, sites[:, 0]-1], atol=1e-8, rtol=0)
        if row['condition'] == 'intermittent':
            first, last = int(.4*full.shape[1]), int(.6*full.shape[1])
            assert not ((sites[:, 0]-1 >= first) & (sites[:, 0]-1 < last)).any()
        else:
            assert len(sites) == full.size
        for cell in cells:
            values = np.array(row['ospa'][cell])
            assert values.shape == (len(sites),) and np.isfinite(values).all()
            assert np.all((values >= 0) & (values <= 12))
            fixed_means[row['sequence'], row['condition'], cell] = float(values.mean())
        fixed_count += len(sites)
    assert fixed_count == snapshot['verified_joint_receiver_scans'] == 19235
    assert len(analysis['fixed_input_rows']) == 200
    for row in analysis['fixed_input_rows']:
        assert math.isclose(row['ospa'], fixed_means[row['sequence'], row['condition'], row['cell']], abs_tol=1e-12)
    assert len(analysis['fixed_input_aggregate']) == 8 and len(analysis['fixed_input_paired']) == 6
    for row in analysis['fixed_input_aggregate']:
        verify_summary([fixed_means[n, row['condition'], row['cell']] for n in names], row['ospa'])
    for row in analysis['fixed_input_paired']:
        values = [fixed_means[n, row['condition'], cells[-1]]-fixed_means[n, row['condition'], row['reference']] for n in names]
        assert np.allclose(values, row['differences'], atol=1e-12, rtol=0)
        verify_summary(values, row['ospa'])

    table = (HERE/'generated/fixed_input_table.tex').read_text()
    summary = {(r['condition'], r['cell']): r['ospa']['mean'] for r in analysis['fixed_input_aggregate']}
    positions = []
    for cell, (space, integral) in zip(cells, [('Base', 'Base'), ('Corrected', 'Base'), ('Base', 'Corrected'), ('Corrected', 'Corrected')]):
        vals = [summary[c, cell] for c in conditions]
        columns = [space, integral]+[f'{v:.3f}' for v in vals]
        if cell == cells[-1]:
            assert all(summary[c, cell] == min(summary[c, cc] for cc in cells) for c in conditions)
            columns = [r'\textbf{'+v+'}' for v in columns]
        expected = ' & '.join(columns)+r' \\'
        assert expected in table
        positions.append(table.index(expected))
    assert positions == sorted(positions)

    expected_facts = {}
    for row in analysis['phase_paired']:
        ref = 'NoAge' if row['reference'] == arms[0] else 'Scalar'
        for stat in ['mean', 'low', 'high']:
            name = 'Phase'+row['condition'].title()+row['phase'].title()+ref+stat.title()
            expected_facts[name] = row['ospa'][stat]
    for row in analysis['fixed_input_paired']:
        aspect = {cells[0]: 'Both', cells[1]: 'Integral', cells[2]: 'Space'}[row['reference']]
        for stat in ['mean', 'low', 'high']:
            expected_facts['Joint'+row['condition'].title()+aspect+stat.title()] = row['ospa'][stat]
    facts = json.loads((HERE/'generated/mechanism_facts.json').read_text())
    assert facts == expected_facts and len(facts) == 54
    macros = dict(re.findall(r'\\newcommand\{\\(\w+)\}\{([^}]+)\}', (HERE/'generated/mechanism_numbers.tex').read_text()))
    assert macros == {key: f'{value:.3f}' for key, value in facts.items()}

    figure = json.loads((data_dir/'gaussian_phases.json').read_text())
    assert figure['point_count'] == 12 and figure['number_of_sequence_measurements'] == 300
    assert figure['groups'] == analysis['phase_paired']
    assert figure['mechanism_analysis_sha256'] == sha(data_dir/'mechanism_analysis.json')
    assert figure['snapshot_sha256'] == sha(data_dir/'mechanism_diagnostic_snapshot.json')
    for row in figure['groups']:
        assert figure['y_limits'][0] < row['ospa']['low'] <= row['ospa']['mean'] <= row['ospa']['high'] < figure['y_limits'][1]
    svg = ET.parse(HERE/'figures/gaussian_phases.svg')
    mean_markers = [node for node in svg.iter() if node.tag.endswith('}use') and 'stroke: #ffffff' in node.get('style', '')]
    assert len(mean_markers) == 12, ('Phase means missing from SVG', len(mean_markers))
    intro = json.loads((data_dir/'intro.json').read_text())
    assert intro['kind'] == 'qualitative_shared_prior_schematic' and not intro['empirical_data']
    assert 'common prior' in intro['assumption'] and intro['width_mm'] == 89
    return dict(complete_sequences=25, native_trajectory_files=150,
                native_receiver_scan_scores=67212, fixed_input_receiver_scans=19235,
                all_four_cells_share_admitted_increments=True, alternative_feedback=False,
                sequence_phase_method_rows=450, phase_estimates=12, phase_pair_measurements=300,
                fixed_input_means=8, additional_numeric_macros=54,
                snapshot_sha256=sha(data_dir/'mechanism_diagnostic_snapshot.json'),
                interpretation='Descriptive diagnostics on complete saved development-corpus trajectories.')


if __name__ == '__main__':
    check()
