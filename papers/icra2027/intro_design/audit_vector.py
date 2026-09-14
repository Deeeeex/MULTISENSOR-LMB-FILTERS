"""Check actual SVG geometry, editable labels, and appearance against the master."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
PAPER = HERE.parent
NUMBER = r'-?(?:\d+\.?\d*|\.\d+)'


def audit():
    source = json.loads((HERE/'vector_scene.json').read_text())
    master = HERE/'concept_v3.png'
    assert hashlib.sha256(master.read_bytes()).hexdigest() == source['master_sha256']
    svg = ET.parse(PAPER/'figures/intro.svg').getroot()
    assert not any(n.tag.endswith('}image') for n in svg.iter())
    width, height = [float(x) for x in svg.attrib['viewBox'].split()][2:]
    factor = np.array([width/source['canvas'][0], height/source['canvas'][1]])
    groups = {n.attrib['id']: n for n in svg.iter()
              if n.tag.endswith('}g') and n.attrib.get('id', '').startswith('master-shape-')}
    assert set(groups) == {p['id'] for p in source['paths']}
    errors = []
    for shape in source['paths']:
        output = [n for n in groups[shape['id']] if n.tag.endswith('}path')]
        assert len(output) == 1
        output = output[0]
        expected = np.array([float(x) for x in re.findall(NUMBER, shape['d'])]).reshape(-1, 2)
        expected = (expected+shape['translate'])*shape['scale']*factor
        actual = np.array([float(x) for x in re.findall(NUMBER, output.attrib['d'])]).reshape(-1, 2)
        assert expected.shape == actual.shape, shape['id']
        errors.append(float(np.max(np.abs(expected-actual))))
        fill = re.search(r'fill:\s*(#[0-9a-fA-F]{6})', output.attrib.get('style', ''))
        assert (fill.group(1) if fill else '#000000').lower() == shape['fill'].lower()
    assert max(errors) < 2e-6, max(errors)
    texts = [n for n in svg.iter() if n.tag.endswith('}text')]
    assert len(texts) == 11
    strings = [''.join(n.itertext()).strip() for n in texts]
    expected_labels = ['Shared history', 'Vehicle A', 'Vehicle B', 'Current observations',
                       'Posterior pool', 'Admit current', 'ratios', 'GCE',
                       'Spatial density', 'Existence', 'shared normalizer']
    assert all(t in strings for t in expected_labels)
    bounds = json.loads((PAPER/'figures/intro_text_bounds.json').read_text())
    assert bounds['text_collisions_checked'] and bounds['passed']
    assert min(t['font_size_pt'] for t in bounds['text_bounds']) >= 7
    latex_lines = [item['text'] for item in bounds['text_bounds'] if item['text'].startswith('$')]
    assert len(latex_lines) == 3
    latex_equations = latex_lines
    caption = (PAPER/'sections/introduction.tex').read_text()
    svg_text = (PAPER/'figures/intro.svg').read_text()
    assert all(equation in caption for equation in latex_equations)
    assert all(f'<!-- {line} -->' in svg_text for line in latex_lines)

    # Appearance is compared descriptively, excluding intentionally re-typeset text.
    with Image.open(PAPER/'figures/intro.png') as preview:
        preview = preview.convert('RGB')
        size = preview.size
        pixels = np.asarray(preview).astype(float)
    with Image.open(master) as original:
        reference = np.asarray(original.convert('RGB').resize(size, Image.Resampling.LANCZOS)).astype(float)
    keep = Image.new('L', size, 255)
    from PIL import ImageDraw
    draw = ImageDraw.Draw(keep)
    sx, sy = size[0]/source['canvas'][0], size[1]/source['canvas'][1]
    for _, (x0, y0, x1, y1) in source['text_regions']:
        draw.rectangle((x0*sx-2, y0*sy-2, x1*sx+2, y1*sy+2), fill=0)
    for item in bounds['text_bounds']:
        x, y, w, h = item['bbox_pixels']
        draw.rectangle((x-2, size[1]-y-h-2, x+w+2, size[1]-y+2), fill=0)
    mask = np.asarray(keep) > 0
    delta = np.abs(pixels-reference)
    report = dict(status='passed', master_sha256=source['master_sha256'],
                  svg_sha256=hashlib.sha256((PAPER/'figures/intro.svg').read_bytes()).hexdigest(),
                  all_source_paths_retained=len(groups), all_path_fills_identical=True,
                  maximum_geometry_error_pt=max(errors), geometry_rounding_tolerance_pt=2e-6,
                  source_canvas_pixels=source['canvas'], final_dimensions_mm=[89, 73],
                  live_svg_text_elements=len(texts), expected_labels=expected_labels,
                  latex_equations_match_caption=True, latex_equations=latex_equations,
                  minimum_font_size_pt=min(t['font_size_pt'] for t in bounds['text_bounds']),
                  no_embedded_raster=True, text_collisions_checked=True,
                  compared_nontext_pixels=int(mask.sum()),
                  mean_absolute_rgb_error_0_to_255=float(delta[mask].mean()),
                  pixel_identical_to_generated_bitmap=False,
                  scope='Source-coordinate vector reconstruction with editable typography; fitting and font rasterization differ from the bitmap. Not experimental evidence.')
    (HERE/'vector_fidelity.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    print(json.dumps(audit(), indent=2))
