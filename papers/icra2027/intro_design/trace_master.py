"""Freeze generated-master geometry as editable paths; no tracing at paper build time.

Optional authoring dependencies: vtracer 0.6.15 and svgpathtools 1.8.0.
The original raster is never modified. Annotation areas are cleared in a
temporary tracing input so the paper builder can typeset editable characters.
"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from PIL import Image, ImageFilter, ImageDraw
import numpy as np

import vtracer
from svgpathtools import parse_path

HERE = Path(__file__).resolve().parent
MASTER = HERE/'concept_v3.png'
PARAMETERS = dict(colormode='color', hierarchical='stacked', mode='spline',
                  filter_speckle=4, color_precision=7, layer_difference=16,
                  corner_threshold=90, length_threshold=5.0, max_iterations=20,
                  splice_threshold=45, path_precision=3)
# Tight source-coordinate regions, measured on the 1385 by 1136 master.
TEXT_REGIONS = [
    ('shared-history', [535, 15, 850, 85]),
    ('vehicle-a', [10, 437, 181, 482]),
    ('vehicle-b', [1205, 437, 1375, 483]),
    ('observations', [534, 429, 855, 473]),
    ('pool-title', [120, 688, 457, 754]),
    ('admission', [557, 699, 829, 789]),
    ('gce-title', [1080, 681, 1213, 748]),
    ('pool-formula', [85, 793, 491, 912]),
    ('gce-formula', [902, 788, 1299, 903]),
    ('weight-bound', [987, 907, 1211, 954]),
    ('spatial-density', [113, 1027, 359, 1078]),
    ('existence', [1061, 1029, 1237, 1079]),
    ('normalizer', [533, 1004, 851, 1050]),
]


def main():
    traced = HERE/'outline_reference.svg'
    working = HERE.parent/'build/intro_trace_input.png'
    working.parent.mkdir(exist_ok=True)
    background_fills = {}
    with Image.open(MASTER) as original:
        image = original.copy()
        pixels = np.asarray(original)
        draw = ImageDraw.Draw(image)
        for name, (x0, y0, x1, y1) in TEXT_REGIONS:
            perimeter = np.concatenate([pixels[y0, x0:x1], pixels[y1, x0:x1],
                                        pixels[y0:y1, x0], pixels[y0:y1, x1]])
            fill = tuple(int(x) for x in np.median(perimeter, axis=0))
            background_fills[name] = fill
            if name in {'vehicle-a', 'vehicle-b'}:
                # Stop at the measured upper vehicle silhouette. The source
                # lettering touches these edges; a full rectangle erases roofs.
                yy, xx = np.mgrid[y0:y1+1, x0:x1+1]
                if name == 'vehicle-a':
                    safe = yy < 578-.60*xx
                else:
                    safe = yy < .64*xx-302
                mask = Image.fromarray((safe*255).astype('uint8'))
                image.paste(fill, (x0, y0, x1+1, y1+1), mask)
            else:
                draw.rectangle((x0, y0, x1, y1), fill=fill)
        image.filter(ImageFilter.GaussianBlur(.35)).resize((2770, 2272), Image.Resampling.LANCZOS).save(working)
    vtracer.convert_image_to_svg_py(str(working), str(traced), **PARAMETERS)
    root = ET.parse(traced).getroot()
    kept = []
    for number, node in enumerate(root):
        assert node.tag.endswith('}path')
        d = node.attrib['d']
        assert set(re.findall('[A-Za-z]', d)) <= {'M', 'L', 'C', 'Z'}
        shift = [float(x) for x in re.findall(r'-?\d+(?:\.\d+)?', node.attrib['transform'])]
        assert len(shift) == 2
        xmin, xmax, ymin, ymax = parse_path(d).bbox()
        bounds = [(xmin+shift[0])/2, (ymin+shift[1])/2, (xmax+shift[0])/2, (ymax+shift[1])/2]
        kept.append(dict(id=f'master-shape-{number:04d}', d=d,
                         fill=node.attrib['fill'], translate=shift, scale=.5, bounds=bounds))
    data = dict(canvas=[1385, 1136], master_sha256=hashlib.sha256(MASTER.read_bytes()).hexdigest(),
                trace_parameters=PARAMETERS, text_regions=TEXT_REGIONS,
                text_background_fills=background_fills, paths=kept,
                preprocessing='0.35-pixel antialias smoothing and 2x Lanczos upsampling; coordinates restored by scale 0.5.',
                treatment='All non-text geometry retained from the selected image master; text rebuilt with live characters.')
    (HERE/'vector_scene.json').write_text(json.dumps(data, separators=(',', ':'))+'\n')
    print('Retained vector paths:', len(kept), 'Editable text regions:', len(TEXT_REGIONS))


if __name__ == '__main__':
    main()
