"""Rebuild the selected illustrated Intro master using paths and live typography."""
from pathlib import Path
import hashlib
import json
import re
import cairosvg

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch
from matplotlib.transforms import Affine2D
from make_gaussian_figures import export
from figure_typography import tex_label

HERE = Path(__file__).resolve().parent
mpl.rcParams.update({'mathtext.fontset': 'stix', 'svg.hashsalt': 'icra-gce-intro-illustrated',
                     'font.sans-serif': ['Arial Narrow', 'Liberation Sans Narrow'],
                     'text.latex.preamble': r'\usepackage[T1]{fontenc}\usepackage{times}\usepackage{amsmath,amssymb}'})


def parse_d(d):
    """Read the absolute M/L/C/Z commands emitted by the frozen vector tracer."""
    tokens = re.findall(r'[MLCZ]|-?(?:\d+\.?\d*|\.\d+)', d)
    vertices, codes = [], []
    i = 0
    start = (0.0, 0.0)
    while i < len(tokens):
        command = tokens[i]
        i += 1
        if command == 'Z':
            vertices.append(start)
            codes.append(MplPath.CLOSEPOLY)
            continue
        assert command in {'M', 'L', 'C'}, command
        count = 6 if command == 'C' else 2
        values = [float(x) for x in tokens[i:i+count]]
        i += count
        pairs = list(zip(values[::2], values[1::2]))
        if command == 'M':
            start = pairs[0]
        vertices.extend(pairs)
        codes.extend([{'M': MplPath.MOVETO, 'L': MplPath.LINETO, 'C': MplPath.CURVE4}[command]]*len(pairs))
    return MplPath(vertices, codes)


def main():
    scene_path = HERE/'intro_design/vector_scene.json'
    scene = json.loads(scene_path.read_text())
    master = HERE/'intro_design/concept_v3.png'
    assert hashlib.sha256(master.read_bytes()).hexdigest() == scene['master_sha256']
    fig = plt.figure(figsize=(89/25.4, 73/25.4), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    width, height = scene['canvas']
    ax.set(xlim=(0, width), ylim=(height, 0))
    ax.axis('off')
    for shape in scene['paths']:
        patch = PathPatch(parse_d(shape['d']), facecolor=shape['fill'], edgecolor='none',
                          linewidth=0, transform=Affine2D().translate(*shape['translate']).scale(shape.get('scale', 1))+ax.transData)
        patch.set_gid(shape['id'])
        ax.add_patch(patch)

    def text(x, y, value, size=8, color='#141414', **kwargs):
        if '$' in value:
            return tex_label(ax, x, y, value, size, color, ha=kwargs.pop('ha', 'center'))
        return ax.text(x, y, value, ha=kwargs.pop('ha', 'center'), va='center',
                       fontsize=size, color=color, fontfamily='sans-serif',
                       fontweight=kwargs.pop('weight', 'bold'), usetex='$' in value,
                       zorder=10, **kwargs)

    text(691, 47, 'Shared history', 9.7)
    text(18, 461, 'Vehicle A', 8.0, '#174c96', ha='left')
    text(1368, 461, 'Vehicle B', 8.0, '#92501a', ha='right')
    text(694, 451, 'Current observations', 7.5)
    text(287, 721, 'Posterior pool', 10.2, '#12466a')
    text(694, 718, 'Admit current', 8.7)
    text(694, 768, 'ratios', 8.7)
    text(1148, 716, 'GCE', 12.0, '#00787e')
    likelihood_equations = [r'f^-(X)\ell_A(X)^{1/2}\ell_B(X)^{1/2}',
                            r'f^-(X)\ell_A(X)^{\omega_A}\ell_B(X)^{\omega_B}']
    for x, equation in zip([287, 1101], likelihood_equations):
        text(x, 860, f'${equation}$', 7.8)
    text(1101, 945, r'$\omega_j=1/2+\bar\kappa_j\in[1/2,1]$', 7.0)
    text(236, 1055, 'Spatial density', 8.0)
    text(1149, 1055, 'Existence', 8.0)
    text(693, 1029, 'shared normalizer', 8.5, '#00848a')
    source = dict(kind='qualitative_shared_prior_schematic', empirical_data=False,
                  check_text_collisions=True, archetype='schematic-led composite',
                  width_mm=89, height_mm=73,
                  assumption='Two sources with common prior, equal base weights, exact local likelihoods.',
                  pool='f_minus * likelihood_A**0.5 * likelihood_B**0.5',
                  corrected='f_minus * likelihood_A**omega_A * likelihood_B**omega_B',
                  exponents='omega_j = 0.5 + admitted_kappa_j in [0.5, 1]',
                  message='Admitted current ratios modify spatial density and existence through one normalizer.',
                  full_restoration='Exact shared-prior Bayes requires both admitted gates equal to one.',
                  generated_master='intro_design/concept_v3.png',
                  generated_master_sha256=scene['master_sha256'],
                  vector_scene_sha256=hashlib.sha256(scene_path.read_bytes()).hexdigest(),
                  source_canvas_pixels=scene['canvas'], traced_scene_paths=len(scene['paths']),
                  live_text_labels=11, latex_equation_lines=3,
                  scene_treatment='Same source-coordinate geometry; typography rebuilt as editable text.',
                  image_integrity='AI-generated conceptual illustration, not a captured test scene or measured result.')
    export(fig, 'intro', source)
    cairosvg.svg2pdf(url=str(HERE/'figures/intro.svg'), write_to=str(HERE/'figures/intro.pdf'))
    cairosvg.svg2png(url=str(HERE/'figures/intro.svg'), write_to=str(HERE/'figures/intro.png'), dpi=300)


if __name__ == '__main__':
    main()
