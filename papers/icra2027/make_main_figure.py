"""Draw a schematic proposal for the paper's main mechanism figure."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'main_figure_draft'
OUT.mkdir(exist_ok=True)
mpl.rcParams.update({
    'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans'],
    'font.size': 7, 'mathtext.fontset': 'dejavusans', 'svg.fonttype': 'none',
    'svg.hashsalt': 'icra-observation-recency-overview', 'pdf.fonttype': 42,
    'axes.unicode_minus': False,
})

INK = '#26343e'
GREY = '#64717a'
PALE = '#f4f6f7'
LINE = '#cbd2d6'
EXIST = '#00856a'
SPACE = '#0072b2'
WIDTH, HEIGHT = 181, 89

fig = plt.figure(figsize=(WIDTH / 25.4, HEIGHT / 25.4))
ax = fig.add_axes([0, 0, 1, 1])
ax.set(xlim=(0, WIDTH), ylim=(0, HEIGHT), aspect='equal')
ax.axis('off')


def text(x, y, value, size=7, color=INK, **kwargs):
    return ax.text(x, y, value, fontsize=size, color=color, va='center', **kwargs)


def box(x, y, w, h, fill='white', edge=LINE, dashed=False):
    patch = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0,rounding_size=1.05',
                          facecolor=fill, edgecolor=edge, linewidth=.65,
                          linestyle=(0, (2, 2)) if dashed else '-')
    ax.add_patch(patch)
    return patch


def arrow(start, end, color=GREY, style='-|>', dashed=False, curve=0, width=.8):
    patch = FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=7,
                            connectionstyle=f'arc3,rad={curve}', color=color,
                            linewidth=width, linestyle=(0, (2.5, 2)) if dashed else '-',
                            shrinkA=0, shrinkB=0)
    ax.add_patch(patch)


def robot(x, y, name):
    for dx in [-2.35, 1.65]:
        ax.add_patch(Rectangle((x + dx, y - 1.2), .7, 2.4, facecolor=INK, edgecolor='none', zorder=6))
    ax.add_patch(Circle((x, y), 1.85, facecolor='white', edgecolor=INK, linewidth=.8, zorder=6))
    text(x, y, name, 6.1, ha='center', weight='bold', zorder=7)


def fov(x, y, radius=8):
    ax.add_patch(Circle((x, y), radius, facecolor='#edf3f6', edgecolor='#c5d4de',
                        linewidth=.6, zorder=0))


def density(x, y, symbol):
    # A qualitative density icon, not a sampled posterior or experimental value.
    u = np.linspace(-1.7, 1.7, 100)
    v = np.exp(-.5 * (u / .63) ** 2)
    ax.plot(x + 2.65 * u, y + 2.4 * v, color=SPACE, lw=.85)
    text(x + 5.2, y + 1.2, symbol, 6.9, SPACE, ha='left')


def title(letter, x, label):
    text(x, 84.5, letter, 9, weight='bold')
    text(x + 5.0, 84.5, label, 7.7, weight='bold')


title('a', 3, 'Different histories at reunion')
title('b', 54, 'Qualify each label')
title('c', 117, 'Two weights, one fused belief')
ax.plot([50.5, 50.5], [5, 79.5], color='#e4e8eb', lw=.7)

# (a) A schematic before/after robot encounter. Positions have no metric scale.
text(3, 78.2, 'Earlier direct opportunity', 6.8, GREY)
fov(17, 65.5)
robot(17, 65.5, 'A')
ax.scatter([24], [68], marker='x', s=23, linewidths=1, color=INK, zorder=5)
arrow((19.4, 66.4), (22.3, 67.5), color=GREY, width=.6)
text(30, 70.5, 'Observed', 6.6)
text(30, 67.0, 'target', 6.6)
ax.plot([27.8, 25.2], [68.5, 68.1], color=GREY, lw=.5)
arrow((15.8, 55.7), (11.2, 43.8), curve=.18, width=.7)
text(19, 53.1, 'Prescribed motion', 6.4, GREY)

text(13, 47.8, 'Now: posterior exchange', 6.8, GREY)
fov(11, 31)
fov(34, 31)
ax.add_patch(Circle((30, 37), 1.2, fill=False, edgecolor=GREY, linewidth=.7,
                    linestyle=(0, (2, 2))))
ax.plot([31.3, 37.5], [37.7, 41.4], color=GREY, lw=.5)
text(48, 43.4, 'Hypothesis', 6.1, GREY, ha='right')
arrow((13.4, 31), (31.6, 31), color=GREY, dashed=True, style='<->', width=.8)
robot(11, 31, 'A')
robot(34, 31, 'B')
text(22.5, 27.0, 'posterior', 6.2, GREY, ha='center')
text(11, 20.0, 'Old direct age', 6.5, ha='center')
text(34, 20.0, 'Recent miss', 6.5, ha='center')
box(3, 5, 45, 9, PALE, edge='none')
text(25.5, 9.5, 'Packet arrival does not reset\nlocal direct-opportunity age', 6.5,
     ha='center', linespacing=1.25)

# (b) The same label has two informed inputs and one untouched prior.
box(54, 62, 49, 15)
text(57, 73.4, 'A  Older observed belief', 7.0, weight='bold')
text(57, 68.6, r'$h_A=1,\quad \Delta_A>0$', 6.9, GREY)
text(57, 64.5, r'High $r_A$', 6.8, EXIST)
density(91, 64.1, r'$p_A$')

box(54, 43, 49, 15)
text(57, 54.4, 'B  Recent missed detection', 7.0, weight='bold')
text(57, 49.6, r'$h_B=1,\quad \Delta_B=0$', 6.9, GREY)
text(57, 45.5, r'Low $r_B$', 6.8, EXIST)
density(91, 45.1, r'$p_B$')

box(54, 24, 49, 15, PALE, dashed=True)
text(57, 35.4, 'U  Untouched prior', 7.0, GREY, weight='bold')
text(57, 30.8, r'$h_U=0$', 6.9, GREY)
text(57, 26.6, 'Excluded here: informed inputs exist', 6.25, GREY)
ax.plot([104.6, 106.6], [30.7, 32.7], color=GREY, lw=.8)
ax.plot([104.6, 106.6], [32.7, 30.7], color=GREY, lw=.8)

text(54, 18.4, r'$h$: observation lineage', 6.5, GREY)
text(54, 14.5, r'$\Delta$: local direct-opportunity age', 6.5, GREY)
box(54, 4.3, 49, 7.1, PALE, edge='none')
text(78.5, 7.85, r'Observable absence enters $\mathcal{E}$ only', 6.35,
     GREY, ha='center')

# Qualified represented inputs feed both blocks. Absence has no spatial density.
ax.plot([103, 109, 109, 103], [69.5, 69.5, 50.5, 50.5], color=GREY, lw=.75)
ax.plot([109, 109], [50.5, 34.7], color=GREY, lw=.75)
arrow((109, 69.5), (116, 69.5), EXIST, width=1.0)
arrow((109, 34.7), (116, 34.7), SPACE, width=1.0)

# (c) The mechanism: age in existence, ordinary eligible spatial weights.
box(117, 50, 61, 27, '#f1f8f5', EXIST)
text(120, 73.1, 'Existence: apply direct age', 7.5, EXIST, weight='bold')
text(120, 65.9, r'$q_j\propto w_j f(\Delta_j),\quad j\in\mathcal{E}$', 8.0)
text(120, 59.1,
     r'$\mathrm{logit}\,r^*=\sum_j q_j\,\mathrm{logit}\,r_j+\log\eta_a$', 7.5)
text(120, 53.5, 'Recent detections and misses both count', 6.5, EXIST)

box(117, 14.7, 61, 26.3, '#f1f6fa', SPACE)
text(120, 37.2, 'Spatial density: retain base weights', 7.25, SPACE, weight='bold')
text(120, 30.6, r'$a_j\propto w_j,\quad j\in\mathcal{P}$', 8.0)
text(120, 24.2, r'$p^*(x)=\eta_a^{-1}\prod_j p_j(x)^{a_j}$', 8.0)
text(120, 17.9, 'Same eligible inputs and spatial weights', 6.5, SPACE)

arrow((173.5, 41.5), (173.5, 49.5), SPACE, width=1.1)
text(168.8, 45.5, r'Spatial overlap $\eta_a$', 6.8, SPACE, ha='right')
text(147.5, 8.0, 'Fixed inputs: spatial pooling stays unchanged', 6.45,
     ha='center', color=GREY)

fig.canvas.draw()
renderer = fig.canvas.get_renderer()
canvas_w, canvas_h = fig.canvas.get_width_height()
checks = []
for item in ax.texts:
    bound = item.get_window_extent(renderer)
    assert bound.x0 >= 0 and bound.y0 >= 0 and bound.x1 <= canvas_w and bound.y1 <= canvas_h, (item.get_text(), bound)
    checks.append({'text': item.get_text(), 'bbox_pixels': list(bound.bounds)})
for i, left in enumerate(ax.texts):
    for right in ax.texts[i + 1:]:
        assert not left.get_window_extent(renderer).overlaps(right.get_window_extent(renderer)), (left.get_text(), right.get_text())

for extension in ['svg', 'pdf', 'png']:
    metadata = {'Date': None} if extension == 'svg' else ({'CreationDate': None, 'ModDate': None} if extension == 'pdf' else None)
    fig.savefig(OUT / f'overview.{extension}', dpi=300, facecolor='white', metadata=metadata)
plt.close(fig)
svg = ET.parse(OUT / 'overview.svg')
live = sum(item.tag.endswith('}text') for item in svg.iter())
assert live > 30 and not any(item.tag.endswith('}image') for item in svg.iter())
(OUT / 'source_data.json').write_text(json.dumps({
    'kind': 'schematic_not_trial_data', 'dimensions_mm': [WIDTH, HEIGHT],
    'inputs': [
        {'robot': 'A', 'lineage': True, 'age': 'older', 'existence': 'high'},
        {'robot': 'B', 'lineage': True, 'age': 'current', 'existence': 'low after a miss'},
        {'robot': 'U', 'lineage': False, 'participation': 'excluded because informed inputs exist'},
    ],
    'illustration': 'Qualitative geometry and density icons; no measurement or trial values are represented.',
    'invariance_scope': 'Fixed input densities, eligibility and ordinary spatial weights only.',
}, indent=2) + '\n')
(OUT / 'qa.json').write_text(json.dumps({
    'status': 'export_and_text_bounds_passed', 'editable_svg_text_elements': live,
    'embedded_raster_images': 0, 'dimensions_mm': [WIDTH, HEIGHT],
    'pairwise_text_overlap': False,
    'text_bounds': checks,
}, indent=2) + '\n')
print('Exported SVG/PDF/PNG main-figure proposal:', OUT)
