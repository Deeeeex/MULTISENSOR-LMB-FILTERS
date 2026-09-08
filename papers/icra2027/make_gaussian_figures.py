"""Render complete sequence comparisons and native communication measurements."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = Path(__file__).resolve().parent
OUT = HERE / 'figures'
DATA = HERE / 'source_data'
PRIMARY = 'marked_gaussian_evidence'
CONDITIONS = [('reliable', 'Reliable links'), ('intermittent', 'Intermittent links')]
INK, TEAL, BLUE, LIGHT, GRID = '#253746', '#008577', '#246699', '#9bafb9', '#e1e7ea'
mpl.rcParams.update({
    'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'DejaVu Sans'],
    'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 9,
    'axes.titleweight': 'bold', 'axes.edgecolor': INK, 'axes.linewidth': .55,
    'text.color': INK, 'axes.labelcolor': INK, 'xtick.color': INK, 'ytick.color': INK,
    'xtick.labelsize': 7.5, 'ytick.labelsize': 8, 'xtick.major.width': .5,
    'xtick.major.size': 3, 'ytick.major.size': 0, 'legend.fontsize': 7.5,
    'svg.fonttype': 'none', 'svg.hashsalt': 'icra-gaussian-results',
    'pdf.fonttype': 42, 'ps.fonttype': 42, 'axes.unicode_minus': True,
})


def export(fig, name, source):
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    width, height = fig.canvas.get_width_height()
    bounds = []
    for item in fig.findobj(mpl.text.Text):
        if not item.get_visible() or not item.get_text():
            continue
        b = item.get_window_extent(renderer)
        assert b.x0 >= -.5 and b.y0 >= -.5 and b.x1 <= width+.5 and b.y1 <= height+.5, (name, item.get_text(), b)
        bounds.append(dict(text=item.get_text(), font_size_pt=item.get_fontsize(), bbox_pixels=list(b.bounds)))
    for suffix in ['svg', 'pdf', 'png']:
        meta = {'Date': None} if suffix == 'svg' else ({'CreationDate': None, 'ModDate': None} if suffix == 'pdf' else None)
        path = OUT / f'{name}.{suffix}'
        fig.savefig(path, dpi=300, facecolor='white', metadata=meta)
        if suffix == 'svg':
            path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    svg = ET.parse(OUT / f'{name}.svg')
    live = sum(node.tag.endswith('}text') for node in svg.iter())
    assert live and not any(node.tag.endswith('}image') for node in svg.iter())
    source['evidence_sha256'] = hashlib.sha256((DATA / 'gaussian_paper_evidence.json').read_bytes()).hexdigest()
    (DATA / f'{name}.json').write_text(json.dumps(source, indent=2, allow_nan=False)+'\n')
    qa = dict(passed=True, dimensions_mm=list(fig.get_size_inches()*25.4),
              canvas_pixels=[width, height], text_bounds=bounds,
              editable_svg_text_elements=live, embedded_raster_images=0,
              point_count=source.get('point_count'), complete_sequence_points=True)
    (OUT / f'{name}_text_bounds.json').write_text(json.dumps(qa, indent=2)+'\n')
    plt.close(fig)
    print('Rendered', name, 'from', source.get('point_count', source.get('number_of_sequence_measurements', 0)), 'sequence measurements.')


def comparisons(data, component=False):
    if component:
        refs = ['marked_asymmetric', PRIMARY+'_no_curvature', PRIMARY+'_no_history', PRIMARY+'_no_mark']
        labels = ['Scalar', 'No curvature guard', 'No history switch', 'No positive score\nconstraint']
        lookup = {(r['condition'], r['reference']): r for r in data['components']}
        name, height, left, gap, aw = 'gaussian_components', 72, .265, .065, .315
    else:
        refs = ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_score', 'marked_ceiling_calibrated', 'marked_asymmetric']
        labels = [data['labels'][r] for r in refs]
        lookup = {(r['condition'], r['reference']): r for r in data['paired']}
        name, height, left, gap, aw = 'gaussian_paired', 82, .175, .065, .36
    groups = []
    for condition, _ in CONDITIONS:
        for reference in refs:
            summary = lookup[condition, reference]['ospa']
            if component:
                points = sorted([dict(sequence=r['sequence'], value=r['ospa']) for r in data['component_sequence_pairs']
                                 if r['condition'] == condition and r['reference'] == reference], key=lambda r: r['sequence'])
            else:
                row = lookup[condition, reference]
                points = [dict(sequence=s, value=v) for s, v in zip(row['sequences'], row['ospa_differences'])]
            assert len(points) == 25 and [r['sequence'] for r in points] == data['sequences']
            assert np.isclose(np.mean([p['value'] for p in points]), summary['mean'], atol=1e-12)
            groups.append(dict(condition=condition, reference=reference, summary=summary, points=points))
    all_values = [p['value'] for g in groups for p in g['points']]
    low, high = min(all_values), max(all_values)
    span = high-low
    limits = [low-.07*span, high+.07*span]
    fig = plt.figure(figsize=(181/25.4, height/25.4), dpi=300)
    axes = [fig.add_axes([left+i*(aw+gap), .22, aw, .64]) for i in range(2)]
    jitter = np.random.default_rng(1972).permutation(np.linspace(-.19, .19, 25))
    for i, ((condition, title), ax) in enumerate(zip(CONDITIONS, axes)):
        color = TEAL if i == 0 else BLUE
        ax.axvline(0, color=INK, lw=.75, zorder=1)
        for y, reference in enumerate(refs):
            row = next(g for g in groups if g['condition'] == condition and g['reference'] == reference)
            values = [p['value'] for p in row['points']]
            stats = row['summary']
            ax.axhline(y, color=GRID, lw=.55, zorder=0)
            ax.scatter(values, y+jitter, s=8, color=LIGHT, alpha=.85, edgecolors='none', zorder=2)
            ax.plot([stats['low'], stats['high']], [y, y], color=color, lw=2, solid_capstyle='round', zorder=4)
            ax.scatter([stats['mean']], [y], s=31, color=color, edgecolors='white', linewidths=.55, zorder=5)
        ax.set(xlim=limits, ylim=(len(refs)-.5, -.5), yticks=range(len(refs)),
               yticklabels=labels if i == 0 else ['']*len(refs))
        ax.set_title(title, pad=10)
        ax.set_xlabel('GCE − control OSPA (m)', labelpad=5)
        for edge in ['top', 'right', 'left']:
            ax.spines[edge].set_visible(False)
        ax.xaxis.set_major_locator(mpl.ticker.MaxNLocator(5, min_n_ticks=3))
    legend = [Line2D([], [], marker='o', color='none', markerfacecolor=LIGHT, markeredgecolor='none', markersize=3, label='One sequence'),
              Line2D([], [], marker='o', color=INK, markerfacecolor=INK, markersize=4, linewidth=1.6, label='Mean and 95% interval')]
    fig.legend(handles=legend, loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(.57, -.004), columnspacing=2)
    export(fig, name, dict(kind='paired_sequence_ospa', groups=groups, point_count=len(all_values),
        x_limits=limits, y_jitter=jitter.tolist(), point_unit='complete sequence',
        interval='10000 percentile bootstrap resamples of the 25 paired sequences',
        development_corpus=True, negative_favors='GCE'))


def communication(data):
    aggregate = {(r['condition'], r['arm']): r for r in data['aggregate']}
    names = ['No-age\nKLA', 'Scalar', 'GCE\nfull', 'GCE\ncodec']
    keys = ['marked_lineage', 'marked_asymmetric', 'full', PRIMARY]
    rows = []
    for condition, _ in CONDITIONS:
        for key, label in zip(keys, names):
            group = [r for r in data['runs'] if r['condition'] == condition and r['arm'] == (PRIMARY if key == 'full' else key)]
            assert len(group) == 25
            means = {metric: float(np.mean([r[('full_gaussian_' if key == 'full' else '')+metric] for r in group])/2**20)
                     for metric in ['raw_bytes', 'wire_bytes']}
            rows.append(dict(condition=condition, arm=key, label=label.replace('\n', ' '), means_mib=means,
                sequences=[dict(sequence=r['sequence'], raw_bytes=r[('full_gaussian_' if key == 'full' else '')+'raw_bytes'],
                                wire_bytes=r[('full_gaussian_' if key == 'full' else '')+'wire_bytes']) for r in group]))
    fig = plt.figure(figsize=(181/25.4, 66/25.4), dpi=300)
    axes = [fig.add_axes([.095+i*.48, .27, .40, .57]) for i in range(2)]
    ymax = max(r['means_mib']['wire_bytes'] for r in rows)*1.15
    for i, ((condition, title), ax) in enumerate(zip(CONDITIONS, axes)):
        for off, metric, color in [(-.17, 'raw_bytes', TEAL), (.17, 'wire_bytes', BLUE)]:
            values = [next(r for r in rows if r['condition'] == condition and r['arm'] == key)['means_mib'][metric] for key in keys]
            bars = ax.bar(np.arange(4)+off, values, width=.29, color=color, zorder=3)
            ax.bar_label(bars, labels=[f'{v:.2f}' for v in values], padding=3, fontsize=7)
        ax.set(ylim=(0, ymax), xticks=range(4), xticklabels=names)
        ax.set_title(title, pad=10)
        ax.set_ylabel('Mean MiB per sequence', labelpad=4)
        ax.yaxis.set_major_locator(mpl.ticker.MaxNLocator(4, integer=True))
        ax.yaxis.grid(True, color=GRID, lw=.55, zorder=0)
        for edge in ['top', 'right']:
            ax.spines[edge].set_visible(False)
    fig.legend(handles=[Patch(facecolor=TEAL, label='Raw payload'), Patch(facecolor=BLUE, label='Fragmented + control')],
               loc='lower center', bbox_to_anchor=(.54, -.005), ncol=2, frameon=False, columnspacing=2)
    export(fig, 'gaussian_communication', dict(kind='actual_native_bytes', rows=rows,
        number_of_sequence_measurements=200, sequence_count=25, bytes_per_mib=2**20,
        full_and_codec_trajectory_parity=True, modeled_fragment_size_bytes=16384))


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    evidence = json.loads((DATA / 'gaussian_paper_evidence.json').read_text())
    comparisons(evidence)
    comparisons(evidence, component=True)
    communication(evidence)
