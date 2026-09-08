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
INK, TEAL, BLUE, LIGHT, GRID = '#263c48', '#007f73', '#446c99', '#b0bec5', '#e7ecef'
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
              point_count=source.get('point_count'), source_sequence_points_complete=True,
              individual_points_displayed=source.get('individual_points_displayed', source.get('kind') == 'paired_sequence_ospa'))
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
        name, height, left, gap, aw = 'gaussian_sequence_differences', 82, .175, .065, .36
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


def paired_summary(data):
    refs = ['marked_lineage', 'marked_er', 'marked_conservative',
            'marked_ceiling_score', 'marked_ceiling_calibrated', 'marked_asymmetric']
    lookup = {(r['condition'], r['reference']): r for r in data['paired']}
    groups = []
    fig = plt.figure(figsize=(181/25.4, 59/25.4), dpi=300)
    axes = [fig.add_axes([.215+i*.405, .245, .355, .62]) for i in range(2)]
    limits = [-.425, .025]
    for i, ((condition, title), ax) in enumerate(zip(CONDITIONS, axes)):
        color, marker = (TEAL, 'o') if i == 0 else (BLUE, 'D')
        for y, reference in enumerate(refs):
            row = lookup[condition, reference]
            stats = row['ospa']
            points = [dict(sequence=seq, value=value) for seq, value in zip(row['sequences'], row['ospa_differences'])]
            assert len(points) == 25 and [p['sequence'] for p in points] == data['sequences']
            assert np.isclose(np.mean([p['value'] for p in points]), stats['mean'], atol=1e-12)
            assert limits[0] < stats['low'] < stats['high'] < limits[1]
            groups.append(dict(condition=condition, reference=reference, summary=stats, points=points))
            if y % 2 == 0:
                ax.axhspan(y-.46, y+.46, color='#f4f7f8', lw=0, zorder=0)
            ax.plot([stats['low'], stats['high']], [y, y], color=color, lw=1.6, solid_capstyle='round', zorder=3)
            ax.plot([stats['low'], stats['low']], [y-.09, y+.09], color=color, lw=.7)
            ax.plot([stats['high'], stats['high']], [y-.09, y+.09], color=color, lw=.7)
            ax.scatter([stats['mean']], [y], s=25, marker=marker, facecolor=color,
                       edgecolor='white', linewidth=.6, zorder=4)
        ax.axvline(0, color=INK, lw=.65, linestyle=(0, (3, 2)), zorder=2)
        ax.set(xlim=limits, ylim=(5.55, -.55), xticks=[-.4, -.2, 0], yticks=range(6),
               yticklabels=[data['labels'][r] for r in refs] if i == 0 else ['']*6)
        ax.set_title(title, fontsize=9, color=color, pad=9)
        ax.set_xlabel('GCE − reference OSPA (m)', fontsize=8, labelpad=5)
        for edge in ['top', 'right', 'left']:
            ax.spines[edge].set_visible(False)
        ax.tick_params(axis='y', pad=9, labelsize=8)
    fig.text(.61, .035, '←  Lower OSPA with GCE', ha='center', va='center', fontsize=7.7, color=INK)
    export(fig, 'gaussian_paired', dict(kind='paired_sequence_ospa', groups=groups, point_count=300,
        rendered_estimate_count=12, plotted_statistic='Equal-sequence mean and paired 95% percentile interval',
        individual_points_displayed=False, individual_points_companion='gaussian_sequence_differences',
        x_limits=limits, point_unit='complete sequence',
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
    for row in rows:
        arm = PRIMARY if row['arm'] == 'full' else row['arm']
        row['mean_ospa_m'] = aggregate[row['condition'], arm]['ospa']['mean']
    fig = plt.figure(figsize=(181/25.4, 57/25.4), dpi=300)
    axes = [fig.add_axes([.095+i*.495, .255, .38, .59]) for i in range(2)]
    styles = {'marked_lineage': ('^', '#798a95', 'No-age KLA'),
              'marked_asymmetric': ('s', BLUE, 'Scalar'),
              'full': ('o', '#82949e', 'GCE, full'),
              PRIMARY: ('o', TEAL, 'GCE')}
    for i, ((condition, title), ax) in enumerate(zip(CONDITIONS, axes)):
        group = {r['arm']: r for r in rows if r['condition'] == condition}
        low = min(r['mean_ospa_m'] for r in group.values())
        high = max(r['mean_ospa_m'] for r in group.values())
        ax.set(xlim=(3.38, 6.38), ylim=(low-.080, high+.065), xticks=[3.5, 4.5, 5.5, 6.0])
        for key in keys:
            row = group[key]
            x, y = row['means_mib']['raw_bytes'], row['mean_ospa_m']
            marker, color, label = styles[key]
            ax.scatter([x], [y], s=43 if key == PRIMARY else 30, marker=marker,
                       facecolor='white' if key == 'full' else color,
                       edgecolor=color, linewidth=1, zorder=4)
            offset = (0, 9) if key == 'marked_lineage' else (0, -14)
            ax.annotate(label, (x, y), xytext=offset, textcoords='offset points', ha='center',
                        fontsize=8, color=TEAL if key == PRIMARY else INK,
                        weight='bold' if key == PRIMARY else 'normal')
        full, encoded = group['full'], group[PRIMARY]
        x0, x1, y = full['means_mib']['raw_bytes'], encoded['means_mib']['raw_bytes'], encoded['mean_ospa_m']
        ax.annotate('', xy=(x1+.10, y), xytext=(x0-.10, y),
                    arrowprops=dict(arrowstyle='->', color=TEAL, linewidth=1.05), zorder=3)
        saving = 100*(1-x1/x0)
        ax.annotate(f'−{saving:.1f}% payload', ((x0+x1)/2, y), xytext=(0, 9),
                    textcoords='offset points', ha='center', color=TEAL, fontsize=8)
        ax.set_title(title, pad=9, fontsize=9)
        ax.set_ylabel('OSPA (m)', labelpad=5)
        ax.set_xlabel('Raw payload (MiB per sequence)', labelpad=6, fontsize=8)
        ax.yaxis.set_major_locator(mpl.ticker.MaxNLocator(4))
        ax.yaxis.set_major_formatter(mpl.ticker.FormatStrFormatter('%.1f'))
        ax.yaxis.grid(True, color=GRID, lw=.45, zorder=0)
        for edge in ['top', 'right']:
            ax.spines[edge].set_visible(False)
    export(fig, 'gaussian_communication', dict(kind='actual_native_bytes_and_accuracy', rows=rows,
        number_of_sequence_measurements=200, sequence_count=25, bytes_per_mib=2**20,
        full_and_codec_trajectory_parity=True, modeled_fragment_size_bytes=16384,
        axes='Native raw bytes versus complete-sequence OSPA; condition-specific OSPA scales'))



if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    evidence = json.loads((DATA / 'gaussian_paper_evidence.json').read_text())
    paired_summary(evidence)
    comparisons(evidence)
    comparisons(evidence, component=True)
    communication(evidence)
