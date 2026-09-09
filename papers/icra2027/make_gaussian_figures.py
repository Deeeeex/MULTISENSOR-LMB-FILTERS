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
    collision_checked = source.get('check_text_collisions', False)
    if collision_checked:
        for i, left in enumerate(bounds):
            x, y, w, h = left['bbox_pixels']
            for right in bounds[i+1:]:
                xx, yy, ww, hh = right['bbox_pixels']
                intersection = [min(x+w, xx+ww)-max(x, xx), min(y+h, yy+hh)-max(y, yy)]
                assert min(intersection) <= .7, (name, 'text overlap', left['text'], right['text'], intersection)
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
              text_collisions_checked=collision_checked,
              point_count=source.get('point_count'), source_sequence_points_complete=True,
              individual_points_displayed=source.get('individual_points_displayed', source.get('kind') == 'paired_sequence_ospa'))
    (OUT / f'{name}_text_bounds.json').write_text(json.dumps(qa, indent=2)+'\n')
    plt.close(fig)
    print('Rendered', name, 'from', source.get('number_of_sequence_measurements', source.get('point_count', 0)), 'sequence measurements.')


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


def paired_sequence_gains(data):
    refs = ['marked_lineage', 'marked_asymmetric']
    lookup = {(r['condition'], r['reference']): r for r in data['paired']}
    panels = []
    fig = plt.figure(figsize=(181/25.4, 83/25.4), dpi=300)
    axes = [fig.add_axes([.095+i*.495, .20, .35, .69]) for i in range(2)]
    styles = {'both': dict(marker='o', color=TEAL, label='Improves both'),
              'one': dict(marker='D', color=BLUE, label='Improves one'),
              'neither': dict(marker='x', color='#a85f3d', label='Improves neither')}
    for i, (reference, ax) in enumerate(zip(refs, axes)):
        by_condition = {}
        for condition, _ in CONDITIONS:
            row = lookup[condition, reference]
            assert row['sequences'] == data['sequences']
            by_condition[condition] = dict(zip(row['sequences'], row['ospa_differences']))
        points = []
        for sequence in data['sequences']:
            x = -by_condition['reliable'][sequence]
            y = -by_condition['intermittent'][sequence]
            positive = int(x > 0)+int(y > 0)
            points.append(dict(sequence=sequence, reliable_gain_m=x,
                               intermittent_gain_m=y, outcome=['neither', 'one', 'both'][positive]))
        counts = {category: sum(p['outcome'] == category for p in points) for category in styles}
        assert len(points) == sum(counts.values()) == 25
        limits = [-.45, 1.20] if i == 0 else [-.20, .65]
        ticks = [-.4, 0, .4, .8, 1.2] if i == 0 else [-.2, 0, .2, .4, .6]
        ax.add_patch(mpl.patches.Rectangle((0, 0), limits[1], limits[1],
                     facecolor='#f0f7f5', edgecolor='none', zorder=0))
        ax.plot(limits, limits, color='#a9b7bd', lw=.65, ls=(0, (3, 3)), zorder=1)
        ax.axhline(0, color='#788b94', lw=.65, zorder=2)
        ax.axvline(0, color='#788b94', lw=.65, zorder=2)
        for category, style in styles.items():
            selected = [p for p in points if p['outcome'] == category]
            x = [p['reliable_gain_m'] for p in selected]
            y = [p['intermittent_gain_m'] for p in selected]
            if category == 'both':
                ax.scatter(x, y, s=19, marker=style['marker'], facecolor=style['color'],
                           edgecolor='white', lw=.5, alpha=.85, zorder=4)
            elif category == 'one':
                ax.scatter(x, y, s=22, marker=style['marker'], facecolor='white',
                           edgecolor=style['color'], lw=.9, zorder=5)
            else:
                ax.scatter(x, y, s=23, marker=style['marker'], color=style['color'],
                           lw=1.0, zorder=6)
        ax.set(xlim=limits, ylim=limits, xticks=ticks, yticks=ticks)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('Reliable-link OSPA gain (m)', fontsize=8, labelpad=6)
        ax.set_ylabel('Intermittent-link OSPA gain (m)', fontsize=8, labelpad=6)
        ax.tick_params(axis='y', length=3, width=.5, pad=4, labelsize=7.5)
        ax.xaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda v, pos: '0' if v == 0 else f'{v:.1f}'))
        ax.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda v, pos: '0' if v == 0 else f'{v:.1f}'))
        for edge in ['top', 'right']:
            ax.spines[edge].set_visible(False)
        center = .27+i*.495
        fig.text(center, .976, f'vs {data["labels"][reference]}', ha='center', va='center', fontsize=9.2, weight='bold')
        fig.text(center, .933, f'{counts["both"]}/25 improve in both', ha='center', va='center', fontsize=8, color=TEAL)
        fig.text(.044+i*.495, .976, 'ab'[i], ha='left', va='center', fontsize=10, weight='bold')
        panels.append(dict(reference=reference, label=data['labels'][reference], points=points,
                           outcome_counts=counts, x_limits=limits, y_limits=limits,
                           equal_axis_scale=True, coordinate_jitter=False))
    handles = [Line2D([], [], marker=style['marker'], linestyle='none',
                      markerfacecolor=style['color'] if category == 'both' else 'white',
                      markeredgecolor=style['color'], markeredgewidth=.9, markersize=4,
                      label=style['label']) for category, style in styles.items()]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False,
               bbox_to_anchor=(.52, .012), fontsize=7.8, columnspacing=2.2, handletextpad=.5)
    export(fig, 'gaussian_paired', dict(kind='paired_cross_condition_ospa_gain', panels=panels,
        point_count=50, number_of_sequence_measurements=100, sequence_count=25,
        individual_points_displayed=True, point_unit='one complete sequence paired across both link conditions',
        plotted_statistic='Reference OSPA minus GCE OSPA for each sequence and condition',
        reference_selection='No-age KLA is the inherited posterior pool; Scalar is the original unguarded existence-only reference. The added Guarded Scalar comparison is reported in the revised tables.',
        all_reference_companion='gaussian_sequence_differences', outcome_rule='Strictly positive gain in both, one, or neither condition',
        intervals_shown=False, positive_favors='GCE', development_corpus=True))


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
    fig = plt.figure(figsize=(89/25.4, 59/25.4), dpi=300)
    ax = fig.add_axes([.16, .20, .81, .68])
    ax.set(xlim=(3.30, 6.52), ylim=(3.34, 4.13), xticks=[3.5, 4.5, 5.5, 6.0],
           yticks=[3.4, 3.6, 3.8, 4.0])
    styles = {'marked_lineage': ('#798a95', 'No-age KLA'),
              'marked_asymmetric': (BLUE, 'Scalar'),
              'full': ('#82949e', 'Full'), PRIMARY: (TEAL, 'GCE')}
    for i, (condition, title) in enumerate(CONDITIONS):
        marker = 'o' if i == 0 else 'D'
        group = {r['arm']: r for r in rows if r['condition'] == condition}
        for key in keys:
            row = group[key]
            x, y = row['means_mib']['raw_bytes'], row['mean_ospa_m']
            color, label = styles[key]
            ax.scatter([x], [y], s=25 if key == PRIMARY else 21, marker=marker,
                       facecolor='white' if key == 'full' else color,
                       edgecolor=color, linewidth=.85, zorder=4)
            offset = (0, 7) if key == 'marked_lineage' else (0, -12)
            if key == 'marked_asymmetric' and i == 1:
                offset = (-3, 7)
            ax.annotate(label, (x, y), xytext=offset, textcoords='offset points', ha='center',
                        fontsize=7.1, color=TEAL if key == PRIMARY else INK,
                        weight='bold' if key == PRIMARY else 'normal')
        full, encoded = group['full'], group[PRIMARY]
        x0, x1, y = full['means_mib']['raw_bytes'], encoded['means_mib']['raw_bytes'], encoded['mean_ospa_m']
        ax.annotate('', xy=(x1+.10, y), xytext=(x0-.10, y),
                    arrowprops=dict(arrowstyle='->', color=TEAL, linewidth=.9), zorder=3)
        saving = 100*(1-x1/x0)
        ax.annotate(f'−{saving:.1f}%', ((x0+x1)/2, y), xytext=(0, 7),
                    textcoords='offset points', ha='center', color=TEAL, fontsize=7.2)
    ax.set_ylabel('OSPA (m)', labelpad=5, fontsize=7.5)
    ax.set_xlabel('Raw payload (MiB per sequence)', labelpad=5, fontsize=7.3)
    ax.tick_params(axis='both', labelsize=7, length=2.5)
    ax.yaxis.set_major_formatter(mpl.ticker.FormatStrFormatter('%.1f'))
    ax.yaxis.grid(True, color=GRID, lw=.45, zorder=0)
    ax.spines[['top', 'right']].set_visible(False)
    handles = [Line2D([], [], marker=marker, linestyle='none', color=INK,
                      markersize=3.5, label=title) for marker, title in [('o', 'Reliable'), ('D', 'Intermittent')]]
    fig.legend(handles=handles, loc='upper center', ncol=2, frameon=False,
               bbox_to_anchor=(.60, 1.018), fontsize=7.2, handletextpad=.2, columnspacing=1.0)
    export(fig, 'gaussian_communication', dict(kind='actual_native_bytes_and_accuracy', rows=rows,
        number_of_sequence_measurements=200, sequence_count=25, bytes_per_mib=2**20,
        full_and_codec_trajectory_parity=True, modeled_fragment_size_bytes=16384,
        axes='Native raw bytes versus complete-sequence OSPA; common scales for both link conditions',
        check_text_collisions=True,
        x_limits=[3.30, 6.52], y_limits=[3.34, 4.13], condition_markers=['o', 'D']))



if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    evidence = json.loads((DATA / 'gaussian_paper_evidence.json').read_text())
    paired_sequence_gains(evidence)
    comparisons(evidence)
    comparisons(evidence, component=True)
    communication(evidence)
