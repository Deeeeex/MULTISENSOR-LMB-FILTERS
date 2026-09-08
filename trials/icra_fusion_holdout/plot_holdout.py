"""Plot every registered paired unit from the complete independent audit."""
from pathlib import Path
import csv
import hashlib
import json
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

OUT = Path(__file__).resolve().parent
DEST = OUT / 'figures'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    summary = json.loads((OUT / 'summary_holdout.json').read_text())
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    assert summary['audited_node_frames'] == 358464 and summary['sequences'] == 25
    assert summary['method_freeze_sha256'] == sha(OUT / 'METHOD_FREEZE.json')
    primary = freeze['primary']
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in summary['runs']}
    seqs = [f'{s:04d}' for s in freeze['units']]
    comparisons = [(condition, reference) for condition in freeze['conditions'] for reference in freeze['primary_references']]
    pairs = {(r['condition'], r['reference']): r for r in summary['paired'] if r['candidate'] == primary}
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'DejaVu Sans'],
                         'font.size': 7, 'axes.labelsize': 7, 'axes.titlesize': 8,
                         'xtick.labelsize': 6.5, 'ytick.labelsize': 7, 'legend.fontsize': 6.5,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.linewidth': .6, 'xtick.major.width': .6, 'ytick.major.width': .6,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none',
                         'svg.hashsalt': 'evidence-ceiling-complete-cohort', 'savefig.facecolor': 'white'})
    DEST.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(183 / 25.4, 90 / 25.4), sharey=True,
                             gridspec_kw={'width_ratios': [1.13, 1]})
    fig.subplots_adjust(left=.175, right=.98, bottom=.17, top=.79, wspace=.20)
    points, bars = [], []
    jitter = np.linspace(-.065, .065, len(seqs))

    def draw(ax, panel, y, values, estimate, metric, condition, reference, color, marker, offset=0):
        assert len(values) == 25 and np.isclose(np.mean(values), estimate['mean'], atol=1e-12)
        ax.scatter(values, y + offset + jitter, s=11, facecolors='none', edgecolors=color,
                   linewidths=.5, marker=marker, alpha=.40, zorder=2)
        ax.plot([estimate['low'], estimate['high']], [y + offset] * 2, color=color, lw=1.6, zorder=4)
        ax.scatter([estimate['mean']], [y + offset], color=color, marker=marker, s=27, zorder=5)
        for seq, value in zip(seqs, values):
            points.append(dict(panel=panel, metric=metric, condition=condition, candidate=primary,
                               reference=reference, sequence=seq, difference=value))
        bars.append(dict(panel=panel, metric=metric, condition=condition, candidate=primary, reference=reference,
                         n=25, mean=estimate['mean'], low=estimate['low'], high=estimate['high']))

    labels = []
    for y, (condition, reference) in enumerate(comparisons):
        name = 'M-No-age' if reference == 'marked_lineage' else 'M-ER'
        labels.append(f"vs {name}\n{condition.capitalize()}")
        pair = pairs[condition, reference]
        delta = [lookup[seq, condition, primary]['ospa'] - lookup[seq, condition, reference]['ospa'] for seq in seqs]
        draw(axes[0], 'a', y, delta, pair['ospa'], 'ospa', condition, reference, '#147D92', 'o')
        for metric, color, marker, offset in [('miss2', '#70539B', '^', -.14), ('false2', '#AF6D17', 's', .14)]:
            delta = [lookup[seq, condition, primary][metric] - lookup[seq, condition, reference][metric] for seq in seqs]
            draw(axes[1], 'b', y, delta, pair[metric], metric, condition, reference, color, marker, offset)
    for ax, panel, title in zip(axes, ['a', 'b'], ['Paired set error', 'Missed / false-target costs']):
        ax.axvline(0, color='#666666', lw=.65, ls=(0, (3, 3)), zorder=1)
        ax.grid(axis='x', color='#E7E7E7', lw=.4, zorder=0)
        ax.set_ylim(3.5, -.5)
        ax.tick_params(axis='y', length=0, pad=7)
        ax.set_title(title, loc='left', pad=13)
        ax.text(-.04, 1.10, panel, transform=ax.transAxes, fontsize=9, weight='bold')
        left, right = ax.get_xlim()
        ax.set_xlim(min(left, -.03 * max(abs(left), abs(right))), max(right, .03 * max(abs(left), abs(right))))
    axes[0].set_yticks(range(4), labels)
    axes[0].set_xlabel('OSPA difference (m)')
    axes[1].set_xlabel('Squared-cost difference (m²)')
    axes[1].legend(handles=[Line2D([], [], color='#70539B', marker='^', lw=1.2, ms=3, label='Missed'),
                            Line2D([], [], color='#AF6D17', marker='s', lw=1.2, ms=3, label='False')],
                   frameon=False, loc='upper right', bbox_to_anchor=(1, 1.11), ncol=2,
                   borderaxespad=0, handlelength=1.0, columnspacing=.8)
    fig.text(.175, .955, 'Evidence ceiling versus matched-information controls', fontsize=10, weight='bold', va='top')
    fig.text(.175, .911, 'M-ECR-S minus comparator; 25 sequences, 5,601 paired frames', fontsize=7, va='top')
    fig.text(.175, .05, 'Open markers: sequences. Filled markers / lines: macro mean / descriptive 95% bootstrap interval.', fontsize=6.5)
    fig.text(.175, .014, 'Negative values favor M-ECR-S. Detection calibration is fixed from development sequences.', fontsize=6.5)
    for name, rows in [('source_points.csv', points), ('source_intervals.csv', bars)]:
        with (DEST / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, list(rows[0]), lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
    assert len(points) == 300 and len(bars) == 12
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    texts = list(fig.texts)
    for ax in axes:
        texts += list(ax.texts) + [ax.title, ax._left_title, ax._right_title, ax.xaxis.label]
        texts += [tick.label1 for tick in ax.xaxis._update_ticks()]
        texts += [tick.label1 for tick in ax.yaxis._update_ticks()]
        if ax.get_legend():
            texts += ax.get_legend().get_texts()
    bounds = []
    for text in texts:
        if not text.get_visible() or not text.get_text():
            continue
        box = text.get_window_extent(renderer)
        assert box.x0 >= -.5 and box.y0 >= -.5 and box.x1 <= fig.bbox.width + .5 and box.y1 <= fig.bbox.height + .5, text.get_text()
        bounds.append(dict(text=text.get_text(), bbox=list(box.extents)))
    for suffix in ['pdf', 'svg', 'png']:
        fig.savefig(DEST / f'paired_holdout.{suffix}', dpi=600)
    svg = ET.parse(DEST / 'paired_holdout.svg')
    live = [x for x in svg.iter() if x.tag.endswith('}text')]
    assert live and not any(x.tag.endswith('}image') for x in svg.iter())
    qa = dict(passed=True, width_mm=183, height_mm=90, source_points=300, source_intervals=12,
              drawn_text_bounds=bounds, live_svg_text_elements=len(live), no_embedded_raster_in_svg=True,
              summary_sha256=sha(OUT / 'summary_holdout.json'), method_freeze_sha256=sha(OUT / 'METHOD_FREEZE.json'),
              plot_script_sha256=sha(Path(__file__)), source_csv_sha256={name: sha(DEST / name) for name in ['source_points.csv', 'source_intervals.csv']})
    (DEST / 'qa.json').write_text(json.dumps(qa, indent=2, allow_nan=False) + '\n')
    print('FIGURE GENERATED: 300/300 sequence points; 12 registered intervals; SVG/PDF/PNG; inspect final appearance.')


if __name__ == '__main__':
    main()
