"""Render every declared phase comparison and the complete fixed-input table."""
from pathlib import Path
import hashlib
import json

import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from make_gaussian_figures import export, INK, BLUE, GRID

HERE = Path(__file__).resolve().parent
DATA = HERE/'source_data'
GEN = HERE/'generated'


def main():
    data = json.loads((DATA/'mechanism_analysis.json').read_text())
    phases = ['before', 'outage', 'after']
    refs = ['marked_lineage', 'marked_asymmetric']
    conditions = ['reliable', 'intermittent']
    lookup = {(r['condition'], r['phase'], r['reference']): r for r in data['phase_paired']}
    fig = plt.figure(figsize=(89/25.4, 73/25.4), dpi=300)
    axes = [fig.add_axes([.205, .57-i*.43, .76, .30]) for i in range(2)]
    styles = [(BLUE, 'o', -.075), ('#7e8a91', 'D', .075)]
    for index, (condition, ax) in enumerate(zip(conditions, axes)):
        ax.axhspan(-.65, 0, color='#f7f9f9', zorder=0)
        ax.axhline(0, color=INK, lw=.65)
        for ref, (color, marker, offset) in zip(refs, styles):
            rows = [lookup[condition, phase, ref]['ospa'] for phase in phases]
            means = np.array([r['mean'] for r in rows])
            lower, upper = np.array([r['low'] for r in rows]), np.array([r['high'] for r in rows])
            assert len(rows) == 3 and all(r['n'] == 25 for r in rows)
            ax.errorbar(np.arange(3)+offset, means, yerr=[means-lower, upper-means],
                        fmt=marker, markersize=3.9, color=color, markeredgecolor='white',
                        markeredgewidth=.4, lw=.85, capsize=2.2, zorder=3)
        ax.set(xlim=(-.42, 2.42), ylim=(-.64, .16), xticks=range(3), yticks=[-.6, -.3, 0])
        ax.set_xticklabels(['First 40%', 'Middle 20%', 'Last 40%'] if index == 0 else ['Before', 'Outage', 'After'], fontsize=7)
        ax.tick_params(axis='x', length=0, pad=3)
        ax.tick_params(axis='y', length=2.5, pad=3, labelsize=7)
        ax.set_title('Reliable links' if index == 0 else 'Intermittent links', fontsize=8.2, pad=5)
        ax.spines[['top', 'right']].set_visible(False)
        ax.yaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda v, p: '0' if v == 0 else f'{v:.1f}'))
    fig.text(.034, .52, 'GCE − reference OSPA (m)', fontsize=7.4, rotation=90, ha='center', va='center')
    handles = [Line2D([], [], marker=m, color=c, lw=.8, markersize=3.6, label=label)
               for (c, m, _), label in zip(styles, ['vs No-age KLA', 'vs Scalar'])]
    fig.legend(handles=handles, loc='upper center', ncol=2, frameon=False,
               bbox_to_anchor=(.58, 1.014), fontsize=7.1, columnspacing=1.3, handletextpad=.5)
    export(fig, 'gaussian_phases', dict(kind='all_sequence_phase_differences',
        point_count=12, number_of_sequence_measurements=300, sequence_count=25,
        individual_points_displayed=False, intervals_shown=True, check_text_collisions=True,
        groups=data['phase_paired'], y_limits=[-.64, .16],
        metric='GCE minus reference position OSPA, p=2, cutoff=12m',
        interval=data['interval'], snapshot_sha256=data['snapshot_sha256'],
        mechanism_analysis_sha256=hashlib.sha256((DATA/'mechanism_analysis.json').read_bytes()).hexdigest(),
        phase_definition='Exact scan boundaries floor(0.4T), floor(0.6T); identical windows under reliable links.'))

    cells = ['base_space_base_integral', 'new_space_base_integral',
             'base_space_new_integral', 'new_space_new_integral']
    lookup = {(r['condition'], r['cell']): r['ospa'] for r in data['fixed_input_aggregate']}
    lines = [r'\begin{table}[t]', r'\centering',
             r'\caption{OSPA on identical GCE inputs (m). All cells share the admitted increments. Means cover delivered receiver-scans, with equal weight per sequence ($n=25$).}',
             r'\label{tab:fixedinput}', r'\small',
             r'\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}llrr@{}}',
             r'\toprule', r'Spatial mean & Existence integral & Reliable & Intermittent \\', r'\midrule']
    names = [('Base', 'Base'), ('Corrected', 'Base'), ('Base', 'Corrected'), ('Corrected', 'Corrected')]
    for index, (cell, (mean, integral)) in enumerate(zip(cells, names)):
        values = [f'{lookup[condition, cell]["mean"]:.3f}' for condition in conditions]
        if index == 3:
            lines += [r'\midrule']
            mean, integral = r'\textbf{'+mean+'}', r'\textbf{'+integral+'}'
            values = [r'\textbf{'+v+'}' for v in values]
        lines.append(' & '.join([mean, integral]+values)+r' \\')
    lines += [r'\bottomrule', r'\end{tabular*}', r'\end{table}', '']
    (GEN/'fixed_input_table.tex').write_text('\n'.join(lines))
    facts = {}
    for row in data['phase_paired']:
        suffix = row['condition'].title()+row['phase'].title()+('NoAge' if row['reference'] == refs[0] else 'Scalar')
        for key in ['mean', 'low', 'high']:
            facts['Phase'+suffix+key.title()] = row['ospa'][key]
    for row in data['fixed_input_paired']:
        short = {'base_space_base_integral': 'Both', 'new_space_base_integral': 'Integral', 'base_space_new_integral': 'Space'}[row['reference']]
        suffix = row['condition'].title()+short
        for key in ['mean', 'low', 'high']:
            facts['Joint'+suffix+key.title()] = row['ospa'][key]
    (GEN/'mechanism_facts.json').write_text(json.dumps(facts, indent=2)+'\n')
    macros = [f'\\newcommand{{\\{name}}}{{{value:.3f}}}' for name, value in facts.items()]
    (GEN/'mechanism_numbers.tex').write_text('\n'.join(macros)+'\n')
    print('Rendered 12 phase estimates and all 8 fixed-input means.')


if __name__ == '__main__':
    main()
