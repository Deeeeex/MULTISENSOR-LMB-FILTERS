"""Render source-backed recursive controls, new data, and model sensitivity."""
from pathlib import Path
import hashlib
import json

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from make_gaussian_tables import write_table
from make_gaussian_figures import export, INK, TEAL, BLUE, GRID

HERE = Path(__file__).resolve().parent
DATA, GEN = HERE / 'source_data', HERE / 'generated'
GCE = 'marked_gaussian_evidence'
GS, FIXED = GCE + '_guarded_scalar', GCE + '_fixed_025'
ARMS = ['marked_lineage', 'marked_asymmetric', GS, GCE]
LABELS = dict(zip(ARMS, ['No-age KLA', 'Scalar', 'Guarded Scalar', 'GCE']))
CONDITIONS = ['reliable', 'intermittent']


def lookup(rows):
    return {(row['condition'], row['arm']): row for row in rows}


def tables(evidence, original, native):
    merged = lookup(original['aggregate'])
    controls = lookup(evidence['controls']['cohorts']['seen_transfer']['aggregate'])
    for condition in CONDITIONS:
        for arm in [GS, FIXED]:
            merged[condition, arm] = controls[condition, arm]
    methods = original['methods'][:-1] + [GS, FIXED, GCE]
    labels = dict(original['labels'], **{GS: 'Guarded Scalar', FIXED: 'Fixed Ratio (0.25)'})
    rows = [(labels[arm], [merged[condition, arm][metric]['mean']
             for condition in CONDITIONS for metric in ['ospa', 'miss2', 'false2']]) for arm in methods]
    write_table('main_table', 'Comparison on 25 previously used sequences with a shared local tracker. OSPA is in m; missed/false GOSPA costs are in m$^2$. Lowest values are bold.',
                'tab:main', rows, ['OSPA', 'Missed', 'False'], 'GCE', separators=(4, 12))
    ablations = [('marked_asymmetric', 'Scalar reference'), (GS, 'Guarded Scalar'),
                 (GCE + '_no_curvature', 'w/o curvature guard'),
                 (GCE + '_no_history', 'w/o history switch'),
                 (GCE + '_no_mark', 'w/o score constraint'), (GCE, 'GCE (complete)')]
    values = lookup(native['aggregate'])
    for condition in CONDITIONS:
        values[condition, GS] = controls[condition, GS]
    rows = [(label, [values[condition, arm][metric]['mean']
             for condition in CONDITIONS for metric in ['ospa', 'miss2', 'false2']]) for arm, label in ablations]
    write_table('ablation_table', 'Recursive controls on the same 25 sequences. Scalar, Guarded Scalar, w/o curvature, and GCE form the scalar/joint $\\times$ guard-off/on comparison. Other rows remove one admission component; units follow Table~\\ref{tab:main}.',
                'tab:ablation', rows, ['OSPA', 'Missed', 'False'], 'GCE (complete)', separators=(3, 5))
    groups = evidence['new_data']['groups']
    all_segments = lookup(groups['all_new_segments']['aggregate'])
    new_recording = lookup(groups['new_recording']['aggregate'])
    methods = ['marked_lineage', 'marked_er', 'marked_asymmetric', GS, GCE + '_no_curvature', FIXED, GCE]
    names = dict(LABELS, **{'marked_er': 'Recency', GCE + '_no_curvature': 'w/o curvature', FIXED: 'Fixed Ratio (0.25)'})
    lines = [r'\begin{table}[t]', r'\centering\small',
        r'\caption{OSPA on newly acquired data (m). All three segments total 748 frames; only the 409-frame segment is from a recording absent from the earlier archives. R/I denote reliable/intermittent links.}',
        r'\label{tab:newdata}', r'\setlength{\tabcolsep}{3pt}',
        r'\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lrrrr@{}}', r'\toprule',
        r'& \multicolumn{2}{c}{All 3 segments} & \multicolumn{2}{c}{New recording} \\',
        r'\cmidrule(lr){2-3}\cmidrule(lr){4-5}', r'Method & R & I & R & I \\', r'\midrule']
    numbers = [[source[condition, arm]['ospa'] for source in [all_segments, new_recording] for condition in CONDITIONS] for arm in methods]
    minima = np.min(numbers, axis=0)
    for arm, values in zip(methods, numbers):
        if arm == GCE:
            lines.append(r'\midrule')
        cells = [r'\textbf{GCE}' if arm == GCE else names[arm]]
        for index, value in enumerate(values):
            cell = f'{value:.3f}'
            if np.isclose(value, minima[index], rtol=0, atol=1e-12):
                cell = r'\textbf{' + cell + '}'
            cells.append(cell)
        lines.append(' & '.join(cells) + r' \\')
    lines += [r'\bottomrule', r'\end{tabular*}', r'\end{table}', '']
    (GEN / 'new_data_table.tex').write_text('\n'.join(lines))


def facts(evidence):
    values, decimals = {}, {}
    def add(name, value, digits=3):
        values[name], decimals[name] = float(value), digits
    for cohort, group in evidence['controls']['cohorts'].items():
        cohort_name = 'Seen' if cohort == 'seen_transfer' else 'Dev'
        for row in group['paired']:
            if row['candidate'] == GCE and row['reference'] in [GS, FIXED]:
                arm = 'GS' if row['reference'] == GS else 'Fixed'
                for statistic in ['mean', 'low', 'high']:
                    digits = 5 if arm == 'GS' and statistic != 'mean' else 3
                    add('Rev' + cohort_name + 'Delta' + arm + row['condition'].title() + statistic.title(), row['ospa'][statistic], digits)
        for row in group['interaction']:
            for statistic in ['mean', 'low', 'high']:
                add('Rev' + cohort_name + 'Interaction' + row['condition'].title() + statistic.title(), row['ospa'][statistic])
    short = {'marked_lineage': 'NoAge', 'marked_asymmetric': 'Scalar', GS: 'GS', GCE: 'GCE'}
    for key, group in evidence['new_data']['groups'].items():
        prefix = {'all_new_segments': 'NewAll', 'new_recording': 'NewRecording', 'related_recording_segments': 'Related'}[key]
        for row in group['aggregate']:
            if row['arm'] in short:
                add('Rev' + prefix + short[row['arm']] + row['condition'].title(), row['ospa'])
    for cohort, group in evidence['sensitivity']['motion'].items():
        prefix = {'development': 'Dev', 'seen_transfer': 'Seen', 'new_validation': 'New'}[cohort]
        for row in group['aggregate']:
            add('RevMotion' + prefix + short[row['arm']] + row['condition'].title(), row['ospa']['mean'])
        for row in group['paired']:
            for statistic in ['mean', 'low', 'high']:
                if statistic in row['ospa']:
                    add('RevMotion' + prefix + 'Delta' + short[row['reference']] + row['condition'].title() + statistic.title(), row['ospa'][statistic])
        if cohort == 'new_validation':
            for row in group['rows']:
                if row['sequence'] == '0000':
                    add('RevMotionNewRecording' + short[row['arm']] + row['condition'].title(), row['ospa'])
    for row in evidence['correlation']['rows']:
        if row['rho'] == .9 and row['arm'] in ['Unit-admission GCE', 'Known-correlation oracle']:
            arm = 'GCE' if row['arm'].startswith('Unit') else 'Oracle'
            add('RevCorrelation' + arm + 'Coverage', 100 * row['coverage_95'], 2)
            add('RevCorrelation' + arm + 'NEES', row['mean_nees_per_dimension'], 3)
    (GEN / 'reviewer_facts.json').write_text(json.dumps(dict(values=values, decimal_places=decimals), indent=2) + '\n')
    (GEN / 'reviewer_numbers.tex').write_text('% Generated from verified reviewer experiment rows.\n' +
        '\n'.join(f'\\newcommand{{\\{name}}}{{{value:.{decimals[name]}f}}}' for name, value in values.items()) + '\n')


def robustness(evidence):
    fig = plt.figure(figsize=(181 / 25.4, 70 / 25.4), dpi=300)
    axes = [fig.add_axes(rect) for rect in [[.065, .22, .255, .57], [.385, .22, .255, .57], [.725, .22, .257, .57]]]
    # Qualitative hues inspired by Paul Tol; teal retains the paper's GCE identity.
    gray, blue, rose, ochre = '#84929C', '#4477AA', '#CC6677', '#A47C2C'
    styles = [
        ('marked_lineage', gray, 's', (0, (4, 2)), 1.1),
        ('marked_asymmetric', blue, 'D', (0, (2, 1.5)), 1.1),
        (GS, rose, '^', (0, (5, 1.5, 1, 1.5)), 1.1),
        (GCE, TEAL, 'o', '-', 1.65),
    ]
    probabilities = [.7, .8, .9, .95]
    pd_rows = []
    for pd in probabilities:
        group = evidence['sensitivity']['detection_probability'][str(pd)]
        for row in group['aggregate']:
            pd_rows.append(dict(pd=pd, condition=row['condition'], arm=row['arm'], ospa=row['ospa']['mean'],
                                sequence_values=row['ospa']['values']))
    mapped = {(row['pd'], row['condition'], row['arm']): row for row in pd_rows}
    endpoints = [row['ospa'] for row in pd_rows]
    limits = [np.floor((min(endpoints) - .04) * 10) / 10, np.ceil((max(endpoints) + .04) * 10) / 10]

    def line_style(color, marker, dash, width, primary=False):
        return dict(color=color, marker=marker, linestyle=dash, linewidth=width,
                    markersize=4.2 if primary else 4.0,
                    markerfacecolor=color if primary else 'white',
                    markeredgecolor='white' if primary else color,
                    markeredgewidth=.65 if primary else .8,
                    solid_capstyle='round', dash_capstyle='round', zorder=5 if primary else 3)

    for index, condition in enumerate(CONDITIONS):
        ax = axes[index]
        for arm, color, marker, dash, width in styles:
            values = [mapped[pd, condition, arm]['ospa'] for pd in probabilities]
            ax.plot(probabilities, values, **line_style(color, marker, dash, width, arm == GCE),
                    label=LABELS[arm], gid='pd_' + condition + '_' + arm)
        ax.set(xlim=(.68, .97), ylim=limits, xticks=probabilities, yticks=[3.8, 4.0, 4.2, 4.4])
        ax.set_xticklabels(['0.70', '0.80', '0.90', '0.95'], fontsize=7.4)
        ax.set_yticklabels(['3.8', '4.0', '4.2', '4.4'], fontsize=7.4)
        ax.set_xlabel('Modeled detection probability', fontsize=7.4, labelpad=6)
        if index == 0:
            ax.set_ylabel('Mean OSPA (m)', fontsize=7.6, labelpad=6)
    fig.legend(handles=[Line2D([], [], **line_style(color, marker, dash, width, arm == GCE), label=LABELS[arm])
                        for arm, color, marker, dash, width in styles],
               loc='upper center', ncol=4, frameon=False, bbox_to_anchor=(.35, .987),
               fontsize=7.6, columnspacing=1.15, handlelength=2, handletextpad=.55)

    ax = axes[2]
    correlations = evidence['correlation']['specification']['rhos']
    correlation_styles = [
        ('No-age spatial pool', gray, 's', (0, (4, 2)), 1.1),
        ('Known-correlation oracle', ochre, '^', (0, (2, 1.5)), 1.1),
        ('Unit-admission GCE', TEAL, 'o', '-', 1.65),
    ]
    for arm, color, marker, dash, width in correlation_styles:
        rows = [next(row for row in evidence['correlation']['rows'] if row['arm'] == arm and row['rho'] == rho) for rho in correlations]
        ax.plot(correlations, [row['coverage_95'] for row in rows],
                **line_style(color, marker, dash, width, arm == 'Unit-admission GCE'),
                gid='correlation_' + arm.replace(' ', '_'))
    ax.axhline(.95, color='#69757D', lw=.7, ls=(0, (5, 3)), zorder=1)
    ax.set(xlim=(-.035, .94), ylim=(.76, 1.015), xticks=[0, .25, .5, .75, .9], yticks=[.8, .85, .9, .95, 1.0])
    ax.set_xticklabels(['0', '0.25', '0.50', '0.75', '0.90'], fontsize=7.4)
    ax.set_yticklabels(['80', '85', '90', '95', '100'], fontsize=7.4)
    ax.set_xlabel('Cross-source correlation', fontsize=7.4, labelpad=6)
    ax.set_ylabel('95% region coverage (%)', fontsize=7.6, labelpad=5)
    ax.text(.92, .997, 'Spatial pool', ha='right', va='center', color='#657784', fontsize=7.4)
    ax.text(.035, .966, 'Nominal 95%', ha='left', va='center', color='#69757D', fontsize=7.3)
    ax.text(.92, .925, 'Known-correlation\noracle', ha='right', va='center',
            color=ochre, fontsize=7.4, linespacing=1.1)
    ax.text(.025, .816, 'Unit-admission GCE', ha='left', va='center', color=TEAL, fontsize=7.4)
    anchor = next(row for row in evidence['correlation']['rows']
                  if row['arm'] == 'Unit-admission GCE' and row['rho'] == .75)
    ax.plot([.61, anchor['rho']], [.817, anchor['coverage_95']],
            color=TEAL, lw=.65, zorder=2, gid='gce_label_leader')
    last = next(row for row in evidence['correlation']['rows'] if row['arm'] == 'Unit-admission GCE' and row['rho'] == .9)
    ax.text(.92, .781, f"{100 * last['coverage_95']:.2f}%", ha='right', va='center', color=TEAL, fontsize=7.4, weight='bold')
    for ax in axes:
        ax.tick_params(axis='both', labelsize=7.4, length=2.5, width=.55, pad=3)
        ax.spines[['top', 'right']].set_visible(False)
        ax.spines[['left', 'bottom']].set_color('#849098')
        ax.spines[['left', 'bottom']].set_linewidth(.55)
        ax.set_axisbelow(True)
        ax.yaxis.grid(color='#E6EAED', lw=.5)
    for x, letter, title in zip([.065, .385, .725], ['a', 'b', 'c'],
                               ['Reliable links', 'Intermittent links', 'Correlated Gaussian control']):
        fig.text(x - .025, .852, letter, fontsize=10, weight='bold', ha='left', va='center')
        fig.text(x + .004, .852, title, fontsize=8.4, weight='bold', ha='left', va='center')
    source = dict(kind='registered_pd_and_correlation_sensitivity', pd_rows=pd_rows,
        correlation_rows=evidence['correlation']['rows'], pd_sequence_count=9, pd_point_count=32,
        correlation_point_count=15, point_count=47, number_of_sequence_measurements=288,
        individual_points_displayed=False, intervals_shown=False, check_text_collisions=True,
        pd_y_limits=limits, coverage_y_limits=[.76, 1.015],
        averaging='Equal complete-sequence means for pD; 10000 independent joint Gaussian samples per correlation setting.',
        correlation_scope='Conditional-existing spatial model with unit admission and all source curvature checks accepted; no real-tracker consistency claim.',
        style_revision=dict(palette={LABELS[arm]: color for arm, color, _, _, _ in styles},
            oracle_color=ochre, shared_tracking_legend=True, direct_correlation_labels=True,
            hollow_baseline_markers=True, redundant_line_styles=True, data_smoothed=False,
            references=['https://www.nature.com/articles/s41592-025-02630-5/figures/4',
                        'https://sronpersonalpages.nl/~pault/',
                        'https://research-figure-guide.nature.com/figures/preparing-figures-our-specifications/']),
        reviewer_evidence_sha256=hashlib.sha256((DATA / 'reviewer_evidence.json').read_bytes()).hexdigest())
    export(fig, 'gaussian_robustness', source)


def main():
    evidence = json.loads((DATA / 'reviewer_evidence.json').read_text())
    original = json.loads((DATA / 'gaussian_paper_evidence.json').read_text())
    native = json.loads((DATA / 'gaussian_main_summary.json').read_text())
    tables(evidence, original, native)
    facts(evidence)
    robustness(evidence)
    print('Rendered recursive controls, all new segments, and all model sensitivity settings.')


if __name__ == '__main__':
    main()
