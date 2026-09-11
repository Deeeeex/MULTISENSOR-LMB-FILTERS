"""Render the completed expanded-corpus comparison and update Fixed Ratio."""
from pathlib import Path
import json
import numpy as np
from make_gaussian_tables import write_table

HERE = Path(__file__).resolve().parent
DATA, GEN = HERE/'source_data', HERE/'generated'
GCE = 'marked_gaussian_evidence'
GS, FIXED = GCE+'_guarded_scalar', GCE+'_fixedx_000'
CONDITIONS = ['reliable', 'intermittent']


def main():
    evidence = json.loads((DATA/'followup_evidence.json').read_text())
    original = json.loads((DATA/'gaussian_paper_evidence.json').read_text())
    merged = {(r['condition'], r['arm']): r for r in original['aggregate']}
    merged.update({(r['condition'], r['arm']): r for r in evidence['seen_25']})
    methods = original['methods'][:-1]+[GS, FIXED, GCE]
    names = dict(original['labels'], **{GS:'Guarded Scalar', FIXED:'Fixed Ratio (0)'})
    rows = [(names[a], [merged[c,a][m]['mean'] for c in CONDITIONS for m in ['ospa','miss2','false2']]) for a in methods]
    write_table('main_table', 'Comparison on 25 previously used sequences with a shared local tracker. OSPA is in m; missed/false GOSPA costs are in m$^2$. Lowest values are bold.',
                'tab:main', rows, ['OSPA','Missed','False'], 'GCE', separators=(4,12))
    values = {(r['dataset'],r['condition'],r['arm']):r['sequence_macro']['ospa'] for r in evidence['aggregate']}
    labels = {'marked_lineage':'No-age KLA','marked_er':'Recency','marked_asymmetric':'Scalar',
              GS:'Guarded Scalar',GCE+'_no_curvature':'w/o curvature',FIXED:'Fixed Ratio (0)',GCE:'GCE'}
    cells = [[values[d,c,a] for d in ['v2v','v2x'] for c in CONDITIONS] for a in evidence['methods']]
    minima = np.min(cells, axis=0)
    lines = [r'\begin{table}[t]',r'\centering\small',
        r'\caption{Expanded-corpus OSPA (m), with equal segment weights. V2V4Real: 43 deduplicated segments, 9,699 paired frames, 17 recordings. V2X-Real: five segments, 619 paired frames, three collection dates. R/I denote reliable/intermittent links.}',
        r'\label{tab:followup}',r'\setlength{\tabcolsep}{3pt}',
        r'\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lrrrr@{}}',r'\toprule',
        r'& \multicolumn{2}{c}{V2V4Real} & \multicolumn{2}{c}{V2X-Real} \\',
        r'\cmidrule(lr){2-3}\cmidrule(lr){4-5}',r'Method & R & I & R & I \\',r'\midrule']
    for arm, row in zip(evidence['methods'], cells):
        if arm == GCE: lines.append(r'\midrule')
        shown = [r'\textbf{GCE}' if arm == GCE else labels[arm]]
        for i, value in enumerate(row):
            s = f'{value:.3f}'
            shown.append(r'\textbf{'+s+'}' if abs(value-minima[i])<1e-12 else s)
        lines.append(' & '.join(shown)+r' \\')
    lines += [r'\bottomrule',r'\end{tabular*}',r'\end{table}','']
    (GEN/'followup_table.tex').write_text('\n'.join(lines))
    facts = {}
    for condition in CONDITIONS:
        suffix = condition.title()
        for dataset, prefix in [('v2v','VTwoV'),('v2x','VTwoX')]:
            for arm, short in [(GCE,'GCE'),('marked_lineage','NoAge'),(GS,'GS')]:
                facts['Follow'+prefix+short+suffix] = [values[dataset,condition,arm],3]
        facts['FollowGainNoAge'+suffix] = [100*(1-values['v2v',condition,GCE]/values['v2v',condition,'marked_lineage']),1]
        for pair in evidence['seen_fixed_paired']:
            if pair['condition'] == condition:
                for stat in ['mean','low','high']: facts['FollowFixed'+suffix+stat.title()] = [pair[stat],3]
        row = next(r for r in evidence['recording_gce_gs'] if r['condition'] == condition)
        for key, short in [('group_macro_difference','Mean'),('low','Low'),('high','High')]:
            facts['FollowRecordingGS'+suffix+short] = [row[key],3]
    (GEN/'followup_facts.json').write_text(json.dumps(facts, indent=2)+'\n')
    (GEN/'followup_numbers.tex').write_text('% Completed, verified follow-up evidence only.\n'+
        '\n'.join(f'\\newcommand{{\\{name}}}{{{value:.{digits}f}}}' for name,(value,digits) in facts.items())+'\n')
    print('Rendered expanded data and fixed-zero comparison.')


if __name__ == '__main__':
    main()
