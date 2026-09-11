"""Generate every displayed scalar and table from portable Gaussian evidence."""
from pathlib import Path
import json
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'generated'
PRIMARY = 'marked_gaussian_evidence'


def write_table(name, caption, label, rows, metrics, emphasized, separators=(), wide=True):
    environment = 'table*' if wide else 'table'
    count = len(metrics)
    lines = [r'\begin{'+environment+r'}[t]', r'\centering\small']
    # The class's first table-caption baseline extends above top-float bounds.
    if name in {'main_table', 'communication_table'}:
        lines.append(r'\vspace*{5pt}')
    lines += ['\\caption{'+caption+'}',
             '\\label{'+label+'}', r'\setlength{\tabcolsep}{'+('6' if wide else '3.5')+r'pt}',
             r'\begin{tabular*}{'+(r'\textwidth' if wide else r'\columnwidth')+r'}{@{\extracolsep{\fill}}l'+('r'*(2*count))+r'@{}}', r'\toprule',
             r'& \multicolumn{'+str(count)+r'}{c}{Reliable links} & \multicolumn{'+str(count)+r'}{c}{Intermittent links} \\',
             r'\cmidrule(lr){2-'+str(count+1)+r'}\cmidrule(lr){'+str(count+2)+'-'+str(2*count+1)+'}',
             'Method & '+' & '.join(metrics*2)+r' \\', r'\midrule']
    minima = np.min(np.array([values for _, values in rows]), axis=0)
    for i, (method, values) in enumerate(rows):
        if i in separators:
            lines.append(r'\midrule')
        if method == emphasized:
            method = r'\textbf{'+method+'}'
        formatted = []
        for j, value in enumerate(values):
            cell = f'{value:.3f}' if wide else f'{value:.2f}'
            if wide and np.isclose(value, minima[j], rtol=0, atol=1e-12):
                cell = r'\textbf{'+cell+'}'
            formatted.append(cell)
        lines.append(method+' & '+' & '.join(formatted)+r' \\')
    lines += [r'\bottomrule', r'\end{tabular*}', r'\end{'+environment+'}', '']
    (OUT / (name+'.tex')).write_text('\n'.join(lines))


def main():
    OUT.mkdir(exist_ok=True)
    data = json.loads((HERE / 'source_data/gaussian_paper_evidence.json').read_text())
    native = json.loads((HERE / 'source_data/gaussian_main_summary.json').read_text())
    ag = {(r['condition'], r['arm']): r for r in data['aggregate']}
    na = {(r['condition'], r['arm']): r for r in native['aggregate']}
    dev = {(r['condition'], r['arm']): r for r in data['development']}
    paired = {(r['condition'], r['reference']): r for r in data['paired']}
    component = {(r['condition'], r['reference']): r for r in data['components']}
    common = {(r['condition'], r['reference']): r for r in data['common'] if r['candidate'] == PRIMARY}
    unmarked = {(r['condition'], r['arm']): r for r in data['unmarked_controls']}
    codec = {r['condition']: r for r in data['codec']}
    facts, formats = {}, {}
    def fact(name, value, decimals=3):
        facts[name] = value
        formats[name] = decimals
    conditions = [('reliable', 'Reliable'), ('intermittent', 'Intermittent')]
    for condition, suffix in conditions:
        for prefix, arm in [('Main', PRIMARY), ('NoAge', 'marked_lineage'), ('Recency', 'marked_er'), ('Scalar', 'marked_asymmetric')]:
            fact(prefix+suffix, ag[condition, arm]['ospa']['mean'])
        for prefix, arm in [('NoAge', 'marked_lineage'), ('Recency', 'marked_er')]:
            fact('Gain'+prefix+suffix, 100*(1-ag[condition, PRIMARY]['ospa']['mean']/ag[condition, arm]['ospa']['mean']), 1)
            for key, middle in [('mean', ''), ('low', 'Low'), ('high', 'High')]:
                fact('Delta'+prefix+middle+suffix, paired[condition, arm]['ospa'][key])
        for metric, prefix in [('miss2', 'Miss'), ('false2', 'False')]:
            fact(prefix+'GainNoAge'+suffix, 100*(1-ag[condition, PRIMARY][metric]['mean']/ag[condition, 'marked_lineage'][metric]['mean']), 1)
        c = common[condition, 'marked_lineage']
        fact('LocMainNoAge'+suffix, c['candidate_rmse'])
        fact('LocNoAge'+suffix, c['reference_rmse'])
        fact('SupportNoAge'+suffix, c['support'], 0)
        s = component[condition, 'marked_asymmetric']
        for key, middle in [('mean', ''), ('low', 'Low'), ('high', 'High')]:
            fact('DeltaScalar'+middle+suffix, s['ospa'][key])
        fact('LocMainScalar'+suffix, s['candidate_common_rmse'])
        fact('LocScalar'+suffix, s['reference_common_rmse'])
        for prefix, arm in [('Main', PRIMARY), ('NoAge', 'marked_lineage'), ('Scalar', 'marked_asymmetric')]:
            fact('Dev'+prefix+suffix, dev[condition, arm]['ospa']['mean'])
        fact('UnmarkedNoAge'+suffix, unmarked[condition, 'lineage']['ospa']['mean'])
        diagnostics = [r for r in data['diagnostics'] if r['arm'] == PRIMARY and r['condition'] == condition]
        assert len(diagnostics) == 25
        for prefix, rule in [('Scalar', 'scalar_AS'), ('Main', 'candidate')]:
            fact('Fixed'+prefix+suffix, float(np.mean([r['same_input'][rule]['ospa'] for r in diagnostics])))
        for key, prefix in [('raw_bytes', 'Raw'), ('wire_bytes', 'Wire')]:
            fact('Codec'+prefix+'Saving'+suffix, 100*codec[condition]['savings'][key], 1)
            fact('Codec'+prefix+'ExtraNoAge'+suffix, 100*(ag[condition, PRIMARY][key]['mean']/ag[condition, 'marked_lineage'][key]['mean']-1), 1)
    macros = ['% Generated from complete experiment summaries; do not edit numbers by hand.']
    for key, value in facts.items():
        shown = f'{value:.{formats[key]}f}' if formats[key] else f'{value:,}'.replace(',', r'\,')
        macros.append('\\newcommand{\\'+key+'}{'+shown+'}')
    (OUT / 'numbers.tex').write_text('\n'.join(macros)+'\n')
    (OUT / 'facts.json').write_text(json.dumps(facts, indent=2, allow_nan=False)+'\n')
    rows = [(data['labels'][arm], [ag[condition, arm][key]['mean']
            for condition, _ in conditions for key in ['ospa', 'miss2', 'false2']]) for arm in data['methods']]
    write_table('main_table', 'Main comparison on 25 sequences with a shared local tracker. OSPA is in m; missed and false GOSPA costs are in m$^2$. Lower is better; best values are bold.',
                'tab:main', rows, ['OSPA', 'Missed', 'False'], 'GCE', separators=(4, 10))
    ablations = [('marked_asymmetric', 'Scalar reference'),
                 (PRIMARY+'_no_curvature', 'w/o curvature guard'),
                 (PRIMARY+'_no_history', 'w/o history switch'),
                 (PRIMARY+'_no_mark', 'w/o score constraint'),
                 (PRIMARY, 'GCE (complete)')]
    rows = [(label, [na[condition, arm][key]['mean'] for condition, _ in conditions
                    for key in ['ospa', 'miss2', 'false2']]) for arm, label in ablations]
    write_table('ablation_table', 'Ablations of the recursive tracker. Each w/o row removes one GCE component; Scalar is a separate existence-only reference. Units and boldface follow Table~\\ref{tab:main}.',
                'tab:ablation', rows, ['OSPA', 'Missed', 'False'], 'GCE (complete)', separators=(1, 4))
    formats = [('marked_lineage', 'No-age KLA', ag), ('marked_asymmetric', 'Scalar', ag),
               (PRIMARY, 'GCE (full)', na), (PRIMARY, 'GCE (encoded)', ag)]
    rows = [(label, [source[condition, arm][key]['mean']/2**20 for condition, _ in conditions
                    for key in ['raw_bytes', 'wire_bytes']]) for arm, label, source in formats]
    write_table('communication_table', 'Mean communication per sequence (MiB). Frag. includes packet fragmentation and control traffic.',
                'tab:communication', rows, ['Raw', 'Frag.'], 'GCE (encoded)', separators=(2,), wide=False)
    print('Generated', len(facts), 'source-backed scalars and three tables.')


if __name__ == '__main__':
    main()
