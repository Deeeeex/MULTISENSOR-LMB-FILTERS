"""Generate every displayed scalar and table from portable Gaussian evidence."""
from pathlib import Path
import json
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'generated'
PRIMARY = 'marked_gaussian_evidence'


def write_table(name, caption, label, rows, headers, emphasized=None):
    lines = [r'\begin{table*}[t]', r'\centering\small', '\\caption{'+caption+'}',
             '\\label{'+label+'}', r'\setlength{\tabcolsep}{6pt}',
             r'\begin{tabular}{lrrrrrr}', r'\toprule',
             r'& \multicolumn{3}{c}{Reliable links} & \multicolumn{3}{c}{Intermittent links} \\',
             r'\cmidrule(lr){2-4}\cmidrule(lr){5-7}',
             'Method & '+' & '.join(headers*2)+r' \\', r'\midrule']
    for method, values in rows:
        if method == emphasized:
            method = r'\textbf{'+method+'}'
        lines.append(method+' & '+' & '.join(values)+r' \\')
    lines += [r'\bottomrule', r'\end{tabular}', r'\end{table*}', '']
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
    rows = []
    for arm in data['methods']:
        values = [f"{ag[condition, arm][key]['mean']:.3f}" for condition, _ in conditions for key in ['ospa', 'miss2', 'false2']]
        rows.append((data['labels'][arm], values))
    write_table('main_table', 'Main evaluation over 25 complete sequences. All methods use the same score-aware local tracker. OSPA is in m; missed and false GOSPA components are squared costs in m$^2$. Lower is better.', 'tab:main', rows, ['OSPA', 'Missed', 'False'], 'GCE')
    rows = []
    for arm in [PRIMARY, 'marked_asymmetric', PRIMARY+'_no_curvature', PRIMARY+'_no_history', PRIMARY+'_no_mark']:
        values = [f"{na[condition, arm][key]['mean']:.3f}" for condition, _ in conditions for key in ['ospa', 'miss2', 'false2']]
        rows.append((data['labels'][arm], values))
    write_table('ablation_table', 'Complete recursive component comparisons on the same 25 sequences. The score constraint ablation retains the common score-aware local update. Units match Table~\\ref{tab:main}.', 'tab:ablation', rows, ['OSPA', 'Missed', 'False'], 'GCE')
    rows = []
    for arm in data['methods']:
        values = [f"{ag[condition, arm][key]['mean']/2**20:.3f}" for condition, _ in conditions for key in ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']]
        rows.append((data['labels'][arm], values))
    rows.append(('GCE full Gaussian packet', [f"{na[condition, PRIMARY][key]['mean']/2**20:.3f}" for condition, _ in conditions for key in ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']]))
    write_table('communication_table', 'Mean actual MiB per sequence over 25 sequences. GCE uses the exact zero-vector codec. Fragmented totals include attempted packet fragments and modeled control traffic; delivered raw bytes count successful deliveries only.', 'tab:communication', rows, ['Raw', 'Delivered', 'Fragmented'], 'GCE')
    print('Generated', len(facts), 'source-backed scalars and three tables.')


if __name__ == '__main__':
    main()
