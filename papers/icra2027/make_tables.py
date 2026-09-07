"""Generate manuscript tables and checked scalar macros from audited outputs."""
from pathlib import Path
import json

OUT=Path(__file__).resolve().parent
DATA=OUT.parents[1]/'trials/icra_reunion_fusion'
GEN=OUT/'generated';GEN.mkdir(exist_ok=True)
SCENES=['split_latebirth','churn_departure','split_no_new']
NAMES={'local':'Local','fov':'FoV','lineage':'Lineage','mil':'MIL-Z','mil_support':'MIL-S',
       'recent':'Age-all','lineage_recent':'L+Age-all','qualified_exist':'ER','confirmed_exist':'Confirmed ER'}

def main():
    summary=DATA/'summary_validation.json'
    if not summary.exists():summary=OUT/'source_data/validation_summary.json'
    d=json.loads(summary.read_text())
    assert d['audited_node_frames']==518400 and len(d['seeds'])==20
    a={(r['scene'],r['arm']):r for r in d['aggregate']}
    p={(r['scene'],r['arm'],r['reference']):r for r in d['paired']}
    lines=[r'\begin{table*}[t]',r'\centering',
           r'\caption{Validation on all twenty paired seeds per family. OSPA is mean $\pm$ episode SD (m); count is mean absolute cardinality error. All nine arms and all seeds are included.}',
           r'\label{tab:main}',r'\small',r'\begin{tabular}{lrrrrrr}',r'\toprule',
           r'& \multicolumn{2}{c}{Split--rejoin} & \multicolumn{2}{c}{Churn--departure} & \multicolumn{2}{c}{No-new control}\\',
           r'Method & OSPA & Count & OSPA & Count & OSPA & Count\\',r'\midrule']
    for arm in d['arms']:
        vals=[]
        for scene in SCENES:
            r=a[scene,arm];vals.extend([f"${r['ospa']['mean']:.3f}\\pm{r['ospa']['sd']:.3f}$",f"{r['count_mae']['mean']:.3f}"])
        lines.append(NAMES[arm]+' & '+' & '.join(vals)+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table*}'])
    (GEN/'main_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table}[t]',r'\centering',
           r'\caption{Residual false-target GOSPA squared cost (m$^2$), averaged over twenty episodes. Departure uses only frames 91--120; no-new uses the entire episode.}',
           r'\label{tab:false}',r'\small',r'\begin{tabular}{lrr}',r'\toprule',r'Method & Departure & No-new\\',r'\midrule']
    for arm in d['arms']:
        lines.append(f"{NAMES[arm]} & {a['churn_departure',arm]['post_departure_false2']['mean']:.3f} & {a['split_no_new',arm]['false2']['mean']:.3f}"+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table}'])
    (GEN/'false_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table}[t]',r'\centering',
           r'\caption{Common-target localization relative to Lineage. Ratio is pooled candidate RMSE divided by reference RMSE; brackets are descriptive 95\% bootstrap intervals. $n_c$ counts common robot--time--truth triples, not independent samples.}',
           r'\label{tab:common}',r'\footnotesize',r'\begin{tabular}{llrr}',r'\toprule',r'Scene & Method & RMSE ratio [95\% interval] & $n_c$\\',r'\midrule']
    for scene,short in zip(SCENES[:2],['Split','Churn']):
        for arm in ['qualified_exist','lineage_recent','confirmed_exist']:
            r=next(r for r in d['common_target'] if r['scene']==scene and r['arm']==arm and r['reference']=='lineage')
            lines.append(f"{short} & {NAMES[arm].replace('Confirmed ER','Confirmed')} & {r['ratio']:.4f} [{r['ratio_low']:.4f}, {r['ratio_high']:.4f}] & {r['support']:,}"+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table}'])
    (GEN/'common_table.tex').write_text('\n'.join(lines)+'\n')
    facts={}
    for scene,key in zip(SCENES,['Split','Churn','Nonew']):
        x,y=a[scene,'qualified_exist'],a[scene,'lineage'];delta=p[scene,'qualified_exist','lineage']['ospa']
        facts['Gain'+key]=100*(1-x['ospa']['mean']/y['ospa']['mean'])
        facts['Er'+key]=x['ospa']['mean'];facts['Lineage'+key]=y['ospa']['mean']
        facts['Delta'+key]=delta['mean'];facts['DeltaLow'+key]=delta['low'];facts['DeltaHigh'+key]=delta['high']
    macros=[]
    for key,value in facts.items():
        digits=1 if key.startswith('Gain') else 3
        macros.append('\\newcommand{\\'+key+'}{'+format(value,f'.{digits}f')+'}')
    (GEN/'numbers.tex').write_text('\n'.join(macros)+'\n')
    (GEN/'facts.json').write_text(json.dumps(facts,indent=2)+'\n')
    print(json.dumps(facts,indent=2))

if __name__=='__main__':main()
