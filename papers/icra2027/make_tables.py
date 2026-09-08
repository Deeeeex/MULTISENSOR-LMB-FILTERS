"""Generate manuscript tables and scalar macros from experiment summaries."""
from pathlib import Path
import json

OUT=Path(__file__).resolve().parent
DATA=OUT.parents[1]/'trials/icra_reunion_fusion'
GEN=OUT/'generated';GEN.mkdir(exist_ok=True)
SCENES=['split_latebirth','churn_departure','split_no_new']
NAMES={'local':'Local','fov':'FoV','lineage':'ER w/o age','mil':'MIL-Z','mil_support':'MIL-S',
       'recent':'Age-all','lineage_recent':'ER (both)','qualified_exist':'ER','confirmed_exist':'Confirmed ER',
       'tc_ospa2_w5':r'TC-OSPA$^2$ (5)','tc_ospa2_w10':r'TC-OSPA$^2$ (10)'}

def extension(name):
    source=DATA.parent/'icra_external_fusion'/('summary_'+name+'.json')
    if not source.exists():source=OUT/'source_data'/('external_'+name+'_summary.json')
    return json.loads(source.read_text())

def write_real_table(d):
    a={(r['condition'],r['arm']):r for r in d['aggregate']}
    lines=[r'\begin{table*}[t]',r'\centering',
           r'\caption{V2V4Real replay over nine sequences (1,993 frames). OSPA: equal-weight mean $\pm$ sequence SD (m); count: cardinality MAE; false: GOSPA squared false-target cost (m$^2$).}',
           r'\label{tab:real}',r'\small',r'\begin{tabular}{lrrrrrr}',r'\toprule',
           r'& \multicolumn{3}{c}{Reliable links} & \multicolumn{3}{c}{Intermittent links}\\',
           r'Method & OSPA & Count & False & OSPA & Count & False\\',r'\midrule']
    for arm in ['local','mil_support','tc_ospa2_w5','tc_ospa2_w10','qualified_exist']:
        vals=[]
        for c in ['reliable','intermittent']:
            r=a[c,arm];vals.extend([f"${r['ospa']['mean']:.3f}\\pm{r['ospa']['sd']:.3f}$",f"{r['countError']['mean']:.3f}",f"{r['false2']['mean']:.2f}"])
        lines.append(('MIL-AM' if arm=='mil_support' else NAMES[arm])+' & '+' & '.join(vals)+r'\\')
    lines += [r'\midrule']
    vals=[]
    for c in ['reliable','intermittent']:
        r=a[c,'lineage'];vals.extend([f"${r['ospa']['mean']:.3f}\\pm{r['ospa']['sd']:.3f}$",f"{r['countError']['mean']:.3f}",f"{r['false2']['mean']:.2f}"])
    lines += [NAMES['lineage']+' (ablation) & '+' & '.join(vals)+r'\\',r'\bottomrule',r'\end{tabular}',r'\end{table*}']
    (GEN/'real_table.tex').write_text('\n'.join(lines)+'\n')

def main():
    summary=DATA/'summary_validation.json'
    if not summary.exists():summary=OUT/'source_data/validation_summary.json'
    d=json.loads(summary.read_text())
    assert d['audited_node_frames']==518400 and len(d['seeds'])==20
    a={(r['scene'],r['arm']):r for r in d['aggregate']}
    ext=extension('case_studies')
    a.update({(r['scene'],r['arm']):r for r in ext['aggregate'] if r['arm'].startswith('tc_')})
    p={(r['scene'],r['arm'],r['reference']):r for r in d['paired']}
    lines=[r'\begin{table*}[t]',r'\centering',
           r'\caption{Mechanism case studies and no-new control: twenty paired trials each. OSPA is mean $\pm$ trial SD (m); count is cardinality MAE. TC uses local track histories without density feedback; MIL-Z/S use shared label keys.}',
           r'\label{tab:main}',r'\small',r'\begin{tabular}{lrrrrrr}',r'\toprule',
           r'& \multicolumn{2}{c}{Split--rejoin} & \multicolumn{2}{c}{Churn--departure} & \multicolumn{2}{c}{No-new control}\\',
           r'Method & OSPA & Count & OSPA & Count & OSPA & Count\\',r'\midrule']
    for arm in ['local','fov','mil','mil_support','tc_ospa2_w5','tc_ospa2_w10','qualified_exist']:
        vals=[]
        for scene in SCENES:
            r=a[scene,arm];vals.extend([f"${r['ospa']['mean']:.3f}\\pm{r['ospa']['sd']:.3f}$",f"{r['count_mae']['mean']:.3f}"])
        lines.append(NAMES[arm]+' & '+' & '.join(vals)+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table*}'])
    (GEN/'main_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table}[t]',r'\centering',
           r'\caption{False-target GOSPA squared cost (m$^2$), averaged over twenty trials. Departure uses frames 91--120; no-new uses the full trial.}',
           r'\label{tab:false}',r'\small',r'\begin{tabular}{lrr}',r'\toprule',r'Method & Departure & No-new\\',r'\midrule']
    for arm in d['arms']:
        lines.append(f"{NAMES[arm]} & {a['churn_departure',arm]['post_departure_false2']['mean']:.3f} & {a['split_no_new',arm]['false2']['mean']:.3f}"+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table}'])
    (GEN/'false_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table}[t]',r'\centering',
           r'\caption{Common-target localization relative to ER w/o age. Each ratio divides the pooled method RMSE by the reference RMSE; brackets give 95\% bootstrap intervals. $n_c$ counts common robot--time--truth triples.}',
           r'\label{tab:common}',r'\footnotesize',r'\begin{tabular}{llrr}',r'\toprule',r'Scene & Method & RMSE ratio [95\% interval] & $n_c$\\',r'\midrule']
    for scene,short in zip(SCENES[:2],['Split','Churn']):
        for arm in ['qualified_exist','lineage_recent','confirmed_exist']:
            r=next(r for r in d['common_target'] if r['scene']==scene and r['arm']==arm and r['reference']=='lineage')
            lines.append(f"{short} & {NAMES[arm].replace('Confirmed ER','Confirmed')} & {r['ratio']:.4f} [{r['ratio_low']:.4f}, {r['ratio_high']:.4f}] & {r['support']:,}"+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table}'])
    (GEN/'common_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table*}[t]',r'\centering',
           r'\caption{Internal mechanism ablations. OSPA (m) is averaged over twenty trials. RMSE ratios use common assignment support relative to ER w/o age; false costs are squared GOSPA components (m$^2$).}',
           r'\label{tab:ablation}',r'\small',r'\begin{tabular}{lrrrrrrr}',r'\toprule',
           r'& \multicolumn{3}{c}{OSPA} & \multicolumn{2}{c}{Common-target RMSE ratio} & \multicolumn{2}{c}{False cost}\\',
           r'Method & Split & Churn & No-new & Split & Churn & Departure & No-new\\',r'\midrule']
    for arm in ['lineage','recent','lineage_recent','confirmed_exist','qualified_exist']:
        vals=[f"{a[s,arm]['ospa']['mean']:.3f}" for s in SCENES]
        for s in SCENES[:2]:
            matches=[r for r in d['common_target'] if r['scene']==s and r['arm']==arm and r['reference']=='lineage']
            vals.append('1.0000' if arm=='lineage' else (f"{matches[0]['ratio']:.4f}" if matches else '--'))
        vals.extend([f"{a['churn_departure',arm]['post_departure_false2']['mean']:.3f}",f"{a['split_no_new',arm]['false2']['mean']:.3f}"])
        lines.append(NAMES[arm]+' & '+' & '.join(vals)+r'\\')
    lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table*}'])
    (GEN/'ablation_table.tex').write_text('\n'.join(lines)+'\n')
    real=extension('v2v4real');assert real['audited_node_frames']==47832 and real['sequences']==9
    write_real_table(real)
    facts={}
    for scene,key in zip(SCENES,['Split','Churn','Nonew']):
        x,y=a[scene,'qualified_exist'],a[scene,'lineage'];delta=p[scene,'qualified_exist','lineage']['ospa']
        facts['Gain'+key]=100*(1-x['ospa']['mean']/y['ospa']['mean'])
        facts['Er'+key]=x['ospa']['mean'];facts['Lineage'+key]=y['ospa']['mean']
        facts['Delta'+key]=delta['mean'];facts['DeltaLow'+key]=delta['low'];facts['DeltaHigh'+key]=delta['high']
    for c,suffix in [('reliable','Reliable'),('intermittent','Intermittent')]:
        rows={r['arm']:r for r in real['aggregate'] if r['condition']==c}
        for arm,prefix in [('qualified_exist','RealEr'),('lineage','RealNoAge'),('mil_support','RealMil'),('tc_ospa2_w5','RealTcFive'),('tc_ospa2_w10','RealTcTen')]:
            facts[prefix+suffix]=rows[arm]['ospa']['mean']
        pair=next(r for r in real['paired'] if r['condition']==c and r['reference']=='lineage')['ospa']
        for field,prefix in [('mean','RealDelta'),('low','RealDeltaLow'),('high','RealDeltaHigh')]:
            facts[prefix+suffix]=pair[field]
    macros=[]
    for key,value in facts.items():
        digits=1 if key.startswith('Gain') else 3
        macros.append('\\newcommand{\\'+key+'}{'+format(value,f'.{digits}f')+'}')
    (GEN/'numbers.tex').write_text('\n'.join(macros)+'\n')
    (GEN/'facts.json').write_text(json.dumps(facts,indent=2)+'\n')
    print(json.dumps(facts,indent=2))

if __name__=='__main__':main()
