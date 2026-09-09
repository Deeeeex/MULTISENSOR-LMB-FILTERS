"""Export native probability traces and the exact aggregate error decomposition."""
from pathlib import Path
import csv
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
FIG = OUT/'figures'
FIG.mkdir(exist_ok=True)
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
gap_path = OUT/'GAP_DIAGNOSIS.json'
trace_path = OUT/'TRACK_DIAGNOSIS.json'
gap, tracks = json.loads(gap_path.read_text()), json.loads(trace_path.read_text())
assert gap['passed'] and tracks['passed']
GCE, NOAGE = 'marked_gaussian_evidence', 'marked_lineage'
COLORS = {GCE:'#BD6738', NOAGE:'#326B80'}
plt.rcParams.update({'font.family':'sans-serif', 'font.sans-serif':['Arial','DejaVu Sans'],
                     'font.size':7.3, 'axes.titlesize':8.3, 'axes.labelsize':7.6,
                     'legend.fontsize':7, 'legend.frameon':False, 'axes.spines.top':False, 'axes.spines.right':False,
                     'axes.linewidth':.65, 'xtick.major.width':.65, 'ytick.major.width':.65,
                     'svg.fonttype':'none', 'svg.hashsalt':'v2x-gce-diagnosis', 'pdf.fonttype':42})
figure, axes = plt.subplots(2,2,figsize=(183/25.4,148/25.4),
                            gridspec_kw={'width_ratios':[1,1.35], 'height_ratios':[1,1]},
                            layout='constrained')
source_rows = []
terms = ['localization','missed','false','cardinality_gap','normalization']
labels = ['Localization','Misses','False outputs','Count imbalance','Normalization']
ax = axes[0,0]
for condition, shift, color, hatch in [('reliable',-.16,'#657D84',None),('intermittent',.16,'#DDE4E6','///')]:
    row = next(r for r in gap['aggregate'] if r['cohort']=='v2x_test' and r['condition']==condition)
    values = [row['sequence_macro']['attribution_'+k] for k in terms]
    ax.barh(np.arange(5)+shift,values,height=.29,color=color,edgecolor='#53666C',linewidth=.5,hatch=hatch,label=condition.capitalize())
    for term, value in zip(terms,values):
        source_rows.append(dict(panel='a',condition=condition,term=term,value=value))
ax.axvline(0,color='#454C50',linewidth=.65)
ax.set_yticks(np.arange(5),labels);ax.invert_yaxis()
ax.set_xlim(-.024,.088);ax.set_xticks([-.02,0,.04,.08])
ax.set_xlabel('Contribution to GCE − No-age OSPA (m)')
ax.set_title('All 14 additional segments',loc='left',pad=8)
ax.legend(loc='lower right',bbox_to_anchor=(1.01,-.015),handlelength=1.2)
ax.text(.04,.03,'Total: +0.106 / +0.025 m',transform=ax.transAxes,fontsize=6.8,
        bbox=dict(facecolor='white',edgecolor='none',pad=1))

loss = next(c for c in tracks['traces'] if c['sequence']=='v2xt_0001' and c['condition']=='reliable')
gain = next(c for c in tracks['traces'] if c['sequence']=='v2xt_0009' and c['condition']=='reliable')


def draw_probabilities(ax,case,first,last,panel):
    for arm in [NOAGE,GCE]:
        part = [r for r in case['rows'] if r['robot']==1 and r['arm']==arm]
        t = [r['frame'] for r in part]
        p = [r['anchor']['r'] if r['anchor'] else np.nan for r in part]
        ax.plot(t,p,color=COLORS[arm],linewidth=1.5,label='No-age KLA' if arm==NOAGE else 'GCE')
        for r,value in zip(part,p):
            source_rows.append(dict(panel=panel,sequence=case['sequence'],condition=case['condition'],
                                    frame=r['frame'],robot=1,arm=arm,term='existence_probability',
                                    value=float(value) if np.isfinite(value) else '',selected=r['anchor_selected']))
    ax.set_xlim(first,last);ax.set_ylim(-.045,1.045)
    ax.set_yticks([0,.5,1]);ax.set_ylabel('Existence probability')
    ax.set_xlabel('Frame');ax.legend(loc='center left',bbox_to_anchor=(.01,.57),handlelength=1.8)


ax=axes[0,1]
draw_probabilities(ax,loss,35,82,'b')
ax.axvspan(53,82,color='#C8CBCE',alpha=.20,zorder=-2)
ax.set_title('Loss case: existence collapses',loc='left',pad=8)
at53=next(r for r in loss['rows'] if r['frame']==53 and r['robot']==1 and r['arm']==GCE)
ax.scatter([53],[at53['anchor']['r']],s=14,color=COLORS[GCE],zorder=5)
ax.annotate('MAP output lost\nat frame 53',xy=(53,at53['anchor']['r']),xytext=(62,.39),
            fontsize=7,arrowprops=dict(arrowstyle='-',color='#555F63',lw=.65),ha='left')
ax.text(.98,.88,'Label [3, 100004]',transform=ax.transAxes,ha='right',va='top',fontsize=6.7)

ax=axes[1,0]
part=[r for r in loss['rows'] if r['robot']==1 and r['arm']==GCE and r['anchor']]
for key,label,color,style in [('positive_log_odds','Positive scalar','#607A83','-'),
                              ('negative_log_odds','Negative scalar',COLORS[GCE],'--')]:
    t=[r['frame'] for r in part];values=[r['anchor'][key] for r in part]
    ax.plot(t,values,label=label,color=color,ls=style,lw=1.3)
    for frame,value in zip(t,values):
        source_rows.append(dict(panel='c',sequence=loss['sequence'],condition='reliable',frame=frame,robot=1,arm=GCE,term=key,value=value))
ax.set_xlim(48,58);ax.set_ylim(-1.12,.18)
ax.set_xticks([48,50,52,54,56,58]);ax.set_yticks([-1,-.5,0])
ax.set_xlabel('Frame');ax.set_ylabel('Extra admitted log-odds')
ax.set_title('Same loss case: repeated negative admission',loc='left',pad=8)
ax.axhline(0,color='#5E666A',lw=.5)
ax.legend(loc='center left',bbox_to_anchor=(.01,.57),handlelength=1.8)
ax.text(.98,.22,'−0.942 each frame',transform=ax.transAxes,ha='right',fontsize=7,color=COLORS[GCE])

ax=axes[1,1]
draw_probabilities(ax,gain,132,178,'d')
ax.set_title('Gain case: earlier target output',loc='left',pad=8)
ax.text(.98,.88,'Label [135, 200004]',transform=ax.transAxes,ha='right',va='top',fontsize=6.7)
ax.legend(loc='center right',bbox_to_anchor=(.98,.53),handlelength=1.8)
ax.text(.98,.10,'Both methods keep their own recursion',transform=ax.transAxes,ha='right',fontsize=6.6)

for letter,ax in zip('abcd',axes.flat):
    ax.text(-.18,1.045,letter,transform=ax.transAxes,fontsize=10,fontweight='bold',va='top')
figure.set_constrained_layout_pads(w_pad=.05,h_pad=.06,wspace=.07,hspace=.08)
data_path=FIG/'figure_source.csv'
fields=list(dict.fromkeys(k for r in source_rows for k in r))
with data_path.open('w',newline='') as handle:
    writer=csv.DictWriter(handle,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(source_rows)
for extension in ['svg','pdf','png']:
    metadata = {'Date':None} if extension=='svg' else ({'CreationDate':None,'ModDate':None} if extension=='pdf' else None)
    figure.savefig(FIG/('gce_diagnosis.'+extension),dpi=300,facecolor='white',metadata=metadata)
plt.close(figure)
report=dict(passed=True,backend='Python/matplotlib',matplotlib=matplotlib.__version__,numpy=np.__version__,
            size_mm=[183,148],inputs={gap_path.name:sha(gap_path),trace_path.name:sha(trace_path)},
            script_sha256=sha(Path(__file__)),source_rows=len(source_rows),source_sha256=sha(data_path),
            outputs={p.name:sha(p) for p in FIG.glob('gce_diagnosis.*')},
            scope='Report figure from native saved traces and exact additive means. Cropped time axes are declared in FIGURE_CONTRACT.md. Full source series retained; no smoothing or simulated data.')
(FIG/'FIGURE_BUILD.json').write_text(json.dumps(report,indent=2)+'\n')
print('FIGURE EXPORTED',len(source_rows),'source rows',flush=True)
