"""Real-detection sequence comparisons from complete independently scored outputs."""
from pathlib import Path
import csv
import json
import shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
from make_figures import save

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'source_data'
EXPERIMENT=HERE.parents[1]/'trials/icra_external_fusion'


def main():
    for name in ['case_studies','v2v4real']:
        original=EXPERIMENT/('summary_'+name+'.json')
        target=SOURCE/('external_'+name+'_summary.json')
        if original.exists():shutil.copyfile(original,target)
        assert target.exists(),target
    d=json.loads((SOURCE/'external_v2v4real_summary.json').read_text())
    assert d['sequences']==9 and d['frames']==1993 and d['audited_node_frames']==47832
    rows={(r['sequence'],r['condition'],r['arm']):r for r in d['runs']}
    arms=['lineage','mil_support','tc_ospa2_w5','tc_ospa2_w10']
    names=['ER w/o age','MIL-AM',r'TC-OSPA$^2$ (5)',r'TC-OSPA$^2$ (10)']
    matrices=[];source=[]
    for condition in ['reliable','intermittent']:
        values=np.array([[rows[f'{seq:04d}',condition,'qualified_exist']['ospa']-
                          rows[f'{seq:04d}',condition,arm]['ospa'] for seq in range(9)] for arm in arms])
        matrices.append(values)
        for i,arm in enumerate(arms):
            for seq in range(9):source.append([f'{seq:04d}',condition,arm,values[i,seq]])
    limit=float(np.ceil(max(abs(m).max() for m in matrices)*2)/2)
    cmap=LinearSegmentedColormap.from_list('signed_difference',['#2166ac','#f7f7f7','#c56a16'])
    norm=TwoSlopeNorm(vmin=-limit,vcenter=0,vmax=limit)
    fig,axes=plt.subplots(1,2,figsize=(7.12,2.30),sharey=True)
    fig.subplots_adjust(left=.135,right=.992,bottom=.34,top=.79,wspace=.11)
    for k,(ax,values,title) in enumerate(zip(axes,matrices,['Reliable links','Intermittent links'])):
        im=ax.pcolormesh(np.arange(10)-.5,np.arange(5)-.5,values,
                         cmap=cmap,norm=norm,shading='flat',rasterized=False)
        ax.set_xlim(-.5,8.5);ax.set_ylim(3.5,-.5)
        ax.set_xticks(range(9),[f'{seq:04d}' for seq in range(9)],fontsize=6.3)
        ax.set_yticks(range(4),names,fontsize=7)
        ax.set_xlabel('Released evaluation sequence',labelpad=5)
        ax.set_title(title,pad=7)
        ax.tick_params(length=0)
        for spine in ax.spines.values():spine.set_visible(False)
        for (i,j),value in np.ndenumerate(values):
            color='white' if abs(value)>.72*limit else '#222222'
            ax.text(j,i,f'{value:+.2f}',ha='center',va='center',fontsize=6.2,color=color)
        ax.text(-.035,1.20,chr(97+k),transform=ax.transAxes,fontsize=9,weight='bold')
    cax=fig.add_axes([.34,.14,.43,.043])
    bar=fig.colorbar(im,cax=cax,orientation='horizontal',ticks=[-limit,0,limit])
    bar.solids.set_rasterized(False)
    bar.outline.set_visible(False);bar.ax.tick_params(labelsize=6.5,length=2)
    fig.text(.555,.012,'ER minus comparator OSPA (m); negative favors ER',ha='center',fontsize=7)
    with (SOURCE/'v2v4real_sequence_differences.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['sequence','condition','reference','er_minus_reference_ospa_m']);writer.writerows(source)
    save(fig,'v2v4real')


if __name__=='__main__':main()
