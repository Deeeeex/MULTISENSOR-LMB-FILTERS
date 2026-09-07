"""Publication plots from saved validation outputs; editable SVG + PDF + PNG."""
from pathlib import Path
import argparse
import gzip
import json
import csv
import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D

OUT=Path(__file__).resolve().parent
DATA=OUT.parents[1]/'trials/icra_reunion_fusion'
FIG=OUT/'figures';FIG.mkdir(exist_ok=True)
SOURCE=OUT/'source_data';SOURCE.mkdir(exist_ok=True)
SCENES=['split_latebirth','churn_departure','split_no_new']
TITLES={'split_latebirth':'Split–rejoin','churn_departure':'Churn–departure','split_no_new':'No-new control'}
LABEL={'local':'Local','fov':'FoV','lineage':'Lineage','mil':'MIL-Z','mil_support':'MIL-S',
       'recent':'Age-all','lineage_recent':'L+Age-all','qualified_exist':'ER','confirmed_exist':'Confirmed ER'}
COLOR={'lineage':'#7560a5','lineage_recent':'#0072b2','qualified_exist':'#00856a','confirmed_exist':'#d55e00','fov':'#777777'}
STYLE={'lineage':'--','lineage_recent':'-.','qualified_exist':'-','confirmed_exist':':','fov':(0,(1,2))}
mpl.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Arial','Helvetica','DejaVu Sans'],
    'font.size':7.4,'axes.labelsize':7.4,'axes.titlesize':8,'xtick.labelsize':6.7,'ytick.labelsize':6.7,
    'legend.fontsize':6.8,'svg.fonttype':'none','pdf.fonttype':42,'ps.fonttype':42,
    'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.65,
    'xtick.major.width':.65,'ytick.major.width':.65,'lines.linewidth':1.3,'legend.frameon':False})

def save(fig,name):
    fig.canvas.draw()
    renderer=fig.canvas.get_renderer();width,height=fig.canvas.get_width_height()
    checked=[]
    texts=list(fig.texts)
    for ax in fig.axes:
        texts.extend([ax.xaxis.label,ax.yaxis.label,ax.title,*ax.texts])
        if ax.get_legend():texts.extend(ax.get_legend().get_texts())
    for legend in fig.legends:texts.extend(legend.get_texts())
    for text in texts:
        if not text.get_visible() or not text.get_text():continue
        box=text.get_window_extent(renderer)
        assert box.x0>=-.5 and box.y0>=-.5 and box.x1<=width+.5 and box.y1<=height+.5,(name,text.get_text(),box,width,height)
        checked.append(text.get_text())
    for suffix in ['svg','pdf','png']:
        fig.savefig(FIG/f'{name}.{suffix}',dpi=300,facecolor='white')
    (FIG/f'{name}_text_bounds.json').write_text(json.dumps({'checked_text':checked,'canvas_pixels':[width,height],'passed':True},indent=2)+'\n')
    plt.close(fig)

def label(ax,letter):
    ax.text(-.12,1.06,letter,transform=ax.transAxes,weight='bold',fontsize=9,va='bottom')

def load(scene,seed=2901):
    path=DATA/'results'/f'{scene}_seed{seed}_validation.json.gz'
    if path.exists():
        with gzip.open(path,'rt') as f:return json.load(f)
    assert seed==2901,'The portable paper archive includes geometry for seed 2901 only.'
    return json.loads((SOURCE/'scene_seed2901.json').read_text())[scene]

def mst_edges(p,radius=24):
    # Independent Kruskal reconstruction for visual display only.
    parent=list(range(8));edges=[]
    def find(i):
        while parent[i]!=i:i=parent[i]
        return i
    candidates=sorted((np.linalg.norm(p[:,i]-p[:,j]),i,j) for i in range(8) for j in range(i+1,8))
    for dist,i,j in candidates:
        a,b=find(i),find(j)
        if dist<=radius and a!=b:parent[a]=b;edges.append((i,j))
    return edges

def scene_plot():
    fig=plt.figure(figsize=(7.12,2.38));gs=fig.add_gridspec(1,3,width_ratios=[1,1,.96],wspace=.34)
    sources={}
    for col,scene in enumerate(SCENES[:2]):
        d=load(scene);p=np.asarray(d['positions']);sources[scene]={key:d[key] for key in ['positions','componentCount','time','truth']}
        ax=fig.add_subplot(gs[0,col]);label(ax,chr(97+col));t=54
        for n in range(8):
            c='#0072b2' if n<4 else '#d55e00'
            ax.plot(p[0,n],p[1,n],color=c,alpha=.42,lw=.75)
            ax.add_patch(Circle(p[:,n,t],14,facecolor=c,edgecolor=c,alpha=.04,lw=.5))
            ax.scatter(*p[:,n,t],color=c,s=19,marker='o',zorder=4,edgecolors='white',linewidths=.35)
        for i,j in mst_edges(p[:,:,t]):ax.plot(p[0,[i,j],t],p[1,[i,j],t],color='#565656',lw=.8,ls='--',zorder=3)
        priors=np.array([[-9.6,9.6,-24.8,24.8,-24.8,24.8],[-1.6,1.6,4.08,4.08,-8,-8]])
        ax.scatter(*priors[:,:4],s=26,marker='+',color='#232323',lw=1,zorder=5)
        ax.scatter(*priors[:,4:],s=19,marker='x',color='#888888',lw=.8,zorder=5)
        for k in [2,3]:ax.text(priors[0,k]+1.5,priors[1,k]+1.6,f'B{k+1}',fontsize=6)
        ax.set(xlim=(-44,44),ylim=(-17,35),xlabel='x (m)',ylabel='y (m)',title=TITLES[scene])
        ax.set_aspect('equal');ax.set_xticks([-40,-20,0,20,40]);ax.set_yticks([-10,10,30])
    ax=fig.add_subplot(gs[0,2]);label(ax,'c')
    for i,scene in enumerate(SCENES[:2]):
        d=load(scene);time=np.asarray(d['time']);ax.step(time,np.asarray(d['componentCount']),where='post',
            color=['#0072b2','#d55e00'][i],ls=['-','--'][i],label=['Split','Churn'][i])
    ax.axvline(17,ls=':',color='#777777',lw=.7);ax.axvline(45,ls=':',color='#777777',lw=.7)
    ax.text(17,2.29,'Birth',ha='center',fontsize=6.7);ax.text(45,2.29,'Departure',ha='center',fontsize=6.7)
    ax.set(xlim=(0,59.5),ylim=(.8,2.48),xlabel='Time (s)',ylabel='Physical components',yticks=[1,2]);ax.legend(loc='lower center',bbox_to_anchor=(.5,1.025),ncol=2)
    fig.subplots_adjust(left=.063,right=.99,bottom=.24,top=.85)
    legend=[Line2D([],[],marker='o',lw=0,color='#0072b2',label='Robots 1–4'),Line2D([],[],marker='o',lw=0,color='#d55e00',label='Robots 5–8'),
            Line2D([],[],color='#555555',ls='--',label='MST at 27 s'),Line2D([],[],color='#222222',marker='+',lw=0,label='Occupied birth regions'),
            Line2D([],[],color='#888888',marker='x',lw=0,label='Empty regions')]
    fig.legend(handles=legend,loc='lower center',ncol=5,bbox_to_anchor=(.5,.005),columnspacing=1.15,handlelength=1.4)
    (SOURCE/'scene_seed2901.json').write_text(json.dumps(sources,separators=(',',':'))+'\n')
    save(fig,'scene')

def mechanism_plot():
    age=np.linspace(0,30,151);f=.25+.75*np.exp(-age/5);q_old=f/(f+1);q_new=1-q_old
    eta_fixed=np.exp(-.5);eta_both=np.exp(-2*q_old*q_new)
    logit=(q_old-q_new)*np.log(9)
    r_fixed=1/(1+np.exp(-(logit+np.log(eta_fixed))))
    r_both=1/(1+np.exp(-(logit+np.log(eta_both))))
    fig,axes=plt.subplots(1,2,figsize=(3.5,1.7),gridspec_kw={'wspace':.43})
    a,b=axes;label(a,'a');label(b,'b')
    for ax in axes:ax.set(xlabel='Older input age (s)',xlim=(0,30),xticks=[0,15,30])
    a.plot(age,np.ones(len(age)),color=COLOR['qualified_exist'],label='ER')
    a.plot(age,2*q_new,color=COLOR['lineage_recent'],ls='-.',label='Both blocks')
    a.set(ylabel='Fused x mean (m)',ylim=(0,2),yticks=[0,1,2]);a.legend(loc='lower right',fontsize=6)
    b.plot(age,r_fixed,color=COLOR['qualified_exist']);b.plot(age,r_both,color=COLOR['lineage_recent'],ls='-.')
    b.set(ylabel='Fused existence',ylim=(0,1),yticks=[0,.5,1])
    fig.subplots_adjust(left=.145,right=.97,bottom=.27,top=.81)
    fig.text(.5,.965,'Illustrative fixed-input Gaussian example',ha='center',va='top',fontsize=7)
    with (SOURCE/'mechanism.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['age_s','mean_x_er','mean_x_both','existence_er','existence_both']);w.writerows(zip(age,np.ones(len(age)),2*q_new,r_fixed,r_both))
    save(fig,'mechanism')

def outcomes_plot(d):
    fig,axes=plt.subplots(2,2,figsize=(7.12,3.15));fig.subplots_adjust(left=.09,right=.98,bottom=.16,top=.93,hspace=.7,wspace=.4)
    rng=np.random.default_rng(1921);main=['qualified_exist','lineage_recent','confirmed_exist']
    rows={(r['scene'],r['seed'],r['arm']):r for r in d['runs']};paired={(r['scene'],r['arm'],r['reference']):r for r in d['paired']}
    a,b,c,e=axes.ravel()
    for ax,letter in zip(axes.ravel(),'abcd'):label(ax,letter)
    for si,scene in enumerate(SCENES):
        for ai,arm in enumerate(main):
            x=si+(ai-1)*.23;values=[rows[scene,s,arm]['ospa']-rows[scene,s,'lineage']['ospa'] for s in d['seeds']]
            ci=paired[scene,arm,'lineage']['ospa'];a.scatter(x+rng.uniform(-.025,.025,20),values,s=5,alpha=.35,color=COLOR[arm])
            a.errorbar(x,ci['mean'],yerr=[[max(0,ci['mean']-ci['low'])],[max(0,ci['high']-ci['mean'])]],fmt='o',ms=3.2,color=COLOR[arm],capsize=2)
    a.axhline(0,color='#555555',lw=.7);a.set(xticks=[0,1,2],xticklabels=['Split','Churn','No-new'],ylabel='Δ OSPA (m)')
    for si,scene in enumerate(SCENES[:2]):
        for ai,arm in enumerate(main):
            x=si+(ai-1)*.23;r=next(r for r in d['common_target'] if r['scene']==scene and r['arm']==arm and r['reference']=='lineage')
            mean=100*(r['ratio']-1);lo=100*(r['ratio_low']-1);hi=100*(r['ratio_high']-1)
            b.errorbar(x,mean,yerr=[[max(0,mean-lo)],[max(0,hi-mean)]],fmt='o',ms=3.5,color=COLOR[arm],capsize=2)
    b.axhline(0,color='#555555',lw=.7);b.set(xticks=[0,1],xticklabels=['Split','Churn'],ylabel='RMSE change (%)')
    controls=['lineage','qualified_exist','confirmed_exist','fov']
    aggregate={(r['scene'],r['arm']):r for r in d['aggregate']}
    for ax,scene,metric in [(c,'churn_departure','post_departure_false2'),(e,'split_no_new','false2')]:
        for ai,arm in enumerate(controls):
            v=[rows[scene,s,arm][metric] for s in d['seeds']];r=aggregate[scene,arm][metric]
            ax.scatter(ai+rng.uniform(-.08,.08,20),v,s=7,alpha=.4,color=COLOR[arm])
            ax.errorbar(ai,r['mean'],yerr=[[max(0,r['mean']-r['low'])],[max(0,r['high']-r['mean'])]],fmt='D',ms=3.5,color=COLOR[arm],capsize=2)
        ax.set(xticks=range(len(controls)),xticklabels=['Lineage','ER','Confirmed','FoV'],ylim=(0,None))
    c.set_ylabel('Departure cost (m²)');e.set_ylabel('No-new false cost (m²)')
    fig.legend(handles=[Line2D([],[],marker='o',color=COLOR[k],lw=0,label=LABEL[k]) for k in main],loc='lower center',ncol=3,bbox_to_anchor=(.5,.002))
    save(fig,'outcomes')

def time_plot(d):
    fig,axes=plt.subplots(1,3,figsize=(7.12,2.02));fig.subplots_adjust(left=.065,right=.99,bottom=.31,top=.84,wspace=.33)
    main=['lineage','lineage_recent','qualified_exist','confirmed_exist'];time=np.arange(120)*.5
    for i,(ax,scene,metric) in enumerate(zip(axes,['split_latebirth','churn_departure','churn_departure'],['ospa','ospa','false2'])):
        label(ax,chr(97+i));arr=np.asarray(d['curves'][scene][metric])
        for arm in main:
            j=d['arms'].index(arm);curve=arr[:,j].mean(0)
            ax.plot(time,curve,color=COLOR[arm],ls=STYLE[arm],label=LABEL[arm],lw=1.1)
        ax.axvline(17,color='#777777',lw=.6,ls=':')
        if i:ax.axvline(45,color='#777777',lw=.6,ls=':')
        ax.set(xlim=(0,59.5),ylim=(0,None),xlabel='Time (s)',ylabel='OSPA (m)' if i<2 else 'False cost (m²)',title=['Split–rejoin','Churn–departure','Departure response'][i])
        if i==2:ax.set_xlim(40,59.5)
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=4,bbox_to_anchor=(.5,.01))
    save(fig,'time')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--initial',action='store_true');args=parser.parse_args()
    scene_plot();mechanism_plot()
    if not args.initial:
        summary=DATA/'summary_validation.json'
        if not summary.exists():summary=SOURCE/'validation_summary.json'
        d=json.loads(summary.read_text());outcomes_plot(d);time_plot(d)
        (SOURCE/'validation_summary.json').write_text(json.dumps(d,separators=(',',':'))+'\n')
        for name in ['validation_runs.csv','validation_common_support.csv']:
            if (DATA/name).exists():(SOURCE/name).write_bytes((DATA/name).read_bytes())
    print('Exported figures:',', '.join(p.name for p in FIG.glob('*.svg')))

if __name__=='__main__':main()
