"""Locate and decompose the first nominal GCE/GS versus No-age difference."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys
import numpy as np
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for name in ['icra_recursion_origin','icra_reviewer_revision']:sys.path.insert(0,str(OUT.parent/name))
from trace_v2 import prepare,rows_at,pool_at,compare,sorted_rows
from review_gaussian_audit import natural,integrate,unpack
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def density(prepared,backend,row,source_labels):
    t,n=map(int,row[:2]);ids=source_labels.reshape(2,2).astype(int)
    present=ids[:,0]>0;b=row[13:15];active=b>0;alpha=(active&present).astype(float);alpha/=alpha.sum()
    priors=[];posts=[]
    for side in range(2):
        if not present[side]:
            priors.append((np.zeros((4,4)),np.zeros(4),0.));posts.append(priors[-1]);continue
        r=prepared['gaussian_lookup'][t,n if side==0 else 3-n,*ids[side]]
        priors.append(natural(r[4:8],unpack(r[8:18])));posts.append(natural(r[18:22],unpack(r[22:32])))
    jp,hp,cp=[np.asarray([x[k] for x in priors]) for k in range(3)]
    ju,hu,cu=[np.asarray([x[k] for x in posts]) for k in range(3)]
    J0=np.sum(alpha[:,None,None]*ju,axis=0);H0=np.sum(alpha[:,None]*hu,axis=0);C0=float(np.sum(alpha*cu))
    base_mean,base_cov,base_i=integrate(J0,H0,C0)
    beta=b if backend=='No-age' else row[28:30]
    kept=np.zeros(2) if backend=='No-age' else row[52:54]
    new_mean,new_cov,new_i=base_mean,base_cov,row[10]
    if backend=='GCE' and kept.any():
        new_mean,new_cov,new_i=integrate(J0+np.sum(kept[:,None,None]*(ju-jp),axis=0),
            H0+np.sum(kept[:,None]*(hu-hp),axis=0),C0+float(np.sum(kept*(cu-cp))))
    logits=np.zeros(2);r=np.clip(row[17:19][active],1e-9,1-1e-9);logits[active]=np.log(r)-np.log1p(-r)
    inherited=float(np.sum(b*logits));history=float(np.sum((beta-b)*logits))
    current=0. if backend=='No-age' else float(np.sum(kept*row[19:21]))
    expected_r=float(expit(inherited+history+current+new_i))
    assert abs(base_i-row[10])<1e-8 and abs(expected_r-row[6])<2e-10
    actual=next(r for r in rows_at(prepared,'fusion',t,n) if np.array_equal(r[2:4],row[2:4]))
    assert np.allclose(new_mean,actual[5:9],atol=1e-7,rtol=0)
    assert np.allclose(new_cov,unpack(actual[9:]),atol=1e-7,rtol=0)
    return dict(backend=backend,inherited_log_odds=inherited,history_change=history,current_scalar_change=current,
        base_spatial_integral=float(base_i),spatial_integral_change=float(new_i-base_i),actual_integral=float(new_i),
        r=float(row[6]),reconstructed_r=expected_r,r_with_no_extra_scalar=float(expit(inherited+base_i)),
        mean=new_mean.tolist(),covariance=new_cov.tolist(),beta=beta.tolist(),kept=kept.tolist())

def detail(left,right,backend,t,n,cfg):
    inputs=[]
    for source in [1,2]:
        inputs.append(dict(source=source,distribution=compare(rows_at(left,'local',t,source),rows_at(right,'local',t,source),cfg),
            exact_increment=np.array_equal(sorted_rows(rows_at(left,'inc',t,source)),sorted_rows(rows_at(right,'inc',t,source))),
            exact_gaussian=np.array_equal(sorted_rows(rows_at(left,'gaussian',t,source)),sorted_rows(rows_at(right,'gaussian',t,source)))))
    a=sorted_rows(rows_at(left,'records',t,n));b=sorted_rows(rows_at(right,'records',t,n))
    asource=sorted_rows(rows_at(left,'source_labels',t,n));bsource=sorted_rows(rows_at(right,'source_labels',t,n))
    amap={tuple(r[2:4]):r for r in a};bmap={tuple(r[2:4]):r for r in b}
    asl={tuple(r[2:4]):r[4:] for r in asource};bsl={tuple(r[2:4]):r[4:] for r in bsource}
    rows=[]
    for label in sorted(amap.keys()&bmap.keys()):
        x=density(left,backend,amap[label],asl[label]);y=density(right,'No-age',bmap[label],bsl[label])
        parts={k:x[k]-y[k] for k in ['inherited_log_odds','history_change','current_scalar_change','base_spatial_integral','spatial_integral_change']}
        rows.append(dict(label=[int(k) for k in label],source_labels_equal=bool(np.array_equal(asl[label],bsl[label])),
            left=x,noage=y,log_odds_difference_parts=parts))
    return dict(frame=t,robot=n,local_sources=inputs,source_matching_equal=bool(np.array_equal(asource,bsource)),
        same_output_labels=amap.keys()==bmap.keys(),rows=rows)

def main():
    destination=OUT/'ORIGIN_TRACE.json';assert not destination.exists()
    freeze=OUT/'TRACE_FREEZE.json';cfg=json.loads(freeze.read_text())
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    origins=[];differences=[];inputs={}
    for condition in ['reliable','intermittent']:
        group={};headers=None
        for cell in cfg['cells']:
            if cell['condition']!=condition:continue
            path=ROOT/cell['path'];assert sha(path)==cell['sha256'];inputs[cell['path']]=sha(path)
            with gzip.open(path,'rt') as stream:data=json.load(stream)
            current={k:data[k] for k in ['truth','truthIds','positions','delivered','time']}
            if headers is None:headers=current
            else:assert current==headers
            assert data['runs']['arm']==cell['arm'] and data['runs']['rangeDetectionMode']=='nominal'
            group[cell['backend']]=prepare(data)
        for backend in ['GCE','Guarded Scalar']:
            left,right=group[backend],group['No-age'];first={k:None for k in ['predicted','local','posterior','fused','target_local','target_posterior']}
            for t in range(1,241):
                idx=np.flatnonzero(np.asarray(headers['truthIds'][t-1]).ravel()==cfg['truth_id']).item()
                xy=np.asarray(headers['truth'][t-1],float).reshape(4,-1)[:2,idx]
                for n in [1,2]:
                    for phase in first:
                        if phase=='fused' and not headers['delivered'][n-1][2-n][t-1]:continue
                        values=[]
                        for prepared in [left,right]:
                            if phase in ['posterior','target_posterior']:pool=pool_at(prepared,t,n);pool=pool[pool[:,4]>.001]
                            else:pool=rows_at(prepared,'fusion' if phase=='fused' else 'local' if phase=='target_local' else phase,t,n)
                            if phase.startswith('target_'):pool=pool[np.sum((pool[:,5:7]-xy)**2,axis=1)<=4]
                            values.append(pool)
                        comparison=compare(*values,cfg)
                        differences.append(dict(condition=condition,backend=backend,frame=t,robot=n,phase=phase,**comparison))
                        if first[phase] is None and not comparison['equal']:first[phase]=dict(frame=t,robot=n,**comparison)
            event=first['fused'];assert event is not None
            origins.append(dict(condition=condition,backend=backend,first=first,
                event=detail(left,right,backend,event['frame'],event['robot'],cfg)))
            print('NOMINAL FIRST DIFFERENCE',condition,backend,json.dumps(first),flush=True)
    with (OUT/'PAIR_DIFFERENCES.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(differences[0]),lineterminator='\n');writer.writeheader();writer.writerows(differences)
    # Bind the already independently verified complete nominal target rows.
    timeline=OUT.parent/'icra_recursion_origin/TARGET_TIMELINE.csv'
    with timeline.open(newline='') as stream:target=[r for r in csv.DictReader(stream) if r['mode']=='nominal']
    assert len(target)==2880 and len({(r['condition'],r['backend'],r['frame'],r['robot']) for r in target})==2880
    result=dict(passed=True,scope='Descriptive nominal comparison; no modified recursion',origins=origins,
        comparisons=len(differences),target_reference_rows=len(target),target_reference_path=str(timeline.relative_to(ROOT)),
        target_reference_sha256=sha(timeline),inputs=inputs,freeze_sha256=sha(freeze),
        source_sha256=sha(Path(__file__)),differences_sha256=sha(OUT/'PAIR_DIFFERENCES.csv'))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('NOMINAL ORIGIN TRACE COMPLETE',len(differences),'comparisons',flush=True)

if __name__=='__main__':main()
