"""Independently find first differences and recount all unchanged nominal target rows."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_recursion_origin'))
from verify_trace import native,get,pool,equality
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'TRACE_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'TRACE_FREEZE.json').read_text());report=json.loads((OUT/'ORIGIN_TRACE.json').read_text());assert report['passed']
    for name,expected in cfg['source_sha256'].items():assert sha(ROOT/name)==expected,name
    assert sha(OUT/'trace.py')==report['source_sha256'] and sha(OUT/'PAIR_DIFFERENCES.csv')==report['differences_sha256']
    with (ROOT/report['target_reference_path']).open(newline='') as stream:rows=[r for r in csv.DictReader(stream) if r['mode']=='nominal']
    assert sha(ROOT/report['target_reference_path'])==report['target_reference_sha256']
    lookup={(r['condition'],r['backend'],int(r['frame']),int(r['robot'])):r for r in rows};assert len(lookup)==len(rows)==2880
    checked=0;firsts=[];components=0
    for condition in ['reliable','intermittent']:
        sources={}
        for cell in cfg['cells']:
            if cell['condition']!=condition:continue
            path=ROOT/cell['path'];assert sha(path)==cell['sha256']
            with gzip.open(path,'rt') as stream:data=json.load(stream)
            sources[cell['backend']]=source=native(data)
            for t in range(1,241):
                truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T
                ti=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[ti,:2]
                for n in [1,2]:
                    r=lookup[condition,cell['backend'],t,n]
                    estimate=np.asarray(data['runs']['estimates'][n-1+2*(t-1)],float).reshape(-1,4)
                    distance=np.array([[math.hypot(a[0]-b[0],a[1]-b[1]) for b in estimate] for a in truth]).reshape(len(truth),len(estimate))
                    a,b=linear_sum_assignment(np.where(distance<=2,distance,1e6))
                    detected=any(x==ti and distance[x,y]<=2 for x,y in zip(a,b));assert detected==(r['detected']=='True')
                    for phase,values in [('predicted',get(source['predicted'],t,n,9)),('local',get(source['local'],t,n,19)),('posterior',pool(source,t,n))]:
                        near=[v for v in values if math.hypot(v[5]-xy[0],v[6]-xy[1])<=2];active=[v for v in near if v[4]>.001]
                        assert len(values)==int(r[phase+'_total_components']) and len(near)==int(r[phase+'_near_components'])
                        assert len(active)==int(r[phase+'_near_active_components'])
                        assert abs(math.fsum(v[4] for v in active)-float(r[phase+'_near_total_r']))<1e-11
                        if active:assert max(v[4] for v in active)==float(r[phase+'_near_max_r'])
                        else:assert not r[phase+'_near_max_r']
                    checked+=1
        for backend in ['GCE','Guarded Scalar']:
            origin=next(r for r in report['origins'] if r['backend']==backend and r['condition']==condition)
            for phase in origin['first']:
                first=None
                for t in range(1,241):
                    data=sources[backend]['data'];ti=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item()
                    xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,ti]
                    for n in [1,2]:
                        if phase=='fused' and not data['delivered'][n-1][2-n][t-1]:continue
                        values=[]
                        for name in [backend,'No-age']:
                            source=sources[name]
                            if phase in ['posterior','target_posterior']:v=pool(source,t,n);v=v[v[:,4]>.001]
                            else:v=get(source['fusion' if phase=='fused' else 'local' if phase=='target_local' else phase],t,n,9 if phase=='predicted' else 19)
                            if phase.startswith('target_'):v=v[np.sum((v[:,5:7]-xy)**2,axis=1)<=4]
                            values.append(v)
                        if not equality(*values,cfg['probability_tolerance'],cfg['spatial_tolerance']):first=(t,n);break
                    if first is not None:break
                saved=origin['first'][phase];assert first==(saved['frame'],saved['robot'])
                firsts.append(dict(condition=condition,backend=backend,phase=phase,frame=first[0],robot=first[1]))
            for record in origin['event']['rows']:
                for key in ['left','noage']:
                    d=record[key]
                    total=math.fsum(d[k] for k in ['inherited_log_odds','history_change','current_scalar_change','base_spatial_integral','spatial_integral_change'])
                    probability=1/(1+math.exp(-total))
                    assert abs(probability-d['r'])<2e-10
                parts=record['log_odds_difference_parts']
                for key,value in parts.items():assert value==record['left'][key]-record['noage'][key]
                a,b=record['left']['r'],record['noage']['r']
                assert abs(math.fsum(parts.values())-(math.log(a)-math.log1p(-a)-math.log(b)+math.log1p(-b)))<1e-8
                components+=1
            print('NOMINAL ORIGIN VERIFIED',condition,backend,flush=True)
    assert checked==2880 and len(firsts)==24
    result=dict(passed=True,target_rows_recounted=checked,first_events=firsts,first_event_scalar_decompositions_checked=components,
        scope='Independent native first-event and target recount, plus scalar decomposition arithmetic; Gaussian reconstruction uses the previously audited moment formulas',
        inputs={name:sha(ROOT/name) for name in report['inputs']},report_sha256=sha(OUT/'ORIGIN_TRACE.json'),
        source_sha256=sha(Path(__file__)),freeze_sha256=sha(OUT/'TRACE_FREEZE.json'))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('NOMINAL TRACE VERIFICATION PASSED',checked,'native target rows',flush=True)

if __name__=='__main__':main()
