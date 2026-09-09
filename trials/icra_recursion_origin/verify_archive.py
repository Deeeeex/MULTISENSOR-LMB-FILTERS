"""Verify frozen sources, exact ports and report totals against all six native trajectories."""
from collections import defaultdict
from pathlib import Path
import ast
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'FINAL_VERIFICATION.json';assert not destination.exists()
    report=json.loads((OUT/'INTERVENTION_RESULTS.json').read_text());assert report['passed']
    freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text());assert freeze['passed']
    protected={};paths={};diagnostics={};checked=[]
    for name,expected in freeze['configurations'].items():
        assert sha(ROOT/name)==expected;protected[name]=expected
        cfg=json.loads((ROOT/name).read_text())
        for source,digest in cfg['source_sha256'].items():assert sha(ROOT/source)==digest,source
        protected.update(cfg['source_sha256'])
        runtime=json.loads((OUT/('runtime_'+cfg['stage']+'.json')).read_text())
        assert len(runtime)==len(cfg['units']) and all(r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files'] for r in runtime)
        audit=json.loads((OUT/('audit_'+cfg['stage']+'.json')).read_text());assert audit['passed']
        for source,digest in audit['inputs'].items():assert sha(ROOT/source)==digest,source
        protected.update(audit['inputs'])
        if cfg['stage']=='recursion_preflight':assert len(audit['parity'])==2 and all(len(p['exact_fields'])>=40 for p in audit['parity'])
        else:assert len(audit['parity'])==3 and all(len(p['exact_prefix_fields'])>=20 for p in audit['parity'])
        diagnostics.update({x['arm']:x for x in audit['diagnostics']})
        for unit in cfg['units']:
            for arm in unit['arms']:paths[arm]=OUT/'results'/cfg['stage']/f"{unit['sequence']}_intermittent_{arm}.json.gz"
            if cfg['stage']=='recursion_preflight':paths['marked_lineage_range']=ROOT/unit['references']['marked_lineage_range']['path']
        with (OUT/('target_frames_'+cfg['stage']+'.csv')).open(newline='') as stream:checked+=list(csv.DictReader(stream))
    lookup={(r['arm'],int(r['frame']),int(r['robot'])):r for r in checked};assert len(lookup)==len(checked)==2880
    trace=json.loads((OUT/'TRACE_FREEZE.json').read_text());mat=loadmat(ROOT/trace['unit']['data_path'])
    total=0;columns=0
    for arm,path in paths.items():
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        run=data['runs'];increments=defaultdict(list);local=defaultdict(list);fused=defaultdict(list)
        for row in run['localIncrementRecords']:increments[int(row[0]),int(row[1])].append(row)
        exists={tuple(row[:4]):row[5] for row in run['localIncrementRecords']}
        for row in run['localGaussianRecords']:local[int(row[0]),int(row[1])].append([*row[:4],exists[tuple(row[:4])],*row[18:32]])
        for row in run['fusionOutputRecords']:fused[int(row[0]),int(row[1])].append(row)
        actual=[]
        for t in range(1,241):
            truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T
            ti=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[ti,:2]
            for n in [1,2]:
                i=n-1+2*(t-1);row=lookup[arm,t,n];estimate=np.asarray(run['estimates'][i],float).reshape(-1,4)
                distance=np.array([[math.hypot(a[0]-b[0],a[1]-b[1]) for b in estimate] for a in truth]).reshape(len(truth),len(estimate))
                a,b=linear_sum_assignment(np.where(distance<=2,distance,1e6))
                detected=any(x==ti and distance[x,y]<=2 for x,y in zip(a,b))
                assert detected==(row['detected']=='True') and len(estimate)==int(row['output_count'])
                source=fused[t,n] if data['delivered'][n-1][2-n][t-1] else local[t,n]
                active=[r for r in source if r[4]>.001];near=[r for r in active if math.hypot(r[5]-xy[0],r[6]-xy[1])<=2]
                assert len(active)==int(row['active_components']) and len(near)==int(row['near_active_components'])
                assert abs(math.fsum(r[4] for r in near)-float(row['near_total_r']))<1e-11
                assert max([r[4] for r in near],default=0)==float(row['near_max_r'])
                z=np.asarray(mat['measurements'][n-1,t-1],float).reshape(2,-1);inc=increments[t,n]
                if row['association_has_column']=='True':
                    assert len(inc) and z.shape[1]
                    distances=[math.hypot(r[0]-xy[0],r[1]-xy[1]) for r in z.T];column=int(np.argmin(distances))
                    W=np.asarray(run['localAssociationWeights'][i],float).reshape(len(inc),z.shape[1]+1)
                    mass=[r[5]*w[column+1] for r,w in zip(inc,W)];summed=math.fsum(mass);p=[x/summed for x in mass] if summed else [0.]*len(mass)
                    entropy=-math.fsum(x*math.log(x) for x in p if x>0)
                    for key,value in [('association_nearest_distance_m',distances[column]),('association_effective_labels',math.exp(entropy)),
                                      ('association_largest_share',max(p)),('association_joint_mass',summed)]:
                        assert abs(value-float(row[key]))<1e-10,(arm,t,n,key)
                    columns+=1
                else:assert not len(inc) or not z.shape[1]
                actual.append(dict(t=t,n=n,detected=detected,near=len(near),all=len(active)))
                total+=1
        target=diagnostics[arm]['target']
        for key,left,right in [('full',1,240),('before_event',1,2),('after_event',3,240),('original_window',53,122)]:
            selected=[r for r in actual if left<=r['t']<=right];expected=target['windows'][key]
            assert sum(r['detected'] for r in selected)==expected['detected'] and len(selected)==expected['robot_frames']
            assert [sum(r['detected'] for r in selected if r['n']==n) for n in [1,2]]==expected['by_robot']
            assert max(r['near'] for r in selected)==expected['maximum_near_components']
            assert max(r['all'] for r in selected)==expected['maximum_components']
        print('ARCHIVE NATIVE TARGET VERIFIED',arm,flush=True)
    assert total==2880 and len(paths)==6
    for receipt_name,target_name,target_field in [('RUNNER_PATCH.json','runRecursionIntervention.m','target_sha256'),
        ('AUDITOR_PATCH.json','intervention_gaussian_audit.py','output_sha256')]:
        receipt=json.loads((OUT/receipt_name).read_text());source=ROOT/receipt['source']
        assert sha(source)==receipt['source_sha256'];content=source.read_text()
        for change in receipt['changes']:
            assert content.count(change['before'])==1;content=content.replace(change['before'],change['after'])
        assert content==(OUT/target_name).read_text() and sha(OUT/target_name)==receipt[target_field]
    rows={r['arm']:r for r in report['rows']};assert set(rows)==set(paths)
    gs='marked_gaussian_evidence_guarded_scalar_range'
    for effect in report['effects']:
        arm=effect['arm']
        for prefix,window in [('window','original_window'),('full','full')]:
            assert effect[prefix+'_detection_delta']==diagnostics[arm]['target']['windows'][window]['detected']-diagnostics[gs]['target']['windows'][window]['detected']
        for field,key in [('ospa_change_percent','ospa'),('wire_byte_change_percent','wire_bytes')]:
            assert effect[field]==100*(rows[arm][key]/rows[gs][key]-1)
    assert report['any_window_improvement']==any(e['window_detection_delta']>0 for e in report['effects'])
    for name,expected in report['inputs'].items():assert sha(ROOT/name)==expected,name
    build=json.loads((OUT/'REPORT_BUILD.json').read_text())
    for name,field in [('build_report.py','source_sha256'),('INTERVENTION_RESULTS.json','results_sha256'),('RESULTS_CN.md','report_sha256'),('ALL_INTERVENTION_SCORES.csv','csv_sha256')]:
        assert sha(OUT/name)==build[field]
    for p in OUT.glob('*.py'):ast.parse(p.read_text())
    for p in OUT.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and '.next.' not in p.name:protected[str(p.relative_to(ROOT))]=sha(p)
    result=dict(passed=True,native_new_runs=5,native_new_robot_frames=2400,native_reused_robot_frames=480,
        independent_target_rows=total,independent_association_columns=columns,trace_target_rows=5760,
        exact_original_controls=2,exact_prefix_interventions=3,source_sha256=sha(Path(__file__)),input_sha256=protected,
        scope='Causal single-case diagnostic complete; no algorithm promotion, selection or extended intervention')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('FINAL RECURSION VERIFICATION PASSED',total,'native target rows;',len(protected),'protected files',flush=True)

if __name__=='__main__':main()
