"""All-seed v4 audit; pair episodes rather than treating frames as replicates."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import numpy as np
from analyze_results import OUT, ROOT, SCENES, inspect_run
from analyze_extension import phase_readout

SEEDS=list(range(2901,2921))
ARMS=['local','fov','lineage','mil','mil_support','recent','lineage_recent','qualified_exist','confirmed_exist']
PAIRS=[('qualified_exist','lineage'),('lineage_recent','lineage'),
       ('confirmed_exist','lineage'),('lineage_recent','qualified_exist'),
       ('confirmed_exist','qualified_exist'),('qualified_exist','fov'),
       ('qualified_exist','mil_support')]
NUMERIC=['ospa','count_mae','gospa','loc2','miss2','false2','matched_rmse',
         'reunion_ospa','post_departure_false2','worst_node','wire_bytes',
         'raw_bytes','delivered_raw_bytes','runtime_s']

def interval(values, resamples):
    values=np.asarray(values,dtype=float)
    return dict(mean=float(values.mean()),low=float(np.quantile(values[resamples].mean(1),.025)),
                high=float(np.quantile(values[resamples].mean(1),.975)),
                sd=float(values.std(ddof=1)),n=len(values))

def main():
    hashes=json.loads((OUT/'validation_source_sha256.json').read_text())
    for path,digest in hashes.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
    paths=[OUT/'results'/f'{scene}_seed{seed}_validation.json.gz' for scene in SCENES for seed in SEEDS]
    assert all(p.exists() for p in paths),'All 60 paired input cases must finish before aggregation.'
    rows=[]; phases=[]; common=[]; curves={}; inputs=[]; audited=0
    for scene in SCENES:
        curves[scene]={k:[] for k in ['ospa','false2','miss2','components','existence3']}
        for seed in SEEDS:
            path=OUT/'results'/f'{scene}_seed{seed}_validation.json.gz'
            with gzip.open(path,'rt') as f: d=json.load(f)
            assert d['mode']=='validation' and d['seed']==seed and d['scene']==scene
            assert [r['arm'] for r in d['runs']]==ARMS
            expected=sum(16384*2*(8-c)+128*8 for c in d['componentCount'])
            assert all(r['totalWireBytes']==expected for r in d['runs'][1:])
            assert all(r['deliveredMessages']==d['runs'][1]['deliveredMessages'] for r in d['runs'][1:])
            assert np.max(np.abs(d['validationOffsets']))<=.6
            matches={}; metric_rows={}; case_curves={k:[] for k in ['ospa','false2','miss2','existence3']}
            for r in d['runs']:
                row,match,metrics,count=inspect_run(d,r);rows.append(row);matches[r['arm']]=match;audited+=count
                metric_rows[r['arm']]=metrics
                phases.extend(dict(scene=scene,seed=seed,arm=r['arm'],**p) for p in phase_readout(d,r))
                case_curves['ospa'].append(np.asarray(r['ospa']).mean(0).tolist())
                for k in ['false2','miss2']:case_curves[k].append(metrics[k].mean(0).tolist())
                case_curves['existence3'].append(np.asarray(r['labelExistence'])[:,2,:].mean(0).tolist())
                assert np.all(np.asarray(r['directOpportunity'])<=np.arange(1,121)[None,None,:])
            for left,right in PAIRS:
                a,b=matches[left],matches[right];joint=np.isfinite(a)&np.isfinite(b)
                assert joint.any()
                common.append(dict(scene=scene,seed=seed,arm=left,reference=right,
                    support=int(joint.sum()),candidate_sse=float(a[joint].sum()),reference_sse=float(b[joint].sum()),
                    candidate_rmse=float(np.sqrt(a[joint].mean())),reference_rmse=float(np.sqrt(b[joint].mean()))))
            for k in case_curves:curves[scene][k].append(case_curves[k])
            curves[scene]['components'].append(d['componentCount'])
            inputs.append(dict(scene=scene,seed=seed,reunions=d['reconnectionFrames'],
                components=d['componentCount'],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('AUDITED',scene,seed,'total node-frames',audited,flush=True)
    rng=np.random.default_rng(8301);resamples=rng.integers(0,len(SEEDS),size=(10000,len(SEEDS)))
    aggregate=[];paired=[];common_aggregate=[]
    lookup={(r['scene'],r['seed'],r['arm']):r for r in rows}
    for scene in SCENES:
        for arm in ARMS:
            selected=[lookup[scene,s,arm] for s in SEEDS]
            a=dict(scene=scene,arm=arm,**{k:interval([r[k] for r in selected],resamples) for k in NUMERIC})
            a['matched_support']=sum(r['matched_support'] for r in selected)
            a['remote_acquisitions']=sum(r['remote_acquisitions'] for r in selected)
            a['remote_queries']=sum(r['remote_queries'] for r in selected)
            aggregate.append(a)
        for left,right in PAIRS:
            record=dict(scene=scene,arm=left,reference=right)
            for k in NUMERIC:
                values=[lookup[scene,s,left][k]-lookup[scene,s,right][k] for s in SEEDS]
                record[k]=interval(values,resamples)
            paired.append(record)
            selected=[r for r in common if r['scene']==scene and r['arm']==left and r['reference']==right]
            a=np.asarray([r['candidate_sse'] for r in selected]);b=np.asarray([r['reference_sse'] for r in selected])
            n=np.asarray([r['support'] for r in selected])
            sampled_a=np.sqrt(a[resamples].sum(1)/n[resamples].sum(1))
            sampled_b=np.sqrt(b[resamples].sum(1)/n[resamples].sum(1))
            ratio=np.sqrt(a.sum()/b.sum())
            common_aggregate.append(dict(scene=scene,arm=left,reference=right,support=int(n.sum()),
                candidate_rmse=float(np.sqrt(a.sum()/n.sum())),reference_rmse=float(np.sqrt(b.sum()/n.sum())),
                ratio=float(ratio),ratio_low=float(np.quantile(sampled_a/sampled_b,.025)),
                ratio_high=float(np.quantile(sampled_a/sampled_b,.975))))
    report=dict(protocol='robot-reunion-fusion-v4',seeds=SEEDS,arms=ARMS,
        audited_node_frames=audited,source_hashes_verified=len(hashes),aggregate=aggregate,
        paired=paired,common_target=common_aggregate,runs=rows,common_runs=common,phases=phases,curves=curves,inputs=inputs,
        bootstrap=dict(replicates=10000,seed=8301,unit='paired episode',interval='95% percentile',multiplicity_adjusted=False),
        development_screen='V2/V3 combinations failed the balanced gate. V1 Age-all passed its initial screen but lost to Lineage and worsened common-target localization. No retuning after validation.')
    (OUT/'summary_validation.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    fields=['scene','seed','arm']+NUMERIC+['matched_support','remote_acquisitions','remote_queries']
    with (OUT/'validation_runs.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    with (OUT/'validation_common_support.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(common[0]));w.writeheader();w.writerows(common)
    print('PASS: all',audited,'node-frames and',len(hashes),'source hashes audited.')
    for x in aggregate:print(x['scene'],x['arm'],'OSPA',round(x['ospa']['mean'],4),'false2',round(x['false2']['mean'],4))

if __name__=='__main__':main()
