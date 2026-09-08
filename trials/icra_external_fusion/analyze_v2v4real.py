"""Independent complete-cohort real-detection replay scoring; no tuning."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import re
import numpy as np
from scipy.io import loadmat
from analyze_case_studies import score

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
ARMS=['local','lineage','qualified_exist','mil_support','tc_ospa2_w5','tc_ospa2_w10']
CONDITIONS=['reliable','intermittent']
PAIRS=['lineage','mil_support','tc_ospa2_w5','tc_ospa2_w10','local']


def interval(values,samples):
    x=np.asarray(values,dtype=float); sampled=x[samples].mean(1)
    return dict(mean=float(x.mean()),sd=float(x.std(ddof=1)),low=float(np.quantile(sampled,.025)),
                high=float(np.quantile(sampled,.975)),n=len(x))


def domain(points,positions):
    x=np.asarray(points,dtype=float).reshape(-1,4)[:,:2]
    distances=((x[:,None,:]-positions.T[None,:,:])**2).sum(-1).min(1)
    return (np.abs(x[:,0])<=70.4)&(np.abs(x[:,1])<=40)&(distances<=1600)&(distances>9)


def main():
    source_hashes=json.loads((OUT/'replay_source_sha256.json').read_text())
    for path,digest in source_hashes.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
    # MATLAB jsondecode makes valid structure fields, limited to namelengthmax=63.
    matlab_keys={re.sub(r'[^A-Za-z0-9_]','_',path)[:63]:digest for path,digest in source_hashes.items()}
    assert len(matlab_keys)==len(source_hashes)
    manifest=json.loads((OUT/'v2v4real_input_manifest.json').read_text())
    transform_manifest=OUT/'v2v4real_transform_manifest.json'
    assert hashlib.sha256(transform_manifest.read_bytes()).hexdigest()==manifest['transforms_manifest_sha256']
    transforms=json.loads(transform_manifest.read_text())
    assert transforms['matrix_count']==3986 and len(transforms['source_files'])==3986
    author=ROOT/'tmp/external_baselines/DMSTrack'
    for path,digest in manifest['source_files'].items():
        assert hashlib.sha256((author/path).read_bytes()).hexdigest()==digest,path
    rows=[];common=[];inputs=[];curves=[];audited=0;original_locals={}
    for seq in manifest['sequences']:
        name=seq['sequence'];T=seq['frames'];mat_path=OUT/'data'/f'v2v4real_{name}.mat'
        assert hashlib.sha256(mat_path.read_bytes()).hexdigest()==seq['input_sha256']
        mat=loadmat(mat_path)
        for condition in CONDITIONS:
            path=OUT/'results_v2v'/f'{name}_{condition}.json.gz'
            with gzip.open(path,'rt') as f:data=json.load(f)
            assert not data['smoke'] and data['sequence']==name and len(data['time'])==T
            assert data['implementation']=='consistent-domain-and-absence-v2'
            assert data['sourceSha256']==matlab_keys
            assert data['inputSha256']==seq['input_sha256']
            assert [r['arm'] for r in data['runs']]==ARMS
            poses=np.asarray(data['positions']);assert np.array_equal(poses,mat['positions'])
            delivery=np.asarray(data['delivered'],dtype=bool);assert delivery.shape==(2,2,T)
            assert not delivery[0,0].any() and not delivery[1,1].any()
            if condition=='reliable':assert delivery.sum()==2*T
            else:assert not delivery[:,:,int(.4*T):int(.6*T)].any()
            for t in range(T):
                truth=np.asarray(data['truth'][t],dtype=float).reshape(4,-1)
                assert np.array_equal(truth,mat['truth'][0,t]),(name,t,'truth')
                assert domain(truth.T,poses[:,:,t]).all()
                for n in range(2):
                    z=mat['measurements'][n,t].T
                    assert domain(np.c_[z,np.zeros((len(z),2))],poses[:,:,t]).all()
                    assert np.all(((z-poses[:,n,t])**2).sum(1)<=1600)
            matches={}
            for run in data['runs']:
                arm=run['arm'];metric={k:[] for k in ['ospa','gospa','loc2','miss2','false2','countError']}
                sse=0.;support=0;matched=[]
                assert run['maximumBernoulliCount']<=2000
                assert np.allclose(np.asarray(run['ospa']).shape,[2,T])
                if arm!='local':
                    assert run['deliveredMessages']==delivery.sum((0,1)).tolist()
                    assert run['attemptedMessages']==[2]*T
                    assert run['controlBytes']==[256]*T
                for t in range(T):
                    for n in range(2):
                        estimates=run['estimates'][n+2*t]
                        raw=run['rawEstimates'][n+2*t]
                        raw_array=np.asarray(raw,dtype=float).reshape(-1,4)
                        keep=domain(raw,poses[:,:,t])
                        assert np.array_equal(np.asarray(estimates,dtype=float).reshape(-1,4),raw_array[keep])
                        value=score(data['truth'][t],estimates)
                        for key in ['ospa','countError','matchedSquaredError','matchedCount']:
                            assert np.isclose(value[key],run[key][n][t],atol=1e-8,rtol=1e-9),(name,condition,arm,n,t,key)
                        for key in metric:metric[key].append(value[key])
                        matched.append(value['match_d2']);sse+=value['matchedSquaredError'];support+=value['matchedCount'];audited+=1
                matches[arm]=np.concatenate(matched)
                if arm=='local':
                    if name in original_locals:assert original_locals[name]==run['rawEstimates']
                    else:original_locals[name]=run['rawEstimates']
                row=dict(sequence=name,condition=condition,arm=arm,frames=T,
                         **{k:float(np.mean(v)) for k,v in metric.items()},
                         matched_sse=float(sse),matched_support=int(support),
                         matched_rmse=float(np.sqrt(sse/support)) if support else None,
                         raw_bytes=sum(run['rawPayloadBytes']),wire_bytes=run['totalWireBytes'],
                         delivered_raw_bytes=sum(run['deliveredRawBytes']),runtime_s=run['runtimeSeconds'],
                         maximum_prefusion_bernoulli_count=run['maximumBernoulliCount'],
                         observable_absences=int(np.asarray(run['observableAbsences']).sum()),
                         total_weight_change=float(np.asarray(run['weightChangeL1']).sum()))
                assert row['wire_bytes']==sum(run['wireBytes'])
                rows.append(row)
                curves.append(dict(sequence=name,condition=condition,arm=arm,time=data['time'],
                                   ospa=np.asarray(run['ospa']).mean(0).tolist(),
                                   count_mae=np.asarray(run['countError']).mean(0).tolist()))
            for reference in PAIRS:
                a,b=matches['qualified_exist'],matches[reference];joint=np.isfinite(a)&np.isfinite(b)
                common.append(dict(sequence=name,condition=condition,reference=reference,
                                   candidate_sse=float(a[joint].sum()),reference_sse=float(b[joint].sum()),support=int(joint.sum())))
            inputs.append(dict(sequence=name,condition=condition,input_sha256=seq['input_sha256'],
                               result_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('AUDITED',name,condition,'node-frames',audited,flush=True)
    assert len(rows)==108 and audited==1993*2*2*6
    samples=np.random.default_rng(8301).integers(0,9,(10000,9))
    numeric=['ospa','gospa','countError','loc2','miss2','false2','raw_bytes','wire_bytes','delivered_raw_bytes']
    aggregate=[];paired=[];common_summary=[]
    lookup={(r['sequence'],r['condition'],r['arm']):r for r in rows}
    for condition in CONDITIONS:
        for arm in ARMS:
            group=[lookup[f'{seq:04d}',condition,arm] for seq in range(9)]
            ss=sum(r['matched_sse'] for r in group);n=sum(r['matched_support'] for r in group)
            aggregate.append(dict(condition=condition,arm=arm,**{key:interval([r[key] for r in group],samples) for key in numeric},
                                  matched_rmse=float(np.sqrt(ss/n)) if n else None,matched_support=n))
        for reference in PAIRS:
            a=[lookup[f'{seq:04d}',condition,'qualified_exist'] for seq in range(9)]
            b=[lookup[f'{seq:04d}',condition,reference] for seq in range(9)]
            paired.append(dict(condition=condition,arm='qualified_exist',reference=reference,
                               **{key:interval([x[key]-y[key] for x,y in zip(a,b)],samples) for key in numeric},
                               ospa_sequence_wins=sum(x['ospa']<y['ospa']-1e-10 for x,y in zip(a,b))))
            values=[r for r in common if r['condition']==condition and r['reference']==reference]
            n=sum(v['support'] for v in values);a=sum(v['candidate_sse'] for v in values);b=sum(v['reference_sse'] for v in values)
            common_summary.append(dict(condition=condition,reference=reference,support=n,
                                       candidate_rmse=float(np.sqrt(a/n)) if n else None,
                                       reference_rmse=float(np.sqrt(b/n)) if n else None))
    report=dict(protocol=manifest['protocol'],implementation='consistent-domain-and-absence-v2',
                source_hashes_verified=len(source_hashes),transform_manifest_verified=True,
                audited_node_frames=audited,frames=1993,sequences=9,
                aggregate=aggregate,paired=paired,common_target=common_summary,runs=rows,inputs=inputs,curves=curves,
                bootstrap=dict(unit='sequence',resamples=10000,seed=8301,interval='95% percentile',multiplicity_adjusted=False),
                scope='Exploratory cropped 2-D two-vehicle real-detection replay; moving ego coordinates; emulated radios; all nine sequences included.')
    (OUT/'summary_v2v4real.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (OUT/'v2v4real_runs.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    for row in aggregate:
        print(row['condition'],row['arm'],'OSPA',round(row['ospa']['mean'],4),'count',round(row['countError']['mean'],4))
    print('COMPLETE REPLAY AUDIT PASSED',audited,'node-frames, 18 complete sequence/condition cases.')


if __name__=='__main__':main()
