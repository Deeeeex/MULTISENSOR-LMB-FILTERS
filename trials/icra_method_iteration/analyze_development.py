"""Independently score the method iteration, including adverse counterfactuals."""
from pathlib import Path
import argparse,csv,gzip,hashlib,json,re,sys
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_case_studies import score
from analyze_v2v4real import domain,interval

METRICS=['ospa','gospa','loc2','miss2','false2','countError']
def read(path):
    with gzip.open(path,'rt') as f:return json.load(f)

def counterfactual(rows,column,poses):
    rows=rows[rows[:,column]>.001]
    probabilities=rows[:,column]-1e-6
    pmf=np.array([1.])
    for p in probabilities:pmf=np.convolve(pmf,[1-p,p])
    count=int(pmf.argmax())
    chosen=rows[np.argsort(-probabilities,kind='stable')[:count]]
    states=np.c_[chosen[:,4:6],np.zeros((count,2))]
    return states[domain(states,poses)].tolist()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--last',type=int,default=8)
    parser.add_argument('--candidate',choices=['ir','cr','cgr'],default='ir')
    parser.add_argument('--accelerated',action='store_true')
    parser.add_argument('--output-dir',type=Path,help='Write new audit summaries here; preserve registered summaries.');args=parser.parse_args()
    destination=args.output_dir.resolve() if args.output_dir else OUT
    conservative=args.candidate=='cr';confirmed=args.candidate=='cgr'
    candidate={'ir':'innovation_recency','cr':'conservative_recency','cgr':'confirmation_recency'}[args.candidate]
    candidate_short=args.candidate.upper()
    assert not confirmed or args.accelerated
    source_name=f'source_sha256_{args.candidate}_fast.json' if args.accelerated else ('source_sha256_cr.json' if conservative else 'source_sha256.json')
    source=json.loads((OUT/source_name).read_text())
    for name,digest in source.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    matlab={re.sub('[^A-Za-z0-9_]','_',p)[:63]:h for p,h in source.items()};assert len(matlab)==len(source)
    original_summary=json.loads((OUT.parent/'icra_external_fusion/summary_v2v4real.json').read_text())
    baseline_hashes={(r['sequence'],r['condition']):r['result_sha256'] for r in original_summary['inputs']}
    original_rows={(r['sequence'],r['condition'],r['arm']):r for r in original_summary['runs']}
    rows=[];diagnostics=[];inputs=[];audited=0;parity=0;slow_parity=0
    for seq in range(args.last+1):
        name=f'{seq:04d}'
        for condition in ['reliable','intermittent']:
            directory={'ir':'results_v2v','cr':'results_v2v_conservative','cgr':'results_v2v_confirmed'}[args.candidate]
            path=OUT/(directory+('_fast' if args.accelerated else ''))/f'{name}_{condition}.json.gz';data=read(path)
            baseline=OUT.parent/'icra_external_fusion/results_v2v'/path.name
            assert hashlib.sha256(baseline.read_bytes()).hexdigest()==baseline_hashes[name,condition]
            previous=read(baseline);T=len(data['time']);poses=np.asarray(data['positions'])
            implementation=f'{args.candidate}-v1'+('-accelerated' if args.accelerated else '')
            assert not data['smoke'] and data['sourceSha256']==matlab and data['implementation']==implementation
            for key in ['truth','truthIds','time','positions','delivered','inputSha256']:
                assert data[key]==previous[key],(name,condition,key)
            for run in ([data['runs']] if isinstance(data['runs'],dict) else data['runs']):
                arm=run['arm'];metrics={key:[] for key in METRICS}
                reference=next(r for r in previous['runs'] if r['arm']=='qualified_exist')
                for key in ['attemptedMessages','deliveredMessages','controlBytes']:
                    assert run[key]==reference[key]
                if arm=='qualified_exist':
                    for key in ['estimates','rawEstimates','labels','ospa','countError']:
                        assert run[key]==reference[key],(name,condition,key,'instrumentation changed original ER')
                    parity+=2*T
                slow_path=OUT/directory/path.name
                if args.accelerated and slow_path.exists():
                    slow=read(slow_path)['runs'];slow=[slow] if isinstance(slow,dict) else slow
                    oldrun=next(r for r in slow if r['arm']==arm)
                    for key in ['estimates','rawEstimates','labels','ospa','countError','iterationRecords']:
                        assert run[key]==oldrun[key],(name,condition,arm,key,'runtime acceleration changed outputs')
                    slow_parity+=2*T
                sse=0.;support=0
                for t in range(T):
                    for n in range(2):
                        value=score(data['truth'][t],run['estimates'][n+2*t]);audited+=1
                        for key in ['ospa','countError','matchedSquaredError','matchedCount']:
                            assert np.isclose(value[key],run[key][n][t],rtol=1e-9,atol=1e-8),(name,condition,arm,key)
                        for key in METRICS:metrics[key].append(value[key])
                        sse+=value['matchedSquaredError'];support+=value['matchedCount']
                        raw=run['rawEstimates'][n+2*t]
                        assert np.array_equal(np.asarray(run['estimates'][n+2*t]).reshape(-1,4),
                                              np.asarray(raw).reshape(-1,4)[domain(raw,poses[:,:,t])])
                rows.append(dict(sequence=name,condition=condition,arm=arm,frames=T,
                                 **{k:float(np.mean(v)) for k,v in metrics.items()},
                                 matched_sse=sse,matched_support=support,
                                 raw_bytes=sum(run['rawPayloadBytes']),wire_bytes=sum(run['wireBytes'])))
                records=np.asarray(run['iterationRecords'],dtype=float).reshape(-1,26)
                assert np.isfinite(records[:,:17]).all() and np.isfinite(records[:,19:]).all()
                assert np.allclose(records[:,23]-records[:,24],records[:,25],atol=1e-10)
                if conservative:
                    assert np.all(records[:,6]<=records[:,7]+1e-12) and np.all(records[:,6]<=records[:,8]+1e-12)
                    assert np.all(records[:,24]<=1e-12)
                if confirmed:
                    gate=((records[:,11:13]>records[:,13:15]+1e-12)&(records[:,17:19]>=.5)&
                          (records[:,13:15]>0)&(records[:,19:21]>0)).any(1)
                    expected=np.where(gate,records[:,8],np.minimum(records[:,7],records[:,8]))
                    assert np.allclose(records[:,6],expected,atol=1e-12,rtol=0)
                # All effects below hold the recursive inputs and spatial pool fixed.
                effects={rule:{key:[] for key in METRICS} for rule in ['no_age','ER',candidate_short]}
                delivery=np.asarray(data['delivered'])
                for t in range(T):
                    for n in range(2):
                        if not delivery[n,1-n,t]:continue
                        rr=records[(records[:,0]==t+1)&(records[:,1]==n+1)]
                        for rule,column in [('no_age',7),('ER',8),(candidate_short,9)]:
                            x=counterfactual(rr,column,poses[:,:,t])
                            v=score(data['truth'][t],x)
                            if (arm=='qualified_exist' and rule=='ER') or (arm==candidate and rule==candidate_short):
                                assert np.isclose(v['ospa'],run['ospa'][n][t],atol=1e-8),(name,arm,'counterfactual extraction')
                            for key in METRICS:effects[rule][key].append(v[key])
                changed=np.abs(records[:,23])>1e-6
                diagnostics.append(dict(sequence=name,condition=condition,arm=arm,labels=len(records),
                                        age_changed_labels=int(changed.sum()),
                                        increased_existence=int((records[:,23]>1e-6).sum()),
                                        decreased_existence=int((records[:,23]<-1e-6).sum()),
                                        posterior_shift_abs=float(np.abs(records[:,23]).sum()),
                                        candidate_correction_abs=float(np.abs(records[:,24]).sum()),
                                        removed_correction_abs=float(np.abs(records[:,25]).sum()),
                                        removed_dominates_changed=int((changed&(np.abs(records[:,25])>np.abs(records[:,24]))).sum()),
                                        controlled_node_frames=len(effects['ER']['ospa']),
                                        same_input={rule:{key:float(np.mean(v)) for key,v in metric.items()} for rule,metric in effects.items()}))
            inputs.append(dict(sequence=name,condition=condition,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('AUDITED',name,condition,'new',audited,'original ER parity',parity,flush=True)
    n=args.last+1;samples=np.random.default_rng(8301).integers(0,n,(10000,n))
    lookup={(r['sequence'],r['condition'],r['arm']):r for r in rows};paired=[];aggregate=[]
    for condition in ['reliable','intermittent']:
        for arm in [candidate,'qualified_exist','lineage','mil_support','tc_ospa2_w5','tc_ospa2_w10','local']:
            group=[lookup.get((f'{seq:04d}',condition,arm),original_rows.get((f'{seq:04d}',condition,arm))) for seq in range(n)]
            aggregate.append(dict(condition=condition,arm=arm,**{key:interval([r[key] for r in group],samples) for key in METRICS}))
        for reference in ['qualified_exist','lineage','mil_support','tc_ospa2_w5','tc_ospa2_w10']:
            a=[lookup[f'{seq:04d}',condition,candidate] for seq in range(n)]
            b=[original_rows[f'{seq:04d}',condition,reference] for seq in range(n)]
            paired.append(dict(condition=condition,reference=reference,
                               **{key:interval([x[key]-y[key] for x,y in zip(a,b)],samples) for key in METRICS},
                               ospa_wins=sum(x['ospa']<y['ospa']-1e-10 for x,y in zip(a,b))))
    result=dict(protocol=data['protocol'],candidate=candidate,scope='Development data, all selected sequences included; descriptive sequence bootstrap.',
                implementation=implementation,sequences=n,audited_node_frames=audited,original_er_bitwise_parity_node_frames=parity,
                slow_runtime_bitwise_parity_node_frames=slow_parity,
                source_hashes_verified=len(source),aggregate=aggregate,paired=paired,runs=rows,diagnostics=diagnostics,inputs=inputs)
    suffix='' if n==9 else f'_first{n}'
    prefix={'ir':'development','cr':'conservative','cgr':'confirmed'}[args.candidate]
    destination.mkdir(parents=True,exist_ok=True)
    (destination/f'summary_{prefix}{suffix}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (destination/f'{prefix}_runs{suffix}.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    for r in aggregate:print(r['condition'],r['arm'],'OSPA',round(r['ospa']['mean'],5),'false',round(r['false2']['mean'],4))
    print('METHOD DEVELOPMENT AUDIT PASSED',audited,'node-frames; original ER bitwise parity checked on',parity,'node-frames.')

if __name__=='__main__':main()
