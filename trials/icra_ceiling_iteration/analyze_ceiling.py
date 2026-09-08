"""Independent full-output and analytic audit of continuous evidence ceilings."""
from pathlib import Path
import argparse,csv,gzip,hashlib,json,re,sys
import numpy as np
from scipy.special import expit

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_case_studies import score
from analyze_v2v4real import domain,interval
sys.path.insert(0,str(OUT.parent/'icra_method_iteration'))
from analyze_development import counterfactual

CANDIDATES=['ceiling_association','ceiling_score','ceiling_calibrated']
METRICS=['ospa','gospa','loc2','miss2','false2','countError','raw_bytes','wire_bytes']


def read(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--last',type=int,default=8)
    parser.add_argument('--output-dir',type=Path);args=parser.parse_args()
    destination=args.output_dir.resolve() if args.output_dir else OUT
    source=json.loads((OUT/'source_sha256.json').read_text())
    for name,digest in source.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    matlab={re.sub('[^A-Za-z0-9_]','_',p)[:63]:h for p,h in source.items()};assert len(matlab)==len(source)
    original=json.loads((OUT.parent/'icra_external_fusion/summary_v2v4real.json').read_text())
    hashes={(r['sequence'],r['condition']):r['result_sha256'] for r in original['inputs']}
    baseline={(r['sequence'],r['condition'],r['arm']):r for r in original['runs']}
    for filename in ['summary_conservative.json','summary_confirmed.json']:
        for r in json.loads((OUT.parent/'icra_method_iteration'/filename).read_text())['runs']:
            baseline[r['sequence'],r['condition'],r['arm']]=r
    rows=[];diagnostics=[];inputs=[];common=[];audited=0;parity=0
    for seq in range(args.last+1):
        name=f'{seq:04d}'
        for condition in ['reliable','intermittent']:
            path=OUT/'results_development'/f'{name}_{condition}.json.gz';data=read(path)
            oldpath=OUT.parent/'icra_external_fusion/results_v2v'/path.name
            assert hashlib.sha256(oldpath.read_bytes()).hexdigest()==hashes[name,condition]
            old=read(oldpath);T=len(data['time']);poses=np.asarray(data['positions']);delivery=np.asarray(data['delivered'])
            assert data['implementation']=='ecr-v1' and not data['smoke'] and data['sourceSha256']==matlab
            for key in ['truth','truthIds','positions','time','delivered','inputSha256']:assert data[key]==old[key],key
            assert [r['arm'] for r in data['runs']]==['qualified_exist']+CANDIDATES
            matches={}
            for run in old['runs']:
                matches[run['arm']]=np.concatenate([score(data['truth'][t],run['estimates'][n+2*t])['match_d2'] for t in range(T) for n in range(2)])
            for run in data['runs']:
                arm=run['arm'];reference=next(r for r in old['runs'] if r['arm']=='qualified_exist')
                for key in ['attemptedMessages','deliveredMessages','controlBytes']:assert run[key]==reference[key],key
                payload=np.asarray(run['rawPayloadBytes']);wire=np.asarray(run['wireBytes']);control=np.asarray(run['controlBytes'])
                assert np.all((payload-64)%216==0) and np.all(wire>=payload+control)
                assert np.all(np.asarray(run['deliveredRawBytes'])<=payload) and run['totalWireBytes']==wire.sum()
                if arm=='qualified_exist':
                    for key in ['estimates','rawEstimates','labels','ospa','countError']:
                        assert run[key]==reference[key],(name,condition,key,'ER parity')
                    parity+=2*T
                values={k:[] for k in METRICS[:6]};sse=0;support=0;matched=[]
                for t in range(T):
                    for n in range(2):
                        output=run['estimates'][n+2*t];raw=run['rawEstimates'][n+2*t]
                        assert np.array_equal(np.asarray(output).reshape(-1,4),np.asarray(raw).reshape(-1,4)[domain(raw,poses[:,:,t])])
                        v=score(data['truth'][t],output);audited+=1
                        for key in ['ospa','countError','matchedSquaredError','matchedCount']:
                            assert np.isclose(v[key],run[key][n][t],rtol=1e-9,atol=1e-8),(name,condition,arm,key,t,n)
                        for key in values:values[key].append(v[key])
                        sse+=v['matchedSquaredError'];support+=v['matchedCount']
                        matched.append(v['match_d2'])
                matches[arm]=np.concatenate(matched)
                rows.append(dict(sequence=name,condition=condition,arm=arm,frames=T,**{k:float(np.mean(v)) for k,v in values.items()},
                                 matched_sse=sse,matched_support=support,raw_bytes=sum(run['rawPayloadBytes']),
                                 wire_bytes=sum(run['wireBytes']),runtime_s=run['runtimeSeconds'],maximum_bernoulli_count=run['maximumBernoulliCount']))
                records=np.asarray(run['iterationRecords'],float).reshape(-1,26)
                b=records[:,13:15];q=records[:,11:13];r=records[:,17:19];evidence=records[:,19:21];active=b>0
                logits=np.zeros_like(b);rr=np.clip(r[active],1e-9,1-1e-9);logits[active]=np.log(rr)-np.log1p(-rr)
                base=(b*logits).sum(1)+records[:,10];age=((q-b)*logits).sum(1)
                assert ((evidence>=0)&(evidence<=1)).all()
                assert (evidence[records[:,21:23]!=records[:,0,None]]==0).all(),(name,arm,'stale support')
                eligible=active&(q>b+1e-12)&(r>=.5)
                cap=np.where(eligible,evidence,0).max(1)
                r0=expit(base);rER=expit(base+age)
                expected=rER if arm=='qualified_exist' else np.minimum(rER,np.maximum(r0,cap))
                assert np.allclose(records[:,7],r0,atol=2e-12,rtol=0)
                assert np.allclose(records[:,8],rER,atol=2e-12,rtol=0)
                assert np.allclose(records[:,6],expected,atol=2e-12,rtol=0),(name,arm,'analytic existence')
                assert np.allclose(records[:,9],expected,atol=2e-12,rtol=0)
                assert np.allclose(records[:,23],age,atol=1e-10)
                assert np.allclose(records[:,23]-records[:,24],records[:,25],atol=1e-10)
                assert (records[:,6]>=np.minimum(r0,rER)-2e-12).all() and (records[:,6]<=rER+2e-12).all()
                if arm in CANDIDATES:assert np.allclose(records[age<0,6],rER[age<0],atol=2e-12)
                effects={rule:{k:[] for k in METRICS[:6]} for rule in ['no_age','ER','candidate']}
                for t in range(T):
                    for n in range(2):
                        if not delivery[n,1-n,t]:continue
                        rr=records[(records[:,0]==t+1)&(records[:,1]==n+1)]
                        for rule,col in [('no_age',7),('ER',8),('candidate',9)]:
                            v=score(data['truth'][t],counterfactual(rr,col,poses[:,:,t]))
                            if rule=='candidate':assert np.isclose(v['ospa'],run['ospa'][n][t],atol=1e-8)
                            for key in v.keys()&effects[rule].keys():effects[rule][key].append(v[key])
                diagnostics.append(dict(sequence=name,condition=condition,arm=arm,labels=len(records),
                                        positive_age_labels=int((age>1e-8).sum()),
                                        support_above_noage=int((cap>r0+1e-8).sum()),
                                        constrained_below_er=int((records[:,6]<rER-1e-8).sum()),
                                        preserved_above_cr=int((records[:,6]>np.minimum(r0,rER)+1e-8).sum()),
                                        same_input={rule:{k:float(np.mean(v)) for k,v in vv.items()} for rule,vv in effects.items()}))
            for candidate in CANDIDATES:
                for reference in ['qualified_exist','lineage','mil_support','tc_ospa2_w5','tc_ospa2_w10']:
                    a,b=matches[candidate],matches[reference];both=np.isfinite(a)&np.isfinite(b)
                    common.append(dict(sequence=name,condition=condition,candidate=candidate,reference=reference,
                                       support=int(both.sum()),candidate_sse=float(a[both].sum()),reference_sse=float(b[both].sum())))
            inputs.append(dict(sequence=name,condition=condition,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('ECR AUDITED',name,condition,audited,'node-frames; ER parity',parity,flush=True)
    n=args.last+1;lookup={**baseline,**{(r['sequence'],r['condition'],r['arm']):r for r in rows if r['arm'] in CANDIDATES}}
    samples=np.random.default_rng(8301).integers(0,n,(10000,n));aggregate=[];paired=[]
    arms=CANDIDATES+['qualified_exist','lineage','conservative_recency','confirmation_recency','mil_support','tc_ospa2_w5','tc_ospa2_w10','local']
    for condition in ['reliable','intermittent']:
        for arm in arms:
            group=[lookup[f'{s:04d}',condition,arm] for s in range(n)]
            aggregate.append(dict(condition=condition,arm=arm,**{k:interval([r[k] for r in group],samples) for k in METRICS}))
        for candidate in CANDIDATES:
            for reference in [a for a in arms if a!=candidate]:
                a=[lookup[f'{s:04d}',condition,candidate] for s in range(n)];b=[lookup[f'{s:04d}',condition,reference] for s in range(n)]
                paired.append(dict(condition=condition,candidate=candidate,reference=reference,
                                   **{k:interval([x[k]-y[k] for x,y in zip(a,b)],samples) for k in METRICS},
                                   ospa_wins=sum(x['ospa']<y['ospa']-1e-10 for x,y in zip(a,b))))
    result=dict(protocol='direct-evidence-ceiling-v1',scope='Development only; all selected sequences, no outcome exclusion; calibration is sequence-held-out.',
                sequences=n,audited_node_frames=audited,original_er_bitwise_parity_node_frames=parity,
                source_hashes_verified=len(source),aggregate=aggregate,paired=paired,runs=rows,diagnostics=diagnostics,inputs=inputs,common_target=common,
                er_packet_note='Parity runner carries unused evidence-ceiling metadata; all headline ER metrics/bytes use the original frozen ER baseline.')
    suffix='' if n==9 else f'_first{n}';destination.mkdir(parents=True,exist_ok=True)
    (destination/f'summary_development{suffix}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (destination/f'development_runs{suffix}.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    for r in aggregate:print(r['condition'],r['arm'],'OSPA',round(r['ospa']['mean'],5),'miss',round(r['miss2']['mean'],3),'false',round(r['false2']['mean'],3))
    print('ECR DEVELOPMENT AUDIT PASSED',audited,'node-frames; original ER parity',parity)


if __name__=='__main__':main()
