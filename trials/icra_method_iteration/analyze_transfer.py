"""Independent full-cohort audit; six reserved sequences, one overlap control."""
from pathlib import Path
import argparse,csv,gzip,hashlib,json,re,sys
import numpy as np
from scipy.io import loadmat
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_case_studies import score
from analyze_v2v4real import domain,interval
ARMS=['local','lineage','qualified_exist','conservative_recency','confirmation_recency','mil_support','tc_ospa2_w5','tc_ospa2_w10']
CANDIDATES=['conservative_recency','confirmation_recency']
KEYS=['ospa','gospa','loc2','miss2','false2','countError','raw_bytes','wire_bytes']

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,help='Write new audit summaries here; preserve registered summaries.');args=parser.parse_args()
    destination=args.output_dir.resolve() if args.output_dir else OUT
    source=json.loads((OUT/'source_sha256_transfer.json').read_text())
    for name,digest in source.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    matlab={re.sub('[^A-Za-z0-9_]','_',p)[:63]:h for p,h in source.items()};assert len(matlab)==len(source)
    manifest=json.loads((OUT/'transfer_input_manifest.json').read_text());rows=[];inputs=[];common=[];audited=0
    assert manifest['selected_sequences']==list(range(0,32,5)) and manifest['primary_reserved_sequences']==[5,10,15,20,25,30]
    for name,digest in manifest['source_files'].items():
        assert hashlib.sha256((ROOT/'tmp/external_baselines/DMSTrack'/name).read_bytes()).hexdigest()==digest,name
    assert hashlib.sha256((OUT/'transfer_overlap_audit.json').read_bytes()).hexdigest()==manifest['overlap_audit_sha256']
    assert hashlib.sha256((OUT/'transfer_transform_manifest.json').read_bytes()).hexdigest()==manifest['transforms_manifest_sha256']
    overlap=json.loads((OUT/'transfer_overlap_audit.json').read_text())
    for name,digest in overlap['source_files'].items():
        assert hashlib.sha256((ROOT/'tmp/external_baselines/DMSTrack/AB3DMOT/scripts/KITTI'/name).read_bytes()).hexdigest()==digest,name
    for r in overlap['train_sequences']:
        if int(r['sequence']) in manifest['primary_reserved_sequences']:assert r['overlap_frame_count']==0
    for seq in manifest['sequences']:
        name=seq['sequence'];T=seq['frames'];matpath=OUT/'data_transfer'/f'v2v4real_{name}.mat'
        assert hashlib.sha256(matpath.read_bytes()).hexdigest()==seq['input_sha256'];mat=loadmat(matpath)
        local=None
        for condition in ['reliable','intermittent']:
            path=OUT/'results_transfer'/f'{name}_{condition}.json.gz'
            with gzip.open(path,'rt') as f:data=json.load(f)
            assert data['implementation']=='cr-cgr-transfer-v1' and data['sourceSha256']==matlab and not data['smoke']
            assert data['inputSha256']==seq['input_sha256'] and data['sequence']==name and len(data['time'])==T
            assert [r['arm'] for r in data['runs']]==ARMS
            poses=np.asarray(data['positions']);assert np.array_equal(poses,mat['positions'])
            delivery=np.asarray(data['delivered']);assert delivery.shape==(2,2,T) and not delivery[0,0].any() and not delivery[1,1].any()
            if condition=='reliable':assert delivery.sum()==2*T
            else:assert not delivery[:,:,int(.4*T):int(.6*T)].any()
            for t in range(T):
                assert np.array_equal(np.asarray(data['truth'][t],float).reshape(4,-1),mat['truth'][0,t])
                assert domain(mat['truth'][0,t].T,poses[:,:,t]).all()
                for n in range(2):
                    z=mat['measurements'][n,t].T;assert domain(np.c_[z,np.zeros((len(z),2))],poses[:,:,t]).all()
                    assert (((z-poses[:,n,t])**2).sum(1)<=1600).all()
            matches={}
            for run in data['runs']:
                arm=run['arm'];values={k:[] for k in KEYS[:6]};sse=0.;support=0;match=[]
                assert run['maximumBernoulliCount']<=2000
                if arm=='local':
                    if local is None:local=run['rawEstimates']
                    else:assert local==run['rawEstimates']
                else:
                    assert run['attemptedMessages']==[2]*T and run['deliveredMessages']==delivery.sum((0,1)).tolist()
                    assert run['controlBytes']==[256]*T
                for t in range(T):
                    for n in range(2):
                        raw=np.asarray(run['rawEstimates'][n+2*t],float).reshape(-1,4)
                        output=run['estimates'][n+2*t]
                        assert np.array_equal(np.asarray(output,float).reshape(-1,4),raw[domain(raw,poses[:,:,t])])
                        v=score(data['truth'][t],output)
                        for k in ['ospa','countError','matchedSquaredError','matchedCount']:
                            assert np.isclose(v[k],run[k][n][t],atol=1e-8,rtol=1e-9),(name,condition,arm,t,n,k)
                        for k in values:values[k].append(v[k])
                        sse+=v['matchedSquaredError'];support+=v['matchedCount'];match.append(v['match_d2']);audited+=1
                matches[arm]=np.concatenate(match)
                rows.append(dict(sequence=name,role=seq['role'],condition=condition,arm=arm,frames=T,
                                 **{k:float(np.mean(v)) for k,v in values.items()},matched_sse=sse,matched_support=support,
                                 raw_bytes=sum(run['rawPayloadBytes']),wire_bytes=sum(run['wireBytes']),
                                 runtime_s=run['runtimeSeconds'],maximum_bernoulli_count=run['maximumBernoulliCount']))
            for candidate in CANDIDATES:
                for ref in [a for a in ARMS if a!=candidate]:
                    a,b=matches[candidate],matches[ref];valid=np.isfinite(a)&np.isfinite(b)
                    common.append(dict(sequence=name,role=seq['role'],condition=condition,candidate=candidate,reference=ref,
                                       support=int(valid.sum()),candidate_sse=float(a[valid].sum()),reference_sse=float(b[valid].sum())))
            inputs.append(dict(sequence=name,role=seq['role'],condition=condition,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('TRANSFER AUDITED',name,seq['role'],condition,audited,flush=True)
    assert len(rows)==7*2*8 and audited==1504*2*2*8
    lookup={(r['sequence'],r['condition'],r['arm']):r for r in rows}
    aggregate=[];paired=[]
    for cohort,sequences in [('reserved',[5,10,15,20,25,30]),('overlap_control',[0])]:
        n=len(sequences);samples=np.random.default_rng(8301).integers(0,n,(10000,n))
        for condition in ['reliable','intermittent']:
            for arm in ARMS:
                group=[lookup[f'{s:04d}',condition,arm] for s in sequences]
                stats={k:(interval([r[k] for r in group],samples) if n>1 else dict(mean=group[0][k],n=1)) for k in KEYS}
                aggregate.append(dict(cohort=cohort,condition=condition,arm=arm,**stats))
            for candidate in CANDIDATES:
                for ref in [a for a in ARMS if a!=candidate]:
                    a=[lookup[f'{s:04d}',condition,candidate] for s in sequences]
                    b=[lookup[f'{s:04d}',condition,ref] for s in sequences]
                    stats={k:(interval([x[k]-y[k] for x,y in zip(a,b)],samples) if n>1 else dict(mean=a[0][k]-b[0][k],n=1)) for k in KEYS}
                    paired.append(dict(cohort=cohort,condition=condition,candidate=candidate,reference=ref,**stats,
                                       ospa_wins=sum(x['ospa']<y['ospa']-1e-10 for x,y in zip(a,b))))
    result=dict(protocol='reserved-v2v4real-algorithm-transfer-v1',source_hashes_verified=len(source),
                primary_reserved_sequences=6,primary_reserved_frames=1357,overlap_control_sequences=1,overlap_control_frames=147,
                audited_node_frames=audited,aggregate=aggregate,paired=paired,runs=rows,common_target=common,inputs=inputs,
                inference='Six sequence units; descriptive 95% percentile bootstrap, 10000 resamples seed 8301; detector train split, potentially related routes.',
                integrity='All seven preselected sequences retained; exact-overlap sequence 0000 excluded from primary aggregate before tracking outcomes.')
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'summary_transfer.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (destination/'transfer_runs.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    for r in aggregate:
        if r['cohort']=='reserved':print(r['condition'],r['arm'],'OSPA',round(r['ospa']['mean'],5),'false',round(r['false2']['mean'],3),'miss',round(r['miss2']['mean'],3))
    print('RESERVED TRANSFER AUDIT PASSED',audited,'node-frames, six reserved sequences and one separate overlap control.')

if __name__=='__main__':main()
