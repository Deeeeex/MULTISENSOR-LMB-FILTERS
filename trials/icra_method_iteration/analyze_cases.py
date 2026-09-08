from pathlib import Path
import argparse,csv,gzip,hashlib,json,re,sys
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_case_studies import previous,interval

def read(path):
    with gzip.open(path,'rt') as f:return json.load(f)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',choices=['cr','cgr'],default='cr')
    parser.add_argument('--output-dir',type=Path,help='Write new audit summaries here; preserve registered summaries.');args=parser.parse_args()
    destination=args.output_dir.resolve() if args.output_dir else OUT
    candidate='confirmation_recency' if args.candidate=='cgr' else 'conservative_recency'
    source=json.loads((OUT/('source_sha256_cgr_cases.json' if args.candidate=='cgr' else 'source_sha256_cases.json')).read_text())
    for name,digest in source.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    matlab={re.sub('[^A-Za-z0-9_]','_',p)[:63]:h for p,h in source.items()};assert len(matlab)==len(source)
    original=OUT.parent/'icra_reunion_fusion';baseline=json.loads((original/'summary_validation.json').read_text())
    rows=[];inputs=[];audited=0;common=[]
    for entry in baseline['inputs']:
        scene=entry['scene'];seed=entry['seed'];stem=f'{scene}_seed{seed}_validation'
        path=original/'results'/f'{stem}.json.gz';assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256']
        old=read(path);path=OUT/('results_cases_confirmed' if args.candidate=='cgr' else 'results_cases')/f'{stem}_{args.candidate}.json.gz';new=read(path)
        assert new['sourceSha256']==matlab
        for key in ['truth','truthRegions','time','positions','visibility','componentCount','validationOffsets','reconnectionFrames']:
            assert new[key]==old[key],(stem,key)
        run=new['runs'];assert run['arm']==candidate
        ref=next(r for r in old['runs'] if r['arm']=='qualified_exist')
        for key in ['attemptedMessages','deliveredMessages','wireBytes','controlBytes']:
            assert run[key]==ref[key],(stem,key)
        row,matched,_,n=previous.inspect_run(new,run);rows.append(row);audited+=n
        for arm in ['qualified_exist','lineage']:
            ref=next(r for r in old['runs'] if r['arm']==arm)
            _,other,_,_=previous.inspect_run(old,ref);valid=np.isfinite(matched)&np.isfinite(other)
            common.append(dict(scene=scene,seed=seed,reference=arm,support=int(valid.sum()),
                               candidate_sse=float(matched[valid].sum()),reference_sse=float(other[valid].sum())))
        inputs.append(dict(scene=scene,seed=seed,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                           cached_input_sha256=hashlib.sha256((original/'results'/f'{stem}.mat').read_bytes()).hexdigest()))
        print('AUDITED',args.candidate,'case',scene,seed,audited,flush=True)
    assert audited==60*8*120 and len(rows)==60
    references=['qualified_exist','lineage','mil_support','confirmed_exist']
    oldrows={(r['scene'],r['seed'],r['arm']):r for r in baseline['runs']}
    lookup={(r['scene'],r['seed'],r['arm']):r for r in rows}
    samples=np.random.default_rng(8301).integers(0,20,(10000,20))
    numeric=['ospa','count_mae','gospa','miss2','false2','loc2','reunion_ospa','post_departure_false2','remote_acquisitions','remote_queries','wire_bytes','raw_bytes']
    aggregate=[];paired=[]
    for scene in previous.SCENES:
        for arm in [candidate]+references:
            group=[(lookup if arm==candidate else oldrows)[scene,seed,arm] for seed in range(2901,2921)]
            aggregate.append(dict(scene=scene,arm=arm,**{key:interval([r[key] for r in group],samples) for key in numeric}))
        for ref in references:
            a=[lookup[scene,s,candidate] for s in range(2901,2921)]
            b=[oldrows[scene,s,ref] for s in range(2901,2921)]
            paired.append(dict(scene=scene,reference=ref,**{key:interval([x[key]-y[key] for x,y in zip(a,b)],samples) for key in numeric},
                               ospa_wins=sum(x['ospa']<y['ospa']-1e-10 for x,y in zip(a,b))))
    result=dict(protocol=new['protocol'],candidate=candidate,source_hashes_verified=len(source),
                audited_node_frames=audited,runs=rows,aggregate=aggregate,paired=paired,inputs=inputs,common_target=common)
    suffix='_confirmed' if args.candidate=='cgr' else ''
    destination.mkdir(parents=True,exist_ok=True)
    (destination/f'summary_cases{suffix}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (destination/f'case_runs{suffix}.csv').open('w',newline='') as f:
        fields=['scene','seed','arm']+numeric
        w=csv.DictWriter(f,fields,extrasaction='ignore',lineterminator='\n');w.writeheader();w.writerows(rows)
    for r in aggregate:print(r['scene'],r['arm'],'OSPA',round(r['ospa']['mean'],5),'missed',round(r['miss2']['mean'],3),'false',round(r['false2']['mean'],3))
    print('CASE',args.candidate,'AUDIT PASSED',audited,'new node-frames; original source and baseline hashes unchanged.')

if __name__=='__main__':main()
