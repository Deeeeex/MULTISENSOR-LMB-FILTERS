"""Read complete audited recursions; apply the declared advancement rule."""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
GCE='marked_gaussian_evidence'
HISTORY=GCE+'_miss_history'
HALF=GCE+'_miss_half'
METRICS=['ospa','gospa','loc2','miss2','false2','countError']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0,str(OUT.parent/'icra_v2x_gce_diagnosis'))
from diagnose_gap import read,matched_truth


def main():
    p=argparse.ArgumentParser();p.add_argument('--preflight-only',action='store_true');args=p.parse_args()
    stages=['miss_history_preflight']+([] if args.preflight_only else ['miss_history_screen'])
    rows=[];hashes={}
    for stage in stages:
        path=OUT/('audit_'+stage+'.json');audit=json.loads(path.read_text());assert audit['passed']
        for name,expected in audit['inputs'].items():assert sha(ROOT/name)==expected,name
        cfg=OUT/'stages'/(stage+'.json');runtime=OUT/('runtime_'+stage+'.json')
        assert sha(cfg)==audit['config_sha256'] and sha(runtime)==audit['runtime_sha256']
        rows+=audit['rows'];hashes[str(path.relative_to(ROOT))]=sha(path)
    summaries=[]
    for dataset in sorted({r['dataset'] for r in rows}):
        chosen=[r for r in rows if r['dataset']==dataset]
        expected=1 if args.preflight_only or dataset=='v2x_test_mechanism' else 9 if dataset=='v2v_development' else 5
        for condition in ['reliable','intermittent']:
            for arm in [GCE,HISTORY,HALF]:
                part=[r for r in chosen if r['condition']==condition and r['arm']==arm]
                assert len(part)==len({r['sequence'] for r in part})==expected
                summaries.append(dict(dataset=dataset,condition=condition,arm=arm,sequences=len(part),
                    sequence_macro={k:float(np.mean([r[k] for r in part])) for k in METRICS},
                    wire_bytes=sum(r['wire_bytes'] for r in part)))
    comparison=[]
    for dataset in sorted({r['dataset'] for r in rows}):
        part=[r for r in rows if r['dataset']==dataset]
        lookup={(r['sequence'],r['condition'],r['arm']):r for r in part}
        scenes=sorted({r['sequence'] for r in part})
        for condition in ['reliable','intermittent']:
            for candidate,reference in [(HISTORY,GCE),(HALF,GCE),(HISTORY,HALF)]:
                deltas={s:lookup[s,condition,candidate]['ospa']-lookup[s,condition,reference]['ospa'] for s in scenes}
                comparison.append(dict(dataset=dataset,condition=condition,candidate=candidate,reference=reference,
                    mean_delta=float(np.mean(list(deltas.values()))),sequence_deltas=deltas,
                    improved=sum(v < -1e-10 for v in deltas.values()),worsened=sum(v > 1e-10 for v in deltas.values())))
    mechanism=[]
    for condition in ['reliable','intermittent']:
        base=OUT.parent/'icra_temporal_association/results/association_screen_selected_test'
        noage=read(base/f'v2xt_0001_{condition}_marked_lineage.json.gz')
        for arm in [GCE,HISTORY,HALF]:
            data=read(OUT/'results/miss_history_preflight'/f'v2xt_0001_{condition}_{arm}.json.gz');run=data['runs']
            records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
            local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
            increments={tuple(row[:4].astype(int)):row for row in np.asarray(run['localIncrementRecords'],float).reshape(-1,12)}
            delivery=np.asarray(data['delivered'],bool)
            target_frames=0;noage_only=0;probabilities=[];fragment=[]
            for t in range(1,len(data['time'])+1):
                ids=np.asarray(data['truthIds'][t-1]).reshape(-1)
                index=np.flatnonzero(ids==5)
                if not len(index):continue
                index=index.item();truth=np.asarray(data['truth'][t-1]).reshape(4,-1)
                for n in [1,2]:
                    flag=matched_truth(truth,run['estimates'][n-1+2*(t-1)],2.)[index]
                    oldflag=matched_truth(truth,noage['runs']['estimates'][n-1+2*(t-1)],2.)[index]
                    if 53<=t<=122:
                        target_frames+=int(flag);noage_only+=int(oldflag and not flag)
                    z=records[(records[:,0]==t)&(records[:,1]==n)]
                    pool='fused'
                    if not delivery[n-1,2-n,t-1]:
                        assert not len(z)
                        current=local[(local[:,0]==t)&(local[:,1]==n)]
                        z=np.zeros((len(current),60));z[:,:4]=current[:,:4];z[:,4:6]=current[:,18:20]
                        z[:,9]=[increments[tuple(row[:4].astype(int))][5] for row in current]
                        pool='local'
                    label=z[(z[:,2]==3)&(z[:,3]==100004)]
                    if t==53 and n==1 and len(label):probabilities.append(float(label[0,9]))
                    if t==119:
                        nearby=z[(np.sum((z[:,4:6]-truth[:2,index])**2,axis=1)<=4)&(z[:,9]>.001)]
                        fragment.append(dict(robot=n,pool=pool,components=len(nearby),max_r=float(max(nearby[:,9],default=0.))))
            mechanism.append(dict(condition=condition,arm=arm,detected_target_robot_frames_53_122=target_frames,
                noage_only_target_robot_frames_53_122=noage_only,frame53_robot1_r=probabilities,
                frame119_fragmentation=fragment))
    advance=None
    if not args.preflight_only:
        changes={dataset:float(np.mean([r['mean_delta'] for r in comparison if r['dataset']==dataset and
                    r['candidate']==HISTORY and r['reference']==GCE])) for dataset in ['v2v_development','v2x_val']}
        advance=all(v<0 for v in changes.values())
    result=dict(passed=True,preflight_only=args.preflight_only,rows=rows,summaries=summaries,comparisons=comparison,
                mechanism=mechanism,advance=advance,inputs=hashes,source_sha256=sha(Path(__file__)),
                exposure='All inputs exposed; no new generalization claim')
    destination=OUT/('PREFLIGHT_ANALYSIS.json' if args.preflight_only else 'SCREEN_SELECTION.json')
    assert not destination.exists();destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('MISS HISTORY SUMMARY',len(rows),'rows','advance',advance,flush=True)
    for row in comparison:print(row['dataset'],row['condition'],row['candidate'],row['reference'],row['mean_delta'],flush=True)
    for row in mechanism:print('MECHANISM',json.dumps(row),flush=True)


if __name__=='__main__':main()
