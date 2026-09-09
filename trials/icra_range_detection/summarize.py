"""Apply all four preregistered gates in both cohorts without curve selection."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
GCE='marked_gaussian_evidence';PRIMARY=GCE+'_range'
REFERENCES=[GCE,GCE+'_constant','marked_lineage_range',GCE+'_guarded_scalar_range']
BASE=[GCE,'marked_lineage',GCE+'_guarded_scalar']
ARMS=[GCE]+[a+s for s in ['_range','_constant'] for a in BASE]
METRICS=['ospa','gospa','loc2','miss2','false2','countError','raw_bytes','delivered_raw_bytes','wire_bytes']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0,str(OUT.parent/'icra_v2x_gce_diagnosis'))
from diagnose_gap import read,matched_truth

def main():
    p=argparse.ArgumentParser();p.add_argument('--preflight-only',action='store_true');args=p.parse_args()
    stages=['range_detection_preflight']+([] if args.preflight_only else ['range_detection_screen'])
    rows=[];hashes={}
    for stage in stages:
        path=OUT/('audit_'+stage+'.json');audit=json.loads(path.read_text());assert audit['passed']
        for name,expected in audit['inputs'].items():assert sha(ROOT/name)==expected,name
        assert sha(OUT/'stages'/(stage+'.json'))==audit['config_sha256']
        assert sha(OUT/('runtime_'+stage+'.json'))==audit['runtime_sha256']
        rows+=audit['rows'];hashes[str(path.relative_to(ROOT))]=sha(path)
    summaries=[];comparisons=[];gates=[]
    for dataset in sorted({r['dataset'] for r in rows}):
        chosen=[r for r in rows if r['dataset']==dataset]
        expected=1 if args.preflight_only or dataset=='v2x_test_mechanism' else 9 if dataset=='v2v_development' else 5
        arms=BASE+[a+s for s in ['_range','_constant'] for a in BASE] if dataset=='v2x_test_mechanism' or args.preflight_only else ARMS
        for condition in ['reliable','intermittent']:
            for arm in arms:
                part=[r for r in chosen if r['condition']==condition and r['arm']==arm]
                assert len(part)==len({r['sequence'] for r in part})==expected
                summaries.append(dict(dataset=dataset,condition=condition,arm=arm,sequences=expected,
                    sequence_macro={k:float(np.mean([r[k] for r in part])) for k in METRICS}))
        lookup={(r['sequence'],r['condition'],r['arm']):r for r in chosen}
        scenes=sorted({r['sequence'] for r in chosen})
        for reference in REFERENCES:
            by_link={}
            for condition in ['reliable','intermittent']:
                deltas={s:lookup[s,condition,PRIMARY]['ospa']-lookup[s,condition,reference]['ospa'] for s in scenes}
                by_link[condition]=deltas
                comparisons.append(dict(dataset=dataset,condition=condition,candidate=PRIMARY,reference=reference,
                    mean_delta=float(np.mean(list(deltas.values()))),sequence_deltas=deltas,
                    improved=sum(v<0 for v in deltas.values()),worsened=sum(v>0 for v in deltas.values())))
            average=float(np.mean([v for link in by_link.values() for v in link.values()]))
            if dataset!='v2x_test_mechanism':gates.append(dict(dataset=dataset,reference=reference,mean_delta=average,passes=average<0))
    mechanism=[]
    for condition in ['reliable','intermittent']:
        for arm in BASE+[a+s for s in ['_range','_constant'] for a in BASE]:
            path=OUT/'results/range_detection_preflight'/f'v2xt_0001_{condition}_{arm}.json.gz'
            data=read(path);run=data['runs'];detected=0;denominator=0
            for t in range(53,123):
                ids=np.asarray(data['truthIds'][t-1]).ravel();index=np.flatnonzero(ids==5).item()
                truth=np.asarray(data['truth'][t-1]).reshape(4,-1)
                for n in [1,2]:
                    detected+=int(matched_truth(truth,run['estimates'][n-1+2*(t-1)],2.)[index]);denominator+=1
            mechanism.append(dict(condition=condition,arm=arm,detected_target_robot_frames_53_122=detected,total=denominator))
    advance=None if args.preflight_only else all(g['passes'] for g in gates)
    if not args.preflight_only:assert len(gates)==8
    result=dict(passed=True,preflight_only=args.preflight_only,rows=rows,summaries=summaries,comparisons=comparisons,
        mechanism=mechanism,gates=gates,advance=advance,inputs=hashes,source_sha256=sha(Path(__file__)),
        exposure='All inputs exposed; case excluded from screen; no independent generalization claim')
    destination=OUT/('PREFLIGHT_ANALYSIS.json' if args.preflight_only else 'SCREEN_SELECTION.json')
    assert not destination.exists();destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('RANGE DETECTION SUMMARY',len(rows),'rows','advance',advance,flush=True)
    for row in gates:print(json.dumps(row),flush=True)
    for row in mechanism:print('MECHANISM',json.dumps(row),flush=True)

if __name__=='__main__':main()
