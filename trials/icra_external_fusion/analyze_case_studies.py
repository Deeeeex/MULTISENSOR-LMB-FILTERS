"""Independent state scoring and complete paired-episode TC readout."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'trials/icra_reunion_fusion'))
import analyze_results as previous
from analyze_validation import interval


def score(truth,estimates):
    x=np.asarray(truth,dtype=float).reshape(4,-1).T
    y=np.asarray(estimates,dtype=float).reshape(-1,4)
    n,m=len(x),len(y); matched=np.full(n,np.nan)
    c2=144.; cost=((x[:,None,:2]-y[None,:,:2])**2).sum(-1)
    rows,cols=linear_sum_assignment(np.minimum(cost,c2))
    d2=cost[rows,cols]; keep=d2<c2
    matched[rows[keep]]=d2[keep]
    loc=float(d2[keep].sum()); support=int(keep.sum())
    miss=(n-support)*c2/2; false=(m-support)*c2/2
    result=dict(ospa=float(np.sqrt((np.minimum(d2,c2).sum()+c2*abs(n-m))/max(n,m))) if max(n,m) else 0.,
                countError=abs(n-m),matchedSquaredError=loc,matchedCount=support,
                gospa=float(np.sqrt(loc+miss+false)),loc2=loc,miss2=miss,false2=false,match_d2=matched)
    if n<=4 and m<=6:
        independent=exhaustive_score(truth,estimates)
        for key in ['ospa','countError','matchedSquaredError','matchedCount','gospa','loc2','miss2','false2']:
            assert np.isclose(result[key],independent[key],atol=1e-10),key
    return result


exhaustive_score=previous.score
previous.score=score


def main():
    reference=json.loads((ROOT/'trials/icra_reunion_fusion/summary_validation.json').read_text())
    source_hashes=json.loads((ROOT/'trials/icra_reunion_fusion/validation_source_sha256.json').read_text())
    for name,digest in source_hashes.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    rows=[]; audited=0; inputs=[]; expected_arms=['tc_ospa2_w5','tc_ospa2_w10']; max_count=0
    for source in reference['inputs']:
        stem=f"{source['scene']}_seed{source['seed']}_validation"
        baseline=ROOT/'trials/icra_reunion_fusion/results'/f'{stem}.json.gz'
        assert hashlib.sha256(baseline.read_bytes()).hexdigest()==source['sha256']
        with gzip.open(baseline,'rt') as f: original=json.load(f)
        result=OUT/'results'/f'{stem}_tc.json.gz'
        with gzip.open(result,'rt') as f: extended=json.load(f)
        for key in ['truth','truthRegions','positions','time','visibility','componentCount','validationOffsets']:
            assert extended[key]==original[key],(stem,key)
        assert [r['arm'] for r in extended['runs']]==expected_arms
        for run in extended['runs']:
            assert run['preEstimates']==original['runs'][0]['estimates']
            assert run['deliveredMessages']==original['runs'][1]['deliveredMessages']
            assert run['attemptedMessages']==original['runs'][1]['attemptedMessages']
            row,_,_,count=previous.inspect_run(extended,run)
            rows.append(row); audited+=count
            max_count=max(max_count,max(len(e) for e in run['estimates']))
        inputs.append(dict(scene=source['scene'],seed=source['seed'],baseline_sha256=source['sha256'],
                           result_sha256=hashlib.sha256(result.read_bytes()).hexdigest()))
    assert len(rows)==120
    reference_arms=['local','fov','lineage','mil_support','qualified_exist']
    rows.extend(r for r in reference['runs'] if r['arm'] in reference_arms)
    rng=np.random.default_rng(8301);samples=rng.integers(0,20,(10000,20))
    numeric=['ospa','count_mae','gospa','loc2','miss2','false2','matched_rmse','reunion_ospa',
             'post_departure_false2','worst_node','wire_bytes','raw_bytes','delivered_raw_bytes']
    aggregate=[]; paired=[]
    lookup={(r['scene'],r['seed'],r['arm']):r for r in rows}
    for scene in previous.SCENES:
        for arm in reference_arms+expected_arms:
            group=[lookup[scene,seed,arm] for seed in range(2901,2921)]
            aggregate.append(dict(scene=scene,arm=arm,**{k:interval([r[k] for r in group],samples) for k in numeric}))
        for reference_arm in ['lineage','mil_support']+expected_arms:
            a=[lookup[scene,s,'qualified_exist'] for s in range(2901,2921)]
            b=[lookup[scene,s,reference_arm] for s in range(2901,2921)]
            paired.append(dict(scene=scene,arm='qualified_exist',reference=reference_arm,
                               **{k:interval([ra[k]-rb[k] for ra,rb in zip(a,b)],samples) for k in numeric}))
    report=dict(protocol='external-tc-case-studies-v1',runs=rows,aggregate=aggregate,paired=paired,
                source_hashes_verified=len(source_hashes),audited_new_node_frames=audited,
                maximum_tc_output_count=max_count,inputs=inputs,
                runtime_note='New TC time covers fusion and history serialization only; Local runtime is cached.',
                adaptation='Author kinematic Algorithms 1/2, common LBP-LMB backend, independent node labels; no network-wide B.3 label-report claim.')
    (OUT/'summary_case_studies.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    fields=['scene','seed','arm']+numeric+['matched_support','runtime_s']
    with (OUT/'case_study_runs.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    for row in aggregate:
        print(row['scene'],row['arm'],'OSPA',round(row['ospa']['mean'],4),'count',round(row['count_mae']['mean'],4))
    print('AUDIT PASSED',audited,'new node-frames; original inputs/results hashes unchanged.')


if __name__=='__main__':main()
