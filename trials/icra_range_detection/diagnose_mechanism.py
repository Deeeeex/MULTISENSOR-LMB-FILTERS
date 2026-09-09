"""Descriptive target-level trace on the predeclared case; never used for selection."""
from pathlib import Path
from collections import defaultdict
import csv
import hashlib
import json
import sys
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_v2x_gce_diagnosis'))
from diagnose_gap import read,matched_truth
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    auditpath=OUT/'audit_range_detection_preflight.json';audit=json.loads(auditpath.read_text());assert audit['passed']
    config=json.loads((OUT/'stages/range_detection_preflight.json').read_text())
    rows=[];sources={}
    for condition in config['conditions']:
        for arm in config['arms']:
            path=OUT/'results/range_detection_preflight'/f'v2xt_0001_{condition}_{arm}.json.gz'
            expected=audit['inputs'][str(path.relative_to(ROOT))];assert sha(path)==expected
            sources[str(path.relative_to(ROOT))]=expected
            data=read(path);run=data['runs'];delivery=np.asarray(data['delivered'],bool)
            fusion=np.asarray(run['fusionOutputRecords'],float).reshape(-1,19)
            local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
            increments={tuple(r[:4].astype(int)):r for r in np.asarray(run['localIncrementRecords'],float).reshape(-1,12)}
            by_fusion,by_local=defaultdict(list),defaultdict(list)
            for row in fusion:by_fusion[tuple(row[:2].astype(int))].append(row)
            for row in local:by_local[tuple(row[:2].astype(int))].append(row)
            for t in range(53,123):
                ids=np.asarray(data['truthIds'][t-1]).ravel();index=np.flatnonzero(ids==5).item()
                truth=np.asarray(data['truth'][t-1]).reshape(4,-1);xy=truth[:2,index]
                for n in [1,2]:
                    delivered=bool(delivery[n-1,2-n,t-1])
                    if delivered:
                        pool=np.asarray(by_fusion[t,n],float).reshape(-1,19)
                    else:
                        current=np.asarray(by_local[t,n],float).reshape(-1,32)
                        pool=np.zeros((len(current),19));pool[:,:4]=current[:,:4];pool[:,5:9]=current[:,18:22]
                        pool[:,4]=[increments[tuple(r[:4].astype(int))][5] for r in current]
                    near=pool[np.sum((pool[:,5:7]-xy)**2,axis=1)<=4]
                    strongest=near[np.argmax(near[:,4])] if len(near) else None
                    slot=n-1+2*(t-1)
                    output_match=bool(matched_truth(truth,run['estimates'][slot],2.)[index])
                    active=int((pool[:,4]>.001).sum());near_active=int((near[:,4]>.001).sum())
                    rows.append(dict(condition=condition,arm=arm,frame=t,robot=n,delivered=delivered,
                        phase='53_96' if t<97 else '97_122',pool='fused' if delivered else 'local',
                        output_match=output_match,all_components=len(pool),active_components=active,
                        near_components=len(near),near_active_components=near_active,
                        strongest_near_r=None if strongest is None else float(strongest[4]),
                        strongest_birth_time=None if strongest is None else int(strongest[2]),
                        strongest_birth_location=None if strongest is None else int(strongest[3])))
    assert len(rows)==2520
    summary=[]
    for condition in config['conditions']:
        for arm in config['arms']:
            for phase in ['53_96','97_122']:
                selected=[r for r in rows if (r['condition'],r['arm'],r['phase'])==(condition,arm,phase)]
                missed=[r for r in selected if not r['output_match']]
                summary.append(dict(condition=condition,arm=arm,phase=phase,robot_frames=len(selected),
                    detected=sum(r['output_match'] for r in selected),
                    missed_without_active_component_within_2m=sum(r['near_active_components']==0 for r in missed),
                    missed_with_active_component_within_2m=sum(r['near_active_components']>0 for r in missed),
                    max_active_components=max(r['active_components'] for r in selected)))
    csvpath=OUT/'MECHANISM_FRAMES.csv';assert not csvpath.exists()
    with csvpath.open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    result=dict(passed=True,scope='Post-outcome descriptive case only; no parameter or selection change',
        note='The second interval coincides with scheduled blackout only for intermittent links; random losses also occur earlier. A nearby component need not have the correct identity, so these categories are not a causal attribution.',
        rows=len(rows),summaries=summary,inputs=sources,audit_sha256=sha(auditpath),
        csv_sha256=sha(csvpath),source_sha256=sha(Path(__file__)))
    path=OUT/'MECHANISM_TRACE.json';assert not path.exists();path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for row in summary:
        if row['condition']=='intermittent' and row['arm'].endswith('_range'):print(json.dumps(row),flush=True)

if __name__=='__main__':main()
