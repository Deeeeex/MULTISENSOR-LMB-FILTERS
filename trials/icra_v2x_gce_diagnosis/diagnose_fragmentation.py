"""Count surviving nearby Bernoullis in the selected native failure case."""
from pathlib import Path
import csv
import gzip
import hashlib
import json

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=OUT/'TRACK_DIAGNOSIS.json';diagnosis=json.loads(source.read_text())
case=next(c for c in diagnosis['traces'] if c['reason']=='longest_strict_interval')
inputs={str(source.relative_to(ROOT)):sha(source),str(Path(__file__).relative_to(ROOT)):sha(Path(__file__))}
rows=[]
for arm in ['marked_gaussian_evidence','marked_lineage']:
    suffix=f"/{case['sequence']}_{case['condition']}_{arm}.json.gz"
    paths=[p for p in diagnosis['inputs'] if p.endswith(suffix)];assert len(paths)==1
    path=ROOT/paths[0];assert sha(path)==diagnosis['inputs'][paths[0]];inputs[paths[0]]=sha(path)
    with gzip.open(path,'rt') as f:data=json.load(f)
    records=np.asarray(data['runs']['iterationRecords'],float).reshape(-1,60)
    for t in range(1,len(data['time'])+1):
        ids=np.asarray(data['truthIds'][t-1]).reshape(-1)
        found=np.flatnonzero(ids==case['truth_id'])
        if not len(found):continue
        xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,found[0]]
        for n in [1,2]:
            z=records[(records[:,0]==t)&(records[:,1]==n)]
            nearby=z[np.linalg.norm(z[:,4:6]-xy,axis=1)<=2]
            kept=nearby[nearby[:,9]>.001];total=float(kept[:,9].sum())
            trace=next(r for r in case['rows'] if r['frame']==t and r['robot']==n and r['arm']==arm)
            assert len(nearby)==trace['best_candidate']['near_components']
            rows.append(dict(sequence=case['sequence'],condition=case['condition'],truth_id=case['truth_id'],frame=t,robot=n,arm=arm,
                             preprune_components=len(nearby),retained_components=len(kept),sum_r=total,
                             max_r=float(kept[:,9].max()) if len(kept) else 0.,
                             probability_effective_count=float(total**2/np.sum(kept[:,9]**2)) if total else 0.,
                             born_in_last_ten_frames=int((kept[:,2]>=t-9).sum()),
                             total_retained_components=int((z[:,9]>.001).sum()),matched=trace['matched']))
csv_path=OUT/'fragmentation_frames.csv'
with csv_path.open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
report=dict(passed=True,sequence=case['sequence'],condition=case['condition'],truth_id=case['truth_id'],rows=rows,
            inputs=inputs,csv_sha256=sha(csv_path),scope='All frames of the previously selected longest strict No-age-only case. Count actual native post-fusion components within 2 m; distinguish pre-pruning and surviving components. No new selection or intervention.')
(OUT/'FRAGMENTATION_DIAGNOSIS.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print('FRAGMENTATION COUNTED',len(rows),'native robot-method frames')
