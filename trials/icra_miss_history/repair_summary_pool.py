"""Preserve v1 and include the actual local pool on communication loss."""
from pathlib import Path
import hashlib
import json
import py_compile

OUT=Path(__file__).resolve().parent
source=OUT/'summarize.py';destination=OUT/'summarize_v2.py'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
text=source.read_text()
changes=[
 ("            records=np.asarray(run['iterationRecords'],float).reshape(-1,60)", """            records=np.asarray(run['iterationRecords'],float).reshape(-1,60)
            local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
            increments={tuple(row[:4].astype(int)):row for row in np.asarray(run['localIncrementRecords'],float).reshape(-1,12)}
            delivery=np.asarray(data['delivered'],bool)"""),
 ("                    z=records[(records[:,0]==t)&(records[:,1]==n)]", """                    z=records[(records[:,0]==t)&(records[:,1]==n)]
                    pool='fused'
                    if not delivery[n-1,2-n,t-1]:
                        assert not len(z)
                        current=local[(local[:,0]==t)&(local[:,1]==n)]
                        z=np.zeros((len(current),60));z[:,:4]=current[:,:4];z[:,4:6]=current[:,18:20]
                        z[:,9]=[increments[tuple(row[:4].astype(int))][5] for row in current]
                        pool='local'"""),
 ("fragment.append(dict(robot=n,components=len(nearby),max_r=float(max(nearby[:,9],default=0.))))", "fragment.append(dict(robot=n,pool=pool,components=len(nearby),max_r=float(max(nearby[:,9],default=0.))))"),
]
for before,after in changes:
    assert text.count(before)==1,before
    text=text.replace(before,after)
assert not destination.exists();destination.write_text(text);py_compile.compile(str(destination),doraise=True)
receipt=dict(reason='The intermittent blackout includes frame 119. A diagnostic count must use the saved local pool on undelivered frames rather than the absent fusion log.',
             original=source.name,original_sha256=sha(source),repaired=destination.name,repaired_sha256=sha(destination),
             replacements=changes,algorithm_or_selection_changed=False)
(OUT/'SUMMARY_POOL_REPAIR.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('PRESERVED SUMMARY V1; CREATED V2 WITH LOCAL-POOL FALLBACK')
