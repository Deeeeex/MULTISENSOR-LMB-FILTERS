"""Freeze both GCE controls and both one-time negative-scalar interventions."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence'
ARM=GCE+'_once_initial_negative'

def main():
    destination=OUT/'INTERVENTION_FREEZE.json';assert not destination.exists() and not (OUT/'results').exists() and not (OUT/'stages').exists()
    prior=OUT.parent/'icra_range_detection/stages/range_detection_preflight.json';old=json.loads(prior.read_text())
    trace=json.loads((OUT/'TRACE_FREEZE.json').read_text());assert json.loads((OUT/'TRACE_VERIFICATION.json').read_text())['passed']
    sources=old['source_sha256'].copy()
    for name,expected in sources.items():assert sha(ROOT/name)==expected,name
    base=deepcopy(trace['unit'])
    for key in ['reference_paths','reference_sha256','noage_reference_paths','noage_reference_sha256','parity_paths','parity_sha256']:base.pop(key)
    base['intervention_frame']=2
    protected=[prior,OUT/'INTERVENTION_PROTOCOL.md',OUT/'PROTOCOL.md',OUT/'TRACE_FREEZE.json',OUT/'TRACE_VERIFICATION.json',
        OUT/'ORIGIN_TRACE.json',OUT/'PAIR_DIFFERENCES.csv',OUT/'RUNNER_PATCH.json',OUT/'AUDITOR_PATCH.json',
        OUT.parent/'icra_recursion_origin/native_summary.py']+list(OUT.glob('*.py'))+list(OUT.glob('*.m'))
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in protected})
    fields=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources];assert len(fields)==len(set(fields))
    stages={}
    for kind,arm in [('controls',GCE),('event',ARM)]:
        units=[]
        for condition in ['reliable','intermittent']:
            unit=deepcopy(base);unit.update(execution_id=condition,conditions=[condition],arms=[arm])
            unit['references']={c['arm']:dict(path=c['path'],sha256=c['sha256']) for c in trace['cells'] if c['condition']==condition}
            for ref in unit['references'].values():assert sha(ROOT/ref['path'])==ref['sha256']
            units.append(unit)
        stage='nominal_origin_'+kind;cfg=dict(protocol='icra-nominal-origin-intervention-v1',stage=stage,cohort=stage,pd=.9,
            created_utc=datetime.now(timezone.utc).isoformat(),units=units,source_sha256=sources,
            truth_id=5,target_window=[53,122],range_m=2.,active_threshold=.001,
            exposure='One previously exposed causal diagnostic case; no method or timing selection')
        path=OUT/'stages'/(stage+'.json');path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
        stages[str(path.relative_to(ROOT))]=sha(path)
    destination.write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),configurations=stages,
        native_trajectories=4,reused_noage_trajectories=2,source_files=len(sources),
        fixture='checkInitialNegative passed before freeze; every native job reruns the fixture'),indent=2)+'\n')
    print('BOTH NOMINAL STAGES FROZEN',len(sources),'sources; 4 native trajectories',flush=True)

if __name__=='__main__':main()
