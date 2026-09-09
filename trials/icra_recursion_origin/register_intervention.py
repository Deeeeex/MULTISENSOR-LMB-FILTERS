"""Freeze both native stages before their outcomes; keep references nested for MATLAB."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence_range'
GS='marked_gaussian_evidence_guarded_scalar_range'

def main():
    destination=OUT/'INTERVENTION_FREEZE.json';assert not destination.exists()
    assert not (OUT/'results').exists() and not (OUT/'stages').exists()
    parent=OUT.parent/'icra_range_detection'
    prior=parent/'stages/range_detection_preflight.json';old=json.loads(prior.read_text())
    trace=json.loads((OUT/'TRACE_FREEZE.json').read_text())
    assert json.loads((OUT/'TRACE_VERIFICATION.json').read_text())['passed']
    sources=old['source_sha256'].copy()
    for name,expected in sources.items():assert sha(ROOT/name)==expected,name
    unit=deepcopy(next(u for u in old['units'] if u['sequence']=='v2xt_0001'))
    unit['intervention_frame']=3
    for key in ['reference_paths','reference_sha256','noage_reference_paths','noage_reference_sha256','parity_paths','parity_sha256']:
        unit.pop(key)
    unit['references']={c['arm']:dict(path=c['path'],sha256=c['sha256']) for c in trace['cells'] if c['mode']=='range' and c['condition']=='intermittent'}
    for item in unit['references'].values():assert sha(ROOT/item['path'])==item['sha256']
    protected=[prior,OUT/'INTERVENTION_PROTOCOL.md',OUT/'PROTOCOL.md',OUT/'TRACE_FREEZE.json',
        OUT/'TRACE_VERIFICATION.json',OUT/'ORIGIN_TRACE.json',OUT/'TRACE_SERIALIZATION_REPAIR.json',
        OUT/'TARGET_TIMELINE.csv',OUT/'PAIR_DIFFERENCES.csv',OUT/'RUNNER_PATCH.json',OUT/'AUDITOR_PATCH.json',
        parent/'audit_stage_v2.py',parent/'probability_audit.py',parent/'range_audit.py']
    protected+=list(OUT.glob('*.py'))+list(OUT.glob('*.m'))
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in protected})
    fields=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources]
    assert len(fields)==len(set(fields))
    plans={}
    for kind in ['preflight','interventions']:
        if kind=='preflight':units=[dict(deepcopy(unit),execution_id='controls',arms=[GCE,GS])]
        else:units=[dict(deepcopy(unit),execution_id='once_'+mode,
            arms=['marked_gaussian_evidence_guarded_scalar_once_'+mode+'_range']) for mode in ['joint','existence','spatial']]
        stage='recursion_'+kind
        cfg=dict(protocol='icra-recursion-intervention-v1',stage=stage,cohort=stage,pd=.9,
            created_utc=datetime.now(timezone.utc).isoformat(),conditions=['intermittent'],units=units,
            source_sha256=sources,exposure='Previously exposed causal diagnostic case; no method selection',
            truth_id=5,range_m=2.,active_threshold=.001,target_window=[53,122])
        path=OUT/'stages'/(stage+'.json');path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
        plans[str(path.relative_to(ROOT))]=sha(path)
    destination.write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),
        configurations=plans,native_trajectories=5,fixture='checkRecursionIntervention passed before freeze; rerun required in every native process',
        source_files=len(sources)),indent=2)+'\n')
    print('FROZEN BOTH STAGES',len(sources),'source files; 2 controls and 3 one-time interventions',flush=True)

if __name__=='__main__':main()
