"""Freeze all six original and all six refined trajectories together."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
ARMS=['marked_gaussian_evidence','marked_gaussian_evidence_guarded_scalar','marked_lineage']

def main():
    destination=OUT/'FREEZE.json';assert not destination.exists() and not (OUT/'results').exists() and not (OUT/'stages').exists()
    prior=OUT.parent/'icra_range_detection/stages/range_detection_preflight.json';old=json.loads(prior.read_text())
    tracepath=OUT.parent/'icra_nominal_origin/TRACE_FREEZE.json';trace=json.loads(tracepath.read_text())
    fixture=json.loads((OUT/'FIXTURE.json').read_text());assert fixture['passed'] and fixture['returncode']==0
    for name,h in fixture['source_sha256'].items():assert sha(ROOT/name)==h,name
    assert sha(ROOT/fixture['log'])==fixture['log_sha256']
    sources=old['source_sha256'].copy()
    for name,h in sources.items():assert sha(ROOT/name)==h,name
    protected=[prior,tracepath,ROOT/fixture['log'],OUT.parent/'icra_label_genealogy/REIMPORT_RESULTS.json',
        OUT.parent/'icra_label_genealogy/REIMPORT_VERIFICATION.json',OUT.parent/'icra_label_genealogy/FINAL_VERIFICATION.json',
        OUT.parent/'icra_recursion_origin/native_summary.py',OUT.parent/'icra_recursion_origin/trace_v2.py']
    protected+=list(OUT.glob('*.py'))+list(OUT.glob('*.m'))+list(OUT.glob('*.md'))+list(OUT.glob('*.json'))
    for path in protected:sources[str(path.relative_to(ROOT))]=sha(path)
    fields=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources];assert len(fields)==len(set(fields))
    base=deepcopy(trace['unit'])
    for key in ['reference_paths','reference_sha256','noage_reference_paths','noage_reference_sha256','parity_paths','parity_sha256']:base.pop(key)
    stages={}
    for kind in ['controls','refined']:
        units=[]
        for condition in ['reliable','intermittent']:
            u=deepcopy(base);u.update(execution_id=condition,conditions=[condition],arms=[a+('_known_censor' if kind=='refined' else '') for a in ARMS])
            u['references']={c['arm']:dict(path=c['path'],sha256=c['sha256']) for c in trace['cells'] if c['condition']==condition}
            assert set(u['references'])==set(ARMS)
            for ref in u['references'].values():assert sha(ROOT/ref['path'])==ref['sha256']
            units.append(u)
        stage='known_censor_'+kind;cfg=dict(protocol='icra-known-censor-v1',stage=stage,cohort=stage,pd=.9,units=units,
            created_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sources,
            truth_id=5,target_window=[53,122],range_m=2.,active_threshold=.001,
            expansion_gate=dict(both_conditions=True,strict_improvement=['ospa','gospa','window_detections','window_target_reimports'],
                                nonworsening=['full_target_detections','wire_bytes']),exposure='Single already exposed causal diagnostic case')
        path=OUT/'stages'/(stage+'.json');path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
        stages[str(path.relative_to(ROOT))]=sha(path)
    destination.write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),configurations=stages,
        native_trajectories=12,source_files=len(sources),fixture_sha256=sha(OUT/'FIXTURE.json')),indent=2)+'\n')
    print('KNOWN CENSOR STAGES FROZEN',len(sources),'sources; 12 native trajectories',flush=True)

if __name__=='__main__':main()
