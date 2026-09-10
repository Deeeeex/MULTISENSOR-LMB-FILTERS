"""Freeze both stages and the fair advancement criteria before native outcomes."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_known_censor'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
ARMS=['marked_gaussian_evidence','marked_gaussian_evidence_guarded_scalar','marked_lineage']

def main():
    assert not (OUT/'FREEZE.json').exists() and not (OUT/'results').exists() and not (OUT/'stages').exists()
    oldpath=OLD/'stages/known_censor_refined.json';old=json.loads(oldpath.read_text())
    auditpath=OLD/'audit_v2_known_censor_refined.json';audit=json.loads(auditpath.read_text());assert audit['passed']
    final=OLD/'FINAL_VERIFICATION.json';assert json.loads(final.read_text())['passed']
    sources=old['source_sha256'].copy()
    extra=json.loads((OLD/'AUDIT_FREEZE_V2.json').read_text())
    for name,h in extra['source_sha256'].items():assert sha(ROOT/name)==h,name
    # Long archived result names collide in MATLAB's 63-character field map.
    # Their complete hashes stay in the protected V2 manifest and references;
    # executable/audit sources retain the original native source-field format.
    sources.update({name:h for name,h in extra['source_sha256'].items() if '/results/' not in name})
    for name,h in sources.items():assert sha(ROOT/name)==h,name
    fixture=json.loads((OUT/'FIXTURE.json').read_text());assert fixture['passed'] and fixture['returncode']==0
    for name,h in fixture['source_sha256'].items():assert sha(ROOT/name)==h,name
    assert sha(ROOT/fixture['log'])==fixture['log_sha256']
    protected=[oldpath,auditpath,final,OLD/'RESULTS.json',OLD/'reentry/RESULTS.json',OLD/'reentry/FINAL_VERIFICATION.json',
        OLD/'DELIVERY_VERIFICATION.json',OLD/'AUDIT_FREEZE_V2.json',OLD/'finish_v2.py',OLD/'population_audit.py',OLD/'event_audit.py',ROOT/fixture['log']]
    protected+=list(OUT.glob('*.py'))+list(OUT.glob('*.m'))+list(OUT.glob('*.md'))+list(OUT.glob('*.json'))
    for p in protected:sources[str(p.relative_to(ROOT))]=sha(p)
    mapped=[re.sub('[^A-Za-z0-9_]','_',p)[:63] for p in sources];assert len(mapped)==len(set(mapped))
    stages={}
    for kind in ['controls','shared']:
        stage='prune_info_'+kind;units=deepcopy(old['units'])
        for u in units:
            u['original_references']=u['references'];u['references']={}
            for a in ARMS:
                path=OLD/'results/known_censor_refined'/f"{u['sequence']}_{u['execution_id']}_{a}_known_censor.json.gz"
                assert sha(path)==audit['inputs'][str(path.relative_to(ROOT))]
                u['references'][a+'_known_censor']=dict(path=str(path.relative_to(ROOT)),sha256=sha(path))
            u['arms']=[a+('_known_censor' if kind=='controls' else '_prune_info') for a in ARMS]
        cfg=dict(protocol='icra-prune-information-v1',stage=stage,cohort=stage,pd=.9,units=units,
            created_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sources,truth_id=5,target_window=[53,122],range_m=2.,
            active_threshold=.001,expansion_gate=dict(both_conditions=True,minimum_ospa_improvement=.01,minimum_gospa_improvement=.01,
                beat_shared_noage_ospa=True,beat_shared_noage_gospa=True,strict_window_detection_gain=True,
                nonworsening_full_detection=True,strict_window_recurrence_reduction=True,maximum_wire_ratio=1.05),
            exposure='Previously exposed complete causal case; no independent validation')
        path=OUT/'stages'/(stage+'.json');path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
        stages[str(path.relative_to(ROOT))]=sha(path)
    (OUT/'FREEZE.json').write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),
        configurations=stages,native_trajectories=12,prior_original_references=6,source_files=len(sources),fixture_sha256=sha(OUT/'FIXTURE.json')),indent=2)+'\n')
    print('PRUNE INFORMATION FROZEN',len(sources),'sources; 12 native trajectories',flush=True)

if __name__=='__main__':main()
