"""Check every declared rule with both density implementations after freeze."""
from pathlib import Path
import gzip
import hashlib
import json
import sys
import numpy as np
from direction_math import RULES,source_state,calculate,check_fixtures
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT))
from independent_math import rebuilt
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    target=OUT/'PREFLIGHT.json';assert not target.exists()
    freeze=OUT/'SCREEN_FREEZE.json';cfg=json.loads(freeze.read_text())
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    assert cfg['rules']==RULES;checks=[]
    for cell in cfg['preflight_cells']:
        path=ROOT/cell['path'];assert sha(path)==cell['sha256']
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        state=source_state(data['runs']);scalar=cell['backend']=='Guarded Scalar'
        rules=[]
        for rule in RULES:
            candidate,values,diag=calculate(state,rule,scalar)
            rec,independent=rebuilt(data['runs'],rule,scalar)
            for key,value in values.items():
                if key in ['allowed','fallback','unchanged','negative_reversal','positive_reversal']:
                    assert np.array_equal(value,independent[key]),(cell,rule,key)
                else:
                    tol=1e-7 if key in ['mean','covariance','kept','multiplier'] else 1e-8 if key in ['log_integral','reference_log_odds','prediction_log_integral','base_direction','original_direction','final_direction'] else 2e-10 if key=='r' else 2e-14
                    assert np.allclose(value,independent[key],atol=tol,rtol=0),(cell,rule,key,float(np.max(abs(value-independent[key]))))
            if rule=='original':assert candidate.tobytes()==rec.tobytes()
            rules.append(dict(rule=rule,**diag))
        checks.append(dict(cell=cell,rules=rules));print('DIRECTION PREFLIGHT',cell['backend'],len(rec),'labels',flush=True)
    target.write_text(json.dumps(dict(passed=True,scope='Frozen two-input density parity; no output-set scores',
        checks=checks,fixtures=check_fixtures(),freeze_sha256=sha(freeze),source_sha256=sha(Path(__file__))),indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
