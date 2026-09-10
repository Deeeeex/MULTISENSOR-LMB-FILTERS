"""Check fixtures and two unchanged source distributions before registration."""
from pathlib import Path
import ast
import gzip
import hashlib
import json
import sys
import numpy as np
from peer_math import source_state,calculate,check_fixtures
sys.path.insert(0,str(Path(__file__).resolve().parent))
from verify_screen import rebuilt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'PREFLIGHT.json';assert not destination.exists() and not (OUT/'SCREEN_FREEZE.json').exists()
    parsed=[]
    for path in OUT.glob('*.py'):ast.parse(path.read_text());parsed.append(path.name)
    old=json.loads((OUT.parent/'icra_joint_admission/SCREEN_FREEZE.json').read_text());checks=[]
    for cell in old['cells'][:2]:
        path=ROOT/cell['path'];assert sha(path)==cell['sha256']
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        state=source_state(data['runs']);candidate,values,_=calculate(state,'original',cell['backend']=='Guarded Scalar')
        assert candidate.tobytes()==state['records'].tobytes()
        rec,independent=rebuilt(data['runs'],'original',cell['backend']=='Guarded Scalar')
        assert rec.tobytes()==candidate.tobytes()
        for key in ['r','mean','covariance','kept','allowed','fallback','unchanged']:
            assert np.array_equal(values[key],independent[key]),key
        checks.append(dict(cell=cell,fusion_labels=len(rec),unchanged_distribution_exact=True))
        print('ORIGINAL PREFLIGHT',cell['sequence'],cell['backend'],len(rec),flush=True)
    result=dict(passed=True,fixtures=check_fixtures(),checks=checks,parsed_python=parsed,
        preselection='Only original rule evaluated, no alternate scores',
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in OUT.glob('*.py')})
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
