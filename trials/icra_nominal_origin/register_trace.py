"""Bind the exact nominal source traces before locating their first difference."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'TRACE_FREEZE.json';assert not destination.exists()
    parent=OUT.parent/'icra_recursion_origin';prior=json.loads((parent/'TRACE_FREEZE.json').read_text())
    assert json.loads((parent/'FINAL_VERIFICATION.json').read_text())['passed']
    cells=[c for c in prior['cells'] if c['mode']=='nominal'];assert len(cells)==6
    for c in cells:assert sha(ROOT/c['path'])==c['sha256'],c['path']
    protected=[OUT/'PROTOCOL.md',parent/'TRACE_FREEZE.json',parent/'TRACE_VERIFICATION.json',parent/'TARGET_TIMELINE.csv',
        parent/'FINAL_VERIFICATION.json',parent/'trace_v2.py',parent/'verify_trace.py',
        OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py',ROOT/prior['unit']['data_path']]
    protected+=list(OUT.glob('*.py'))
    result=dict(protocol='icra-nominal-origin-trace-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        unit=prior['unit'],cells=cells,truth_id=5,range_m=2.,active_threshold=.001,
        probability_tolerance=1e-9,spatial_tolerance=1e-7,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in protected})
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('NOMINAL ORIGIN TRACE FROZEN',len(cells),'native trajectories',flush=True)

if __name__=='__main__':main()
