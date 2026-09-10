"""Freeze the full cohort and all implementation before real substitutions."""
from datetime import datetime,timezone
from pathlib import Path
import ast
import hashlib
import json
from direction_math import RULES,DIRECTION_TOL,STEPS,check_fixtures

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    target=OUT/'SCREEN_FREEZE.json';assert not target.exists() and not (OUT/'results').exists()
    old=OUT.parent/'icra_peer_detection';cfg=json.loads((old/'SCREEN_FREEZE.json').read_text())
    sources=cfg['source_sha256'].copy();cells=cfg['cells']
    assert len(cells)==56 and len({c['sequence'] for c in cells})==14
    assert not any(c['sequence']=='v2xt_0001' for c in cells)
    for name,h in sources.items():assert sha(ROOT/name)==h,name
    for p in OUT.glob('*.py'):ast.parse(p.read_text())
    protected=list(OUT.glob('*.py'))+list(OUT.glob('*.md'))+list(OUT.glob('*.json'))
    protected += [old/name for name in ['SCREEN_FREEZE.json','SCREEN_RESULTS.json','SCREEN_VERIFICATION.json',
        'FINAL_VERIFICATION.json','RESULTS_CN.md','MOTIVATING_TRACE.json','MOTIVATING_TRACE.csv','NUMERICAL_NOTE.md']]
    prior=OUT.parent/'icra_prune_information'
    stage=json.loads((prior/'stages/prune_info_controls.json').read_text())
    reference_inputs={u['execution_id']:u['original_references']['marked_gaussian_evidence'] for u in stage['units']}
    for info in reference_inputs.values():
        assert sha(ROOT/info['path'])==info['sha256'];sources[info['path']]=info['sha256']
    for path in protected:sources[str(path.relative_to(ROOT))]=sha(path)
    freeze=dict(protocol='icra-direction-constraint-fixed-input-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        primary='nonreversal',rules=RULES,direction_tolerance=DIRECTION_TOL,bisections=STEPS,
        selection_backend='GCE',conditions=['reliable','intermittent'],cells=cells,
        gates=dict(ospa='strictly lower in each dataset/link group',gospa='nonincreasing in each dataset/link group'),
        preflight_cells=cells[:2],reference_inputs=reference_inputs,source_sha256=sources,fixtures=check_fixtures(),
        exposure='All fourteen source segments exposed; diagnostic case excluded; no recursion or wire-cost claim')
    target.write_text(json.dumps(freeze,indent=2,allow_nan=False)+'\n')
    print('DIRECTION CONSTRAINT FROZEN',len(cells),'source runs;',len(sources),'protected files',flush=True)

if __name__=='__main__':main()
