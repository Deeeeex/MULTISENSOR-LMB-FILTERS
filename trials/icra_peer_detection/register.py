"""Freeze the unchanged cohort and one new primary before alternate scores."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import sys

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT))
from peer_math import RULES,check_fixtures
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    target=OUT/'SCREEN_FREEZE.json';assert not target.exists() and not (OUT/'results').exists()
    oldpath=OUT.parent/'icra_joint_admission/SCREEN_FREEZE.json';old=json.loads(oldpath.read_text())
    cells=old['cells'];assert len(cells)==56 and not any(c['sequence']=='v2xt_0001' for c in cells)
    assert len({c['sequence'] for c in cells})==14
    sources=old['source_sha256'].copy()
    for name,h in sources.items():assert sha(ROOT/name)==h,name
    prior=OUT.parent/'icra_prune_information'
    prior_manifest=json.loads((prior/'FINAL_VERIFICATION.json').read_text())
    prior_cfg=json.loads((prior/'stages/prune_info_controls.json').read_text())
    assert sha(prior/'stages/prune_info_controls.json')==prior_manifest['input_sha256']['trials/icra_prune_information/stages/prune_info_controls.json']
    motivation={}
    for unit in prior_cfg['units']:
        condition=unit['execution_id']
        for mode in ['original','local','shared']:
            for base in ['marked_gaussian_evidence','marked_lineage']:
                arm=base+('_known_censor' if mode=='local' else '_prune_info' if mode=='shared' else '')
                path=ROOT/unit['original_references'][base]['path'] if mode=='original' else prior/'results'/('prune_info_controls' if mode=='local' else 'prune_info_shared')/f"{unit['sequence']}_{condition}_{arm}.json.gz"
                name=str(path.relative_to(ROOT));digest=sha(path)
                expected=unit['original_references'][base]['sha256'] if mode=='original' else prior_manifest['input_sha256'][name]
                assert digest==expected,name
                motivation[name]=digest
    assert len(motivation)==12
    sources.update(motivation)
    protected=[oldpath]+[OUT.parent/p for p in [
        'icra_joint_admission/joint_math.py','icra_joint_admission/FINAL_VERIFICATION.json',
        'icra_joint_admission/WEIGHT_ARITHMETIC_VERIFICATION.json','icra_joint_admission/RESULTS_CN.md',
        'icra_compatible_admission/PROTOCOL.md','icra_admission_final/RESULTS_CN.md',
        'icra_curvature_conflict/FINAL_VERIFICATION.json','icra_curvature_conflict/RESULTS_CN.md',
        'icra_nominal_origin/FINAL_VERIFICATION.json','icra_nominal_origin/RESULTS_CN.md',
        'icra_prune_information/FINAL_VERIFICATION.json','icra_prune_information/RESULTS_CN.md',
        'icra_prune_information/RESULTS.json','icra_prune_information/stages/prune_info_controls.json']]
    protected+=list(OUT.glob('*.py'))+list(OUT.glob('*.md'))+list(OUT.glob('*.json'))
    for p in protected:sources[str(p.relative_to(ROOT))]=sha(p)
    cfg=dict(protocol='icra-peer-detection-fixed-input-v1',created_utc=datetime.now(timezone.utc).isoformat(),rules=RULES,
        primary='peer_joint',selection_backend='GCE',conditions=['reliable','intermittent'],
        gates=dict(ospa='strictly lower in every dataset/link group',gospa='nonincreasing in every dataset/link group'),
        cells=cells,source_sha256=sources,motivation_inputs=motivation,fixtures=check_fixtures(),
        exposure='All source inputs previously exposed; v2xt_0001 excluded from selection; no recursion or physical byte-cost claim')
    target.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('PEER DETECTION FROZEN',len(cells),'source runs;',len(sources),'protected files',flush=True)

if __name__=='__main__':main()
