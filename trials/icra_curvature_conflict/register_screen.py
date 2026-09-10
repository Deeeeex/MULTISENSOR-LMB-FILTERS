"""Bind the complete old roster and this sole primary before alternate scores."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
from conflict_math import RULES,check_fixtures

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'SCREEN_FREEZE.json'
    assert not destination.exists() and not (OUT/'SCREEN_RESULTS.json').exists()
    parent=OUT.parent/'icra_joint_admission'
    prior=json.loads((parent/'SCREEN_FREEZE.json').read_text())
    verification=json.loads((parent/'FINAL_VERIFICATION.json').read_text())
    assert verification['passed'] and len(prior['cells'])==56
    sources=verification['source_sha256'].copy()
    for p in [parent/'FINAL_VERIFICATION.json',OUT/'PROTOCOL.md',OUT/'SCREEN_PORT.json',*OUT.glob('*.py')]:
        sources[str(p.relative_to(ROOT))]=sha(p)
    for name,digest in sources.items():assert sha(ROOT/name)==digest,name
    for name,port in json.loads((OUT/'SCREEN_PORT.json').read_text()).items():
        assert sha(ROOT/port['source'])==port['source_sha256']
        old=(ROOT/port['source']).read_text()
        for patch in port['replacements']:
            assert old.count(patch['old'])==1
            old=old.replace(patch['old'],patch['new'])
        assert old==(OUT/name).read_text() and sha(OUT/name)==port['output_sha256']
    cfg=dict(protocol='icra-curvature-conflict-fixed-input-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        primary='paired_veto',rules=RULES,selection_backend='GCE',references=['original'],
        conditions=['reliable','intermittent'],cells=prior['cells'],source_sha256=sources,
        fixtures=check_fixtures(),exposure='All inputs already exposed; fixed-input substitution only',
        advancement='Strictly lower primary OSPA on all four dataset-by-radio GCE-state cells')
    destination.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('FROZEN',len(cfg['cells']),'source runs;',len(sources),'protected files',flush=True)


if __name__=='__main__':main()
