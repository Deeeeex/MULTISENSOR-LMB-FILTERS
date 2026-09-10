"""Freeze the descriptive complete nominal roster before constructing ancestry."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
from census import check_fixtures

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'CENSUS_FREEZE.json';assert not destination.exists()
    parent=OUT.parent/'icra_nominal_origin/TRACE_FREEZE.json';old=json.loads(parent.read_text())
    assert len(old['cells'])==6
    protected=old['source_sha256'].copy()
    for p in [parent,OUT/'.gitignore',OUT/'PROTOCOL.md',*OUT.glob('*.py'),
              OUT.parent/'icra_nominal_origin/TRACE_VERIFICATION.json',
              OUT.parent/'icra_range_detection/runRangeDetectionReplay.m',
              ROOT/'lmb/lmbPredictionStep.m',OUT.parent/'icra_recursion_origin/trace_v2.py',
              OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py']:
        protected[str(p.relative_to(ROOT))]=sha(p)
    for name,digest in protected.items():assert sha(ROOT/name)==digest,name
    cfg=dict(protocol='icra-nominal-label-genealogy-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        unit=old['unit'],cells=old['cells'],truth_id=5,range_m=2.,active_threshold=.001,
        fixed_window=[53,122],source_sha256=protected,fixtures=check_fixtures(),
        scope='All nominal labels, both robots and all frames; exposed case, diagnostic birth ancestry only')
    destination.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('CENSUS FROZEN',len(cfg['cells']),'source runs;',len(protected),'protected files',flush=True)


if __name__=='__main__':main()
