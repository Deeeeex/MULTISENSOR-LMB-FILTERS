"""Protect the complete declared case before computing the origin trace."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'TRACE_FREEZE.json';assert not destination.exists()
    parent=OUT.parent/'icra_range_detection'
    cfgpath=parent/'stages/range_detection_preflight.json';cfg=json.loads(cfgpath.read_text())
    auditpath=parent/'audit_range_detection_preflight.json';audit=json.loads(auditpath.read_text());assert audit['passed']
    unit=next(u for u in cfg['units'] if u['sequence']=='v2xt_0001')
    cells=[];protected={}
    for mode,suffix in [('nominal',''),('range','_range')]:
        for condition in cfg['conditions']:
            for backend,base in [('GCE','marked_gaussian_evidence'),('Guarded Scalar','marked_gaussian_evidence_guarded_scalar'),('No-age','marked_lineage')]:
                path=parent/'results/range_detection_preflight'/f"{unit['sequence']}_{condition}_{base+suffix}.json.gz"
                key=str(path.relative_to(ROOT));assert sha(path)==audit['inputs'][key]
                protected[key]=sha(path);cells.append(dict(mode=mode,condition=condition,backend=backend,arm=base+suffix,path=key,sha256=sha(path)))
    files=[cfgpath,auditpath,parent/'FINAL_VERIFICATION.json',OUT/'PROTOCOL.md']+list(OUT.glob('*.py'))
    files+=[ROOT/unit['data_path'],ROOT/unit['pose_path'],
        OUT.parent/'icra_range_detection/diagnose_mechanism.py',
        OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py',
        OUT.parent/'icra_v2x_gce_diagnosis/diagnose_gap.py']
    protected.update({str(p.relative_to(ROOT)):sha(p) for p in files})
    previous=json.loads((parent/'FINAL_VERIFICATION.json').read_text());assert previous['passed']
    for key,digest in previous['input_sha256'].items():assert sha(ROOT/key)==digest,key
    result=dict(protocol='icra-recursion-origin-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        sequence=unit['sequence'],truth_id=5,range_m=2.,probability_tolerance=1e-9,spatial_tolerance=1e-7,
        active_threshold=.001,unit=unit,cells=cells,source_sha256=protected,
        scope='Descriptive only; no modified native trajectory or selection')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('TRACE FROZEN',len(cells),'native source files;',len(protected),'direct dependencies',flush=True)


if __name__=='__main__':main()
