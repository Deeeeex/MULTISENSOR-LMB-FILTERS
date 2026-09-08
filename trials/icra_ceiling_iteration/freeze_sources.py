"""Freeze after analytic preflight, before any ECR tracking outcomes."""
from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
base=json.loads((OUT.parent/'icra_evidence_iteration/source_sha256.json').read_text())
for name,digest in base.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
files=list(OUT.glob('*.m'))+list(OUT.glob('*.py'))+[OUT/'PROTOCOL.md',OUT/'calibration.json']+list((OUT/'data_marks').glob('*.mat'))
files+=[OUT.parent/'icra_evidence_iteration/summary_development.json',OUT.parent/'icra_evidence_iteration/detection_score_diagnosis.csv']
for p in files:base[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
target=OUT/'source_sha256.json';assert not target.exists(),'An existing source freeze is immutable.'
target.write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('ECR SOURCE FREEZE',len(base),'files')
