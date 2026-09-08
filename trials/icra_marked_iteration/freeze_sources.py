from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
base=json.loads((OUT.parent/'icra_ceiling_iteration/source_sha256.json').read_text())
for name,digest in base.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
files=list(OUT.glob('*.m'))+list(OUT.glob('*.py'))+[OUT/'PROTOCOL.md',OUT/'likelihood_manifest.json']+list((OUT/'data_likelihoods').glob('*.mat'))
for p in files:base[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
target=OUT/'source_sha256.json';assert not target.exists()
target.write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('MARKED SOURCES FROZEN',len(base),'files')
