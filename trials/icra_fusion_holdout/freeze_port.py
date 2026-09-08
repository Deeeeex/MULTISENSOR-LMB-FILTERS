from pathlib import Path
import hashlib,json,re
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
base=json.loads((OUT.parent/'icra_marked_iteration/source_sha256_stable.json').read_text())
for p,h in base.items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h,p
paths=list(OUT.glob('*.m'))+list(OUT.glob('*.py'))+list(OUT.glob('*PROTOCOL.md'))
paths+=[OUT/'input_manifest.json',OUT/'transform_manifest.json']+list((OUT/'data').glob('*.mat'))
for p in paths:base[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
assert len({re.sub('[^A-Za-z0-9_]','_',p)[:63] for p in base})==len(base)
target=OUT/'source_sha256_port.json';assert not target.exists()
target.write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('PORT SOURCE FREEZE',len(base),'files; no holdout tracking authorization file created.')
