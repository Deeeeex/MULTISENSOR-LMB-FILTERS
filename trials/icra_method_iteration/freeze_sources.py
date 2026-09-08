from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
base=json.loads((out.parent/'icra_external_fusion/replay_source_sha256.json').read_text())
for name,digest in base.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for path in list(out.glob('*.m'))+[out/'PROTOCOL.md',out/'make_replay_runner.py']:
    base[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
(out/'source_sha256.json').write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('Frozen',len(base),'source files; all original frozen sources unchanged.')
