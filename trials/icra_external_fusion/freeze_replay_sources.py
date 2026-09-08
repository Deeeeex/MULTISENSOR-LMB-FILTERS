"""Record tracker source before the complete replay; verify during analysis."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
old=json.loads((ROOT/'trials/icra_reunion_fusion/validation_source_sha256.json').read_text())
for path,digest in old.items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
paths=set(old)
paths.update(str(path.relative_to(ROOT)) for path in OUT.rglob('*.m'))
paths.update(str(path.relative_to(ROOT)) for path in [OUT/'V2V4REAL_PROTOCOL.md',OUT/'prepare_v2v4real.py'])
report={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in sorted(paths)}
(OUT/'replay_source_sha256.json').write_text(json.dumps(report,indent=2)+'\n')
print('Frozen source files:',len(report),'; existing validation hashes verified:',len(old))
