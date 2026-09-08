from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
base=json.loads((OUT/'source_sha256.json').read_text())
for name,h in base.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
for name in ['STABLE_ADAPTER_AMENDMENT.md','make_stable_adapter.py','updateMarkedLmbStable.m',
             'runMarkedEvidenceReplayStable.m','checkStableMarkedEvidence.m','freeze_stable_adapter.py','fuseMarkedInputsStable.m']:
    p=OUT/name;base[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
target=OUT/'source_sha256_stable.json';assert not target.exists()
target.write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('STABLE ADAPTER SOURCE FREEZE',len(base),'files; original marked snapshot unchanged.')
