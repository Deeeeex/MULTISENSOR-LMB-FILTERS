"""Generate CR replay without editing the frozen IR runner or source manifest."""
from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
s=(out/'runMethodReplay.m').read_text()
s=s.replace('runMethodReplay','runConservativeReplay')
s=s.replace("arms={'qualified_exist','innovation_recency'}","arms={'conservative_recency'}")
s=s.replace("'results_v2v'","'results_v2v_conservative'")
s=s.replace("'source_sha256.json'","'source_sha256_cr.json'")
s=s.replace("'current-innovation-recency-v1'","'conservative-existence-recency-v1'")
s=s.replace("'ir-v1'","'cr-v1'")
s=s.replace('fuseInnovationRecency(inputs','fuseConservativeRecency(inputs')
s=s.replace('innovationLmbPacket(local{n}','gaussianLmbPacket(local{n}')
# No local log-odds increment is needed, evaluated or transmitted by CR.
begin=s.index('        for j=1:numel(local{n})\n            assert(strcmp(key(local{n}(j)),key(predicted(j))));')
end=s.index('        local{n}=reduce(local{n},model);',begin)
s=s[:begin]+s[end:]
(out/'runConservativeReplay.m').write_text(s)
base=json.loads((out/'source_sha256.json').read_text())
for name,digest in base.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for name in ['AMENDMENT_CR.md','fuseConservativeRecency.m','runConservativeReplay.m','make_conservative_runner.py','checkConservativeRecency.m']:
    p=out/name;base[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'source_sha256_cr.json').write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('CR source frozen; original ER and IR sources unchanged.',len(base),'files')
