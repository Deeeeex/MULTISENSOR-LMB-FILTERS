"""Generate a third fixed candidate; preserve all preceding frozen methods."""
from pathlib import Path
import hashlib,json
out=Path(__file__).resolve().parent;root=out.parents[1]
s=(out/'fuseConservativeRecency.m').read_text().replace('fuseConservativeRecency','fuseConfirmedRecency')
s=s.replace('conservative_recency','confirmation_recency')
s=s.replace('% ER objective subject to r <= the same-input no-age result.',
            '% Admit an upward age correction only with current local confirmation.')
s=s.replace('stamps=zeros(size(weights));confirmed=false;',
            'stamps=zeros(size(weights));confirmed=false;currentConfirmation=false(size(weights));')
s=s.replace('o=pieces{s};stamps(s)', 'o=pieces{s};currentConfirmation(s)=o.positiveConfirmation;stamps(s)')
s=s.replace('rEr=one.r;one.r=min(r0,rEr);one.positiveConfirmation=confirmed;\n    assert(one.r<=r0 && one.r<=rEr && isfinite(one.r));',
'''rEr=one.r;
    gate=any(active & q>b+1e-12 & rec.inputExistence>=.5 & currentConfirmation);
    if gate,one.r=rEr;else,one.r=min(r0,rEr);end
    one.positiveConfirmation=confirmed;
    assert(one.r<=rEr && isfinite(one.r));
    if ~gate,assert(one.r<=r0);end''')
s=s.replace('% Columns 10 and 25 are CR r and its nonpositive correction; the\n        % other two scalar slots are zeros (CR uses no increment metadata).',
            '% Columns 20--21 contain current source confirmation; column 25\n        % contains the admitted signed age correction.')
s=s.replace('shift=sum((q-b).*logits);',
            'shift=sum((q-b).*logits);admitted=min(shift,0);if gate,admitted=shift;end')
s=s.replace('rec.inputExistence,0,0,stamps,shift,min(shift,0),max(shift,0)',
            'rec.inputExistence,double(currentConfirmation),stamps,shift,admitted,shift-admitted')
(out/'fuseConfirmedRecency.m').write_text(s)
s=(out/'runConservativeReplayFast.m').read_text().replace('runConservativeReplayFast','runConfirmedReplayFast')
s=s.replace('conservative_recency','confirmation_recency').replace('fuseConservativeRecency','fuseConfirmedRecency')
s=s.replace("'results_v2v_conservative_fast'","'results_v2v_confirmed_fast'")
s=s.replace("'source_sha256_cr_fast.json'","'source_sha256_cgr_fast.json'")
s=s.replace("'conservative-existence-recency-v1'","'confirmation-gated-recency-v1'").replace("'cr-v1-accelerated'","'cgr-v1-accelerated'")
needle="direct={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};"
assert needle in s
s=s.replace(needle,needle+"\nhits={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};\nlastUpdate={containers.Map('KeyType','char','ValueType','double'),containers.Map('KeyType','char','ValueType','double')};")
s=s.replace('        local{n}=reduce(local{n},model);', '''        for j=1:numel(local{n})
            id=key(local{n}(j));mass=0;
            if ~isempty(measurements{n,t}) && stamp(direct{n},id)==t
                mass=local{n}(j).detectionAssociationMass;
            end
            [count,confirmed]=updateLocalConfirmation(stamp(hits{n},id),stamp(lastUpdate{n},id), ...
                t,mass,stamp(direct{n},id)==t,~isempty(measurements{n,t}));
            hits{n}(id)=count;lastUpdate{n}(id)=t;
            local{n}(j).positiveConfirmation=confirmed;
        end
        local{n}=reduce(local{n},model);''')
s=s.replace('posterior{n}(j).lastDirectOpportunity=stamp(direct{n},key(posterior{n}(j)));',
            'posterior{n}(j).lastDirectOpportunity=stamp(direct{n},key(posterior{n}(j)));\n            posterior{n}(j).positiveConfirmation=stamp(lastUpdate{n},key(posterior{n}(j)))==t && stamp(hits{n},key(posterior{n}(j)))>=2;')
(out/'runConfirmedReplayFast.m').write_text(s)
s=(out/'runConservativeCases.m').read_text().replace('runConservativeCases','runConfirmedCases')
s=s.replace('conservative_recency','confirmation_recency').replace('fuseConservativeRecency','fuseConfirmedRecency')
s=s.replace("'results_cases'","'results_cases_confirmed'").replace("'_cr.json'","'_cgr.json'")
s=s.replace("'source_sha256_cases.json'","'source_sha256_cgr_cases.json'")
s=s.replace("'conservative-recency-case-studies-v1'","'confirmation-recency-case-studies-v1'")
s=s.replace('local{n}(j).positiveConfirmation=local{n}(j).positiveConfirmation || localHits(n,region)>=2;',
            'local{n}(j).positiveConfirmation=localHits(n,region)>=2;')
s=s.replace('posterior{n}(k).lastDirectOpportunity=directTimes(n,j);',
            'posterior{n}(k).lastDirectOpportunity=directTimes(n,j);\n            posterior{n}(k).positiveConfirmation=localHits(n,j)>=2;')
(out/'runConfirmedCases.m').write_text(s)
for variant,base_name,target in [('fast','source_sha256_cr_fast.json','runConfirmedReplayFast.m'),('cases','source_sha256_cases.json','runConfirmedCases.m')]:
    manifest=json.loads((out/base_name).read_text())
    for p,h in manifest.items():assert hashlib.sha256((root/p).read_bytes()).hexdigest()==h,p
    for name in ['AMENDMENT_CGR.md','make_confirmed_runner.py','fuseConfirmedRecency.m','updateLocalConfirmation.m','checkConfirmedRecency.m',target]:
        p=out/name;manifest[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/f'source_sha256_cgr_{variant}.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
print('CGR generated and frozen; all earlier source hashes verified.')
