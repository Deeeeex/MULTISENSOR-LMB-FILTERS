"""Trial-local mixture packet and truth-free cached-case runner."""
from pathlib import Path
import hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
old=OUT.parent/'icra_method_iteration'
s=(old/'runConservativeCases.m').read_text()
def replace(a,b):
    global s
    assert a in s,a
    s=s.replace(a,b)
replace('function runConservativeCases(', 'function runEvidenceCeilingCases(')
replace("fullfile(root,'trials','icra_group_tracking'));", "fullfile(root,'trials','icra_group_tracking'));\npriorIteration=fullfile(root,'trials','icra_method_iteration');addpath(priorIteration);\nexternal=fullfile(root,'trials','icra_external_fusion');addpath(external);\nruntime=fullfile(priorIteration,'runtime');addpath(runtime);\ncleanup=onCleanup(@()rmpath(runtime)); %#ok<NASGU>\ncheckEvidenceCeiling();")
replace("'source_sha256_cases.json'", "'source_sha256_cases.json'")
replace("        fprintf('START CR case", "        for j=1:numel(data.model.birthParameters),data.model.birthParameters(j).directEvidenceCeiling=0;end\n        data.model.object=data.model.birthParameters([]);\n        fprintf('START ECR case")
replace('runArm(data.model,data.truth,data.measurements', 'runArm(data.model,data.measurements')
replace("'conservative_recency',data.scene.T);", "'ceiling_association',data.scene.T);\n        run=scoreSavedOutputs(run,data.truth);")
replace("'conservative-recency-case-studies-v1'", "'evidence-ceiling-case-studies-v1'")
replace("'_cr.json'", "'_ecr.json'")
replace("'DONE CR case", "'DONE ECR case")
replace("'COMPLETED CR cases", "'COMPLETED ECR cases")
replace('function run=runArm(model,truth,measurements', 'function run=runArm(model,measurements')
replace('        if isempty(predicted), local{n}=predicted;\n        else, local{n}=updateLmbWithSensorMeasurement(predicted,measurements{n,t},model,n,t); end',
'''        association=[];
        if isempty(predicted), local{n}=predicted;
        else, [local{n},~,association]=updateLmbWithAssociationWeights(predicted,measurements{n,t},model,n,t); end
        support=directEvidenceCeilings(association,ones(1,numel(measurements{n,t})),[local{n}.lastDirectOpportunity]==t);
        for j=1:numel(local{n}),local{n}(j).directEvidenceCeiling=support(j);end''')
replace('        [run.preOspa(n,t),run.preCountError(n,t)]=score(truth{t},states);', '')
replace('fuseConservativeRecency(', 'fuseEvidenceCeiling(')
replace('        [run.ospa(n,t),run.countError(n,t),run.matchedSquaredError(n,t),run.matchedCount(n,t)]=score(truth{t},states);', '')
replace('double(o.hasObservationLineage),o.lastDirectOpportunity,double(o.positiveConfirmation)]',
        'double(o.hasObservationLineage),o.lastDirectOpportunity,double(o.positiveConfirmation),o.directEvidenceCeiling]')
replace('o.lastDirectOpportunity=v(cursor+5); o.positiveConfirmation=logical(v(cursor+6)); cursor=cursor+7;',
        'o.lastDirectOpportunity=v(cursor+5); o.positiveConfirmation=logical(v(cursor+6)); o.directEvidenceCeiling=v(cursor+7); cursor=cursor+8;')
replace('    assert(a(k).positiveConfirmation==b(k).positiveConfirmation);',
        '    assert(a(k).positiveConfirmation==b(k).positiveConfirmation);\n    assert(a(k).directEvidenceCeiling==b(k).directEvidenceCeiling);')
s+='''
function run=scoreSavedOutputs(run,truth)
[N,T]=size(run.estimates);
for t=1:T
    for n=1:N
        [run.preOspa(n,t),run.preCountError(n,t)]=score(truth{t},run.preEstimates{n,t});
        [run.ospa(n,t),run.countError(n,t),run.matchedSquaredError(n,t),run.matchedCount(n,t)]=score(truth{t},run.estimates{n,t});
    end
end
end
'''
(OUT/'runEvidenceCeilingCases.m').write_text(s)
base=json.loads((OUT/'source_sha256.json').read_text())
for p,h in base.items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h,p
for p in [OUT/'CASES_PROTOCOL.md',OUT/'make_cases.py',OUT/'runEvidenceCeilingCases.m']:
    base[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
target=OUT/'source_sha256_cases.json';assert not target.exists()
target.write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('ECR CASES FROZEN',len(base),'files; no case outputs used.')
