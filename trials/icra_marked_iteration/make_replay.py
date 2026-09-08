"""Generate five fixed fair-control arms from the frozen ECR runner."""
from pathlib import Path
OUT=Path(__file__).resolve().parent
OLD=OUT.parent/'icra_ceiling_iteration'
s=(OLD/'runEvidenceCeilingReplay.m').read_text()
def replace(a,b):
    global s
    assert a in s,a
    s=s.replace(a,b)
replace('function runEvidenceCeilingReplay(', 'function runMarkedEvidenceReplay(')
replace("arms={'qualified_exist','ceiling_association','ceiling_score','ceiling_calibrated'}",
        "arms={'marked_lineage','marked_er','marked_ceiling_association','marked_ceiling_score','marked_ceiling_calibrated'}")
replace('checkEvidenceCeiling();', "priorCeiling=fullfile(root,'trials','icra_ceiling_iteration');addpath(priorCeiling);\ncheckMarkedEvidence();")
replace("marks=load(fullfile(out,'data_marks'", "marks=load(fullfile(priorCeiling,'data_marks'")
replace('    T=double(data.T);if smoke', "    ratios=load(fullfile(out,'data_likelihoods',sprintf('likelihoods_%04d.mat',seq)));\n    T=double(data.T);if smoke")
replace('runDensity(model,measurements,positions,delivered,arms{a},marks)', 'runDensity(model,measurements,positions,delivered,arms{a},marks,ratios)')
replace("'direct-evidence-ceiling-v1','implementation','ecr-v1'", "'marked-evidence-ceiling-v1','implementation','mecr-v1'")
replace('function run=runDensity(model,measurements,positions,delivered,arm,marks)', 'function run=runDensity(model,measurements,positions,delivered,arm,marks,ratios)')
replace('updateLmbWithAssociationWeights(predicted,num2cell(measurements{n,t},1),model,n,t)',
        'updateMarkedLmb(predicted,num2cell(measurements{n,t},1),model,n,t,ratios.likelihoodRatios{n,t})')
replace("strcmp(arm,'ceiling_score')", "strcmp(arm,'marked_ceiling_score')")
replace("strcmp(arm,'ceiling_calibrated')", "strcmp(arm,'marked_ceiling_calibrated')")
replace('fuseEvidenceCeiling(', 'fuseMarkedInputs(')
(OUT/'runMarkedEvidenceReplay.m').write_text(s)
print('Generated five-arm marked local-likelihood comparison.')
