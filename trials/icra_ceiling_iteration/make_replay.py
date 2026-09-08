"""Generate the trial-local adapter and runner before source freeze."""
from pathlib import Path

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_method_iteration'
s=(ROOT/'multisensorLmb/updateLmbWithSensorMeasurement.m').read_text()
s=s.replace('[updatedObjects, diagnostics] = updateLmbWithSensorMeasurement(',
            '[updatedObjects, diagnostics, W] = updateLmbWithAssociationWeights(')
s=s.replace('diagnostics = defaultDiagnostics();', 'W = [];\ndiagnostics = defaultDiagnostics();',1)
(OUT/'updateLmbWithAssociationWeights.m').write_text(s)
s=(OLD/'runMethodReplayFast.m').read_text()
def replace(a,b):
    global s
    assert a in s,a
    s=s.replace(a,b)
replace('function runMethodReplayFast(', 'function runEvidenceCeilingReplay(')
replace("arms={'qualified_exist','innovation_recency'}", "arms={'qualified_exist','ceiling_association','ceiling_score','ceiling_calibrated'}")
replace("original=fullfile(root,'trials','icra_external_fusion');", "original=fullfile(root,'trials','icra_external_fusion');\npriorIteration=fullfile(root,'trials','icra_method_iteration');addpath(priorIteration);\ncheckEvidenceCeiling();")
replace("runtimePath=fullfile(out,'runtime')", "runtimePath=fullfile(priorIteration,'runtime')")
replace("'results_v2v_fast'", "'results_development'")
replace("'source_sha256_ir_fast.json'", "'source_sha256.json'")
replace("T=double(data.T);if smoke", "marks=load(fullfile(out,'data_marks',sprintf('marks_%04d.mat',seq)));\n    T=double(data.T);if smoke")
replace('runDensity(model,measurements,positions,delivered,arms{a})', 'runDensity(model,measurements,positions,delivered,arms{a},marks)')
replace("'current-innovation-recency-v1','implementation','ir-v1-accelerated'", "'direct-evidence-ceiling-v1','implementation','ecr-v1'")
replace('template.localLogOddsIncrement=0', 'template.directEvidenceCeiling=0')
replace('function run=runDensity(model,measurements,positions,delivered,arm)', 'function run=runDensity(model,measurements,positions,delivered,arm,marks)')
a=s.index('        if isempty(predicted),local{n}=predicted;')
b=s.index('        local{n}=reduce(local{n},model);',a)
s=s[:a]+'''        W=[];
        if isempty(predicted),local{n}=predicted;
        else,[local{n},~,W]=updateLmbWithAssociationWeights(predicted,num2cell(measurements{n,t},1),model,n,t);end
        mark=ones(1,size(measurements{n,t},2));
        if strcmp(arm,'ceiling_score'),mark=marks.rawScores{n,t};end
        if strcmp(arm,'ceiling_calibrated'),mark=marks.calibratedScores{n,t};end
        current=[local{n}.lastDirectOpportunity]==t;
        ceiling=directEvidenceCeilings(W,mark,current);
        for j=1:numel(local{n})
            assert(strcmp(key(local{n}(j)),key(predicted(j))));
            local{n}(j).directEvidenceCeiling=ceiling(j);
            if ~isempty(W),assert(ceiling(j)<=local{n}(j).detectionAssociationMass+1e-12);end
        end
''' +s[b:]
replace('innovationLmbPacket(', 'ceilingLmbPacket(')
replace('fuseInnovationRecency(', 'fuseEvidenceCeiling(')
(OUT/'runEvidenceCeilingReplay.m').write_text(s)
print('Generated unchanged local-update adapter and ECR development runner.')
