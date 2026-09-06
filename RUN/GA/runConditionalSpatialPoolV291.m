function [reportPath,result]=runConditionalSpatialPoolV291(options)
% One conditional-pooling control. No new routing, local FoV split or sweep.
s=load(options.baselineTracePath,'trace'); baseline=s.trace;
s=load(options.milSummaryPath,'result'); cache=s.result;
T=options.maximumTime; out=options.outputRoot;
assert(ismember(T,[2,40]) && T<=baseline.maximumTime && cache.maximumTime==T);
assert(cache.seed==baseline.seed && strcmp(cache.presetName,baseline.presetName));
assert(strcmp(baseline.presetName,'x36-formation-fov-temporal-coupled-formation-braid') ...
    && baseline.seed==1301);
if exist(out,'dir')~=7, mkdir(out); end
rawPath=fullfile(out,'CONDITIONAL_SPATIAL_POOL_V291_RAW.mat');
if exist(rawPath,'file')==2
    s=load(rawPath,'run'); run=s.run;
    assert(strcmp(run.contractVersion,'conditional-spatial-pool-v291-v1') && ...
        run.maximumTime==T && strcmp(run.baselineTracePath,options.baselineTracePath));
else
    inputs=generateDynamicTopologyScenarioInputs(baseline.presetName,baseline.seed);
    assert(inputs.config.simulationLength==baseline.originalTimeCount);
    inputs=cropInputs(inputs,T);
    model=removeRealizedTargetTruthFromDynamicTopologyModel(inputs.model);
    config=buildCausalMinimumFormationBackboneV242Config(inputs.config);
    config.lmbFusionRule='arithmetic-existence-geometric-spatial';
    config.missingLabelFusionMode='strict-common-label';
    config.diagnosticsProgressInterval=5;
    assert(strcmp(config.fusionWeightMode,'metropolis') && ...
        config.mixtureAwareMaxFusedComponents==8 && ~config.receiverSafeLabelFusionEnabled);
    context=buildCausalMinimumFormationBackboneV242ExecutionContext(baseline.presetName,baseline.seed,T);
    protocol=getCausalMinimumFormationBackboneV242Protocol();
    [~,commit]=system('git rev-parse HEAD');
    rng(baseline.seed+protocol.filterSeedOffset,'twister');
    fprintf('V291 conditional geometric pool: X36 seed %d, prefix 1:%d of %d.\n', ...
        baseline.seed,T,baseline.originalTimeCount);
    started=tic;
    [estimates,d]=runEventTriggeredDistributedLmbFilter(model,inputs.measurements, ...
        inputs.sensorTrajectories,inputs.neighborMap,inputs.commConfig,config,context);
    run=struct('contractVersion','conditional-spatial-pool-v291-v1', ...
        'generationGitCommit',strtrim(commit),'baselineTracePath',options.baselineTracePath, ...
        'milSummaryPath',options.milSummaryPath,'presetName',baseline.presetName, ...
        'seed',baseline.seed,'maximumTime',T,'originalTimeCount',baseline.originalTimeCount, ...
        'elapsedSeconds',toc(started),'completedAt',datestr(now,31), ...
        'fusionRule',config.lmbFusionRule,'maximumFusedComponents',8, ...
        'config',inputs.config,'groundTruthRfs',inputs.groundTruthRfs,'estimates',{estimates}, ...
        'attempted',d.attempted,'delivered',d.delivered, ...
        'attemptedPayloadBytes',d.attemptedPayloadBytes,'developmentEvidenceOnly',true);
    save('-mat7-binary',[rawPath,'.partial'],'run'); movefile([rawPath,'.partial'],rawPath);
    fprintf('V291 raw output saved after %.1f s: %s\n',run.elapsedSeconds,rawPath);
end
candidate=summarizeLmbTrackingPrefix(run); reference=cache.reference; mil=cache.candidate;
common=isfinite(reference.rmseBySensorTime)&isfinite(candidate.rmseBySensorTime);
commonMil=isfinite(mil.rmseBySensorTime)&isfinite(candidate.rmseBySensorTime);
formationChange=100*(candidate.perFormationEospa./reference.perFormationEospa-1);
result=struct('contractVersion','conditional-spatial-pool-v291-analysis-v1', ...
    'generationGitCommit',run.generationGitCommit,'rawPath',rawPath, ...
    'maximumTime',T,'seed',run.seed,'presetName',run.presetName, ...
    'reference',reference,'mil',mil,'candidate',candidate, ...
    'commonFiniteCellCount',nnz(common), ...
    'commonReferenceRmse',finiteMean(reference.rmseBySensorTime(common)), ...
    'commonCandidateRmse',finiteMean(candidate.rmseBySensorTime(common)), ...
    'commonMilCellCount',nnz(commonMil),'commonMilRmse',finiteMean(mil.rmseBySensorTime(commonMil)), ...
    'commonMilCandidateRmse',finiteMean(candidate.rmseBySensorTime(commonMil)), ...
    'eospaImprovementPercent',100*(1-candidate.eospa/reference.eospa), ...
    'attemptedByteRatio',candidate.attemptedBytes/reference.attemptedBytes, ...
    'formationEospaChangePercent',formationChange, ...
    'attemptedRouteDifferences',nnz(xor(run.attempted,baseline.attempted(:,:,1:T))), ...
    'deliveredRouteDifferences',nnz(xor(run.delivered,baseline.delivered(:,:,1:T))), ...
    'elapsedSeconds',run.elapsedSeconds,'screenEvaluated',T==40,'developmentEvidenceOnly',true);
result.commonRmseChangePercent=100*(result.commonCandidateRmse/result.commonReferenceRmse-1);
result.screenPassed=result.screenEvaluated && result.eospaImprovementPercent>=1 && ...
    candidate.countError<reference.countError && ...
    candidate.representativeDisagreement<=reference.representativeDisagreement && ...
    isfinite(result.commonRmseChangePercent) && result.commonRmseChangePercent<=1 && ...
    max(formationChange)<=1 && result.attemptedByteRatio<=1.05 && ...
    result.attemptedRouteDifferences==0 && result.deliveredRouteDifferences==0;
save('-mat7-binary',fullfile(out,'CONDITIONAL_SPATIAL_POOL_V291.mat'),'result');
reportPath=fullfile(out,'CONDITIONAL_SPATIAL_POOL_V291_REPORT.md');
writeOutputs(reportPath,out,result,run.config);
fprintf('V291 prefix %d: E-OSPA %.6f -> %.6f, RMSE %.6f -> %.6f, bytes %.4fx; screen %d/%d.\n', ...
    T,reference.eospa,candidate.eospa,reference.conditionalRmse,candidate.conditionalRmse, ...
    result.attemptedByteRatio,result.screenEvaluated,result.screenPassed);
end

function writeOutputs(path,out,r,cfg)
fields={'eospa','conditionalRmse','countError','representativeDisagreement', ...
    'worstSensorEospa','attemptedBytes','deliveredBytes','attemptedMessages', ...
    'deliveredMessages','finiteRmseFraction'};
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# V291: conditional spatial-pooling control\n\n');
fprintf(fid,'X36 seed 1301, original 160-step scene prefix 1--%d; source `%s`. Self-check only.\n\n',r.maximumTime,r.generationGitCommit);
fprintf(fid,'Not standard LMB-KLA/MIL, not a reproduced Uney method and not new routing. Compared with V288, retain arithmetic existence, zero extension and existence-weighted conditional weights; replace only the spatial pooling operator, including its documented numerical approximation. Recursive inputs can subsequently differ.\n\n');
fprintf(fid,'| Metric | V242 KLA | V288 MIL | Conditional geometric |\n| --- | ---: | ---: | ---: |\n');
for k=1:numel(fields)
    f=fields{k}; fprintf(fid,'| %s | %.9f | %.9f | %.9f |\n',f,r.reference.(f),r.mil.(f),r.candidate.(f));
end
fprintf(fid,'\nCommon-finite RMSE versus KLA: %.6f -> %.6f (%d cells). Versus MIL: %.6f -> %.6f (%d cells). Matching identities can differ.\n', ...
    r.commonReferenceRmse,r.commonCandidateRmse,r.commonFiniteCellCount,r.commonMilRmse,r.commonMilCandidateRmse,r.commonMilCellCount);
fprintf(fid,'Route-mask differences attempted/delivered: %d/%d; filter %.1f s.\n',r.attemptedRouteDifferences,r.deliveredRouteDifferences,r.elapsedSeconds);
fprintf(fid,'\n| Formation | KLA E | V291 E | KLA RMSE | V291 RMSE |\n| --- | ---: | ---: | ---: | ---: |\n');
groups=unique(cfg.sensorGroupIds,'stable');
for g=1:numel(groups)
    selected=cfg.sensorGroupIds==groups(g);
    fprintf(fid,'| %d | %.6f | %.6f | %.6f | %.6f |\n',groups(g),r.reference.perFormationEospa(g), ...
        r.candidate.perFormationEospa(g),finiteMean(r.reference.rmseBySensorTime(selected,:)), ...
        r.candidate.perFormationRmse(g));
end
fprintf(fid,'\nScreen evaluated %d; passed %d. Two steps are integration only. No automatic full/M24 extension and no claim of across-seed significance.\n',r.screenEvaluated,r.screenPassed);
clear cleanup;
fid=fopen(fullfile(out,'V291_JOINT_METRICS.csv'),'w'); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'metric,kla_reference,mil,conditional_geometric\n');
for k=1:numel(fields)
    f=fields{k}; fprintf(fid,'%s,%.12g,%.12g,%.12g\n',f,r.reference.(f),r.mil.(f),r.candidate.(f));
end
clear cleanup;
fid=fopen(fullfile(out,'V291_SENSOR_METRICS.csv'),'w'); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'sensor,formation,reference_eospa,candidate_eospa,reference_rmse,candidate_rmse\n');
for n=1:numel(cfg.sensorGroupIds)
    fprintf(fid,'%d,%d,%.12g,%.12g,%.12g,%.12g\n',n,cfg.sensorGroupIds(n), ...
        r.reference.perSensorEospa(n),r.candidate.perSensorEospa(n),r.reference.perSensorRmse(n),r.candidate.perSensorRmse(n));
end
end

function inputs=cropInputs(inputs,T)
inputs.measurements=inputs.measurements(:,1:T); inputs.config.simulationLength=T;
inputs.model.simulationLength=T; inputs.model.dynamicTopologyScenario.config.simulationLength=T;
for n=1:numel(inputs.sensorTrajectories),inputs.sensorTrajectories{n}=inputs.sensorTrajectories{n}(:,1:T);end
for name={'x','mu','Sigma'},inputs.groundTruthRfs.(name{1})=inputs.groundTruthRfs.(name{1})(1:T);end
inputs.groundTruthRfs.cardinality=inputs.groundTruthRfs.cardinality(1:T);
if ndims(inputs.commConfig.pDropByEdge)>=3,inputs.commConfig.pDropByEdge=inputs.commConfig.pDropByEdge(:,:,1:T);end
if isfield(inputs.commConfig,'linkUniforms') && ndims(inputs.commConfig.linkUniforms)>=3
    inputs.commConfig.linkUniforms=inputs.commConfig.linkUniforms(:,:,1:T);
end
inputs.graphData.physicalAdjacency=inputs.graphData.physicalAdjacency(:,:,1:T);
inputs.graphData.positions=inputs.graphData.positions(:,:,1:T);
end

function v=finiteMean(x)
x=x(isfinite(x)); if isempty(x),v=NaN;else,v=mean(x(:));end
end
