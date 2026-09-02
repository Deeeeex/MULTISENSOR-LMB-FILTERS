function [reportPath, result] = ...
        runCausalGatewayEmbeddingV250H3Oracle(options)
% RUNCAUSALGATEWAYEMBEDDINGV250H3ORACLE Paired finite-horizon value.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getCausalGatewayEmbeddingV250Protocol());
presetName = char(getField(options, 'presetName', ...
    protocol.allowedPresets{1}));
seed = getField(options, 'seed', protocol.allowedSeeds(1));
resume = logical(getField(options, 'resume', true));
writeReport = logical(getField(options, 'writeReport', true));
if ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        ~islogical(resume) || ~isscalar(resume) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('CausalGatewayEmbeddingV250:InvalidOracleRequest', ...
        'The V250 H=3 oracle request violates its M24 protocol.');
end

gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('CausalGatewayEmbeddingV250:DirtyOracleSource', ...
        'Official V250 H=3 execution requires clean source.');
end
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', ...
    protocol.oracleOutputRoot));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7, mkdir(outputRoot); end
matPath = fullfile(outputRoot, ...
    getField(protocol, 'oracleMatName', ...
        'CAUSAL_GATEWAY_EMBEDDING_V250_H3_ORACLE.mat'));
reportPath = fullfile(outputRoot, ...
    getField(protocol, 'oracleReportName', ...
        'CAUSAL_GATEWAY_EMBEDDING_V250_H3_ORACLE.md'));

[cachePath, cacheSummary] = ...
    generateCausalGatewayEmbeddingV250ReferenceCache(struct( ...
        'presetName', presetName, 'seed', seed, ...
        'overwrite', false, 'protocol', protocol));
loadedCache = load(cachePath, 'behaviorBundle');
bundle = loadedCache.behaviorBundle;
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);

expected = initializeResult(protocol, gitState.commit, ...
    presetName, seed, cachePath, cacheSummary, matPath, reportPath);
result = expected;
if resume && exist(matPath, 'file') == 2
    loaded = load(matPath, 'result');
    validateExistingResult(loaded.result, expected);
    result = loaded.result;
end

for windowIdx = 1:numel(protocol.anchorTimes)
    anchorTime = protocol.anchorTimes(windowIdx);
    endTime = anchorTime + protocol.horizonSteps - 1;
    [initialPosteriors, history] = ...
        extractBehaviorContinuationSnapshot( ...
            bundle, anchorTime, inputs.config.numberOfSensors);
    bankContext = buildBankContext( ...
        inputs, identity, anchorTime, history);
    bank = buildCausalGatewayEmbeddingCandidateBankV250( ...
        bankContext, protocol.maximumCandidateCount);
    if isempty(result.windows{windowIdx})
        window = initializeWindow( ...
            bank, anchorTime, endTime, protocol);
    else
        window = result.windows{windowIdx};
        validateExistingWindow(window, bank, anchorTime, endTime);
    end
    cropped = cropInputs(inputs, endTime);
    for candidateIdx = 1:bank.candidateCount
        if armComplete(window.arms{candidateIdx})
            continue;
        end
        candidate = bank.candidates(candidateIdx);
        if candidateIdx == bank.referenceCandidateIndex
            config = buildCausalMinimumFormationBackboneV242Config( ...
                cropped.config);
            if isIndependentM24TeacherProtocol(protocol)
                executionContext = ...
                    buildIndependentM24GatewayTeacherV252ExecutionContext( ...
                        presetName, seed, endTime, ...
                        'candidate-reference', anchorTime, candidateIdx);
            else
                executionContext = ...
                    buildCausalMinimumFormationBackboneV242ExecutionContext( ...
                        presetName, seed, endTime);
            end
        else
            config = buildCausalGatewayEmbeddingV250Config( ...
                cropped.config, candidate.gatewayAssignment);
            if isIndependentM24TeacherProtocol(protocol)
                executionContext = ...
                    buildIndependentM24GatewayTeacherV252ExecutionContext( ...
                        presetName, seed, endTime, ...
                        'candidate-action', anchorTime, candidateIdx);
            else
                executionContext = ...
                    buildCausalGatewayEmbeddingV250ExecutionContext( ...
                        presetName, seed, endTime, anchorTime, ...
                        candidateIdx);
            end
        end
        config = attachContinuation( ...
            config, initialPosteriors, history, anchorTime, ...
            getField(protocol, 'continuationSource', ...
                'v250-v242-reference-cache'));
        filterModel = ...
            removeRealizedTargetTruthFromDynamicTopologyModel( ...
                cropped.model);
        rng(seed + protocol.filterSeedOffset, 'twister');
        timerId = tic;
        fprintf('%s H=3 t=%d candidate %02d/%02d %s ...\n', ...
            getField(protocol, 'runLabel', 'V250'), ...
            anchorTime, candidateIdx, bank.candidateCount, ...
            candidate.candidateType);
        [estimates, diagnostics] = ...
            runEventTriggeredDistributedLmbFilter( ...
                filterModel, cropped.measurements, ...
                cropped.sensorTrajectories, cropped.neighborMap, ...
                cropped.commConfig, config, executionContext);
        arm = scoreWindow(estimates, diagnostics, ...
            cropped.groundTruthRfs, cropped.config, ...
            anchorTime, endTime, candidate, toc(timerId));
        assertArm(arm, cropped.config, protocol, candidateIdx);
        window.arms{candidateIdx} = arm;
        result.windows{windowIdx} = window;
        result.lastCompletedAnchorTime = anchorTime;
        result.lastCompletedCandidateIndex = candidateIdx;
        save('-mat7-binary', matPath, 'result');
        fprintf(['  E-OSPA %.3f; RMSE %.3f; consistency %.3f; ', ...
            'bytes %.0f; applied %d/%d; %.1f s\n'], ...
            arm.positionEospa, arm.positionRmse, ...
            arm.interFormationPositionOspa, arm.attemptedPayloadBytes, ...
            arm.requestedAssignmentAppliedPageCount, ...
            protocol.horizonSteps, arm.elapsedSeconds);
    end
    window = finalizeWindow(window, protocol);
    result.windows{windowIdx} = window;
    save('-mat7-binary', matPath, 'result');
end

result = finalizeResult(result, protocol);
result.completedAt = datestr(now, 31);
save('-mat7-binary', matPath, 'result');
if writeReport
    writeReportFile(reportPath, result);
end
fprintf(['%s H=3 oracle complete: joint-positive anchors %d/%d; ', ...
    'aggregate pass=%d; ridge authorized=%d\n'], ...
    getField(protocol, 'runLabel', 'V250'), ...
    result.jointPositiveAnchorCount, numel(protocol.anchorTimes), ...
    result.oraclePassed, result.ridgeTrainingAuthorized);
fprintf('%s H=3 report: %s\n', ...
    getField(protocol, 'runLabel', 'V250'), reportPath);
end

function result = initializeResult(protocol, commit, presetName, seed, ...
        cachePath, cacheSummary, matPath, reportPath)
result = struct();
result.contractVersion = getField(protocol, 'resultContractVersion', ...
    'causal-gateway-embedding-v250-h3-oracle-result-v1');
result.protocolId = protocol.id;
result.protocol = protocol;
result.startedAt = datestr(now, 31);
result.completedAt = '';
result.generationGitCommit = commit;
result.presetName = presetName;
result.seed = seed;
result.cachePath = cachePath;
result.cacheGenerationGitCommit = cacheSummary.generationGitCommit;
result.cacheElapsedSeconds = cacheSummary.elapsedSeconds;
result.windows = cell(1, numel(protocol.anchorTimes));
result.lastCompletedAnchorTime = 0;
result.lastCompletedCandidateIndex = 0;
result.jointPositiveAnchorCount = 0;
result.aggregateComparison = struct();
result.oraclePassed = false;
result.ridgeTrainingAuthorized = false;
result.gnnTrainingAuthorized = false;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.matPath = matPath;
result.reportPath = reportPath;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function validateExistingResult(actual, expected)
required = {'contractVersion', 'protocolId', ...
    'generationGitCommit', 'presetName', 'seed', ...
    'cachePath', 'cacheGenerationGitCommit', 'windows'};
valid = isstruct(actual) && isscalar(actual) && ...
    all(isfield(actual, required)) && ...
    strcmp(actual.contractVersion, expected.contractVersion) && ...
    strcmp(actual.protocolId, expected.protocolId) && ...
    strcmp(actual.generationGitCommit, ...
        expected.generationGitCommit) && ...
    strcmp(actual.presetName, expected.presetName) && ...
    actual.seed == expected.seed && ...
    strcmp(actual.cachePath, expected.cachePath) && ...
    strcmp(actual.cacheGenerationGitCommit, ...
        expected.cacheGenerationGitCommit) && ...
    numel(actual.windows) == numel(expected.windows);
if ~valid
    error('CausalGatewayEmbeddingV250:InvalidOracleResume', ...
        'The V250 checkpoint belongs to another source or cache.');
end
end

function window = initializeWindow( ...
        bank, anchorTime, endTime, protocol)
window = struct();
window.contractVersion = getField(protocol, 'windowContractVersion', ...
    'causal-gateway-embedding-v250-h3-window-v1');
window.anchorTime = anchorTime;
window.endTime = endTime;
window.candidateCount = bank.candidateCount;
window.referenceCandidateIndex = bank.referenceCandidateIndex;
window.rawGlobalAssignmentCount = bank.rawGlobalAssignmentCount;
window.receiverCoverageByFormation = ...
    bank.receiverCoverageByFormation;
window.candidateAssignments = ...
    {bank.candidates.gatewayAssignment};
window.candidateTypes = {bank.candidates.candidateType};
window.arms = cell(1, bank.candidateCount);
window.comparisons = cell(1, bank.candidateCount);
window.bestAvailableCandidateIndex = bank.referenceCandidateIndex;
window.selectedCandidateIndex = bank.referenceCandidateIndex;
window.jointPositiveCandidateFound = false;
end

function validateExistingWindow(window, bank, anchorTime, endTime)
valid = isstruct(window) && isscalar(window) && ...
    isfield(window, 'contractVersion') && ...
    strcmp(window.contractVersion, ...
        getField(protocol, 'windowContractVersion', ...
            'causal-gateway-embedding-v250-h3-window-v1')) && ...
    window.anchorTime == anchorTime && window.endTime == endTime && ...
    window.candidateCount == bank.candidateCount && ...
    window.referenceCandidateIndex == bank.referenceCandidateIndex && ...
    numel(window.candidateAssignments) == bank.candidateCount && ...
    numel(window.arms) == bank.candidateCount;
if valid
    for candidateIdx = 1:bank.candidateCount
        valid = valid && isequal( ...
            window.candidateAssignments{candidateIdx}, ...
            bank.candidates(candidateIdx).gatewayAssignment);
    end
end
if ~valid
    error('CausalGatewayEmbeddingV250:WindowDrift', ...
        'The resumed V250 candidate bank changed.');
end
end

function context = buildBankContext(inputs, identity, anchorTime, history)
nodeCount = inputs.config.numberOfSensors;
formationCount = numel(unique(inputs.config.sensorGroupIds));
context = struct();
context.localPosteriorBySensor = ...
    repmat({struct([])}, 1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', inputs.config, ...
    'staticAdjacency', logical(inputs.graphData.staticAdjacency)));
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
context.physicalAdjacency = logical( ...
    inputs.graphData.physicalAdjacency(:, :, anchorTime));
context.positions = inputs.graphData.positions(:, :, anchorTime);
context.currentTime = anchorTime;
context.commConfig = struct('pDropByEdge', ...
    inputs.commConfig.pDropByEdge(:, :, anchorTime));
context.directedMessageBudget = nodeCount + 2 * (formationCount - 1);
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.previousAdjacencyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.selectedDirectedEdgeHistory);
context.previousAdjacencyHistoryCount = ...
    size(context.previousAdjacencyHistory, 3);
context.previousAdjacencyHistoryTimes = history.times;
end

function config = attachContinuation( ...
        config, posteriors, history, anchorTime, source)
config.filterInitialLocalPosteriorBySensor = posteriors;
config.filterInitialLocalPosteriorTime = anchorTime;
config.filterInitialPreviousAdjacencyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.selectedDirectedEdgeHistory);
config.filterInitialPreviousAdjacencyTimes = history.times;
config.filterInitialPreviousAdjacencyHistorySource = source;
end

function arm = scoreWindow(estimates, diagnostics, truth, config, ...
        anchorTime, endTime, candidate, elapsedSeconds)
times = anchorTime:endTime;
sensorCount = numel(estimates);
eospa = zeros(sensorCount, numel(times));
rmse = nan(sensorCount, numel(times));
cardinality = zeros(sensorCount, numel(times));
for sensorIdx = 1:sensorCount
    allRmse = computeSetRmseOverTime(estimates{sensorIdx}, truth);
    rmse(sensorIdx, :) = allRmse(times);
    for timeIdx = 1:numel(times)
        currentTime = times(timeIdx);
        components = computePositionEuclideanOspa( ...
            truth.x{currentTime}, ...
            estimates{sensorIdx}.mu{currentTime}, ...
            config.ospaPositionCutoff, 2, [1, 2]);
        eospa(sensorIdx, timeIdx) = components(1);
        cardinality(sensorIdx, timeIdx) = abs( ...
            numel(estimates{sensorIdx}.mu{currentTime}) - ...
            numel(truth.x{currentTime}));
    end
end
representatives = selectFormationRepresentatives( ...
    config.sensorGroupIds);
consistency = zeros(1, numel(times));
for timeIdx = 1:numel(times)
    currentTime = times(timeIdx);
    pairValues = zeros(1, nchoosek(numel(representatives), 2));
    cursor = 0;
    for left = 1:numel(representatives)-1
        for right = left+1:numel(representatives)
            cursor = cursor + 1;
            components = computePositionEuclideanOspa( ...
                estimates{representatives(left)}.mu{currentTime}, ...
                estimates{representatives(right)}.mu{currentTime}, ...
                config.ospaPositionCutoff, 2, [1, 2]);
            pairValues(cursor) = components(1);
        end
    end
    consistency(timeIdx) = mean(pairValues);
end
groupIds = reshape(config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
perFormationEospa = zeros(1, numel(groups));
perFormationRmse = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    rows = groupIds == groups(formationIdx);
    perFormationEospa(formationIdx) = mean( ...
        reshape(eospa(rows, :), 1, []));
    perFormationRmse(formationIdx) = finiteMean( ...
        reshape(rmse(rows, :), 1, []));
end
attemptedBytes = diagnostics.attemptedPayloadBytes(:, :, times);
delivered = logical(diagnostics.delivered(:, :, times));
scheduleApplied = false(1, numel(times));
scheduleFallback = false(1, numel(times));
for timeIdx = 1:numel(times)
    schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
        times(timeIdx)};
    scheduleApplied(timeIdx) = logical(getField( ...
        schedule, 'requestedGatewayAssignmentApplied', true));
    scheduleFallback(timeIdx) = logical(getField( ...
        schedule, 'gatewayFallbackUsed', false));
end
strong = false(1, numel(times));
for timeIdx = 1:numel(times)
    page = logical(diagnostics.topologyActiveEdge( ...
        :, :, times(timeIdx)))';
    strong(timeIdx) = isStronglyConnected(page);
end
arm = struct();
arm.completed = true;
arm.candidateIndex = candidate.candidateIndex;
arm.candidateType = candidate.candidateType;
arm.gatewayAssignment = candidate.gatewayAssignment;
arm.elapsedSeconds = elapsedSeconds;
arm.positionEospa = mean(eospa(:));
arm.positionRmse = finiteMean(rmse(:));
arm.interFormationPositionOspa = mean(consistency);
arm.terminalInterFormationPositionOspa = consistency(end);
arm.meanAbsoluteCardinalityError = mean(cardinality(:));
arm.perFormationPositionEospa = perFormationEospa;
arm.perFormationPositionRmse = perFormationRmse;
arm.attemptedPayloadBytes = sum(attemptedBytes(:));
arm.deliveredPayloadBytes = sum(attemptedBytes(delivered));
arm.messageCountByTime = reshape( ...
    diagnostics.topologyDirectedMessageCount(times), 1, []);
arm.physicalFeasibilityPassed = ...
    all(diagnostics.topologyFeasible(times));
arm.instantaneousStrongPassed = all(strong);
arm.policyTruthUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyTruthUsed(times));
arm.policyFutureOutcomeUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyFutureOutcomeUsed(times));
arm.requestedAssignmentAppliedByTime = scheduleApplied;
arm.requestedAssignmentAppliedPageCount = nnz(scheduleApplied);
arm.gatewayFallbackByTime = scheduleFallback;
arm.gatewayFallbackCount = nnz(scheduleFallback);
end

function assertArm(arm, config, protocol, candidateIdx)
formationCount = numel(unique(config.sensorGroupIds));
expectedMessages = config.numberOfSensors + 2 * (formationCount - 1);
passed = arm.physicalFeasibilityPassed && ...
    arm.instantaneousStrongPassed && ...
    ~arm.policyTruthUsed && ~arm.policyFutureOutcomeUsed && ...
    all(arm.messageCountByTime == expectedMessages);
if candidateIdx > 1
    passed = passed && arm.requestedAssignmentAppliedByTime(1);
end
if ~passed || numel(arm.messageCountByTime) ~= protocol.horizonSteps
    error('CausalGatewayEmbeddingV250:OracleArmContract', ...
        'A V250 H=3 arm violated its paired runtime contract.');
end
end

function window = finalizeWindow(window, protocol)
reference = window.arms{window.referenceCandidateIndex};
bestIndex = window.referenceCandidateIndex;
bestScore = -Inf;
jointPositive = false;
for candidateIdx = 1:window.candidateCount
    arm = window.arms{candidateIdx};
    comparison = compareArm(arm, reference, protocol);
    window.comparisons{candidateIdx} = comparison;
    if candidateIdx == window.referenceCandidateIndex || ...
            ~comparison.tailAndByteConstraintPassed
        continue;
    end
    if comparison.jointScorePercent > bestScore + 1e-12 || ...
            (abs(comparison.jointScorePercent - bestScore) <= 1e-12 && ...
             candidateIdx < bestIndex)
        bestIndex = candidateIdx;
        bestScore = comparison.jointScorePercent;
    end
end
if bestIndex ~= window.referenceCandidateIndex
    jointPositive = window.comparisons{bestIndex}.jointPositive;
end
window.bestAvailableCandidateIndex = bestIndex;
window.jointPositiveCandidateFound = jointPositive;
if jointPositive
    window.selectedCandidateIndex = bestIndex;
else
    window.selectedCandidateIndex = window.referenceCandidateIndex;
end
end

function comparison = compareArm(candidate, reference, protocol)
comparison = struct();
comparison.eospaGainPercent = lowerGain( ...
    reference.positionEospa, candidate.positionEospa);
comparison.rmseGainPercent = lowerGain( ...
    reference.positionRmse, candidate.positionRmse);
comparison.consistencyGainPercent = lowerGain( ...
    reference.interFormationPositionOspa, ...
    candidate.interFormationPositionOspa);
comparison.attemptedByteSavingPercent = lowerGain( ...
    reference.attemptedPayloadBytes, candidate.attemptedPayloadBytes);
comparison.formationEospaGainPercent = lowerGainVector( ...
    reference.perFormationPositionEospa, ...
    candidate.perFormationPositionEospa);
comparison.formationRmseGainPercent = lowerGainVector( ...
    reference.perFormationPositionRmse, ...
    candidate.perFormationPositionRmse);
comparison.minimumFormationEospaGainPercent = ...
    min(comparison.formationEospaGainPercent);
comparison.minimumFormationRmseGainPercent = ...
    min(comparison.formationRmseGainPercent);
comparison.jointScorePercent = min([ ...
    comparison.eospaGainPercent, comparison.rmseGainPercent, ...
    comparison.consistencyGainPercent]);
comparison.tailAndByteConstraintPassed = ...
    comparison.attemptedByteSavingPercent >= ...
        protocol.minimumAggregateByteSavingPercent && ...
    comparison.minimumFormationEospaGainPercent >= ...
        -protocol.maximumFormationRegressionPercent && ...
    comparison.minimumFormationRmseGainPercent >= ...
        -protocol.maximumFormationRegressionPercent;
comparison.jointPositive = ...
    comparison.jointScorePercent > ...
        protocol.minimumAggregateCoreGainPercent && ...
    comparison.tailAndByteConstraintPassed;
end

function result = finalizeResult(result, protocol)
selected = cell(1, numel(result.windows));
references = cell(1, numel(result.windows));
jointCount = 0;
for windowIdx = 1:numel(result.windows)
    window = result.windows{windowIdx};
    selected{windowIdx} = ...
        window.arms{window.selectedCandidateIndex};
    references{windowIdx} = ...
        window.arms{window.referenceCandidateIndex};
    jointCount = jointCount + window.jointPositiveCandidateFound;
end
aggregateCandidate = aggregateArms(selected);
aggregateReference = aggregateArms(references);
aggregateComparison = compareArm( ...
    aggregateCandidate, aggregateReference, protocol);
result.jointPositiveAnchorCount = jointCount;
result.aggregateCandidate = aggregateCandidate;
result.aggregateReference = aggregateReference;
result.aggregateComparison = aggregateComparison;
result.oraclePassed = jointCount >= ...
        protocol.minimumJointPositiveAnchorCount && ...
    aggregateComparison.jointPositive;
result.ridgeTrainingAuthorized = result.oraclePassed;
result.gnnTrainingAuthorized = false;
if result.oraclePassed
    result.nextMethodDecision = getField(protocol, ...
        'oraclePassDecision', ...
        'fit-causal-ridge-ranker-before-any-gnn');
else
    result.nextMethodDecision = getField(protocol, ...
        'oracleFailDecision', ...
        'stop-v250-gateway-action-space-before-learning');
end
end

function aggregate = aggregateArms(arms)
aggregate = struct();
aggregate.positionEospa = mean(cellfun( ...
    @(value) value.positionEospa, arms));
aggregate.positionRmse = mean(cellfun( ...
    @(value) value.positionRmse, arms));
aggregate.interFormationPositionOspa = mean(cellfun( ...
    @(value) value.interFormationPositionOspa, arms));
aggregate.attemptedPayloadBytes = sum(cellfun( ...
    @(value) value.attemptedPayloadBytes, arms));
formationCount = numel(arms{1}.perFormationPositionEospa);
formationEospa = zeros(numel(arms), formationCount);
formationRmse = zeros(numel(arms), formationCount);
for armIdx = 1:numel(arms)
    formationEospa(armIdx, :) = ...
        arms{armIdx}.perFormationPositionEospa;
    formationRmse(armIdx, :) = ...
        arms{armIdx}.perFormationPositionRmse;
end
aggregate.perFormationPositionEospa = mean(formationEospa, 1);
aggregate.perFormationPositionRmse = mean(formationRmse, 1);
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('CausalGatewayEmbeddingV250:OracleReportOpen', ...
        'Could not write the V250 H=3 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# %s\n\n', getField(result.protocol, 'reportTitle', ...
    'V250 causal sensor-gateway H=3 oracle'));
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Source commit: `%s`\n', ...
    result.generationGitCommit);
fprintf(fid, '- Joint-positive anchors: `%d / %d`\n', ...
    result.jointPositiveAnchorCount, numel(result.windows));
fprintf(fid, '- Oracle / ridge / GNN authorized: `%d / %d / %d`\n', ...
    result.oraclePassed, result.ridgeTrainingAuthorized, ...
    result.gnnTrainingAuthorized);
fprintf(fid, '- Next decision: `%s`\n\n', ...
    result.nextMethodDecision);
fprintf(fid, ['| Anchor | Best available | Selected | Type | ', ...
    'E gain | RMSE gain | Consistency gain | Byte saving | ', ...
    'Weakest formation E / RMSE | Applied pages |\n']);
fprintf(fid, '|--:|--:|--:|:--|--:|--:|--:|--:|:--|:--:|\n');
for windowIdx = 1:numel(result.windows)
    window = result.windows{windowIdx};
    bestIdx = window.bestAvailableCandidateIndex;
    selectedIdx = window.selectedCandidateIndex;
    comparison = window.comparisons{bestIdx};
    arm = window.arms{bestIdx};
    fprintf(fid, ['| %d | %d | %d | %s | %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% | %+.3f%% | %+.3f%% / %+.3f%% | %d/%d |\n'], ...
        window.anchorTime, bestIdx, selectedIdx, ...
        window.candidateTypes{bestIdx}, ...
        comparison.eospaGainPercent, comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, ...
        comparison.attemptedByteSavingPercent, ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent, ...
        arm.requestedAssignmentAppliedPageCount, ...
        result.protocol.horizonSteps);
end
gain = result.aggregateComparison;
fprintf(fid, '\n## Aggregate selected-or-reference result\n\n');
fprintf(fid, '| Metric | Gain |\n|:--|--:|\n');
fprintf(fid, '| E-OSPA | `%+.3f%%` |\n', gain.eospaGainPercent);
fprintf(fid, '| RMSE | `%+.3f%%` |\n', gain.rmseGainPercent);
fprintf(fid, '| Consistency | `%+.3f%%` |\n', ...
    gain.consistencyGainPercent);
fprintf(fid, '| Attempted-byte saving | `%+.3f%%` |\n', ...
    gain.attemptedByteSavingPercent);
fprintf(fid, '| Weakest formation E-OSPA | `%+.3f%%` |\n', ...
    gain.minimumFormationEospaGainPercent);
fprintf(fid, '| Weakest formation RMSE | `%+.3f%%` |\n', ...
    gain.minimumFormationRmseGainPercent);
fprintf(fid, '\n## Frozen decision rule\n\n');
fprintf(fid, ['At each anchor, the oracle maximizes the minimum of the ', ...
    'E-OSPA, RMSE and inter-formation consistency percentage gains. ', ...
    'A candidate is joint-positive only if all three gains are positive, ', ...
    'attempted bytes do not increase and neither formation-level E-OSPA ', ...
    'nor RMSE regresses by more than %.1f%%. The requested gateway ', ...
    'assignment persists for H=3 when physical and falls back to V242 ', ...
    'when it is not.\n\n'], ...
    result.protocol.maximumFormationRegressionPercent);
fprintf(fid, '## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function inputs = cropInputs(inputs, maximumTime)
inputs.measurements = inputs.measurements(:, 1:maximumTime);
inputs.config.simulationLength = maximumTime;
inputs.model.simulationLength = maximumTime;
if isfield(inputs.model, 'dynamicTopologyScenario') && ...
        isfield(inputs.model.dynamicTopologyScenario, 'config')
    inputs.model.dynamicTopologyScenario.config.simulationLength = ...
        maximumTime;
end
for sensorIdx = 1:numel(inputs.sensorTrajectories)
    inputs.sensorTrajectories{sensorIdx} = ...
        inputs.sensorTrajectories{sensorIdx}(:, 1:maximumTime);
end
fields = {'x', 'mu', 'Sigma'};
for fieldIdx = 1:numel(fields)
    name = fields{fieldIdx};
    inputs.groundTruthRfs.(name) = ...
        inputs.groundTruthRfs.(name)(1:maximumTime);
end
inputs.groundTruthRfs.cardinality = ...
    inputs.groundTruthRfs.cardinality(1:maximumTime);
if ndims(inputs.commConfig.pDropByEdge) >= 3
    inputs.commConfig.pDropByEdge = ...
        inputs.commConfig.pDropByEdge(:, :, 1:maximumTime);
end
if isfield(inputs.commConfig, 'linkUniforms') && ...
        ndims(inputs.commConfig.linkUniforms) >= 3
    inputs.commConfig.linkUniforms = ...
        inputs.commConfig.linkUniforms(:, :, 1:maximumTime);
end
if isfield(inputs, 'graphData')
    inputs.graphData.physicalAdjacency = ...
        inputs.graphData.physicalAdjacency(:, :, 1:maximumTime);
    inputs.graphData.positions = ...
        inputs.graphData.positions(:, :, 1:maximumTime);
end
end

function sensors = selectFormationRepresentatives(groupIds)
groups = unique(groupIds, 'stable');
sensors = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    sensors(groupIdx) = find(groupIds == groups(groupIdx), 1);
end
end

function complete = armComplete(arm)
complete = isstruct(arm) && isscalar(arm) && ...
    isfield(arm, 'completed') && logical(arm.completed);
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = mean(values); end
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = lowerGainVector(reference, candidate)
value = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function value = anyFiniteNonzero(values)
values = values(isfinite(values));
value = any(values ~= 0);
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, ...
        find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && (path(1) == '/' || ...
    (~isempty(regexp(path, '^[A-Za-z]:[\\/]', 'once'))));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end

function detected = isIndependentM24TeacherProtocol(protocol)
detected = isstruct(protocol) && isscalar(protocol) && ...
    isfield(protocol, 'id') && ...
    strcmp(protocol.id, ...
        'independent-m24-gateway-teacher-v252-v1');
end
