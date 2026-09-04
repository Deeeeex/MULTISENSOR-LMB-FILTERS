function [reportPath, result] = ...
        runRiskTriggeredFormationResidualPulseV260Continuation(options)
% RUNRISKTRIGGEREDFORMATIONRESIDUALPULSEV260CONTINUATION Two causal arms.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getRiskTriggeredFormationResidualPulseV260Protocol();
tracePath = char(getField(options, 'tracePath', ''));
resume = logical(getField(options, 'resume', true));
if isempty(tracePath) || exist(tracePath, 'file') ~= 2 || ...
        ~islogical(resume) || ~isscalar(resume)
    error('RiskTriggeredPulseV260:InvalidRunRequest', ...
        'V260 requires the completed V259 trace and a scalar resume flag.');
end
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('RiskTriggeredPulseV260:DirtySource', ...
        'Official V260 execution requires clean source.');
end
loaded = load(tracePath, 'result', 'behaviorBundle');
if ~isfield(loaded, 'result') || ~isfield(loaded, 'behaviorBundle')
    error('RiskTriggeredPulseV260:MalformedTrace', ...
        'The V259 trace lacks its result or behavior bundle.');
end
trace = loaded.result;
bundle = loaded.behaviorBundle;
validateTrace(trace, bundle, protocol);
referenceEnvelope = load(trace.referenceResultPath, 'result');
reference = referenceEnvelope.result;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', protocol.outputRoot));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7, mkdir(outputRoot); end
matPath = fullfile(outputRoot, ...
    'RISK_TRIGGERED_FORMATION_RESIDUAL_PULSE_V260_CONTINUATION.mat');
reportPath = fullfile(outputRoot, ...
    'RISK_TRIGGERED_FORMATION_RESIDUAL_PULSE_V260_CONTINUATION.md');

inputs = generateDynamicTopologyScenarioInputs( ...
    bundle.presetName, bundle.seed);
inputs = cropInputs(inputs, protocol.continuationEndTime);
startTime = protocol.continuationStartTime;
endTime = protocol.continuationEndTime;
times = startTime:endTime;
initialPosteriors = bundle.posteriorSnapshots{startTime};
initialHistory = bundle.preDecisionTopologyHistoryByTime{startTime};
referenceWindowBytes = reconstructReferenceWindowBytes( ...
    inputs, bundle, times);
v242 = scoreStoredArm(trace.passiveArm, inputs.config, ...
    times, referenceWindowBytes, 'V242 minimum backbone');
fixed = scoreStoredArm(reference.fixedTree, inputs.config, ...
    times, NaN, 'fixed formation tree');
full = scoreStoredArm(reference.fullCausal, inputs.config, ...
    times, NaN, 'full causal repair');

expected = initializeResult(protocol, gitState.commit, tracePath, ...
    trace, reference, v242, fixed, full, matPath, reportPath);
result = expected;
if resume && exist(matPath, 'file') == 2
    checkpoint = load(matPath, 'result');
    validateCheckpoint(checkpoint.result, expected);
    result = checkpoint.result;
end

filterModel = removeRealizedTargetTruthFromDynamicTopologyModel( ...
    inputs.model);
for armIdx = 1:numel(protocol.pulseWeights)
    if isCompleteArm(result.candidates{armIdx})
        continue;
    end
    pulseWeight = protocol.pulseWeights(armIdx);
    config = buildRiskTriggeredFormationResidualPulseV260Config( ...
        inputs.config, pulseWeight);
    config = attachContinuation(config, initialPosteriors, ...
        initialHistory, startTime, tracePath);
    execution = ...
        buildRiskTriggeredFormationResidualPulseV260ExecutionContext( ...
            bundle.presetName, bundle.seed, endTime, pulseWeight);
    rng(bundle.seed + protocol.filterSeedOffset, 'twister');
    fprintf('V260 continuation w=%.2f, t=%d--%d ...\n', ...
        pulseWeight, startTime, endTime);
    timerId = tic;
    [estimates, diagnostics] = ...
        runEventTriggeredDistributedLmbFilter( ...
            filterModel, inputs.measurements, ...
            inputs.sensorTrajectories, inputs.neighborMap, ...
            inputs.commConfig, config, execution);
    arm = scoreCandidate(estimates, diagnostics, ...
        inputs.groundTruthRfs, inputs.config, times, pulseWeight, ...
        toc(timerId), trace, reference, referenceWindowBytes, protocol);
    assertCandidate(arm, inputs.config, protocol);
    result.candidates{armIdx} = arm;
    result.comparisons{armIdx} = compareArms(arm, v242, protocol);
    result.lastCompletedArmIndex = armIdx;
    save('-mat7-binary', matPath, 'result');
    comparison = result.comparisons{armIdx};
    fprintf(['  E/R/C %+.3f%%/%+.3f%%/%+.3f%%; ', ...
        'F4 event-73 E/R %+.3f%%/%+.3f%%; ', ...
        'spliced static byte saving %+.3f%%; gate %d\n'], ...
        comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, ...
        comparison.eventFormationEospaGainPercent, ...
        comparison.eventFormationRmseGainPercent, ...
        arm.splicedStaticByteSavingPercent, ...
        comparison.shortHorizonGatePassed);
end

result = finalizeResult(result, protocol);
result.completedAt = datestr(now, 31);
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V260 complete: best arm %d; screen passed=%d\n', ...
    result.selectedArmIndex, result.actionMechanismPassed);
fprintf('V260 report: %s\n', reportPath);
end

function validateTrace(trace, bundle, protocol)
valid = isstruct(trace) && isscalar(trace) && ...
    isfield(trace, 'contractVersion') && ...
    strcmp(trace.contractVersion, ...
        'risk-decomposed-horizon-v259-passive-trace-result-v1') && ...
    isfield(trace, 'passiveComparison') && ...
    trace.passiveComparison.exactOutcomeReproductionPassed && ...
    isstruct(bundle) && isscalar(bundle) && ...
    strcmp(bundle.presetName, protocol.allowedPresets{1}) && ...
    bundle.seed == protocol.allowedSeeds(1) && ...
    numel(bundle.posteriorSnapshots) >= ...
        protocol.continuationEndTime && ...
    numel(bundle.preDecisionTopologyHistoryByTime) >= ...
        protocol.continuationEndTime;
if ~valid
    error('RiskTriggeredPulseV260:TraceContract', ...
        'The V259 trace does not satisfy the V260 source contract.');
end
end

function result = initializeResult(protocol, commit, tracePath, ...
        trace, reference, v242, fixed, full, matPath, reportPath)
result = struct();
result.contractVersion = ...
    'risk-triggered-formation-residual-pulse-v260-result-v1';
result.protocolId = protocol.id;
result.protocol = protocol;
result.startedAt = datestr(now, 31);
result.completedAt = '';
result.generationGitCommit = commit;
result.tracePath = tracePath;
result.traceGenerationGitCommit = trace.generationGitCommit;
result.referenceGenerationGitCommit = reference.generationGitCommit;
result.presetName = trace.presetName;
result.seed = trace.seed;
result.v242 = v242;
result.fixedTree = fixed;
result.fullCausal = full;
result.candidates = cell(1, numel(protocol.pulseWeights));
result.comparisons = cell(1, numel(protocol.pulseWeights));
result.lastCompletedArmIndex = 0;
result.selectedArmIndex = 0;
result.actionMechanismPassed = false;
result.fullM24RunAuthorized = false;
result.matPath = matPath;
result.reportPath = reportPath;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function validateCheckpoint(actual, expected)
required = {'contractVersion', 'protocolId', ...
    'generationGitCommit', 'tracePath', ...
    'traceGenerationGitCommit', 'presetName', 'seed', 'candidates'};
valid = isstruct(actual) && isscalar(actual) && ...
    all(isfield(actual, required)) && ...
    strcmp(actual.contractVersion, expected.contractVersion) && ...
    strcmp(actual.protocolId, expected.protocolId) && ...
    strcmp(actual.generationGitCommit, expected.generationGitCommit) && ...
    strcmp(actual.tracePath, expected.tracePath) && ...
    strcmp(actual.traceGenerationGitCommit, ...
        expected.traceGenerationGitCommit) && ...
    strcmp(actual.presetName, expected.presetName) && ...
    actual.seed == expected.seed && ...
    numel(actual.candidates) == numel(expected.candidates);
if ~valid
    error('RiskTriggeredPulseV260:CheckpointDrift', ...
        'The V260 checkpoint belongs to another source or commit.');
end
end

function arm = scoreStoredArm(source, config, times, bytes, name)
eospa = source.positionEospaBySensorTime(:, times);
rmse = source.positionRmseBySensorTime(:, times);
cardinality = source.absoluteCardinalityErrorBySensorTime(:, times);
arm = scoreMatrices(eospa, rmse, cardinality, config);
arm.name = name;
arm.pulseWeight = 0;
arm.windowAttemptedPayloadBytes = bytes;
arm.interFormationPositionOspa = mean( ...
    source.interFormationPositionOspaByTime(times));
eventTimes = max(58, times(1)):min(73, times(end));
eventColumns = eventTimes - times(1) + 1;
eventMembers = config.sensorGroupIds == 4;
arm.eventFormationEospa = mean(reshape( ...
    eospa(eventMembers, eventColumns), 1, []));
arm.eventFormationRmse = finiteMean(reshape( ...
    rmse(eventMembers, eventColumns), 1, []));
arm.eventFormationCardinality = mean(reshape( ...
    cardinality(eventMembers, eventColumns), 1, []));
arm.pulseTimes = zeros(1, 0);
arm.pulseFormationIds = zeros(1, 0);
arm.messageCountByTime = nan(1, numel(times));
arm.completed = true;
end

function arm = scoreCandidate(estimates, diagnostics, truth, config, ...
        times, pulseWeight, elapsedSeconds, trace, reference, ...
        referenceWindowBytes, protocol)
sensorCount = numel(estimates);
eospa = zeros(sensorCount, numel(times));
rmse = nan(sensorCount, numel(times));
cardinality = zeros(sensorCount, numel(times));
for sensorIdx = 1:sensorCount
    allRmse = computeSetRmseOverTime(estimates{sensorIdx}, truth);
    rmse(sensorIdx, :) = allRmse(times);
    for localTime = 1:numel(times)
        currentTime = times(localTime);
        components = computePositionEuclideanOspa( ...
            truth.x{currentTime}, estimates{sensorIdx}.mu{currentTime}, ...
            config.ospaPositionCutoff, 2, [1, 2]);
        eospa(sensorIdx, localTime) = components(1);
        cardinality(sensorIdx, localTime) = abs( ...
            numel(estimates{sensorIdx}.mu{currentTime}) - ...
            numel(truth.x{currentTime}));
    end
end
arm = scoreMatrices(eospa, rmse, cardinality, config);
arm.name = sprintf('V260 pulse weight %.2f', pulseWeight);
arm.pulseWeight = pulseWeight;
arm.elapsedSeconds = elapsedSeconds;
arm.interFormationPositionOspa = candidateConsistency( ...
    estimates, config.sensorGroupIds, config.ospaPositionCutoff, times);
attempted = diagnostics.attemptedPayloadBytes(:, :, times);
arm.windowAttemptedPayloadBytes = sum(attempted(:));
arm.referenceWindowAttemptedPayloadBytes = referenceWindowBytes;
arm.windowByteChangeOverV242Percent = 100 * ...
    (referenceWindowBytes - arm.windowAttemptedPayloadBytes) / ...
        max(referenceWindowBytes, eps);
arm.splicedFullTraceAttemptedPayloadBytes = ...
    trace.passiveArm.attemptedPayloadBytes - referenceWindowBytes + ...
        arm.windowAttemptedPayloadBytes;
arm.splicedStaticByteSavingPercent = 100 * ...
    (reference.fixedTree.attemptedPayloadBytes - ...
        arm.splicedFullTraceAttemptedPayloadBytes) / ...
        max(reference.fixedTree.attemptedPayloadBytes, eps);
arm.messageCountByTime = reshape( ...
    diagnostics.topologyDirectedMessageCount(times), 1, []);
arm.physicalFeasibilityPassed = all( ...
    diagnostics.topologyFeasible(times));
arm.policyTruthUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyTruthUsed(times));
arm.policyFutureOutcomeUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyFutureOutcomeUsed(times));
arm.instantaneousStrongPassed = true;
for currentTime = times
    arm.instantaneousStrongPassed = ...
        arm.instantaneousStrongPassed && isStronglyConnected( ...
            logical(diagnostics.topologyActiveEdge( ...
                :, :, currentTime)));
end
arm.pulseAppliedByTime = false(1, numel(times));
arm.pulseRequestedByTime = false(1, numel(times));
arm.pulseFormationIdByTime = zeros(1, numel(times));
arm.residualCountByTime = zeros(1, numel(times));
arm.centralizedDevelopmentController = true;
arm.distributedControlSynopsisCostIncluded = false;
for localTime = 1:numel(times)
    schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
        times(localTime)};
    arm.pulseAppliedByTime(localTime) = logical(getField( ...
        schedule, 'pulseApplied', false));
    arm.pulseRequestedByTime(localTime) = logical(getField( ...
        schedule, 'pulseRequested', false));
    arm.pulseFormationIdByTime(localTime) = getField( ...
        schedule, 'selectedLocalizationFormationId', 0);
    arm.residualCountByTime(localTime) = getField( ...
        schedule, 'selectedResidualCount', 0);
end
arm.pulseTimes = times(arm.pulseAppliedByTime);
arm.pulseFormationIds = ...
    arm.pulseFormationIdByTime(arm.pulseAppliedByTime);
arm.noConsecutivePulsePassed = ...
    ~any(arm.pulseAppliedByTime(1:end-1) & ...
        arm.pulseAppliedByTime(2:end));
arm.maximumResidualCount = max(arm.residualCountByTime);
eventTimes = max(protocol.localizationEventWindow(1), times(1)): ...
    min(protocol.localizationEventWindow(2), times(end));
eventColumns = eventTimes - times(1) + 1;
eventMembers = config.sensorGroupIds == ...
    protocol.localizationEventFormation;
arm.eventFormationEospa = mean(reshape( ...
    eospa(eventMembers, eventColumns), 1, []));
arm.eventFormationRmse = finiteMean(reshape( ...
    rmse(eventMembers, eventColumns), 1, []));
arm.eventFormationCardinality = mean(reshape( ...
    cardinality(eventMembers, eventColumns), 1, []));
arm.completed = true;
end

function arm = scoreMatrices(eospa, rmse, cardinality, config)
groupIds = reshape(config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
arm = struct();
arm.positionEospa = mean(eospa(:));
arm.positionRmse = finiteMean(rmse(:));
arm.meanAbsoluteCardinalityError = mean(cardinality(:));
arm.perFormationPositionEospa = zeros(1, numel(groups));
arm.perFormationPositionRmse = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    rows = groupIds == groups(formationIdx);
    arm.perFormationPositionEospa(formationIdx) = mean( ...
        reshape(eospa(rows, :), 1, []));
    arm.perFormationPositionRmse(formationIdx) = finiteMean( ...
        reshape(rmse(rows, :), 1, []));
end
end

function value = candidateConsistency( ...
        estimates, groupIds, cutoff, times)
groups = unique(groupIds, 'stable');
representatives = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    representatives(groupIdx) = find(groupIds == groups(groupIdx), 1);
end
values = zeros(1, numel(times));
for localTime = 1:numel(times)
    currentTime = times(localTime);
    pairs = zeros(1, nchoosek(numel(representatives), 2));
    cursor = 0;
    for left = 1:numel(representatives)-1
        for right = left+1:numel(representatives)
            cursor = cursor + 1;
            components = computePositionEuclideanOspa( ...
                estimates{representatives(left)}.mu{currentTime}, ...
                estimates{representatives(right)}.mu{currentTime}, ...
                cutoff, 2, [1, 2]);
            pairs(cursor) = components(1);
        end
    end
    values(localTime) = mean(pairs);
end
value = mean(values);
end

function comparison = compareArms(candidate, reference, protocol)
comparison = struct();
comparison.eospaGainPercent = lowerGain( ...
    reference.positionEospa, candidate.positionEospa);
comparison.rmseGainPercent = lowerGain( ...
    reference.positionRmse, candidate.positionRmse);
comparison.consistencyGainPercent = lowerGain( ...
    reference.interFormationPositionOspa, ...
    candidate.interFormationPositionOspa);
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
target = protocol.localizationEventFormation;
comparison.eventFormationEospaGainPercent = lowerGain( ...
    reference.eventFormationEospa, candidate.eventFormationEospa);
comparison.eventFormationRmseGainPercent = lowerGain( ...
    reference.eventFormationRmse, candidate.eventFormationRmse);
comparison.shortHorizonGatePassed = ...
    comparison.eventFormationEospaGainPercent >= ...
        protocol.minimumEventFormationEospaGainPercent && ...
    comparison.eventFormationRmseGainPercent > ...
        protocol.minimumEventFormationRmseGainPercent && ...
    min([comparison.eospaGainPercent, comparison.rmseGainPercent, ...
         comparison.consistencyGainPercent]) >= ...
        -protocol.maximumNetworkRegressionPercent && ...
    comparison.minimumFormationEospaGainPercent >= ...
        -protocol.maximumFormationRegressionPercent && ...
    comparison.minimumFormationRmseGainPercent >= ...
        -protocol.maximumFormationRegressionPercent && ...
    candidate.splicedStaticByteSavingPercent > 0;
comparison.targetFormationIndex = target;
end

function result = finalizeResult(result, protocol)
if any(cellfun(@(arm) ~isCompleteArm(arm), result.candidates))
    error('RiskTriggeredPulseV260:IncompleteResult', ...
        'At least one registered V260 arm is incomplete.');
end
passed = cellfun(@(value) value.shortHorizonGatePassed, ...
    result.comparisons);
candidateIndices = find(passed);
if isempty(candidateIndices)
    candidateIndices = 1:numel(result.candidates);
end
bestIdx = candidateIndices(1);
bestKey = selectionKey(result, bestIdx);
for candidateIdx = candidateIndices(2:end)
    key = selectionKey(result, candidateIdx);
    if lexicographicallyGreater(key, bestKey)
        bestIdx = candidateIdx;
        bestKey = key;
    end
end
result.selectedArmIndex = bestIdx;
result.selectedPulseWeight = ...
    result.candidates{bestIdx}.pulseWeight;
result.actionMechanismPassed = any(passed);
result.fullM24RunAuthorized = result.actionMechanismPassed;
result.nextMethodDecision = conditionalValue( ...
    result.actionMechanismPassed, ...
    'run-one-full-causal-m24-arm', ...
    'reject-local-residual-pulse-and-reconsider-zero-byte-cycle-switch');
end

function key = selectionKey(result, idx)
comparison = result.comparisons{idx};
candidate = result.candidates{idx};
key = [double(comparison.shortHorizonGatePassed), ...
    comparison.eventFormationRmseGainPercent, ...
    comparison.eventFormationEospaGainPercent, ...
    min([comparison.eospaGainPercent, comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent]), ...
    candidate.splicedStaticByteSavingPercent, -idx];
end

function greater = lexicographicallyGreater(left, right)
greater = false;
for idx = 1:numel(left)
    if left(idx) > right(idx) + 1e-12
        greater = true;
        return;
    elseif left(idx) < right(idx) - 1e-12
        return;
    end
end
end

function bytes = reconstructReferenceWindowBytes(inputs, bundle, times)
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
nodeCount = inputs.config.numberOfSensors;
bytes = 0;
for currentTime = times
    history = bundle.preDecisionTopologyHistoryByTime{currentTime};
    context = struct();
    context.localPosteriorBySensor = ...
        bundle.posteriorSnapshots{currentTime};
    context.model = inputs.model;
    context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
    context.physicalAdjacency = logical( ...
        inputs.graphData.physicalAdjacency(:, :, currentTime));
    context.positions = inputs.graphData.positions(:, :, currentTime);
    context.currentTime = currentTime;
    context.commConfig = struct('pDropByEdge', ...
        inputs.commConfig.pDropByEdge(:, :, currentTime));
    context.directedMessageBudget = 2 * nodeCount;
    context.sensorPhysicalUids = identity.sensorPhysicalUids;
    context.formationPhysicalUidsBySensor = ...
        identity.formationPhysicalUidsBySensor;
    context.previousAdjacencyHistory = ...
        convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
            history.selectedDirectedEdgeHistory);
    context.previousAdjacencyHistoryCount = ...
        size(context.previousAdjacencyHistory, 3);
    context.previousAdjacencyHistoryTimes = history.times;
    adjacency = ...
        selectCausalMinimumFormationBackboneV242Policy(context);
    senderBytes = zeros(1, nodeCount);
    for sender = 1:nodeCount
        stats = estimateLmbPayloadSize( ...
            bundle.posteriorSnapshots{currentTime}{sender}, ...
            inputs.model, 2);
        senderBytes(sender) = stats.estimatedBytes;
    end
    bytes = bytes + sum(sum(adjacency, 1) .* senderBytes);
end
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

function inputs = cropInputs(inputs, maximumTime)
inputs.measurements = inputs.measurements(:, 1:maximumTime);
inputs.config.simulationLength = maximumTime;
inputs.model.simulationLength = maximumTime;
inputs.model.dynamicTopologyScenario.config.simulationLength = maximumTime;
for sensorIdx = 1:numel(inputs.sensorTrajectories)
    inputs.sensorTrajectories{sensorIdx} = ...
        inputs.sensorTrajectories{sensorIdx}(:, 1:maximumTime);
end
for name = {'x', 'mu', 'Sigma'}
    inputs.groundTruthRfs.(name{1}) = ...
        inputs.groundTruthRfs.(name{1})(1:maximumTime);
end
inputs.groundTruthRfs.cardinality = ...
    inputs.groundTruthRfs.cardinality(1:maximumTime);
inputs.commConfig.pDropByEdge = ...
    inputs.commConfig.pDropByEdge(:, :, 1:maximumTime);
inputs.commConfig.linkUniforms = ...
    inputs.commConfig.linkUniforms(:, :, 1:maximumTime);
inputs.graphData.physicalAdjacency = ...
    inputs.graphData.physicalAdjacency(:, :, 1:maximumTime);
inputs.graphData.positions = ...
    inputs.graphData.positions(:, :, 1:maximumTime);
end

function assertCandidate(arm, config, protocol)
formationCount = numel(unique(config.sensorGroupIds));
minimumMessages = config.numberOfSensors + 2 * (formationCount - 1);
passed = arm.completed && arm.physicalFeasibilityPassed && ...
    arm.instantaneousStrongPassed && ~arm.policyTruthUsed && ...
    ~arm.policyFutureOutcomeUsed && arm.noConsecutivePulsePassed && ...
    arm.maximumResidualCount <= ...
        protocol.maximumFormationResidualEdges && ...
    all(arm.messageCountByTime >= minimumMessages) && ...
    all(arm.messageCountByTime <= minimumMessages + ...
        protocol.maximumFormationResidualEdges) && ...
    ~isempty(arm.pulseTimes);
if ~passed
    error('RiskTriggeredPulseV260:RuntimeContract', ...
        'The V260 continuation violated its route or evidence contract.');
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('RiskTriggeredPulseV260:ReportOpen', ...
        'Unable to write the V260 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V260 risk-triggered formation-local pulse\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Generation commit: `%s`\n', ...
    result.generationGitCommit);
fprintf(fid, '- Continuation: `t=%d--%d` from the V259 V242 state\n', ...
    result.protocol.continuationStartTime, ...
    result.protocol.continuationEndTime);
fprintf(fid, '- Selected pulse weight: `%.2f`\n', ...
    result.selectedPulseWeight);
fprintf(fid, '- Mechanism / full-M24 authorization: `%d / %d`\n\n', ...
    result.actionMechanismPassed, result.fullM24RunAuthorized);
fprintf(fid, ['| Arm | E-OSPA | RMSE | Consistency | F4 event E / RMSE | ', ...
    'Window bytes | Spliced static saving | Pulse pages | Gate |\n']);
fprintf(fid, '|:--|--:|--:|--:|:--|--:|--:|:--|:--:|\n');
writeArm(fid, result.v242, [], result.protocol);
for armIdx = 1:numel(result.candidates)
    writeArm(fid, result.candidates{armIdx}, ...
        result.comparisons{armIdx}, result.protocol);
end
fprintf(fid, '\n## Candidate gains over V242\n\n');
fprintf(fid, ['| Weight | Network E / R / C | F4 event E / R | ', ...
    'Weakest formation E / R | Window byte change |\n']);
fprintf(fid, '|--:|:--|:--|:--|--:|\n');
for armIdx = 1:numel(result.candidates)
    arm = result.candidates{armIdx};
    comparison = result.comparisons{armIdx};
    fprintf(fid, ['| %.2f | `%+.3f%% / %+.3f%% / %+.3f%%` | ', ...
        '`%+.3f%% / %+.3f%%` | `%+.3f%% / %+.3f%%` | `%+.3f%%` |\n'], ...
        arm.pulseWeight, comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, comparison.consistencyGainPercent, ...
        comparison.eventFormationEospaGainPercent, ...
        comparison.eventFormationRmseGainPercent, ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent, ...
        arm.windowByteChangeOverV242Percent);
end
fprintf(fid, '\n## Decision\n\n');
if result.actionMechanismPassed
    fprintf(fid, ['At least one causal pulse improves the diagnosed F4 ', ...
        'localization event without violating the registered short-horizon ', ...
        'network, formation or communication guards. Run only the selected ', ...
        'weight as a complete M24 candidate; do not tune another pulse ', ...
        'strength on this opened continuation.\n\n']);
else
    fprintf(fid, ['Neither registered pulse repairs F4 under the bounded ', ...
        'short-horizon guard. Reject residual-bundle pulsing before a full ', ...
        'episode and reconsider a zero-extra-message local-cycle action.\n\n']);
end
fprintf(fid, '## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function writeArm(fid, arm, comparison, protocol)
eventE = arm.eventFormationEospa;
eventR = arm.eventFormationRmse;
gate = true;
if isempty(comparison)
    gateText = '--';
else
    gate = comparison.shortHorizonGatePassed;
    gateText = sprintf('%d', gate);
end
if isempty(arm.pulseTimes)
    pulseText = '--';
else
    pulseText = mat2str(arm.pulseTimes);
end
if isfield(arm, 'splicedStaticByteSavingPercent') && ...
        isfinite(arm.splicedStaticByteSavingPercent)
    saving = arm.splicedStaticByteSavingPercent;
else
    saving = NaN;
end
fprintf(fid, ['| %s | %.3f | %.3f | %.3f | %.3f / %.3f | ', ...
    '%.0f | %.3f%% | `%s` | %s |\n'], ...
    arm.name, arm.positionEospa, arm.positionRmse, ...
    arm.interFormationPositionOspa, eventE, eventR, ...
    arm.windowAttemptedPayloadBytes, saving, pulseText, gateText);
end

function complete = isCompleteArm(arm)
complete = isstruct(arm) && isscalar(arm) && ...
    isfield(arm, 'completed') && logical(arm.completed);
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = lowerGainVector(reference, candidate)
value = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = mean(values); end
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
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; ...
        %#ok<AGROW>
end
passed = all(visited);
end

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
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
