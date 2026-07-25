function test_dynamic_topology_scenarios()
% TEST_DYNAMIC_TOPOLOGY_SCENARIOS Scenario, safety and KLA reference tests.

testScenarioPresets();
testTeacherSceneDifficultyGates();
testExplicitSensorHeadingGate();
testScalableTopologyCandidatePools();
testTopologyTaskRiskOrdering();
testTeacherLabelExcludesSwitchPenalty();
testContinuationFromLocalPosteriorSnapshot();
testConstraintAwareScreenDecision();
testContinuationCacheRejectsConfigDrift();
testScheduledBirths();
testTimeVaryingDropAccounting();
testInfeasiblePhysicalGraphFailsClosed();
testMixtureAwareReferenceBoundary();
testD12OracleCallbackSmoke();
fprintf('test_dynamic_topology_scenarios passed\n');
end

function testScenarioPresets()
names = {'r8-legacy', 'd12-handover', ...
    'm24-handover', 'm24-link', 'm24-composite', ...
    'x36-topology', 'x36-joint', 'x36-matched', ...
    'x36-clean-scale', ...
    'd12-hard', 'm24-hard', 'x36-hard'};
expectedSensors = [ ...
    8, 12, 24, 24, 24, 36, 36, 36, 36, 12, 24, 36];
for presetIdx = 1:numel(names)
    rng(7);
    config = buildDynamicTopologyScenarioConfig(names{presetIdx});
    [sensors, ~] = generateMultiFormationTrajectories(config);
    [targets, ~] = generateCorridorTargetTrajectories(config);
    graphs = buildDynamicTopologyGraphs(config, sensors);
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graphs);
    assert(validation.isValid);
    assert(config.numberOfSensors == expectedSensors(presetIdx));
    assert(validation.staticEdgeCount <= config.edgeBudget);
    assert(validation.staticPhysicalViolationCount == 0);
end

rng(7);
config = buildDynamicTopologyScenarioConfig('d12-handover');
[sensors, ~] = generateMultiFormationTrajectories(config);
graphs = buildDynamicTopologyGraphs(config, sensors);
assert(size(graphs.candidateAdjacency, 3) == 48);
for candidateIdx = 1:size(graphs.candidateAdjacency, 3)
    assert(nnz(triu( ...
        graphs.candidateAdjacency(:, :, candidateIdx), 1)) == 14);
end
end

function testTeacherSceneDifficultyGates()
names = { ...
    'd12-hard', 'm24-hard', 'x36-matched', ...
    'x36-clean-scale', 'x36-hard'};
for presetIdx = 1:numel(names)
    rng(17);
    config = buildDynamicTopologyScenarioConfig(names{presetIdx});
    [sensors, ~] = generateMultiFormationTrajectories(config);
    [targets, ~] = generateCorridorTargetTrajectories(config);
    graphs = buildDynamicTopologyGraphs(config, sensors);
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graphs);
    difficulty = validation.difficulty;
    requirements = config.difficultyRequirements;
    assert(difficulty.blackoutFraction <= ...
        requirements.maxBlackoutFraction + 1e-12);
    assert(difficulty.singleFormationFraction >= ...
        requirements.minSingleFormationFraction - 1e-12);
    assert(difficulty.multiFormationFraction >= ...
        requirements.minMultiFormationFraction - 1e-12);
    assert(difficulty.focusHandovers >= ...
        requirements.minFocusHandovers);
    assert(difficulty.focusCloseEncounterTimeFraction >= ...
        requirements.minCrossGroupCloseEncounterFraction - 1e-12);
    assert(difficulty.formationOwnershipEntropy >= ...
        requirements.minFormationOwnershipEntropy - 1e-12);
    assert(difficulty.blockageFocusOverlapFraction >= ...
        requirements.minBlockageFocusOverlapFraction - 1e-12);
end

rng(17);
config = buildDynamicTopologyScenarioConfig( ...
    'd12-hard', struct('fovRange', 40));
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graphs = buildDynamicTopologyGraphs(config, sensors);
failedClosed = false;
try
    validateDynamicTopologyScenario(config, sensors, targets, graphs);
catch errorInfo
    failedClosed = ~isempty(strfind( ...
        errorInfo.message, 'difficulty-blackout')); %#ok<STREMP>
end
assert(failedClosed);
end

function testExplicitSensorHeadingGate()
model = struct();
model.detectionProbability = 0.9;
model.Q = {eye(2)};
model.sensorTrajectories = {zeros(4, 1)};
model.sensorMotionEnabled = true;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = 30;
model.sensorFovRange = 100;
model.sensorFovHeadingRad = 0;

[forwardPd, ~, forwardInfo] = evaluateSensorQuality( ...
    model, 1, [50; 0; 0; 0], 1);
[sidePd, ~, sideInfo] = evaluateSensorQuality( ...
    model, 1, [0; 50; 0; 0], 1);
assert(forwardPd > 0 && forwardInfo.inFov);
assert(sidePd == 0 && ~sideInfo.inFov);

model.sensorFovHeadingRad = pi / 2;
[rotatedPd, ~, rotatedInfo] = evaluateSensorQuality( ...
    model, 1, [0; 50; 0; 0], 1);
assert(rotatedPd > 0 && rotatedInfo.inFov);
end

function testScalableTopologyCandidatePools()
names = { ...
    'm24-hard', 'x36-matched', 'x36-clean-scale', 'x36-hard'};
for presetIdx = 1:numel(names)
    rng(23);
    config = buildDynamicTopologyScenarioConfig(names{presetIdx});
    [sensors, ~] = generateMultiFormationTrajectories(config);
    graphs = buildDynamicTopologyGraphs(config, sensors);
    context = makeCandidatePoolContext(config, sensors, graphs);
    [candidates, metadata] = ...
        buildDynamicTopologyCandidatePool(context);
    assert(strcmp(metadata.source, 'projected-general'));
    assert(size(candidates, 3) >= 5);
    sameGroup = bsxfun(@eq, ...
        config.sensorGroupIds(:), config.sensorGroupIds(:)');
    for candidateIdx = 1:size(candidates, 3)
        candidate = candidates(:, :, candidateIdx);
        assert(nnz(triu(candidate, 1)) == config.edgeBudget);
        assert(max(sum(candidate, 2)) <= config.maxNodeDegree);
        assert(max(sum(candidate & ~sameGroup, 2)) <= ...
            config.maxInterFormationDegree);
        assert(isConnectedTest(candidate));
    end
    context.previousAdjacency = graphs.staticAdjacency;
    [localCandidates, ~] = ...
        buildDynamicTopologyCandidatePool(context);
    removable = zeros(1, size(localCandidates, 3));
    for candidateIdx = 1:size(localCandidates, 3)
        removable(candidateIdx) = nnz(triu( ...
            context.previousAdjacency & ...
            ~localCandidates(:, :, candidateIdx), 1));
    end
    assert(sum(removable <= ...
        config.maxEdgeReplacementsPerStep) >= 5);
    for policyMode = {'reliability', 'discrepancy'}
        [policyAdjacency, policyDetails] = ...
            selectProjectedTopologyPolicy( ...
                context, policyMode{1});
        assert(policyDetails.validCandidateCount >= 1);
        assert(nnz(triu(policyAdjacency, 1)) == ...
            config.edgeBudget);
        assert(isConnectedTest(policyAdjacency));
    end
end
end

function testTopologyTaskRiskOrdering()
model = generateMultisensorModel( ...
    1, 0, 0.9, 3, 'GA', 'LBP');
model.birthTimeByLocation = 1;
model.dynamicTopologyScenario = struct( ...
    'config', struct( ...
        'targetSpeedLimit', 15, ...
        'ospaPositionCutoff', 100), ...
    'targetTrajectories', {{[zeros(4, 1), zeros(4, 1)]}});
good = makeObject(model, 1, 1, 0.95, ...
    [1; 0; 0; 0], eye(4));
poor = makeObject(model, 1, 1, 0.45, ...
    [80; 60; 8; -8], 100 * eye(4));
goodRisk = evaluateLmbTopologyTaskRisk( ...
    good, model, 1, struct('horizonSteps', 0));
poorRisk = evaluateLmbTopologyTaskRisk( ...
    poor, model, 1, struct('horizonSteps', 0));
assert(goodRisk < poorRisk);
meanRisk = evaluateLmbTopologyTaskRisk( ...
    {good, poor}, model, 1, struct( ...
        'horizonSteps', 0, ...
        'sensorAggregationMode', 'mean'));
riskSensitive = evaluateLmbTopologyTaskRisk( ...
    {good, poor}, model, 1, struct( ...
        'horizonSteps', 0, ...
        'sensorAggregationMode', 'mean-cvar', ...
        'sensorCvarFraction', 0.5, ...
        'sensorCvarWeight', 0.5));
assert(riskSensitive > meanRisk);
end

function testTeacherLabelExcludesSwitchPenalty()
inputs = generateDynamicTopologyScenarioInputs('d12-hard', 19);
timeIdx = 50;
sensorCount = inputs.config.numberOfSensors;
truth = inputs.targetTrajectories{1}(:, timeIdx);
posteriors = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    shift = [0; 0; 0; 0];
    if mod(sensorIdx, 3) == 0
        shift = [90; -50; 4; -2];
    end
    posteriors{sensorIdx} = makeObject( ...
        inputs.model, ...
        inputs.model.birthTimeByLocation(1), 1, ...
        0.85 - 0.1 * mod(sensorIdx, 2), ...
        truth + shift, (4 + sensorIdx) * eye(4));
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = inputs.model;
context.commConfig = inputs.commConfig;
context.currentTime = timeIdx;
context.previousAdjacency = inputs.graphData.staticAdjacency;
context.baseAdjacency = inputs.graphData.staticAdjacency;
context.physicalAdjacency = ...
    inputs.graphData.physicalAdjacency(:, :, timeIdx);
context.edgeScores = double(context.physicalAdjacency);
context.edgeBudget = inputs.config.edgeBudget;
context.positions = zeros(2, sensorCount);
for sensorIdx = 1:sensorCount
    context.positions(:, sensorIdx) = ...
        inputs.sensorTrajectories{sensorIdx}(1:2, timeIdx);
end

zeroPenalty = buildMixtureAwareKlaReferenceConfig(struct( ...
    'topologyTeacherSwitchPenaltyWeight', 0));
context.triggerConfig = zeroPenalty;
[~, zeroDetails] = ...
    selectCounterfactualTopologyTeacher(context, 'current');
highPenalty = zeroPenalty;
highPenalty.topologyTeacherSwitchPenaltyWeight = 10;
context.triggerConfig = highPenalty;
[~, highDetails] = ...
    selectCounterfactualTopologyTeacher(context, 'current');
finite = isfinite(zeroDetails.candidateTaskRisks) & ...
    isfinite(highDetails.candidateTaskRisks);
assert(any(finite));
assert(max(abs( ...
    zeroDetails.candidateTaskRisks(finite) - ...
    highDetails.candidateTaskRisks(finite))) < 1e-12);
assert(~zeroDetails.labelIncludesSwitchPenalty);
assert(~highDetails.labelIncludesSwitchPenalty);
assert(zeroDetails.taskRiskSpread > 0);

balancedConfig = zeroPenalty;
balancedConfig.topologyTeacherRiskOptions = struct( ...
    'sensorAggregationMode', 'mean-cvar', ...
    'sensorCvarFraction', 0.25, ...
    'sensorCvarWeight', 0.5);
context.triggerConfig = balancedConfig;
[~, balancedDetails] = ...
    selectCounterfactualTopologyTeacher(context, 'current');
balancedFinite = isfinite(balancedDetails.candidateTaskRisks);
assert(any(balancedFinite));
for candidateIdx = find(balancedFinite)
    nodeRisk = balancedDetails.candidateNodeTaskRisks( ...
        candidateIdx, :);
    nodeRisk = nodeRisk(isfinite(nodeRisk));
    tailCount = max(1, ceil(0.25 * numel(nodeRisk)));
    sortedRisk = sort(nodeRisk, 'descend');
    expectedRisk = 0.5 * mean(nodeRisk) + ...
        0.5 * mean(sortedRisk(1:tailCount));
    assert(abs(expectedRisk - ...
        balancedDetails.candidateTaskRisks(candidateIdx)) < 1e-12);
end
sharedFinite = balancedFinite & ...
    isfinite(zeroDetails.candidateTaskRisks);
assert(any(abs( ...
    balancedDetails.candidateTaskRisks(sharedFinite) - ...
    zeroDetails.candidateTaskRisks(sharedFinite)) > 1e-8));

registered = inputs.graphData.candidateAdjacency;
removedFromBase = zeros(1, size(registered, 3));
for candidateIdx = 1:size(registered, 3)
    removedFromBase(candidateIdx) = nnz(triu( ...
        inputs.graphData.staticAdjacency & ...
        ~registered(:, :, candidateIdx), 1));
end
[maximumRemoved, farCandidateIdx] = max(removedFromBase);
assert(maximumRemoved > ...
    inputs.config.maxEdgeReplacementsPerStep);
context.previousAdjacency = registered(:, :, farCandidateIdx);
context.triggerConfig = zeroPenalty;
[~, farDetails] = ...
    selectCounterfactualTopologyTeacher(context, 'current');
assert(isfinite(farDetails.baselineTaskRisk));
assert(~farDetails.selectionValidCandidates( ...
    farDetails.baselineCandidateIndex));
allCandidateConfig = zeroPenalty;
allCandidateConfig.topologyTeacherEvaluateAllCandidates = true;
context.triggerConfig = allCandidateConfig;
[~, allCandidateDetails] = ...
    selectCounterfactualTopologyTeacher(context, 'current');
assert(allCandidateDetails.candidateIndex == ...
    farDetails.candidateIndex);
evaluatedByDefault = isfinite(farDetails.candidateTaskRisks);
assert(max(abs( ...
    farDetails.candidateTaskRisks(evaluatedByDefault) - ...
    allCandidateDetails.candidateTaskRisks(evaluatedByDefault))) < 1e-12);
assert(sum(isfinite(allCandidateDetails.candidateTaskRisks)) > ...
    sum(evaluatedByDefault));

context.previousAdjacency = inputs.graphData.staticAdjacency;
zeroPenalty.topologyTeacherClosedLoopHorizonSteps = 1;
context.triggerConfig = zeroPenalty;
rolloutData = struct( ...
    'measurements', {inputs.measurements}, ...
    'continuationAdjacency', inputs.graphData.staticAdjacency);
[~, zeroClosedDetails] = ...
    selectClosedLoopCounterfactualTopologyTeacher( ...
        context, rolloutData);
highPenalty = zeroPenalty;
highPenalty.topologyTeacherSwitchPenaltyWeight = 10;
context.triggerConfig = highPenalty;
[~, highClosedDetails] = ...
    selectClosedLoopCounterfactualTopologyTeacher( ...
        context, rolloutData);
finiteClosed = isfinite(zeroClosedDetails.candidateTaskRisks) & ...
    isfinite(highClosedDetails.candidateTaskRisks);
assert(any(finiteClosed));
assert(max(abs( ...
    zeroClosedDetails.candidateTaskRisks(finiteClosed) - ...
    highClosedDetails.candidateTaskRisks(finiteClosed))) < 1e-12);
assert(zeroClosedDetails.usesFutureMeasurements);
assert(~zeroClosedDetails.labelIncludesSwitchPenalty);
end

function testContinuationFromLocalPosteriorSnapshot()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
for birthIdx = 1:numel(model.birthParameters)
    model.birthParameters(birthIdx).r = 0.9;
end
measurements = repmat({{}}, 2, 3);
neighborMap = {[1, 2], [1, 2]};
commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropBySensor', [0, 0]);
fullConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'topologyPosteriorCaptureTimes', 2);
[fullEstimates, fullDiagnostics] = ...
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, [], neighborMap, ...
        commConfig, fullConfig);
snapshot = ...
    fullDiagnostics.topologyLocalPosteriorSnapshot{2};
assert(numel(snapshot) == 2);

continuationConfig = fullConfig;
continuationConfig.topologyPosteriorCaptureTimes = [];
continuationConfig.filterInitialLocalPosteriorBySensor = snapshot;
continuationConfig.filterInitialLocalPosteriorTime = 2;
continuationConfig.filterInitialPreviousAdjacency = [0, 1; 1, 0];
[continuationEstimates, continuationDiagnostics] = ...
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, [], neighborMap, ...
        commConfig, continuationConfig);
assert(continuationDiagnostics.usedInitialLocalPosterior);
assert(continuationDiagnostics.continuationStartTime == 2);
assert(isequal( ...
    continuationDiagnostics.continuationPreviousDirectedEdgeMask, ...
    logical([0, 1; 1, 0])));
for sensorIdx = 1:2
    for timeIdx = 2:3
        assert(isequal( ...
            fullEstimates{sensorIdx}.labels{timeIdx}, ...
            continuationEstimates{sensorIdx}.labels{timeIdx}));
        assertStateCellsEqual( ...
            fullEstimates{sensorIdx}.mu{timeIdx}, ...
            continuationEstimates{sensorIdx}.mu{timeIdx});
        assertStateCellsEqual( ...
            fullEstimates{sensorIdx}.Sigma{timeIdx}, ...
            continuationEstimates{sensorIdx}.Sigma{timeIdx});
    end
end
assert(isequal( ...
    fullDiagnostics.topologyActiveEdge(:, :, 2:3), ...
    continuationDiagnostics.topologyActiveEdge(:, :, 2:3)));
fullAttempted = ...
    fullDiagnostics.attemptedPayloadBytes(:, :, 2:3);
continuationAttempted = ...
    continuationDiagnostics.attemptedPayloadBytes(:, :, 2:3);
assert(sum(fullAttempted(:)) == sum(continuationAttempted(:)));
end

function assertStateCellsEqual(left, right)
assert(numel(left) == numel(right));
for idx = 1:numel(left)
    assert(norm(left{idx} - right{idx}, 'fro') < 1e-12);
end
end

function testConstraintAwareScreenDecision()
options = struct( ...
    'maxTimeSteps', 1, ...
    'armNames', {{'robust-static', 'local'}}, ...
    'writeReport', false);
[~, summary] = runDynamicTopologyOracleGapScreen( ...
    'd12-handover', 7, options);
assert(summary.decision.constraintEligibleArmCount == 1);
assert(strcmp( ...
    summary.decision.bestObservedArm, ...
    'All-time geometry static'));
assert(strcmp( ...
    summary.decision.status, ...
    'stop-no-observed-dynamic-gain'));
assert(summary.decision. ...
    minimumPracticalTrackingGainPercent == 5);
end

function testContinuationCacheRejectsConfigDrift()
cacheDirectory = tempname();
mkdir(cacheDirectory);
cleanup = onCleanup(@() cleanupDirectory(cacheDirectory)); %#ok<NASGU>
teacherOptions = struct( ...
    'snapshotTimes', 1, ...
    'teacherHorizonSteps', 0, ...
    'closedLoopHorizonSteps', 0, ...
    'behaviorCacheDirectory', cacheDirectory, ...
    'writeReport', false);
runDynamicTopologyTeacherSignalScreen( ...
    'd12-handover', 31, teacherOptions);

continuationOptions = struct( ...
    'maxTimeSteps', 1, ...
    'continuationStartTime', 1, ...
    'armNames', {{'robust-static'}}, ...
    'behaviorCacheDirectory', cacheDirectory, ...
    'writeReport', false);
runDynamicTopologyOracleGapScreen( ...
    'd12-handover', 31, continuationOptions);

continuationOptions.scenarioOverrides = struct( ...
    'measurementNoiseStd', 9);
rejected = false;
try
    runDynamicTopologyOracleGapScreen( ...
        'd12-handover', 31, continuationOptions);
catch errorInfo
    rejected = ~isempty(strfind( ...
        errorInfo.message, ...
        'metadata or posterior snapshot mismatches')); %#ok<STREMP>
end
assert(rejected);

continuationOptions.scenarioOverrides = struct();
continuationOptions.fusionOverrides = struct( ...
    'mixtureAwareTopComponents', 2);
rejected = false;
try
    runDynamicTopologyOracleGapScreen( ...
        'd12-handover', 31, continuationOptions);
catch errorInfo
    rejected = ~isempty(strfind( ...
        errorInfo.message, ...
        'metadata or posterior snapshot mismatches')); %#ok<STREMP>
end
assert(rejected);
end

function cleanupDirectory(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
end

function testScheduledBirths()
inputs = generateDynamicTopologyScenarioInputs('d12-handover', 7);
model = inputs.model;
objects = model.object;
objects = lmbPredictionStep(objects, model, 1);
assert(numel(objects) == 3);
objects = lmbPredictionStep(objects, model, 2);
assert(numel(objects) == 3);
for timeIdx = 3:11
    objects = lmbPredictionStep(objects, model, timeIdx);
end
assert(numel(objects) == 6);
birthTimes = sort([objects.birthTime]);
assert(isequal(birthTimes, [1, 1, 1, 11, 11, 11]));
end

function testTimeVaryingDropAccounting()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
for birthIdx = 1:numel(model.birthParameters)
    model.birthParameters(birthIdx).r = 0.9;
end
measurements = repmat({{}}, 2, 2);
neighborMap = {[1, 2], [1, 2]};
pDrop = zeros(2, 2, 2);
pDrop(:, :, 2) = 1;
commConfig = struct( ...
    'pDropBySensor', [0, 0], ...
    'pDropByEdge', pDrop, ...
    'linkUniforms', 0.5 * ones(2, 2, 2));
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, [], neighborMap, commConfig, triggerConfig);
assert(diagnostics.summary.attemptCount == 4);
assert(diagnostics.summary.deliveryCount == 2);
assert(diagnostics.summary.attemptedPayloadBytes > ...
    diagnostics.summary.payloadBytes);
assert(diagnostics.summary.payloadBytes > 0);
end

function testInfeasiblePhysicalGraphFailsClosed()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
model.sensorCommRange = 1;
measurements = repmat({{}}, 2, 2);
sensorTrajectories = { ...
    repmat([0; 0; 0; 0], 1, 2), ...
    repmat([100; 0; 0; 0], 1, 2)};
neighborMap = {[1, 2], [1, 2]};
commConfig = struct('forceDelivery', true, 'pDropBySensor', [0, 0]);
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'dynamicTopologyEnabled', true, ...
    'dynamicTopologyEdgeBudget', 1);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
assert(all(diagnostics.topologyUndirectedEdgeCount == 0));
assert(all(~diagnostics.topologyFeasible));
assert(diagnostics.summary.attemptCount == 0);
end

function testMixtureAwareReferenceBoundary()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
left = makeObject(model, 1, 1, 0.8, ...
    [-2; 0; 0; 0], 2 * eye(4));
right = makeObject(model, 1, 1, 0.8, ...
    [2; 0; 0; 0], 3 * eye(4));
config = buildMixtureAwareKlaReferenceConfig();
details = struct('eventType', [0, 2]);
fused = fuseLmbPosteriorsByLabel( ...
    {left, right}, [0.5, 0.5], model, [0.5, 0.5], ...
    details, config);
expectedCovariance = inv( ...
    0.5 * inv(left.Sigma{1}) + 0.5 * inv(right.Sigma{1}));
expectedMean = expectedCovariance * ( ...
    0.5 * (left.Sigma{1} \ left.mu{1}) + ...
    0.5 * (right.Sigma{1} \ right.mu{1}));
assert(fused.numberOfGmComponents == 1);
assert(norm(fused.mu{1} - expectedMean) < 1e-10);
assert(norm(fused.Sigma{1} - expectedCovariance, 'fro') < 1e-10);

bimodal = makeObject(model, 1, 2, 0.8, ...
    [-8; 0; 0; 0], eye(4));
bimodal.numberOfGmComponents = 2;
bimodal.w = [0.5, 0.5];
bimodal.mu = {[-8; 0; 0; 0], [8; 0; 0; 0]};
bimodal.Sigma = {eye(4), eye(4)};
fixedPoint = fuseLmbPosteriorsByLabel( ...
    {bimodal, bimodal}, [0.5, 0.5], model, [0.5, 0.5], ...
    details, config);
assert(fixedPoint.numberOfGmComponents >= 2);
[meanVector, covariance] = objectMoments(fixedPoint);
[referenceMean, referenceCovariance] = objectMoments(bimodal);
assert(norm(meanVector - referenceMean) < 1e-6);
assert(norm(covariance - referenceCovariance, 'fro') < 1e-3);
assert(abs(fixedPoint.r - bimodal.r) < 1e-3);
end

function testD12OracleCallbackSmoke()
inputs = generateDynamicTopologyScenarioInputs('d12-handover', 13);
inputs.measurements = inputs.measurements(:, 1);
inputs.sensorTrajectories = cellfun( ...
    @(x) x(:, 1), inputs.sensorTrajectories, ...
    'UniformOutput', false);
inputs.commConfig.pDropByEdge = ...
    inputs.commConfig.pDropByEdge(:, :, 1);
inputs.commConfig.linkUniforms = ...
    inputs.commConfig.linkUniforms(:, :, 1);
config = buildMixtureAwareKlaReferenceConfig();
config.dynamicTopologyEnabled = true;
config.dynamicTopologyEdgeBudget = inputs.config.edgeBudget;
config.topologyPolicyName = 'oracle-consensus';
config.topologyPolicyFcn = ...
    @(context) selectD12TopologyPolicy(context, 'oracle-consensus');
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    inputs.model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, config);
assert(diagnostics.topologyFeasible(1));
assert(diagnostics.topologyUndirectedEdgeCount(1) == ...
    inputs.config.edgeBudget);
assert(isfinite(diagnostics.topologyPolicyCandidateIndex(1)));
assert(isfinite(diagnostics.topologyPolicyObjective(1)));

uncachedConfig = config;
uncachedConfig.topologyOracleFusionCacheEnabled = false;
[~, uncachedDiagnostics] = runEventTriggeredDistributedLmbFilter( ...
    inputs.model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, uncachedConfig);
assert(uncachedDiagnostics.topologyPolicyCandidateIndex(1) == ...
    diagnostics.topologyPolicyCandidateIndex(1));
assert(abs(uncachedDiagnostics.topologyPolicyObjective(1) - ...
    diagnostics.topologyPolicyObjective(1)) < 1e-10);
end

function object = makeObject(model, birthTime, birthLocation, r, mu, Sigma)
object = model.birthParameters(1);
object.birthTime = birthTime;
object.birthLocation = birthLocation;
object.r = r;
object.numberOfGmComponents = 1;
object.w = 1;
object.mu = {mu};
object.Sigma = {Sigma};
end

function [meanVector, covariance] = objectMoments(object)
weights = reshape(object.w, 1, []);
weights = weights / sum(weights);
meanVector = zeros(size(object.mu{1}));
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(size(object.Sigma{1}));
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
end

function context = makeCandidatePoolContext(config, sensors, graphs)
sensorCount = config.numberOfSensors;
scenario = struct( ...
    'config', config, ...
    'candidateAdjacency', graphs.candidateAdjacency);
context = struct();
context.model = struct( ...
    'dynamicTopologyScenario', scenario, ...
    'xDimension', 4);
context.physicalAdjacency = graphs.physicalAdjacency(:, :, 1);
context.baseAdjacency = graphs.staticAdjacency;
context.previousAdjacency = false(sensorCount);
context.edgeBudget = config.edgeBudget;
context.edgeScores = double(context.physicalAdjacency);
context.positions = zeros(2, sensorCount);
for sensorIdx = 1:sensorCount
    context.positions(:, sensorIdx) = ...
        sensors{sensorIdx}(1:2, 1);
end
context.localPosteriorBySensor = ...
    repmat({struct([])}, 1, sensorCount);
context.commConfig = struct( ...
    'pDropByEdge', zeros(sensorCount));
context.currentTime = 1;
end

function connected = isConnectedTest(adjacency)
nodeCount = size(adjacency, 1);
visited = false(1, nodeCount);
queue = 1;
visited(1) = true;
while ~isempty(queue)
    node = queue(1);
    queue(1) = [];
    neighbors = find(adjacency(node, :) & ~visited);
    visited(neighbors) = true;
    queue = [queue, neighbors]; %#ok<AGROW>
end
connected = all(visited);
end
