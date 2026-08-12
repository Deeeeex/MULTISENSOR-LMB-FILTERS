function test_counterfactual_regret_gate_v133()
% TEST_COUNTERFACTUAL_REGRET_GATE_V133 Protocol and one-page action.

protocol = getCounterfactualRegretGateV133Protocol();
assert(strcmp(protocol.actionContract.payloadMode, 'abstention-only'));
assert(strcmp(protocol.actionContract.wireRepresentation, ...
    'control-synopsis-only'));
assert(~protocol.gnnAuthorized);
assert(protocol.routeExecutionAuthorized);
assert(protocol.trackingOutcomeScoringAuthorized);
assert(~protocol.modelTrainingAuthorized);
assert(protocol.reportingPolicy.failedCandidatesRepositoryOnly);
allSeeds = [protocol.developmentSeeds, protocol.calibrationSeeds, ...
    protocol.validationSeeds];
assert(numel(unique(allSeeds)) == numel(allSeeds));
assert(protocol.scaleCases(1).horizonSteps == 4);
assert(protocol.scaleCases(2).horizonSteps == 6);

[context, adjacency, weights, groupIds] = buildCase('renormalize');
renormalized = scorePayloadAbstentionCounterfactualV133( ...
    context, adjacency, weights, groupIds, 2, protocol);
assert(renormalized.singleRoundExact);
assert(renormalized.runtimeAbstentionSemanticsMatched);
assert(renormalized.runtimeRouteUnchanged);
assert(~renormalized.topologyActionClaimed);
assert(renormalized.abstainedInputCount == 1);
assert(renormalized.abstainedInputMask(3, 2));
assert(~renormalized.counterfactualEnumeratorAdjacency(3, 2));
assert(abs(renormalized. ...
    counterfactualEnumeratorFusionWeights(3, 3) - 0.75) <= 1e-12);
assert(abs(renormalized. ...
    counterfactualEnumeratorFusionWeights(3, 4) - 0.25) <= 1e-12);
assert(all(isfinite([ ...
    renormalized.referenceTaskRisk, ...
    renormalized.counterfactualTaskRisk, ...
    renormalized.existenceRetentionRisk])));
assert(~renormalized.truthUsed && ~renormalized.futureOutcomeUsed);
assertGuaranteedMissingEquivalent( ...
    context, adjacency, weights, renormalized, protocol);

context.triggerConfig.missingNeighborWeightMode = 'self';
selfWeighted = scorePayloadAbstentionCounterfactualV133( ...
    context, adjacency, weights, groupIds, 2, protocol);
assert(abs(selfWeighted. ...
    counterfactualEnumeratorFusionWeights(3, 3) - 0.8) <= 1e-12);
assert(abs(selfWeighted. ...
    counterfactualEnumeratorFusionWeights(3, 4) - 0.2) <= 1e-12);
assertGuaranteedMissingEquivalent( ...
    context, adjacency, weights, selfWeighted, protocol);

fprintf('PASS: V133 counterfactual regret gate contract\n');
end

function assertGuaranteedMissingEquivalent( ...
        context, adjacency, weights, score, protocol)
forcedMissing = context;
forcedMissing.commConfig.pDropByEdge(2, 3) = 1;
options = struct('maximumIncomingCount', ...
    protocol.maximumIncomingCountForOutcomeEnumeration);
[runtimeRisk, runtimeDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        forcedMissing, adjacency, weights, options);
[projectedRisk, projectedDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        forcedMissing, ...
        score.counterfactualEnumeratorAdjacency, ...
        score.counterfactualEnumeratorFusionWeights, options);
assert(abs(runtimeRisk - projectedRisk) <= 1e-12);
[runtimeTaskRisk, runtimeTaskDetails] = ...
    computeExpectedLmbPosteriorTaskRiskV92( ...
        runtimeDetails, [1, 1, 2, 2], forcedMissing.model, ...
        protocol.riskOptions);
[projectedTaskRisk, projectedTaskDetails] = ...
    computeExpectedLmbPosteriorTaskRiskV92( ...
        projectedDetails, [1, 1, 2, 2], forcedMissing.model, ...
        protocol.riskOptions);
assert(abs(runtimeTaskRisk - projectedTaskRisk) <= 1e-12);
assert(max(abs(runtimeTaskDetails.expectedReceiverRisk - ...
    projectedTaskDetails.expectedReceiverRisk)) <= 1e-12);
end

function [context, adjacency, weights, groupIds] = buildCase(mode)
sensorCount = 4;
currentTime = 3;
groupIds = [1, 1, 2, 2];
model = generateMultisensorModel( ...
    sensorCount, zeros(1, sensorCount), ...
    0.9 * ones(1, sensorCount), ...
    2 * ones(1, sensorCount), 'GA', 'LBP');
model.sensorMotionEnabled = true;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = 60 * ones(1, sensorCount);
model.sensorFovRange = 150 * ones(1, sensorCount);
model.sensorFovHeadingRad = zeros(sensorCount, currentTime);
model.sensorTrajectories = cell(1, sensorCount);
positions = [0, 10, 20, 30; 0, 0, 0, 0];
for sensorIdx = 1:sensorCount
    model.sensorTrajectories{sensorIdx} = repmat( ...
        [positions(:, sensorIdx); 1; 0], 1, currentTime);
end
model.sensorQuality = struct('enabled', false);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
model = attachCurrentObservableSensorGeometry( ...
    model, positions, currentTime, sensorCount);

posteriors = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    object = model.birthParameters(1);
    object.birthTime = 1;
    object.birthLocation = 1;
    object.r = 0.65 + 0.05 * sensorIdx;
    object.numberOfGmComponents = 1;
    object.w = 1;
    object.mu = {[15 + sensorIdx; 2 * sensorIdx; 0; 0]};
    object.Sigma = {diag([4, 5, 1, 1])};
    object.associationEntropy = 0;
    object.detectionAssociationEntropy = 0;
    object.detectionAssociationMass = 0.8;
    object.associationAmbiguity = 0.2;
    object.associationConfidence = 0.8;
    object.trajectoryLength = 1;
    object.trajectory = object.mu;
    object.timestamps = 1;
    posteriors{sensorIdx} = object;
end

triggerConfig = buildMixtureAwareKlaReferenceConfig(struct( ...
    'topologyPolicySensorObservationEnabled', true, ...
    'missingNeighborWeightMode', mode, ...
    'missingLabelFusionMode', 'fov-aware-censored'));
context = buildObservableTopologyPolicyContext( ...
    buildFullContext(posteriors, model, triggerConfig, ...
        positions, currentTime));

adjacency = logical([ ...
    0, 1, 0, 0; ...
    1, 0, 0, 0; ...
    0, 1, 0, 1; ...
    0, 0, 1, 0]);
weights = zeros(sensorCount);
weights(adjacency) = 0.2;
weights(1:sensorCount+1:end) = 1 - sum(weights, 2)';
end

function context = buildFullContext( ...
        posteriors, model, triggerConfig, positions, currentTime)
sensorCount = numel(posteriors);
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct('pDropByEdge', 0.1 * ones(sensorCount));
context.triggerConfig = triggerConfig;
context.currentTime = currentTime;
context.previousAdjacency = false(sensorCount);
context.previousAdjacencyHistory = false(sensorCount, sensorCount, 2);
context.previousAdjacencyHistoryCount = 2;
context.previousAdjacencyHistoryTimes = [1, 2];
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
context.previousAdjacencyHistorySource = ...
    'selected-validated-topology';
context.baseAdjacency = false(sensorCount);
context.physicalAdjacency = true(sensorCount) & ~eye(sensorCount);
context.edgeScores = zeros(sensorCount);
context.edgeBudget = 0;
context.directedMessageBudget = 5;
context.positions = positions;
context.localInnovationHistory = zeros(sensorCount, 3);
context.localAssociationConfidenceHistory = zeros(sensorCount, 3);
context.localNisNormHistory = zeros(sensorCount, 3);
context.localNisDeviationHistory = zeros(sensorCount, 3);
context.localUpdateHistoryTimes = 1:currentTime;
end
