function test_joint_source_trust_routing()
% TEST_JOINT_SOURCE_TRUST_ROUTING Focused v8/v9 projector and routing tests.

testVectorDominantWeights();
testFiniteModeProjector();
testValueGateMask();
testFormationTailGuard();
testX36SelectorIntegration();
fprintf('test_joint_source_trust_routing passed\n');
end

function testFormationTailGuard()
groupIds = [1, 1, 2, 2];
nodeCount = numel(groupIds);
modeCount = 2;
pairRiskByMode = zeros(nodeCount, nodeCount, modeCount, modeCount);
for leftIdx = 1:(nodeCount - 1)
    for rightIdx = (leftIdx + 1):nodeCount
        sameFormation = groupIds(leftIdx) == groupIds(rightIdx);
        if sameFormation && groupIds(leftIdx) == 1
            modeRisk = [1, 1; 1, 3];
        elseif sameFormation
            modeRisk = [1, 1; 1, 0];
        else
            modeRisk = [1, 0.5; 0.5, 0.5];
        end
        for leftMode = 1:modeCount
            for rightMode = 1:modeCount
                value = modeRisk(leftMode, rightMode);
                pairRiskByMode( ...
                    leftIdx, rightIdx, leftMode, rightMode) = value;
                pairRiskByMode( ...
                    rightIdx, leftIdx, rightMode, leftMode) = value;
            end
        end
    end
end
objective = repmat([0, 5], 2, 1);
payload = ones(2, modeCount);
baseOptions = struct( ...
    'referenceModeIndex', 1, ...
    'riskLimit', 1, ...
    'primaryPayloadLimitBytes', 2, ...
    'maximumPayloadLimitBytes', 2, ...
    'modeTrustWeights', [0.70, 0.50]);
unprotected = selectJointSourceTrustFormationModes( ...
    pairRiskByMode, groupIds, objective, payload, baseOptions);
assert(isequal(unprotected, [2, 2]));

guardedOptions = baseOptions;
guardedOptions.formationRiskGuardMode = 'reference-mean-tail';
guardedOptions.formationRiskTailFraction = 0.5;
guardedOptions.formationRiskTailWeight = 1;
guardedOptions.formationRiskToleranceFraction = 0;
[guarded, details] = selectJointSourceTrustFormationModes( ...
    pairRiskByMode, groupIds, objective, payload, guardedOptions);
assert(isequal(guarded, [1, 2]));
assert(details.formationRiskGuardEnabled);
assert(details.formationRiskRejectedCandidateCount >= 1);
assert(details.formationRiskConstraintPassed);
assert(all(details.selectedFormationRisk <= ...
    details.formationRiskLimit + 1e-12));
assert(strcmp(details.contractVersion, ...
    'joint-source-trust-formation-tail-projector-v2'));
end

function testValueGateMask()
groupIds = [1, 1, 2, 2];
nodeCount = numel(groupIds);
modeCount = 3;
pairRiskByMode = ones( ...
    nodeCount, nodeCount, modeCount, modeCount);
for nodeIdx = 1:nodeCount
    pairRiskByMode(nodeIdx, nodeIdx, :, :) = 0;
end
objective = repmat([0, 5, 10], 2, 1);
payload = ones(2, modeCount);
allowedModeMask = [true, false, false; true, true, true];
[selectedModes, details] = selectJointSourceTrustFormationModes( ...
    pairRiskByMode, groupIds, objective, payload, struct( ...
        'referenceModeIndex', 1, ...
        'riskLimit', 1, ...
        'primaryPayloadLimitBytes', 2, ...
        'maximumPayloadLimitBytes', 2, ...
        'modeTrustWeights', [0.70, 0.30, 0.70], ...
        'allowedModeMask', allowedModeMask));
assert(isequal(selectedModes, [1, 3]));
assert(details.evaluatedCandidateCount == 9);
assert(details.eligibleCandidateCount == 3);
assert(isequal(details.allowedModeMask, allowedModeMask));
end

function testVectorDominantWeights()
nodeCount = 4;
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.physicalAdjacency = true(nodeCount);
context.physicalAdjacency(1:(nodeCount + 1):end) = false;
dominantAdjacency = false(nodeCount);
residualAdjacency = false(nodeCount);
for receiverIdx = 1:nodeCount
    dominantAdjacency(receiverIdx, mod(receiverIdx, nodeCount) + 1) = true;
    residualAdjacency(receiverIdx, mod(receiverIdx - 2, nodeCount) + 1) = true;
end
[scalarAdjacency, scalarWeights, scalarDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, 0.70, 0.05);
[vectorAdjacency, vectorWeights, vectorDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        0.70 * ones(1, nodeCount), 0.05);
assert(isequal(scalarAdjacency, vectorAdjacency));
assert(max(abs(scalarWeights(:) - vectorWeights(:))) < 1e-12);
assert(max(abs(scalarDetails.fusionWeightMatrix(:) - ...
    vectorDetails.fusionWeightMatrix(:))) < 1e-12);

dominantWeights = [0.30, 0.50, 0.70, 0.30];
[~, ~, heterogeneousDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        dominantWeights, 0.05);
assert(max(abs(sum( ...
    heterogeneousDetails.fusionWeightMatrix, 2) - 1)) < 1e-12);
assert(isequal(heterogeneousDetails.dominantWeightsByReceiver, ...
    dominantWeights));
assert(max(abs(heterogeneousDetails.selfWeightsByReceiver - ...
    (1 - dominantWeights - 0.05))) < 1e-12);
assert(all(heterogeneousDetails.fusionWeightMatrix(:) >= 0));
end

function testFiniteModeProjector()
groupIds = [1, 1, 2, 2];
nodeCount = numel(groupIds);
modeCount = 3;
modePairRisk = [ ...
    2.0, 1.5, 3.0; ...
    1.5, 1.0, 3.0; ...
    3.0, 3.0, 3.0];
pairRiskByMode = zeros(nodeCount, nodeCount, modeCount, modeCount);
for leftIdx = 1:(nodeCount - 1)
    for rightIdx = (leftIdx + 1):nodeCount
        for leftMode = 1:modeCount
            for rightMode = 1:modeCount
                value = modePairRisk(leftMode, rightMode);
                pairRiskByMode( ...
                    leftIdx, rightIdx, leftMode, rightMode) = value;
                pairRiskByMode( ...
                    rightIdx, leftIdx, rightMode, leftMode) = value;
            end
        end
    end
end
objective = repmat([0, 5, 10], 2, 1);
payload = ones(2, modeCount);
[selectedModes, details] = selectJointSourceTrustFormationModes( ...
    pairRiskByMode, groupIds, objective, payload, struct( ...
        'referenceModeIndex', 1, ...
        'riskLimit', 2, ...
        'primaryPayloadLimitBytes', 2, ...
        'maximumPayloadLimitBytes', 2, ...
        'modeTrustWeights', [0.70, 0.30, 0.70]));
assert(isequal(selectedModes, [2, 2]));
assert(details.selectedRisk < details.referenceRisk);
assert(details.evaluatedCandidateCount == 9);
assert(~details.payloadRelaxationUsed);
assert(details.constraintPassed);

fallbackPayload = repmat([5, 6, 6], 2, 1);
[fallbackModes, fallbackDetails] = ...
    selectJointSourceTrustFormationModes( ...
        pairRiskByMode, groupIds, objective, fallbackPayload, struct( ...
            'referenceModeIndex', 1, ...
            'riskLimit', 2, ...
            'primaryPayloadLimitBytes', 9, ...
            'maximumPayloadLimitBytes', 10, ...
            'modeTrustWeights', [0.70, 0.30, 0.70]));
assert(isequal(fallbackModes, [1, 1]));
assert(fallbackDetails.payloadRelaxationUsed);
assert(fallbackDetails.referenceActionFeasible);
assert(fallbackDetails.selectedPayloadBytes == 10);
end

function testX36SelectorIntegration()
groupIds = repelem(1:6, 6);
context = makeContext(groupIds);
nodeCount = numel(groupIds);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.localInnovationHistory = zeros(nodeCount, 1);
context.localAssociationConfidenceHistory = ones(nodeCount, 1);
context.localNisNormHistory = nan(nodeCount, 1);
context.localNisDeviationHistory = nan(nodeCount, 1);
context.localUpdateHistoryTimes = context.currentTime;
for sensorIdx = 1:nodeCount
    context.localPosteriorBySensor{sensorIdx} = makeObject( ...
        context.model, 1, 1, 0.8, zeros(4, 1), eye(4));
end
senderPayloadBytes = 10000 * ones(1, nodeCount);
for groupId = 1:6
    members = find(groupIds == groupId);
    senderPayloadBytes(members(1)) = 100;
end
[adjacency, details] = selectLabelSetAdaptiveDominantRoutingPolicy( ...
    context, ['composite-balanced-certified-overlap-', ...
        'consensus-joint-trust-payload-aware'], struct( ...
            'payloadMarginFraction', 0.02, ...
            'candidateDominantWeightGrid', [0.30, 0.50, 0.70], ...
            'consensusAggregationMode', 'mean', ...
            'consensusTailFraction', 0.25, ...
            'maximumConsensusProjectionFormationCount', 8, ...
            'consensusPayloadFallbackMode', 'reference-cap', ...
            'senderPayloadBytes', senderPayloadBytes));
assert(nnz(adjacency) >= 42 && nnz(adjacency) <= 60);
assert(details.jointTrustProjectionEnabled);
assert(details.consensusProjectionConstraintPassed);
assert(details.sensorWindowStrongConnected);
assert(details.formationWindowStrongConnected);
assert(details.consensusProjection.exactFiniteModeProjection);
assert(details.consensusProjection.exactEnumeration);
assert(details.consensusProjection.singleRoundExact);
assert(details.consensusProjection.evaluatedCandidateCount == 4096);
assert(details.consensusProjection.selectedRisk <= ...
    details.consensusProjection.referenceRisk + 1e-12);
assert(details.payloadLimitPassed);
assert(details.selectedPayloadBytes <= details.baselinePayloadBytes + 1e-12);
assert(abs(details.selectedPayloadBytes - ...
    sum(sum(adjacency, 1) .* senderPayloadBytes)) < 1e-12);
assert(max(abs(sum(details.fusionWeightMatrix, 2) - 1)) < 1e-12);
assert(all(ismember(round(100 * ...
    details.selectedDominantWeightsByReceiver), [30, 50, 70])));
assert(details.jointTrustReferenceFormationCount + ...
    details.jointTrustLowTrustFormationCount + ...
    details.jointTrustFullTrustCandidateFormationCount == 6);
protocol = getLabelSetSimulatorPolicyProtocol();
assert(strcmp(details.contractVersion, ...
    protocol.adaptiveDominantJointTrustRoutingContractVersion));
assert(~details.truthUsed);

[valueGatedAdjacency, valueGatedDetails] = ...
    selectLabelSetAdaptiveDominantRoutingPolicy( ...
        context, ['composite-balanced-certified-overlap-', ...
            'consensus-joint-trust-value-gated-payload-aware'], struct( ...
                'payloadMarginFraction', 0.02, ...
                'candidateDominantWeightGrid', [0.30, 0.50, 0.70], ...
                'minimumDynamicObjectiveAdvantage', 0, ...
                'consensusAggregationMode', 'mean', ...
                'consensusTailFraction', 0.25, ...
                'maximumConsensusProjectionFormationCount', 8, ...
                'consensusPayloadFallbackMode', 'reference-cap', ...
                'senderPayloadBytes', senderPayloadBytes));
assert(nnz(valueGatedAdjacency) >= 42 && ...
    nnz(valueGatedAdjacency) <= 60);
assert(valueGatedDetails.consensusProjection.valueGateEnabled);
assert(valueGatedDetails.minimumDominantObjectiveAdvantage >= -1e-12);
assert(strcmp(valueGatedDetails.contractVersion, protocol. ...
    adaptiveDominantJointTrustStabilityRoutingContractVersion));
assert(~valueGatedDetails.truthUsed);
end

function context = makeContext(groupIds)
sensorCount = numel(groupIds);
model = generateMultisensorModel( ...
    sensorCount, [0, 0], 0.9 * ones(1, sensorCount), ...
    3 * ones(1, sensorCount), 'GA', 'LBP');
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, sensorCount);
context.model = model;
context.commConfig = struct('pDropByEdge', zeros(sensorCount));
context.triggerConfig = struct();
context.currentTime = 1;
context.previousAdjacency = false(sensorCount);
context.previousAdjacencyHistory = false(sensorCount, sensorCount, 0);
context.previousAdjacencyHistoryCount = 0;
context.previousAdjacencyHistoryTimes = [];
context.baseAdjacency = logical(ones(sensorCount) - eye(sensorCount));
context.physicalAdjacency = context.baseAdjacency;
context.edgeScores = double(context.physicalAdjacency);
context.edgeBudget = sensorCount;
context.directedMessageBudget = sensorCount;
context.positions = zeros(2, sensorCount);
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
