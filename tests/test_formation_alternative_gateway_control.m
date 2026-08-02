function test_formation_alternative_gateway_control()
% TEST_FORMATIONALTERNATEGATEWAYCONTROL Structural v32 contracts.

context = buildContext(3, 4);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
baselineSources = reshape(referenceDetails. ...
    residualBaselinePolicyDetails.selectedSourcesByReceiver, 1, []);
dominantSources = reshape( ...
    referenceDetails.dominantSourcesByReceiver, 1, []);
nodeCount = numel(baselineSources);
scoreMatrix = -inf(nodeCount);
for receiver = 1:nodeCount
    for sender = find(context.physicalAdjacency(receiver, :))
        scoreMatrix(receiver, sender) = ...
            1000 - 10 * receiver - sender;
    end
end
ranked = enumerateTopKSplicedResidualFormationCycles( ...
    context.model.dynamicTopologyScenario.config.sensorGroupIds, ...
    baselineSources, dominantSources, context.physicalAdjacency, ...
    scoreMatrix, 'counter-clockwise', 5);
assert(numel(ranked) == 5);
assert(all(diff([ranked.predictedObjective]) <= 1e-12));
assert(isequal(ranked(1).formationOrder, [1, 3, 2]));
keys = cell(1, numel(ranked));
for candidateIdx = 1:numel(ranked)
    candidate = ranked(candidateIdx);
    assert(candidate.rank == candidateIdx);
    assert(candidate.crossFormationEdgeCount == 3);
    assert(candidate.residualSensorStrongConnected);
    assert(candidate.residualFormationStrongConnected);
    assert(all(sum(candidate.residualAdjacency, 1) == 1));
    assert(all(sum(candidate.residualAdjacency, 2) == 1));
    keys{candidateIdx} = sprintf( ...
        '%d,', candidate.residualSourcesByReceiver);
end
assert(numel(unique(keys)) == numel(keys));

anchored = enumerateTopKSplicedResidualFormationCycles( ...
    context.model.dynamicTopologyScenario.config.sensorGroupIds, ...
    baselineSources, dominantSources, context.physicalAdjacency, ...
    scoreMatrix, 'counter-clockwise', 5, ...
    referenceDetails.spliceSelection.cutReceivers, 1);
assert(numel(anchored) == 4);
assert(all([anchored.trustRegionEnabled]));
assert(all([anchored.referenceCutChangeCount] <= 1));
assert(any([anchored.referenceCutChangeCount] == 0));
assert(any([anchored.referenceCutChangeCount] == 1));

protocol = getFormationAlternativeGatewayProbeProtocol();
control = buildFormationAlternativeGatewayReconnectControl(context);
assert(strcmp(control.contractVersion, ...
    'formation-alternative-gateway-control-v1'));
assert(control.nodeCount == 12);
assert(control.formationCount == 3);
assert(control.candidateCount >= 1);
assert(control.candidateCount <= ...
    protocol.maximumGatewayCandidates);
assert(all(control.candidateSafetyMask));
assert(~any(control.candidateEligibilityMask));
assert(control.referenceFallbackUsed);
assert(isnan(control.selectedCandidateIndex));
assert(strcmp(control.selectedActionName, 'reference'));
assert(isequal(control.selectedAdjacency, referenceAdjacency));
assert(control.messageCountParityWithReference);
assert(control.formationGraphParityWithReference);
assert(control.residualWeightParityWithReference);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

assert(strcmp(protocol.contractVersion, ...
    'formation-alternative-gateway-probe-protocol-v1'));
assert(strcmp(protocol.formationOrientation, 'counter-clockwise'));
assert(protocol.maximumGatewayCandidates == 16);
assert(protocol.maximumGatewayCutChanges == 1);
assert(protocol.minimumDisagreementImprovementFraction == 0.0025);
assert(protocol.expectedCandidateCount == 12);
assert(~any(protocol.expectedCandidateSafetyMask));
assert(~any(protocol.expectedCandidateEligibilityMask));
assert(protocol.expectedReferenceFallback);
assert(protocol.messageCountParityRequired);
assert(protocol.formationGraphParityRequired);
assert(protocol.residualWeightParityRequired);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);

fprintf('PASS: formation alternative-gateway control tests\n');
end

function context = buildContext(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    object = model.birthParameters(1);
    object.birthTime = 1;
    object.birthLocation = 1;
    object.r = 0.9;
    object.numberOfGmComponents = 1;
    object.w = 1;
    object.mu = {zeros(model.xDimension, 1)};
    object.Sigma = {eye(model.xDimension)};
    posteriors{sensorIdx} = object;
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(nodeCount), ...
    'outageSchedule', []);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.currentTime = 1;
context.physicalAdjacency = ...
    logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = context.physicalAdjacency;
context.edgeBudget = nodeCount;
context.directedMessageBudget = 2 * nodeCount;
context.positions = zeros(2, nodeCount);
[referenceAdjacency, ~] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
context.previousAdjacency = referenceAdjacency;
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
context.previousAdjacencyHistoryCount = 2;
context.previousAdjacencyHistoryTimes = [-1, 0];
end
