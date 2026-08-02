function test_formation_safe_gateway_trust_backoff_control()
% TEST_FORMATIONSAFEGATEWAYTRUSTBACKOFFCONTROL Structural v33 contracts.

context = buildContext(3, 4);
sourceControl = ...
    buildFormationAlternativeGatewayReconnectControl(context);
assert(sourceControl.candidateCount >= 1);
sourceControl.candidateDisagreementImprovementFraction(1) = 0.01;
sourcePreflight = struct('gatewayControl', sourceControl);

control = buildFormationSafeGatewayTrustBackoffControl( ...
    context, sourcePreflight);
protocol = getFormationSafeGatewayTrustBackoffProtocol();
assert(strcmp(control.contractVersion, ...
    'formation-safe-gateway-trust-backoff-control-v1'));
assert(control.nodeCount == 12);
assert(control.formationCount == 3);
assert(isequal(control.promisingSourceCandidateIndices, 1));
assert(control.promisingCandidateCount == 1);
assert(isequal(control.alternativeResidualWeightGrid, ...
    [0.0125, 0.025, 0.0375]));
assert(all(control.routeSafetyMask));
assert(all(control.routeEligibilityMask));
assert(abs(control.maximumSafeWeightByCandidate - 0.0375) < 1e-12);
assert(~control.referenceFallbackUsed);
assert(control.selectedSourceCandidateIndex == 1);
assert(abs(control.selectedAlternativeResidualWeight - 0.0375) < 1e-12);
assert(~strcmp(control.selectedActionName, 'reference'));
assert(numel(control.selectedChangedCrossReceivers) == 2);
assert(control.messageCountParityWithReference);
assert(control.formationGraphParityWithReference);
assert(control.dominantRouteParityWithReference);
assert(control.alternativeTrustNeverExceedsReference);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.evaluatedRouteCount == 3);
assert(control.referenceScoreReusedFromFrozenV32);
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

assert(strcmp(protocol.contractVersion, ...
    'formation-safe-gateway-trust-backoff-protocol-v1'));
assert(isequal(protocol.expectedPromisingCandidateIndices, ...
    [8, 10, 12]));
assert(~any(protocol.expectedRouteSafetyMask(:)));
assert(~any(protocol.expectedRouteEligibilityMask(:)));
assert(protocol.expectedReferenceFallback);
assert(protocol.maximumControlRouteEvaluations == 48);
assert(protocol.minimumDisagreementImprovementFraction == 0.0025);
assert(protocol.messageCountParityRequired);
assert(protocol.formationGraphParityRequired);
assert(protocol.dominantRouteParityRequired);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);

fprintf('PASS: formation safe gateway trust-backoff tests\n');
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
