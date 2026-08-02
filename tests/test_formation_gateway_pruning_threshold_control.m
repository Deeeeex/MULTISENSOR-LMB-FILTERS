function test_formation_gateway_pruning_threshold_control()
% TEST_FORMATIONGATEWAYPRUNINGTHRESHOLDCONTROL Structural v34 contracts.

context = buildContext(4, 6);
sourceControl = ...
    buildFormationAlternativeGatewayReconnectControl(context);
assert(sourceControl.candidateCount == 12);
sourcePreflight = struct('gatewayControl', sourceControl);
control = buildFormationGatewayPruningThresholdControl( ...
    context, sourcePreflight);
protocol = getFormationGatewayPruningThresholdProtocol();

assert(strcmp(control.contractVersion, ...
    'formation-gateway-pruning-threshold-control-v1'));
assert(control.nodeCount == 24);
assert(control.formationCount == 4);
assert(isequal(control.sourceCandidateIndices, [8, 10, 12]));
assert(isequal(control.lowResidualWeightGrid, ...
    [0.0001, 0.00025, 0.0005, 0.001, 0.0025, 0.005, 0.010]));
assert(isequal(size(control.routeSafetyMask), [3, 7]));
assert(isequal(size(control.routeUsefulnessMask), [3, 7]));
assert(isequal(size(control.routeEligibilityMask), [3, 7]));
assert(all(control.routeSafetyMask(:)));
assert(all(abs(control.maximumSafeWeightByCandidate - 0.01) < 1e-12));
assert(control.evaluatedRouteCount == 21);
assert(control.selectedScore.safe);
assert(control.messageCountParityWithReference);
assert(control.formationGraphParityWithReference);
assert(control.dominantRouteParityWithReference);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(~control.truthUsed && ~control.futureOutcomeUsed);
assert(control.referenceFallbackUsed == ...
    (control.eligibleRouteCount == 0));

assert(strcmp(protocol.contractVersion, ...
    'formation-gateway-pruning-threshold-protocol-v1'));
assert(protocol.maximumControlRouteEvaluations == 21);
assert(~any(protocol.expectedRouteSafetyMask(:)));
assert(~any(protocol.expectedRouteUsefulnessMask(:)));
assert(~any(protocol.expectedRouteEligibilityMask(:)));
assert(protocol.expectedReferenceFallback);
assert(protocol.v33NoBenefitUpperWeight == 0.025);
assert(protocol.v33UsefulLowerWeight == 0.0375);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);

fprintf('PASS: formation gateway pruning-threshold tests\n');
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
