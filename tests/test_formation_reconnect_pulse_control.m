function test_formation_reconnect_pulse_control()
% TEST_FORMATIONRECONNECTPULSECONTROL Structural v31 contracts.

context = buildContext(4, 6);
protocol = getFormationReconnectPulseProbeProtocol();
control = buildFormationReconnectPulseControl(context);
assert(strcmp(control.contractVersion, ...
    'formation-reconnect-pulse-control-v1'));
assert(control.nodeCount == 24);
assert(control.formationCount == 4);
assert(control.referenceFallbackUsed);
assert(isempty(control.requestedFormationIds));
assert(isempty(control.selectedFormationIds));
assert(control.messageCountParityWithReference);
assert(control.selectedScore.safe);
assert(all(control.singleActionAvailableMask(:)));
assert(all(control.singleActionSafetyMask(:)));
assert(~any(control.singleActionEligibleMask(:)));
assert(all(control.singleDisagreementImprovementFraction(:) < ...
    protocol.minimumDisagreementImprovementFraction));
assert(control.evaluatedRouteCount == 13);
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(~control.truthUsed && ~control.futureOutcomeUsed);

assert(strcmp(protocol.contractVersion, ...
    'formation-reconnect-pulse-probe-protocol-v1'));
assert(isequal(protocol.pulseResidualWeights, ...
    [0.075, 0.10, 0.15]));
assert(protocol.minimumDisagreementImprovementFraction == 0.0025);
assert(protocol.maximumControlRouteEvaluations == 33);
assert(isequal(protocol.expectedSingleSafetyMask, ...
    logical([1, 1, 0; zeros(3, 3)])));
assert(~any(protocol.expectedSingleEligibleMask(:)));
assert(protocol.expectedReferenceFallback);
assert(~protocol.trackingOutcomeRerunAuthorized);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);

fprintf('PASS: formation reconnect-pulse control tests\n');
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
