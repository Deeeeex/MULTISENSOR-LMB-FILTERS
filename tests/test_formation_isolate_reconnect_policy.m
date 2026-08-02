function test_formation_isolate_reconnect_policy()
% TEST_FORMATION_ISOLATE_RECONNECT_POLICY Structural v30 contracts.

context = buildContext(4, 6);
control = buildFormationRetentionDebtControl(context);
assert(strcmp(control.contractVersion, ...
    'formation-retention-debt-control-v1'));
assert(control.nodeCount == 24);
assert(control.formationCount == 4);
assert(control.referenceFallbackUsed);
assert(isempty(control.requestedFormationIds));
assert(isempty(control.selectedFormationIds));
assert(all(abs(control.formationRetentionDebtFraction) <= 1e-12));
assert(all(control.singleActionAvailableMask));
assert(all(control.singleActionSafetyMask));
assert(control.selectedScore.safe);
assert(control.selectedMessageCount <= ...
    control.referenceMessageCount);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

initial = buildTemporalCrossEdgeSuspensionActionBank( ...
    context, struct('scorePosteriorSafety', false));
initial.actionPosteriorSafetyMask(2) = true;
initial.networkDisagreementRisk(:) = 1;
initial.retentionRisk(:) = 0;
context.currentTime = 2;
[recoveryAdjacency, recoveryDetails] = ...
    selectFormationIsolateReconnectRuntimePolicy( ...
        context, 1, initial, [2, 1, 1]);
assert(isequal(recoveryAdjacency, ...
    control.selectedAdjacency));
assert(~recoveryDetails.isolationPhase);
assert(recoveryDetails.recoveryPhase);
assert(~recoveryDetails.runtimeFrozenGraphReplay);
assert(recoveryDetails.posteriorUsed);
assert(recoveryDetails.currentLinkReliabilityUsed);
assert(~recoveryDetails.truthUsed && ...
    ~recoveryDetails.futureOutcomeUsed);

[referenceAdjacency, referenceDetails] = ...
    selectFormationIsolateReconnectRuntimePolicy( ...
        context, 1, initial, [1, 1, 1]);
assert(nnz(referenceAdjacency) == ...
    initial.actionMessageCounts(1));
assert(strcmp(referenceDetails.actionName, 'reference'));
assert(~referenceDetails.isolationPhase);
assert(~referenceDetails.recoveryPhase);
assert(~referenceDetails.posteriorUsed);
assert(referenceDetails.referenceFallbackUsed);

protocol = getFormationIsolateReconnectProbeProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-retention-debt-probe-protocol-v1'));
assert(protocol.horizonSteps == 3);
assert(protocol.isolationDurationSteps == 1);
assert(protocol.recoveryDurationSteps == 2);
assert(protocol.retentionDebtOnFraction == 0.02);
assert(protocol.retentionDebtOffFraction == 0.01);
assert(protocol.maximumControlRouteEvaluations == 17);
assert(isequal(protocol.expectedPrimarySelectedFormationIds, ...
    [2, 3, 4]));
assert(protocol.terminalConsensusGainThresholdPercent == 0);
assert(protocol.recoveryCurrentPosteriorRequired);
assert(protocol.recoveryCurrentLinkReliabilityRequired);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('runFormationIsolateReconnectOpenedScreen') == 1);
assert(nargin('auditFormationIsolateReconnectV30Preflight') == 1);

fprintf('PASS: formation isolate-reconnect policy tests\n');
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
