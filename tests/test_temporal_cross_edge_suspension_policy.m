function test_temporal_cross_edge_suspension_policy()
% TEST_TEMPORAL_CROSS_EDGE_SUSPENSION_POLICY Structural v29 contracts.

context = buildContext(4, 6);
[registeredAdjacency, registeredDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
context.previousAdjacencyHistory = repmat( ...
    registeredAdjacency, 1, 1, 2);

[referenceAdjacency, referenceDetails] = ...
    selectTemporalCrossEdgeSuspensionPolicy(context, []);
assert(isequal(referenceAdjacency, registeredAdjacency));
assert(max(abs(referenceDetails.fusionWeightMatrix(:) - ...
    registeredDetails.fusionWeightMatrix(:))) < 1e-12);
assert(referenceDetails.messageCount == 40);
assert(referenceDetails.messageSavingCount == 0);
assert(all(referenceDetails.rollingB3SensorPass));
assert(all(referenceDetails.rollingB3FormationPass));

bank = buildTemporalCrossEdgeSuspensionActionBank( ...
    context, struct('scorePosteriorSafety', false));
assert(strcmp(bank.contractVersion, ...
    'temporal-cross-edge-suspension-action-bank-v1'));
assert(bank.actionCount == 16);
assert(bank.referenceActionIndex == 1);
assert(isequal(bank.safeActionIndices, 1));
assert(all(bank.actionMessageCounts == 40 - ...
    bank.actionSuspendedCrossEdgeCounts));
assert(all(abs(bank.actionMessageSavingFractions - ...
    bank.actionMessageSavingCounts ./ ...
        bank.actionMessageCounts(1)) < 1e-12));
assert(all(bank.actionWithinReferencePayload));
assert(bank.allPhysical);
assert(bank.allRowStochastic);
assert(bank.allMessageCountsNoGreaterThanReference);
assert(bank.allRollingB3SensorStrongConnected);
assert(bank.allRollingB3FormationStrongConnected);
assert(~bank.oneStepStrongConnectivityRequired);
assert(~bank.oneStepDisagreementHardGateEnabled);
assert(~bank.truthUsed && ~bank.futureOutcomeUsed);

protocol = getFormationTemporalSuspensionProbeProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-temporal-suspension-probe-protocol-v1'));
assert(protocol.interventionDurationSteps == 1);
assert(protocol.horizonSteps == 3);
assert(protocol.maximumActionCount == 256);
assert(~protocol.oneStepDisagreementHardGateEnabled);
assert(protocol.selectedRollingConnectivityRequired);
assert(~protocol.finalModelTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('runFormationTemporalSuspensionOpenedScreen') == 1);

[singleAdjacency, single] = ...
    selectTemporalCrossEdgeSuspensionPolicy(context, 2);
assert(nnz(singleAdjacency) == 39);
assert(single.messageSavingCount == 1);
assert(abs(single.messageSavingFraction - 1/40) < 1e-12);
assert(single.suspendedCrossEdgeCount == 1);
assert(isequal(single.suspendedFormationIds, 2));
assert(numel(single.suspendedReceivers) == 1);
assert(numel(single.suspendedSenders) == 1);
assert(abs(single.suspendedWeights - 0.05) < 1e-12);
assert(max(abs(sum(single.fusionWeightMatrix, 2) - 1)) < 1e-12);
assert(all(single.rollingB3SensorPass));
assert(all(single.rollingB3FormationPass));
assert(~single.truthUsed && ~single.futureOutcomeUsed);

[multiAdjacency, multi] = ...
    selectTemporalCrossEdgeSuspensionPolicy(context, [2, 3, 4]);
assert(nnz(multiAdjacency) == 37);
assert(multi.messageSavingCount == 3);
assert(multi.crossFormationMessageCount == 1);
assert(~multi.instantaneousFormationStrongConnected);
assert(all(multi.rollingB3SensorPass));
assert(all(multi.rollingB3FormationPass));

[allAdjacency, allDetails] = ...
    selectTemporalCrossEdgeSuspensionPolicy(context, [1, 2, 3, 4]);
assert(nnz(allAdjacency) == 36);
assert(allDetails.messageSavingCount == 4);
assert(allDetails.crossFormationMessageCount == 0);
assert(~allDetails.instantaneousFormationStrongConnected);
assert(all(allDetails.rollingB3SensorPass));
assert(all(allDetails.rollingB3FormationPass));

failed = false;
try
    selectTemporalCrossEdgeSuspensionPolicy(context, [1, 1]);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'TemporalSuspension:InvalidContract');
end
assert(failed);

for formationCount = [2, 4, 6, 8]
    scaleContext = buildContext(formationCount, 6);
    [scaleReference, ~] = ...
        selectBackboneResidualSplicedCyclePolicy( ...
            scaleContext, 'fixed-counter-clockwise', struct( ...
                'dominantWeight', 0.70, 'residualWeight', 0.05));
    scaleContext.previousAdjacencyHistory = repmat( ...
        scaleReference, 1, 1, 2);
    [candidate, details] = ...
        selectTemporalCrossEdgeSuspensionPolicy( ...
            scaleContext, 1:formationCount);
    assert(nnz(candidate) == 9 * formationCount);
    assert(details.referenceMessageCount == 10 * formationCount);
    assert(details.messageSavingCount == formationCount);
    assert(all(details.rollingB3SensorPass));
    assert(all(details.rollingB3FormationPass));
end

fprintf('PASS: temporal cross-edge suspension tests\n');
end

function context = buildContext(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = struct();
context.model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
context.commConfig = struct();
context.currentTime = 1;
context.physicalAdjacency = ~logical(eye(nodeCount));
context.directedMessageBudget = 2 * nodeCount;
end
