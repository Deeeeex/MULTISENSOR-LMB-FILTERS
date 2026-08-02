function test_conflict_aware_formation_mixing_policy()
% TEST_CONFLICT_AWARE_FORMATION_MIXING_POLICY Structural v28 actions.

context = buildContext(4, 6);
[registeredAdjacency, registeredDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
[referenceAdjacency, referenceDetails] = ...
    selectConflictAwareFormationMixingPolicy( ...
        context, struct('mode', 'reference'));
assert(isequal(referenceAdjacency, registeredAdjacency));
assert(max(abs(referenceDetails.fusionWeightMatrix(:) - ...
    registeredDetails.fusionWeightMatrix(:))) < 1e-12);
assert(referenceDetails.messageCount == 40);
assert(referenceDetails.crossFormationMessageCount == 4);
assert(abs(referenceDetails.candidateCrossResidualTrust - 0.20) < ...
    1e-12);
assert(referenceDetails.combinedSensorStrongConnected);
assert(referenceDetails.formationStrongConnected);
assert(~referenceDetails.posteriorUsed);

bank = buildConflictAwareFormationMixingActionBank( ...
    context, struct('scorePosteriorSafety', false));
assert(strcmp(bank.contractVersion, ...
    'conflict-aware-formation-mixing-action-bank-v1'));
assert(bank.actionCount == 23);
assert(bank.referenceActionIndex == 1);
assert(isequal(bank.safeActionIndices, 1));
assert(bank.safeNonreferenceActionCount == 0);
assert(all(bank.actionMessageCounts == 40));
assert(bank.messageCountParityWithReference);
assert(bank.allPhysical);
assert(bank.allCombinedOneStepStrongConnected);
assert(bank.allFormationStrongConnected);
assert(~bank.truthUsed && ~bank.futureOutcomeUsed);
assert(bank.acceptedSpecificationCount == 23);
assert(bank.duplicateSpecificationCount == 0);
assert(bank.unavailableSpecificationCount == 0);

protocol = getFormationConflictAwareMixingProbeProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-conflict-aware-mixing-probe-protocol-v1'));
assert(strcmp(protocol.presetName, 'm24-formation-fov'));
assert(protocol.seed == 211);
assert(protocol.primarySnapshotTime == 72);
assert(protocol.maximumRetentionRisk == 0.01);
assert(protocol.minimumFormationMeanCardinalityChange == -0.05);
assert(protocol.minimumSupportedLabelRetentionRatio == 0.80);
assert(protocol.maximumDecisionThresholdCrossingCount == 0);
assert(~protocol.finalModelTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('runFormationConflictAwareMixingOpenedScreen') == 1);

[dampedAdjacency, damped] = ...
    selectConflictAwareFormationMixingPolicy( ...
        context, struct( ...
            'mode', 'damp-reference-edge', ...
            'formationId', 2));
assert(isequal(dampedAdjacency, referenceAdjacency));
assert(damped.messageCount == 40);
assert(damped.crossFormationMessageCount == 4);
assert(abs(damped.crossResidualTrustChange + 0.025) < 1e-12);
assert(nnz(abs(damped.fusionWeightMatrix(:) - ...
    referenceDetails.fusionWeightMatrix(:)) > 1e-12) == 2);

modes = {'pair-redistribute', 'pair-add-low', 'pair-add-reference'};
expectedTrustChanges = [0, 0.05, 0.10];
for modeIdx = 1:numel(modes)
    action = struct( ...
        'mode', modes{modeIdx}, ...
        'formationPair', [1, 3]);
    [adjacency, details] = ...
        selectConflictAwareFormationMixingPolicy( ...
            context, action);
    assert(nnz(adjacency) == nnz(referenceAdjacency));
    assert(details.messageCount == 40);
    assert(details.crossFormationMessageCount == 6);
    assert(abs(details.crossResidualTrustChange - ...
        expectedTrustChanges(modeIdx)) < 1e-12);
    assert(numel(details.selectedPairReceivers) == 2);
    assert(numel(details.selectedPairSenders) == 2);
    assert(details.combinedSensorStrongConnected);
    assert(details.formationStrongConnected);
    assert(details.posteriorUsed);
    assert(~details.truthUsed && ~details.futureOutcomeUsed);
    assert(all(sum(details.residualAdjacency, 2) == 1));
    assert(all(sum(details.residualAdjacency, 1) == 1));
    assert(max(abs(sum(details.fusionWeightMatrix, 2) - 1)) < ...
        1e-12);
end

[firstAdjacency, firstDetails] = ...
    selectConflictAwareFormationMixingPolicy( ...
        context, struct( ...
            'mode', 'pair-redistribute', ...
            'formationPair', [1, 4]));
[secondAdjacency, secondDetails] = ...
    selectConflictAwareFormationMixingPolicy( ...
        context, struct( ...
            'mode', 'pair-redistribute', ...
            'formationPair', [1, 4]));
assert(isequal(firstAdjacency, secondAdjacency));
assert(isequal(firstDetails.selectedPairReceivers, ...
    secondDetails.selectedPairReceivers));
assert(isequal(firstDetails.selectedPairSenders, ...
    secondDetails.selectedPairSenders));

for formationCount = [2, 4, 6, 8]
    scaleContext = buildContext(formationCount, 6);
    [scaleReference, ~] = ...
        selectConflictAwareFormationMixingPolicy( ...
            scaleContext, struct('mode', 'reference'));
    [scaleCandidate, scaleDetails] = ...
        selectConflictAwareFormationMixingPolicy( ...
            scaleContext, struct( ...
                'mode', 'pair-redistribute', ...
                'formationPair', [1, formationCount]));
    assert(nnz(scaleCandidate) == nnz(scaleReference));
    assert(nnz(scaleCandidate) == 10 * formationCount);
    assert(scaleDetails.crossFormationMessageCount == ...
        formationCount + 2);
    assert(abs(scaleDetails.crossResidualTrustChange) < 1e-12);
    assert(scaleDetails.combinedSensorStrongConnected);
    assert(scaleDetails.formationStrongConnected);
end

failed = false;
try
    selectConflictAwareFormationMixingPolicy( ...
        context, struct( ...
            'mode', 'pair-redistribute', ...
            'formationPair', [1, 1]));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ConflictMixing:InvalidContract');
end
assert(failed);

fprintf('PASS: conflict-aware formation-mixing policy tests\n');
end

function context = buildContext(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = struct();
context.model.xDimension = 4;
context.model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
context.commConfig = struct();
context.currentTime = 1;
context.physicalAdjacency = ~logical(eye(nodeCount));
context.directedMessageBudget = 2 * nodeCount;
end
