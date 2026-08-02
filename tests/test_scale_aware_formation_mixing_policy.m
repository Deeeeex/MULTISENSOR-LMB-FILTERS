function test_scale_aware_formation_mixing_policy()
% TEST_SCALE_AWARE_FORMATION_MIXING_POLICY Structural v27 contracts.

context = buildContext(4, 6);
[registeredAdjacency, registeredDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
[referenceAdjacency, referenceDetails] = ...
    selectScaleAwareFormationMixingPolicy( ...
        context, 'reference-cycle');
assert(isequal(referenceAdjacency, registeredAdjacency));
assert(max(abs(referenceDetails.fusionWeightMatrix(:) - ...
    registeredDetails.fusionWeightMatrix(:))) < 1e-12);
assert(referenceDetails.gatewayLaneCount == 1);
assert(referenceDetails.crossFormationMessageCount == 4);
assert(referenceDetails.messageCount == 40);
assert(referenceDetails.messageCountParityWithReference);
assert(referenceDetails.combinedSensorStrongConnected);
assert(referenceDetails.formationStrongConnected);
assert(~referenceDetails.truthUsed && ...
    ~referenceDetails.futureOutcomeUsed);

modes = { ...
    'cycle', 'dual-gateway-cycle', 'bidirectional-cycle', ...
    'cycle-antipodal', 'bidirectional-antipodal'};
expectedLaneCounts = [1, 2, 2, 2, 3];
expectedCrossCounts = 4 * expectedLaneCounts;
for modeIdx = 1:numel(modes)
    [adjacency, details] = ...
        selectScaleAwareFormationMixingPolicy( ...
            context, modes{modeIdx}, struct( ...
                'crossResidualWeight', 0.10, ...
                'fallbackMode', 'error'));
    assert(details.gatewayLaneCount == ...
        expectedLaneCounts(modeIdx));
    assert(details.crossFormationMessageCount == ...
        expectedCrossCounts(modeIdx));
    assert(nnz(adjacency) == nnz(referenceAdjacency));
    assert(details.messageCountParityWithReference);
    assert(details.dominantResidualDuplicateCount == 8);
    assert(details.referenceDominantResidualDuplicateCount == 8);
    assert(details.combinedSensorStrongConnected);
    assert(details.formationStrongConnected);
    assert(all(sum(details.gatewaySelection. ...
        residualAdjacency, 2) == 1));
    assert(all(sum(details.gatewaySelection. ...
        residualAdjacency, 1) == 1));
    assert(max(abs(sum(details.fusionWeightMatrix, 2) - 1)) < ...
        1e-12);
    grouped = details.formationMixingMatrix;
    crossMass = sum(grouped, 2) - diag(grouped);
    expectedMass = expectedLaneCounts(modeIdx) * 0.10 / 6;
    assert(max(abs(crossMass - expectedMass)) < 1e-12);
end

[~, expanderDetails] = ...
    selectScaleAwareFormationMixingPolicy( ...
        context, 'bidirectional-antipodal', struct( ...
            'crossResidualWeight', 0.10, ...
            'fallbackMode', 'error'));
assert(expanderDetails.formationMixingSpectralGapProxy > ...
    referenceDetails.formationMixingSpectralGapProxy + 1e-3);

bank = buildScaleAwareFormationMixingActionBank(context);
assert(strcmp(bank.contractVersion, ...
    'scale-aware-formation-mixing-action-bank-v1'));
assert(bank.actionCount == 11);
assert(bank.referenceActionIndex == 1);
assert(strcmp(bank.actionModes{1}, 'reference-cycle'));
assert(all(bank.actionMessageCounts == 40));
assert(isequal(bank.actionGatewayLaneCounts, ...
    [1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3]));
assert(isequal(bank.actionCrossFormationMessageCounts, ...
    [4, 4, 4, 8, 8, 8, 8, 8, 8, 12, 12]));
assert(bank.messageCountParityWithReference);
assert(bank.allPhysical);
assert(bank.allCombinedOneStepStrongConnected);
assert(bank.allFormationStrongConnected);
assert(~bank.truthUsed && ~bank.futureOutcomeUsed);
assert(bank.actionFormationMixingSpectralGapProxy(end) > ...
    bank.actionFormationMixingSpectralGapProxy(1));
assert(all(bank.actionWithinReferencePayload));
assert(isequal(bank.actionModeIndex, 1:bank.actionCount));

protocol = getFormationScaleAwareMixingProbeProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-scale-aware-residual-mixing-probe-protocol-v1'));
assert(strcmp(protocol.presetName, 'm24-formation-fov'));
assert(protocol.seed == 211);
assert(protocol.primarySnapshotTime == 72);
assert(isequal(protocol.openedSnapshotTimes, [60, 72, 104, 124]));
assert(protocol.horizonSteps == 3);
assert(protocol.maximumActionCount == 11);
assert(protocol.minimumStrongOpenedStateCount == 3);
assert(~protocol.finalModelTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('runFormationScaleAwareMixingOpenedScreen') == 1);

restricted = context;
groupIds = context.model.dynamicTopologyScenario.config.sensorGroupIds;
sameFormation = groupIds(:) == groupIds(:)';
restricted.physicalAdjacency = logical( ...
    sameFormation | registeredAdjacency | registeredAdjacency');
restricted.physicalAdjacency(1:25:end) = false;
[fallbackAdjacency, fallbackDetails] = ...
    selectScaleAwareFormationMixingPolicy( ...
        restricted, 'cycle-antipodal');
[restrictedReference, ~] = ...
    selectScaleAwareFormationMixingPolicy( ...
        restricted, 'reference-cycle');
assert(isequal(fallbackAdjacency, restrictedReference));
assert(fallbackDetails.projectionFallbackUsed);
assert(strcmp(fallbackDetails.realizedMode, 'reference-cycle'));
assert(strcmp(fallbackDetails.requestedMode, 'cycle-antipodal'));

failed = false;
try
    selectScaleAwareFormationMixingPolicy( ...
        restricted, 'cycle-antipodal', ...
        struct('fallbackMode', 'error'));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ScaleMixing:Infeasible');
end
assert(failed);

failed = false;
try
    selectScaleAwareFormationMixingPolicy( ...
        context, 'cycle', struct( ...
            'crossResidualWeight', 0.30));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ScaleMixing:InvalidContract');
end
assert(failed);

for formationCount = [2, 4, 6, 8]
    scaleContext = buildContext(formationCount, 6);
    [scaleReference, scaleReferenceDetails] = ...
        selectScaleAwareFormationMixingPolicy( ...
            scaleContext, 'reference-cycle');
    [scaleCandidate, scaleDetails] = ...
        selectScaleAwareFormationMixingPolicy( ...
            scaleContext, 'bidirectional-antipodal', ...
            struct('fallbackMode', 'error'));
    assert(scaleDetails.gatewayLaneCount == 3);
    assert(scaleDetails.crossFormationMessageCount == ...
        3 * formationCount);
    assert(nnz(scaleCandidate) == nnz(scaleReference));
    assert(nnz(scaleCandidate) == 10 * formationCount);
    assert(scaleDetails.combinedSensorStrongConnected);
    assert(scaleDetails.formationStrongConnected);
    assert(scaleDetails.formationMixingSpectralGapProxy > ...
        scaleReferenceDetails.formationMixingSpectralGapProxy);
end

fprintf('test_scale_aware_formation_mixing_policy passed\n');
end

function context = buildContext(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = struct();
context.model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
context.currentTime = 1;
context.physicalAdjacency = ~logical(eye(nodeCount));
context.directedMessageBudget = 2 * nodeCount;
end
