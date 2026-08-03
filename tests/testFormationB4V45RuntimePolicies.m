function testFormationB4V45RuntimePolicies()
% Structural, temporal and causal-boundary tests for the three V45 arms.

presets = {'m24-formation-fov-convoy', ...
    'x36-formation-fov-crossing'};
for presetIdx = 1:numel(presets)
    context = buildRuntimeContext(presets{presetIdx}, 41);
    nodeCount = numel(context.localPosteriorBySensor);
    referenceByPhase = cell(1, 4);
    syncCounts = zeros(1, 4);
    staggeredCounts = zeros(1, 4);
    expectedStaggeredCounts = zeros(1, 4);
    for phase = 1:4
        context = setCurrentTime(context, phase);
        [reference, referenceDetails] = ...
            selectFormationB4V45ReferenceRuntimePolicy(context);
        [synchronized, synchronizedDetails] = ...
            selectFormationB4V45SynchronizedRuntimePolicy(context);
        [staggered, staggeredDetails] = ...
            selectFormationB4V45FormationStaggeredRuntimePolicy(context);
        referenceByPhase{phase} = reference;
        syncCounts(phase) = nnz(synchronized);
        staggeredCounts(phase) = nnz(staggered);
        expectedStaggeredCounts(phase) = ...
            staggeredDetails.messageCountsByPhase(phase);

        assertRuntimePage(context, reference, referenceDetails, reference);
        assertRuntimePage(context, synchronized, ...
            synchronizedDetails, reference);
        assertRuntimePage(context, staggered, staggeredDetails, reference);
        assert(referenceDetails.currentAbsolutePhase == phase);
        assert(synchronizedDetails.currentAbsolutePhase == phase);
        assert(staggeredDetails.currentAbsolutePhase == phase);
        assert(strcmp(referenceDetails.armId, ...
            'v43-reference-a70-e05'));
        assert(strcmp(synchronizedDetails.armId, ...
            'v44-sync-all-b4-e20-mc'));
        assert(strcmp(staggeredDetails.armId, ...
            'v44-formation-all-b4-e20-mc'));
        assert(strcmp(synchronizedDetails.phasePattern, ...
            'synchronized'));
        assert(strcmp(staggeredDetails.phasePattern, ...
            'formation-staggered'));
        assertFixedRule(referenceDetails, 1, false);
        assertFixedRule(synchronizedDetails, 2, true);
        assertFixedRule(staggeredDetails, 3, true);
    end

    assert(all(cellfun(@(page) isequal(page, referenceByPhase{1}), ...
        referenceByPhase)));
    assert(isequal(syncCounts, [2 * nodeCount, nodeCount, ...
        nodeCount, nodeCount]));
    assert(isequal(staggeredCounts, expectedStaggeredCounts));
    assert(sum(staggeredCounts) == 5 * nodeCount);

    context = setCurrentTime(context, 5);
    [~, wrapDetails] = ...
        selectFormationB4V45SynchronizedRuntimePolicy(context);
    assert(wrapDetails.currentAbsolutePhase == 1);
end

context = buildRuntimeContext('m24-formation-fov-convoy', 41);
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy( ...
    rmfield(context, 'sensorPhysicalUids')), ...
    'FormationB4V45:InvalidContext');
assertErrorId(@() selectFormationB4V45SynchronizedRuntimePolicy( ...
    rmfield(context, 'formationPhysicalUidsBySensor')), ...
    'FormationB4V45:InvalidContext');
invalid = context;
invalid.sensorPhysicalUids(2) = invalid.sensorPhysicalUids(1);
assertErrorId(@() ...
    selectFormationB4V45FormationStaggeredRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidContext');
invalid = setCurrentTime(context, 0);
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidContext');
invalid = context;
invalid.observableInputContract.contractVersion = ...
    'topology-policy-observable-input-v2';
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.observableInputContract.linkUniformsAbsent = false;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.observableInputContract.targetTruthAbsent = false;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.triggerConfig.topologyDirectedEnabled = false;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidDirectedRuntime');
invalid = context;
invalid.triggerConfig.topologyDirectedMessageBudget = ...
    invalid.triggerConfig.topologyDirectedMessageBudget - 1;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidDirectedRuntime');
invalid = context;
invalid.linkUniforms = zeros(numel(context.sensorPhysicalUids));
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.targetTruth = 1;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.oracleScore = 1;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.truth = 1;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = context;
invalid.futureOutcome = 1;
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
assertErrorId(@() buildFormationB4V45FixedRuntimeArm( ...
    context, 'unregistered-arm'), 'FormationB4V45:InvalidArm');

fprintf('PASS: FormationB4V45 runtime policy tests\n');
end

function context = buildRuntimeContext(presetName, seed)
[context, metadata] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        presetName, seed);
context = rmfield(context, 'auditBoundary');
context.model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', ...
        context.model.dynamicTopologyScenario.config.sensorGroupIds));
context.commConfig = struct( ...
    'pDropByEdge', context.commConfig.pDropByEdge);
context.physicalIdentityRegistryCanonicalSha256 = ...
    metadata.physicalUidAssignmentCanonicalSha256;
nodeCount = numel(context.localPosteriorBySensor);
context.triggerConfig.topologyDirectedEnabled = true;
context.triggerConfig.topologyDirectedMessageBudget = 2 * nodeCount;
context.observableInputContract = struct( ...
    'contractVersion', ...
        'topology-policy-observable-input-v3-physical-uid', ...
    'enforced', true, ...
    'passed', true, ...
    'currentTime', context.currentTime, ...
    'targetTruthAbsent', true, ...
    'linkUniformsAbsent', true, ...
    'futurePDropPagesAbsent', true, ...
    'physicalIdentityPresent', true, ...
    'physicalIdentitySchemaRestricted', true, ...
    'directedTopologyRuntimeSemanticsPresent', true, ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount, ...
    'physicalIdentityRegistryCanonicalSha256', ...
        context.physicalIdentityRegistryCanonicalSha256);
end

function context = setCurrentTime(context, currentTime)
context.currentTime = currentTime;
context.observableInputContract.currentTime = currentTime;
end

function assertRuntimePage(context, adjacency, details, reference)
nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
weights = details.fusionWeightMatrix;
support = adjacency | logical(eye(nodeCount));
assert(islogical(adjacency));
assert(~any(diag(adjacency)));
assert(~any(adjacency(:) & ~physical(:)));
assert(~any(adjacency(:) & ~reference(:)));
assert(all(abs(sum(weights, 2) - 1) <= 1e-12));
assert(~any(weights(:) < -1e-12));
assert(~any(weights(:) > 1e-12 & ~support(:)));
assert(details.currentMessageCount == nnz(adjacency));
assert(details.referenceMessageCount == nnz(reference));
assert(details.currentAdjacencyIsReferenceSubset);
assert(details.observableContextOnly);
assert(details.directedTopologyRuntimeRequired);
assert(details.topologyDirectedEnabled);
assert(details.topologyDirectedMessageBudget == 2 * nodeCount);
assert(~details.standaloneAttestationAuthority);
assert(details.targetTruthAbsent);
assert(details.linkUniformsAbsent);
assert(details.futurePDropPagesAbsent);
assert(~details.posteriorUsed && ~details.currentPosteriorUsed);
assert(~details.truthUsed && ~details.futureOutcomeUsed);
assert(~details.realizedDeliveryUniformsUsed);
end

function assertFixedRule(details, expectedOrdinal, candidate)
assert(strcmp(details.contractVersion, ...
    'formation-b4-v45-fixed-runtime-policy-v1'));
assert(details.armOrdinal == expectedOrdinal);
assert(details.runtimeRuleFrozen);
assert(~details.policyOptionsAccepted);
assert(details.period == 4);
assert(strcmp(details.dutyLayer, 'all'));
assert(details.dominantWeight == 0.70);
assert(details.referenceResidualWeight == 0.05);
assert(details.activeResidualWeight == 0.20);
assert(details.residualMassMatchedToReference);
if candidate
    assert(details.referenceMessageCountPerStep == ...
        2 * size(details.fusionWeightMatrix, 1));
    assert(details.messagesPerPeriod == ...
        5 * size(details.fusionWeightMatrix, 1));
    assert(abs(details.messageSavingFractionPerPeriod - 3 / 8) ...
        <= 1e-12);
else
    assert(details.messageSavingFractionPerPeriod == 0);
end
end

function assertErrorId(handle, expected)
failed = false;
try
    handle();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expected);
end
assert(failed);
end
