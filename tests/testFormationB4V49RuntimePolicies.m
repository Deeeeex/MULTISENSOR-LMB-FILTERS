function testFormationB4V49RuntimePolicies()
% Focused checks for selection, phase behavior and posterior independence.

protocol = getFormationB4V49RuntimeProtocol();
context = buildContext('m24-formation-fov-convoy', 41);
nodeCount = numel(context.localPosteriorBySensor);
[triggerConfig, details] = buildFormationB4V49FixedTriggerConfig( ...
    protocol.candidateArmId, nodeCount);
context.triggerConfig = triggerConfig;
assert(details.parentConfigReusedWithoutRelaxation);
assert(strcmp(triggerConfig.topologyPolicyName, ...
    'selectFormationB4V49SynchronizedRuntimePolicy'));

first = context;
first.localPosteriorBySensor(:) = {struct('value', ones(2))};
second = context;
second.localPosteriorBySensor(:) = {struct('value', -ones(5))};
[firstAdjacency, firstDetails] = ...
    selectFormationB4V49SynchronizedRuntimePolicy(first);
[secondAdjacency, secondDetails] = ...
    selectFormationB4V49SynchronizedRuntimePolicy(second);
firstDetails.selectionSeconds = 0;
secondDetails.selectionSeconds = 0;
assert(isequal(firstAdjacency, secondAdjacency));
assert(isequaln(firstDetails, secondDetails));
assert(firstDetails.scheduleCertificate.cycleSelected);
assert(~firstDetails.referenceFallbackUsed);
assertRuntimePage(firstAdjacency, firstDetails, nodeCount, 1);

dominant = firstDetails.dominantAdjacency;
for phase = 2:4
    current = context;
    current.currentTime = phase;
    current.observableInputContract.currentTime = phase;
    [adjacency, phaseDetails] = ...
        selectFormationB4V49SynchronizedRuntimePolicy(current);
    assert(isequal(adjacency, dominant));
    assert(~phaseDetails.scheduleCertificate.cycleSelectionPerformed);
    assertRuntimePage(adjacency, phaseDetails, nodeCount, phase);
end

fallback = context;
fallback.physicalAdjacency = logical(fallback.baseAdjacency);
[fallbackAdjacency, fallbackDetails] = ...
    selectFormationB4V49SynchronizedRuntimePolicy(fallback);
assert(fallbackDetails.referenceFallbackUsed);
assert(strcmp(fallbackDetails.scheduleCertificate.fallbackReason, ...
    'no-physical-cycle'));
assert(isequal(fallbackAdjacency, ...
    fallbackDetails.incumbentRuntimeAdjacency));

future = context;
future.commConfig.pDropByEdge = repmat( ...
    future.commConfig.pDropByEdge, 1, 1, 2);
assertErrorId(@() selectFormationB4V49SynchronizedRuntimePolicy(future), ...
    'FormationB4V49RouteContext:FuturePageOrInvalidLinkState');
fprintf('PASS: FormationB4V49 runtime policy test\n');
end

function context = buildContext(presetName, seed)
[context, metadata] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        presetName, seed);
context = rmfield(context, 'auditBoundary');
context.commConfig = struct( ...
    'pDropByEdge', context.commConfig.pDropByEdge);
context.physicalIdentityRegistryCanonicalSha256 = ...
    metadata.physicalUidAssignmentCanonicalSha256;
nodeCount = numel(context.localPosteriorBySensor);
context.observableInputContract = struct( ...
    'contractVersion', ...
        'topology-policy-observable-input-v3-physical-uid', ...
    'enforced', true, 'passed', true, ...
    'currentTime', context.currentTime, ...
    'targetTruthAbsent', true, 'linkUniformsAbsent', true, ...
    'futurePDropPagesAbsent', true, ...
    'physicalIdentityPresent', true, ...
    'physicalIdentitySchemaRestricted', true, ...
    'directedTopologyRuntimeSemanticsPresent', true, ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount, ...
    'physicalIdentityRegistryCanonicalSha256', ...
        context.physicalIdentityRegistryCanonicalSha256);
end

function assertRuntimePage(adjacency, details, nodeCount, phase)
weights = details.fusionWeightMatrix;
positive = weights > 1e-12;
positive(1:nodeCount+1:end) = false;
assert(isequal(positive, logical(adjacency)));
assert(nnz(adjacency) == nodeCount * (1 + double(phase == 1)));
assert(all(abs(sum(weights, 2) - 1) <= 1e-12));
end

function assertErrorId(action, expectedId)
failed = false;
try
    action();
catch errorInfo
    failed = true;
    assert(strcmp(errorInfo.identifier, expectedId));
end
assert(failed);
end
