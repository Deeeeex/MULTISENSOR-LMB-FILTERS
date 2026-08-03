function test_causal_minimal_edit_formation_backbone()
% Exact edit, V43-realized assignment and UID-equivariance contracts.

fixture = buildCrossingFixture();
testRealT158RepairAndNoOp(fixture);
testPermutationAndGroupRelabel(fixture);
testPDropSenderReceiverOrientation();
testHallConflictChoosesAlternative();
testNoHallFeasibleConnectedGraphFailsClosed();
testX48DegreeSevenStarFailsClosed();
testFormationCountDevelopmentLimit();
testFuturePageRejected(fixture);
fprintf('PASS: causal minimal-edit formation-backbone tests\n');
end

function fixture = buildCrossingFixture()
rng(41, 'twister');
config = buildDynamicTopologyScenarioConfig( ...
    'x36-formation-fov-crossing');
[trajectories, metadata] = generateMultiFormationTrajectories(config);
graph = buildDynamicTopologyGraphs(config, trajectories);
[pDropByEdge, ~] = buildDynamicTopologyLinkSchedule(config, graph);
registry = FormationB4V45BuildCausalRegisteredBaseGraph( ...
    config.formationBackboneMode, config.sensorGroupIds, ...
    metadata.sensorPhysicalUids, ...
    metadata.formationPhysicalUidsBySensor, graph.positions(:, :, 1), ...
    graph.physicalAdjacency(:, :, 1));
fixture = struct('config', config, 'metadata', metadata, ...
    'graph', graph, 'pDropByEdge', pDropByEdge, ...
    'registry', registry);
end

function testRealT158RepairAndNoOp(fixture)
c = fixture.config;
m = fixture.metadata;
g = fixture.graph;
pDrop = fixture.pDropByEdge;
registered = fixture.registry.baseAdjacency;

[noOpBase, noOp] = projectAt(fixture, 157);
assert(isequal(noOpBase, registered));
assert(noOp.registeredRemovalCount == 0);
assert(noOp.forcedRegisteredRemovalCount == 0);
assert(noOp.optionalRegisteredRemovalCount == 0);
assert(noOp.nonregisteredAdditionCount == 0);
assert(isempty(noOp.removedRegisteredPairPhysicalUids));
assert(isempty(noOp.addedNonregisteredPairPhysicalUids));
assert(noOp.searchCandidateCounts.selectedLayerTopologyCount == 1);

registeredContext = v43Context(c, m, g, pDrop, 157, registered);
projectedContext = v43Context(c, m, g, pDrop, 157, noOpBase);
[registeredRoute, registeredDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(registeredContext);
[projectedRoute, projectedDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(projectedContext);
assert(isequal(registeredRoute, projectedRoute));
assert(isequal(registeredDetails.fusionWeightMatrix, ...
    projectedDetails.fusionWeightMatrix));
registeredRouteHash = computeCanonicalValueSha256(struct( ...
    'adjacency', registeredRoute, ...
    'weights', registeredDetails.fusionWeightMatrix));
projectedRouteHash = computeCanonicalValueSha256(struct( ...
    'adjacency', projectedRoute, ...
    'weights', projectedDetails.fusionWeightMatrix));
assert(strcmp(registeredRouteHash, projectedRouteHash));

[projectedBase, certificate] = projectAt(fixture, 158);
assert(isequal(certificate.removedRegisteredPairPhysicalUids, ...
    [700404, 700505]));
assert(isequal(certificate.addedNonregisteredPairPhysicalUids, ...
    [700202, 700606]));
assert(certificate.registeredRemovalCount == 1);
assert(certificate.forcedRegisteredRemovalCount == 1);
assert(certificate.optionalRegisteredRemovalCount == 0);
assert(certificate.nonregisteredAdditionCount == 1);
assert(certificate.registeredRemovalCount == ...
    certificate.forcedRegisteredRemovalCount + ...
    certificate.optionalRegisteredRemovalCount);
assert(certificate.searchCandidateCounts. ...
    selectedLayerMatchingFeasibleCount == 4);
assert(abs(certificate.selectedScore.bottleneckReliability - ...
    0.752240320) < 1e-8);
assert(certificate.currentGeometryUsed && ...
    certificate.currentPhysicalPageUsed && ...
    certificate.currentLinkProbabilityPageUsed);
assert(~certificate.futureGeometryUsed && ...
    ~certificate.futurePhysicalPageUsed && ...
    ~certificate.posteriorUsed && ~certificate.truthUsed && ...
    ~certificate.measurementUsed && ...
    ~certificate.realizedDeliveryUniformsUsed && ...
    ~certificate.runtimeOverridesAccepted && ...
    ~certificate.scoreFunctionAccepted);
identityPayload = rmfield(certificate, ...
    {'canonicalSha256', 'arrayOrderExecutionCanonicalSha256'});
assert(strcmp(certificate.canonicalSha256, ...
    computeCanonicalValueSha256(identityPayload)));

context = v43Context(c, m, g, pDrop, 158, projectedBase);
[route, details] = ...
    selectIndexEquivariantFormationBackbonePolicy(context);
assert(nnz(route) == 2 * c.numberOfSensors);
assert(all(sum(route, 2) == 2));
physicalPage = g.physicalAdjacency(:, :, 158);
assert(~any(route(:) & ~physicalPage(:)));
assert(isStronglyConnected(route));
assert(details.instantaneousSensorStrongConnected);
assert(details.instantaneousFormationStrongConnected);
assert(all(abs(sum(details.fusionWeightMatrix, 2) - 1) < 1e-12));
assertProjectionWitnessesMatchV43( ...
    certificate.matchingWitnesses, details.construction);
end

function testPermutationAndGroupRelabel(fixture)
[baselineBase, baseline] = projectAt(fixture, 158);
c = fixture.config;
m = fixture.metadata;
g = fixture.graph;
pDrop = fixture.pDropByEdge(:, :, 158);
nodeCount = c.numberOfSensors;
order = [2:2:nodeCount, 1:2:nodeCount];
permutedGroups = c.sensorGroupIds(order);
originalPermutedGroups = permutedGroups;
labels = unique(c.sensorGroupIds);
replacementLabels = [91, 17, 63, 42, 88, 5];
for labelIdx = 1:numel(labels)
    permutedGroups(originalPermutedGroups == labels(labelIdx)) = ...
        replacementLabels(labelIdx);
end
[permutedBase, permuted] = ...
    projectCausalMinimalEditFormationBackbone( ...
        permutedGroups, m.sensorPhysicalUids(order), ...
        m.formationPhysicalUidsBySensor(order), ...
        g.positions(:, order, 158), ...
        fixture.registry.baseAdjacency(order, order), ...
        g.physicalAdjacency(order, order, 158), pDrop(order, order));
restored = false(nodeCount);
restored(order, order) = permutedBase;
assert(isequal(restored, baselineBase));
assert(strcmp(permuted.canonicalSha256, baseline.canonicalSha256));
assert(strcmp(permuted.currentInputCanonicalSha256, ...
    baseline.currentInputCanonicalSha256));
assert(~strcmp(permuted.arrayOrderExecutionCanonicalSha256, ...
    baseline.arrayOrderExecutionCanonicalSha256));
assert(isequal(permuted.registeredPairPhysicalUids, ...
    baseline.registeredPairPhysicalUids));
assert(isequal(permuted.projectedPairPhysicalUids, ...
    baseline.projectedPairPhysicalUids));
assert(isequal(permuted.matchingWitnesses, baseline.matchingWitnesses));
end

function testPDropSenderReceiverOrientation()
fixture = twoFormationOrientationFixture();
[~, certificate] = callFixture(fixture);
receiverIdx = find( ...
    [certificate.matchingWitnesses.receiverFormationPhysicalUid] == 20);
assert(numel(receiverIdx) == 1);
witness = certificate.matchingWitnesses(receiverIdx);
assert(isequal(witness.receiverSensorPhysicalUids, 201));
assert(isequal(witness.senderSensorPhysicalUids, 103));
end

function testHallConflictChoosesAlternative()
fixture = hallFixture(true);
[~, certificate] = callFixture(fixture);
assert(isequal(certificate.removedRegisteredPairPhysicalUids, [20, 30]));
assert(isequal(certificate.addedNonregisteredPairPhysicalUids, [20, 40]));
assert(certificate.registeredRemovalCount == 1);
assert(certificate.nonregisteredAdditionCount == 1);
end

function testNoHallFeasibleConnectedGraphFailsClosed()
fixture = hallFixture(false);
assertErrorId(@() callFixture(fixture), ...
    'CausalFormationProjection:NoFeasibleBackbone');
end

function testX48DegreeSevenStarFailsClosed()
formationCount = 8;
sensorsPerFormation = 6;
fixture = blankFixture(formationCount, sensorsPerFormation);
pairs = [(ones(formationCount - 1, 1)), (2:formationCount)'];
fixture.registered = placeholderFromPairs( ...
    formationCount, sensorsPerFormation, pairs);
for leaf = 2:formationCount
    centerMembers = formationMembers(1, sensorsPerFormation);
    leafMembers = formationMembers(leaf, sensorsPerFormation);
    fixture.physical(centerMembers, leafMembers) = true;
    fixture.physical(leafMembers, centerMembers) = true;
end
assertErrorId(@() callFixture(fixture), ...
    'CausalFormationProjection:NoFeasibleBackbone');
end

function testFormationCountDevelopmentLimit()
fixture = blankFixture(9, 3);
pairs = [(1:8)', (2:9)'];
fixture.registered = placeholderFromPairs(9, 3, pairs);
fixture.physical = fixture.registered;
assertErrorId(@() callFixture(fixture), ...
    'CausalFormationProjection:InvalidInput');
end

function testFuturePageRejected(fixture)
c = fixture.config;
m = fixture.metadata;
g = fixture.graph;
futurePhysical = cat(3, g.physicalAdjacency(:, :, 158), ...
    g.physicalAdjacency(:, :, 159));
assertErrorId(@() projectCausalMinimalEditFormationBackbone( ...
    c.sensorGroupIds, m.sensorPhysicalUids, ...
    m.formationPhysicalUidsBySensor, g.positions(:, :, 158), ...
    fixture.registry.baseAdjacency, futurePhysical, ...
    fixture.pDropByEdge(:, :, 158)), ...
    'CausalFormationProjection:InvalidInput');
end

function [base, certificate] = projectAt(fixture, currentTime)
c = fixture.config;
m = fixture.metadata;
g = fixture.graph;
[base, certificate] = projectCausalMinimalEditFormationBackbone( ...
    c.sensorGroupIds, m.sensorPhysicalUids, ...
    m.formationPhysicalUidsBySensor, g.positions(:, :, currentTime), ...
    fixture.registry.baseAdjacency, ...
    g.physicalAdjacency(:, :, currentTime), ...
    fixture.pDropByEdge(:, :, currentTime));
end

function context = v43Context(config, metadata, graph, pDrop, ...
        currentTime, baseAdjacency)
nodeCount = config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', config.sensorGroupIds), ...
    'staticAdjacency', logical(baseAdjacency)));
context.baseAdjacency = logical(baseAdjacency);
context.physicalAdjacency = logical( ...
    graph.physicalAdjacency(:, :, currentTime));
context.positions = graph.positions(:, :, currentTime);
context.sensorPhysicalUids = metadata.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    metadata.formationPhysicalUidsBySensor;
context.commConfig = struct('pDropByEdge', ...
    pDrop(:, :, currentTime));
context.directedMessageBudget = 2 * nodeCount;
end

function fixture = twoFormationOrientationFixture()
fixture = blankFixture(2, 3);
fixture.sensorUids = [101, 102, 103, 201, 202, 203];
fixture.formationUids = [10, 10, 10, 20, 20, 20];
fixture.registered = placeholderFromPairs(2, 3, [1, 2]);
fixture.physical(4, 1:3) = true;
fixture.physical(1:3, 4) = true;
fixture.pDrop(:) = 0.90;
fixture.pDrop(1:size(fixture.pDrop, 1)+1:end) = 1;
fixture.pDrop(1, 4) = 0.20;
fixture.pDrop(2, 4) = 0.30;
fixture.pDrop(3, 4) = 0.01;
fixture.pDrop(4, 1) = 0.40;
fixture.pDrop(4, 2) = 0.50;
fixture.pDrop(4, 3) = 0.60;
end

function fixture = hallFixture(includeAlternative)
fixture = blankFixture(4, 3);
fixture.registered = placeholderFromPairs( ...
    4, 3, [1, 2; 2, 3; 3, 4]);
fixture.physical = false(12);
fixture.physical = addSymmetricEdge(fixture.physical, 1, 4);   % 1-2
fixture.physical = addSymmetricEdge(fixture.physical, 7, 10);  % 3-4
fixture.physical = addSymmetricEdge(fixture.physical, 1, 7);   % bad 1-3
if includeAlternative
    fixture.physical = addSymmetricEdge( ...
        fixture.physical, 5, 11);                              % good 2-4
end
end

function fixture = blankFixture(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
fixture = struct();
fixture.groupIds = repelem(1:formationCount, sensorsPerFormation);
fixture.sensorUids = zeros(1, nodeCount);
fixture.formationUids = zeros(1, nodeCount);
for formationIdx = 1:formationCount
    members = formationMembers(formationIdx, sensorsPerFormation);
    fixture.sensorUids(members) = ...
        formationIdx * 100 + (1:sensorsPerFormation);
    fixture.formationUids(members) = formationIdx * 10;
end
fixture.positions = [1:nodeCount; zeros(1, nodeCount)];
fixture.registered = false(nodeCount);
fixture.physical = false(nodeCount);
fixture.pDrop = 0.20 * ones(nodeCount);
fixture.pDrop(1:nodeCount+1:end) = 1;
end

function [base, certificate] = callFixture(fixture)
[base, certificate] = projectCausalMinimalEditFormationBackbone( ...
    fixture.groupIds, fixture.sensorUids, fixture.formationUids, ...
    fixture.positions, fixture.registered, fixture.physical, ...
    fixture.pDrop);
end

function adjacency = placeholderFromPairs( ...
        formationCount, sensorsPerFormation, pairs)
nodeCount = formationCount * sensorsPerFormation;
adjacency = false(nodeCount);
for pairIdx = 1:size(pairs, 1)
    left = formationMembers(pairs(pairIdx, 1), sensorsPerFormation);
    right = formationMembers(pairs(pairIdx, 2), sensorsPerFormation);
    adjacency(left(1), right(1)) = true;
    adjacency(right(1), left(1)) = true;
end
end

function members = formationMembers(formationIdx, sensorsPerFormation)
members = (formationIdx - 1) * sensorsPerFormation + ...
    (1:sensorsPerFormation);
end

function adjacency = addSymmetricEdge(adjacency, left, right)
adjacency(left, right) = true;
adjacency(right, left) = true;
end

function assertProjectionWitnessesMatchV43(witnesses, construction)
assert(numel(witnesses) == numel(construction.crossAssignmentDetails));
for formationIdx = 1:numel(witnesses)
    expected = witnesses(formationIdx);
    actual = construction.crossAssignmentDetails{formationIdx};
    assert(actual.receiverFormationPhysicalUid == ...
        expected.receiverFormationPhysicalUid);
    assert(isequal(actual.senderFormationPhysicalUids, ...
        expected.senderFormationPhysicalUids));
    assert(isequal(actual.receiverSensorPhysicalUids, ...
        expected.receiverSensorPhysicalUids));
    assert(isequal(actual.senderSensorPhysicalUids, ...
        expected.senderSensorPhysicalUids));
    assert(isequal(actual.score, expected.score));
end
actualRows = sortrows([ ...
    reshape(construction.crossSenderFormationPhysicalUids, [], 1), ...
    reshape(construction.crossReceiverFormationPhysicalUids, [], 1), ...
    reshape(construction.crossSenderPhysicalUids, [], 1), ...
    reshape(construction.crossReceiverPhysicalUids, [], 1)], ...
    [1, 2, 3, 4]);
expectedRows = zeros(0, 4);
for formationIdx = 1:numel(witnesses)
    witness = witnesses(formationIdx);
    expectedRows = [expectedRows; ... %#ok<AGROW>
        reshape(witness.senderFormationPhysicalUids, [], 1), ...
        repmat(witness.receiverFormationPhysicalUid, ...
            numel(witness.senderFormationPhysicalUids), 1), ...
        reshape(witness.senderSensorPhysicalUids, [], 1), ...
        reshape(witness.receiverSensorPhysicalUids, [], 1)];
end
expectedRows = sortrows(expectedRows, [1, 2, 3, 4]);
assert(isequal(actualRows, expectedRows));
end

function assertErrorId(callable, expectedId)
failed = false;
try
    callable();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expectedId);
    if ~failed
        rethrow(errorInfo);
    end
end
assert(failed, 'Expected error was not thrown: %s', expectedId);
end

function connected = isStronglyConnected(adjacency)
connected = reachableAll(adjacency') && reachableAll(adjacency);
end

function connected = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
connected = all(visited);
end
