function test_index_equivariant_formation_backbone_reference()
% Structural, orientation and coordinate-equivariance contracts for v43.

testRealSceneReferenceContract();
testPhysicalIdentityRegistryBeforeStoragePermutation();
testArbitraryNodePermutationAndGroupRelabel();
testAsymmetricPDropOrientation();
testMissingOrDuplicateUidFailsClosed();
testEnumerationLimitFailsClosed();
fprintf('PASS: index-equivariant formation-backbone reference tests\n');
end

function testEnumerationLimitFailsClosed()
sensorsPerFormation = 10;
groupIds = repelem(1:2, sensorsPerFormation);
formationUids = repelem([10, 20], sensorsPerFormation);
sensorUids = [1001:1010, 2001:2010];
nodeCount = numel(groupIds);
positions = [1:nodeCount; zeros(1, nodeCount)];
registered = logical(ones(nodeCount) - eye(nodeCount));
physical = registered;
pDrop = 0.2 * ones(nodeCount);
assertErrorId(@() buildIndexEquivariantFormationBackboneReference( ...
    groupIds, sensorUids, formationUids, positions, registered, ...
    physical, pDrop), ...
    'IndexEquivariantFormationRoute:EnumerationLimitExceeded');
end

function testPhysicalIdentityRegistryBeforeStoragePermutation()
config = buildDynamicTopologyScenarioConfig( ...
    'm24-formation-fov-convoy');
baseline = buildDynamicTopologyPhysicalIdentityRegistry(config);
formationOrder = [3, 1, 4, 2];
reordered = config;
reordered.sensorCenterWaypoints = ...
    config.sensorCenterWaypoints(formationOrder);
reordered.sensorLocalRoleUidsByFormation = ...
    config.sensorLocalRoleUidsByFormation(formationOrder);
reordered.formationStochasticRoleUidsByFormation = ...
    config.formationStochasticRoleUidsByFormation(formationOrder);
actual = buildDynamicTopologyPhysicalIdentityRegistry(reordered);
for newFormationIdx = 1:numel(formationOrder)
    oldFormationIdx = formationOrder(newFormationIdx);
    assert(actual.formationPhysicalUids(newFormationIdx) == ...
        baseline.formationPhysicalUids(oldFormationIdx));
    oldMembers = (oldFormationIdx - 1) * config.sensorsPerFormation + ...
        (1:config.sensorsPerFormation);
    newMembers = (newFormationIdx - 1) * config.sensorsPerFormation + ...
        (1:config.sensorsPerFormation);
    assert(isequal(actual.sensorPhysicalUids(newMembers), ...
        baseline.sensorPhysicalUids(oldMembers)));
end

rng(821, 'twister');
[baselineFormationTrajectories, baselineFormationMetadata] = ...
    generateMultiFormationTrajectories(config);
reordered = rmfield(reordered, ...
    'physicalIdentityRegistryCanonicalSha256');
rng(821, 'twister');
[reorderedFormationTrajectories, reorderedFormationMetadata] = ...
    generateMultiFormationTrajectories(reordered);
for sensorIdx = 1:config.numberOfSensors
    uid = baselineFormationMetadata.sensorPhysicalUids(sensorIdx);
    actualIdx = find( ...
        reorderedFormationMetadata.sensorPhysicalUids == uid);
    assert(numel(actualIdx) == 1);
    assert(isequal(reorderedFormationTrajectories{actualIdx}, ...
        baselineFormationTrajectories{sensorIdx}));
end

roleOrder = [3, 1, 6, 2, 5, 4];
roleReordered = config;
roleReordered.sensorLocalRoleUidsByFormation = repmat( ...
    {roleOrder}, 1, config.formationCount);
roleRegistry = buildDynamicTopologyPhysicalIdentityRegistry( ...
    roleReordered);
rng(823, 'twister');
[baselineTrajectories, baselineMetadata] = ...
    generateMultiFormationTrajectories(config);
roleReordered = rmfield(roleReordered, ...
    'physicalIdentityRegistryCanonicalSha256');
rng(823, 'twister');
[roleTrajectories, roleMetadata] = ...
    generateMultiFormationTrajectories(roleReordered);
for sensorIdx = 1:config.numberOfSensors
    uid = baselineMetadata.sensorPhysicalUids(sensorIdx);
    actualIdx = find(roleMetadata.sensorPhysicalUids == uid);
    assert(numel(actualIdx) == 1);
    assert(isequal(roleTrajectories{actualIdx}, ...
        baselineTrajectories{sensorIdx}));
end
assert(isequal(roleMetadata.sensorPhysicalUids, ...
    roleRegistry.sensorPhysicalUids));
end

function testRealSceneReferenceContract()
[context, metadata] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        'm24-formation-fov', 41);
[adjacency, details] = ...
    selectIndexEquivariantFormationBackbonePolicy(context);
nodeCount = metadata.nodeCount;
assert(strcmp(details.contractVersion, ...
    'index-equivariant-formation-backbone-policy-v1'));
assert(nnz(adjacency) == 2 * nodeCount);
assert(all(sum(adjacency, 2) == 2));
assert(~any(adjacency(:) & ~context.physicalAdjacency(:)));
assert(all(details.dominantSourcesByReceiver ~= ...
    details.residualSourcesByReceiver));
assert(details.crossFormationMessageCount == ...
    nnz(details.construction.registeredFormationAdjacency));
assert(details.construction.fullPositiveGraphStronglyConnected);
assert(~details.construction.residualSingleDirectedCycleRequired);
assert(details.currentGeometryUsed);
assert(details.currentLinkReliabilityUsed);
assert(~details.arrayIndexTieBreakUsed);
assert(~details.posteriorUsed && ~details.truthUsed && ...
    ~details.futureOutcomeUsed);
assert(all(abs(sum(details.fusionWeightMatrix, 2) - 1) < 1e-12));

uids = details.formationPhysicalUids;
[candidate, candidateDetails] = ...
    selectTemporalIndexEquivariantFormationInputBundleSuspensionPolicy( ...
        context, uids(1));
assert(candidateDetails.messageSavingCount >= 1);
assert(nnz(candidate) == nnz(adjacency) - ...
    candidateDetails.messageSavingCount);
assert(candidateDetails.oneStepTopologyReservePassed);
end

function testArbitraryNodePermutationAndGroupRelabel()
[context, ~] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        'm24-formation-fov-convoy', 43);
[referenceAdjacency, referenceDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(context);
referenceWeights = referenceDetails.fusionWeightMatrix;
nodeCount = size(referenceAdjacency, 1);
groupIds = context.model.dynamicTopologyScenario.config.sensorGroupIds;

rng(971, 'twister');
orders = cell(1, 4);
orders{1} = randperm(nodeCount);
orders{2} = [find(groupIds == 3), find(groupIds == 1), ...
    find(groupIds == 4), find(groupIds == 2)];
within = 1:nodeCount;
members = find(groupIds == 2);
within(members) = fliplr(members);
orders{3} = within;
orders{4} = orders{1}(orders{3});
for orderIdx = 1:numel(orders)
    order = orders{orderIdx};
    [permuted, permutation] = ...
        permuteFormationIndexEquivariantContext(context, order);
    [actualAdjacency, actualDetails] = ...
        selectIndexEquivariantFormationBackbonePolicy(permuted);
    assertRestoresExactly(referenceAdjacency, actualAdjacency, order);
    assertRestoresExactly(referenceWeights, ...
        actualDetails.fusionWeightMatrix, order);
    assert(permutation.pureNodeCoordinatePermutation);
end

order = orders{1};
reorderedGroups = groupIds(order);
originalLabels = unique(reorderedGroups);
newLabels = [91, 17, 63, 42];
relabeled = reorderedGroups;
for labelIdx = 1:numel(originalLabels)
    relabeled(reorderedGroups == originalLabels(labelIdx)) = ...
        newLabels(labelIdx);
end
[permuted, permutation] = ...
    permuteFormationIndexEquivariantContext( ...
        context, order, relabeled);
[actualAdjacency, actualDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(permuted);
assert(permutation.formationLabelsChanged);
assertRestoresExactly(referenceAdjacency, actualAdjacency, order);
assertRestoresExactly(referenceWeights, ...
    actualDetails.fusionWeightMatrix, order);
end

function testAsymmetricPDropOrientation()
[groupIds, sensorUids, formationUids, positions, registered, physical] = ...
    buildSymmetricFixture();
nodeCount = numel(groupIds);
pDrop = 0.90 * ones(nodeCount);
pDrop(1:nodeCount+1:end) = 1;
% Repository orientation is sender row, receiver column.  Geometry is an
% exact tie, so only the correctly oriented page selects UID 103 -> 201.
positions(:) = 0;
pDrop(3, 4) = 0.01;
[dominant, residual, details] = ...
    buildIndexEquivariantFormationBackboneReference( ...
        groupIds, sensorUids, formationUids, positions, ...
        registered, physical, pDrop);
crossIdx = find(details.crossReceiverPhysicalUids == 201 & ...
    details.crossSenderFormationPhysicalUids == 10);
assert(numel(crossIdx) == 1);
assert(details.crossSenderPhysicalUids(crossIdx) == 103);

order = [4, 2, 6, 1, 5, 3];
[permutedDominant, permutedResidual] = ...
    buildIndexEquivariantFormationBackboneReference( ...
        groupIds(order), sensorUids(order), formationUids(order), ...
        positions(:, order), registered(order, order), ...
        physical(order, order), pDrop(order, order));
assertRestoresExactly(dominant, permutedDominant, order);
assertRestoresExactly(residual, permutedResidual, order);
end

function testMissingOrDuplicateUidFailsClosed()
[groupIds, sensorUids, formationUids, positions, registered, ...
        physical] = buildSymmetricFixture();
pDrop = 0.2 * ones(numel(groupIds));
assertErrorId(@() buildIndexEquivariantFormationBackboneReference( ...
    groupIds, sensorUids(1:end-1), formationUids, positions, ...
    registered, physical, pDrop), ...
    'IndexEquivariantFormationRoute:InvalidContract');
duplicate = sensorUids;
duplicate(2) = duplicate(1);
assertErrorId(@() buildIndexEquivariantFormationBackboneReference( ...
    groupIds, duplicate, formationUids, positions, registered, ...
    physical, pDrop), ...
    'IndexEquivariantFormationRoute:InvalidContract');
badFormation = formationUids;
badFormation(4:6) = formationUids(1);
assertErrorId(@() buildIndexEquivariantFormationBackboneReference( ...
    groupIds, sensorUids, badFormation, positions, registered, ...
    physical, pDrop), ...
    'IndexEquivariantFormationRoute:InvalidContract');
end

function [groupIds, sensorUids, formationUids, positions, ...
        registered, physical] = buildSymmetricFixture()
groupIds = [1, 1, 1, 2, 2, 2];
sensorUids = [101, 102, 103, 201, 202, 203];
formationUids = [10, 10, 10, 20, 20, 20];
angles = 2 * pi * (0:2) / 3;
positions = [[cos(angles); sin(angles)], ...
    [5 + cos(angles); sin(angles)]];
nodeCount = numel(groupIds);
physical = logical(ones(nodeCount) - eye(nodeCount));
registered = false(nodeCount);
registered(1:3, 1:3) = true;
registered(4:6, 4:6) = true;
registered(1:3, 4:6) = true;
registered(4:6, 1:3) = true;
registered(1:nodeCount+1:end) = false;
end

function assertRestoresExactly(expected, actual, newToOld)
restored = zeros(size(actual));
if islogical(actual)
    restored = false(size(actual));
end
restored(newToOld, newToOld) = actual;
assert(isequal(restored, expected));
end

function assertErrorId(callable, expectedId)
thrown = false;
try
    callable();
catch errorInfo
    thrown = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Unexpected error identifier: %s', errorInfo.identifier);
end
assert(thrown, 'Expected error was not thrown.');
end
