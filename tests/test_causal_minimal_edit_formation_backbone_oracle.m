function test_causal_minimal_edit_formation_backbone_oracle()
% Independent exhaustive oracle for the frozen V46 projection objective.
%
% The oracle never calls a projector helper.  It independently collapses
% the registered/current formation graphs, enumerates every current-
% quotient edge subset, and asks the unchanged V43 constructor for the
% deterministic sensor assignment realized by each connected candidate.
% Exactness here is therefore over candidate topologies conditional on the
% frozen V43 constructor; it is not a joint re-optimization of V43 edges.

fixtures = {buildF3NoOpFixture(), ...
    buildF4ForcedRepairFixture(), ...
    buildF5HallConflictFixture(), ...
    buildF6TwoRepairFixture()};
for fixtureIdx = 1:numel(fixtures)
    fixture = fixtures{fixtureIdx};
    verifyProjectorAgainstOracle(fixture);
    verifyProjectorAgainstOracle(permutedAndRelabeled(fixture));
end
testLocalCycleBrokenCompositionBoundary();
fprintf('PASS: causal minimal-edit formation-backbone oracle tests\n');
end

function verifyProjectorAgainstOracle(fixture)
[projectedBase, actual] = ...
    projectCausalMinimalEditFormationBackbone( ...
        fixture.groupIds, fixture.sensorUids, ...
        fixture.formationUidsBySensor, fixture.positions, ...
        fixture.registeredBaseAdjacency, fixture.physicalAdjacency, ...
        fixture.pDropByEdge);
expected = exhaustiveOracle(fixture);

assert(isequal(actual.projectedPairPhysicalUids, ...
    expected.projectedPairPhysicalUids), fixture.name);
assert(isequal(projectedBase, expected.projectedBaseAdjacency), ...
    fixture.name);
assert(actual.registeredRemovalCount == ...
    expected.registeredRemovalCount, fixture.name);
assert(actual.nonregisteredAdditionCount == ...
    expected.nonregisteredAdditionCount, fixture.name);
assert(actual.forcedRegisteredRemovalCount == ...
    expected.forcedRegisteredRemovalCount, fixture.name);
assert(actual.optionalRegisteredRemovalCount == ...
    expected.optionalRegisteredRemovalCount, fixture.name);
assertScoreEqual(actual.selectedScore, expected.selectedScore, ...
    fixture.name);
assertWitnessesEqual(actual.matchingWitnesses, ...
    expected.matchingWitnesses, fixture.name);

% Re-run the chosen candidate through the unchanged downstream constructor.
[~, ~, composed] = buildIndexEquivariantFormationBackboneReference( ...
    fixture.groupIds, fixture.sensorUids, ...
    fixture.formationUidsBySensor, fixture.positions, projectedBase, ...
    fixture.physicalAdjacency, fixture.pDropByEdge);
composedWitnesses = detailsToWitnesses(composed);
assertWitnessesEqual(actual.matchingWitnesses, composedWitnesses, ...
    [fixture.name, '-downstream-parity']);
end

function expected = exhaustiveOracle(fixture)
formation = canonicalMembership(fixture.groupIds, fixture.sensorUids, ...
    fixture.formationUidsBySensor);
registeredFormation = collapseRegistered( ...
    fixture.registeredBaseAdjacency, formation.members);
currentQuotient = collapseCurrentPhysical( ...
    fixture.physicalAdjacency, formation.members);
registeredPairs = upperPairs(registeredFormation);
quotientPairs = upperPairs(currentQuotient);
if size(quotientPairs, 1) > 20
    error('OracleFixture:TooDense', ...
        'The deterministic oracle fixture is unexpectedly dense.');
end

best = struct();
bestFound = false;
subsetCount = 2 ^ size(quotientPairs, 1);
for subsetMask = 0:subsetCount-1
    selectedMask = logical(bitget(subsetMask, 1:size(quotientPairs, 1)));
    candidatePairs = quotientPairs(selectedMask, :);
    candidateFormation = pairsToAdjacency( ...
        candidatePairs, formation.count);
    if ~isConnected(candidateFormation)
        continue;
    end
    candidateBase = materializePlaceholder( ...
        candidatePairs, formation.members, fixture.sensorUids, ...
        numel(fixture.sensorUids));
    try
        [~, ~, details] = ...
            buildIndexEquivariantFormationBackboneReference( ...
                fixture.groupIds, fixture.sensorUids, ...
                fixture.formationUidsBySensor, fixture.positions, ...
                candidateBase, fixture.physicalAdjacency, ...
                fixture.pDropByEdge);
    catch errorInfo
        if ismember(errorInfo.identifier, { ...
                'IndexEquivariantFormationRoute:InvalidBackbone', ...
                'IndexEquivariantFormationRoute:NoCrossAssignment'})
            continue;
        end
        rethrow(errorInfo);
    end
    removedCount = nnz(~ismember(registeredPairs, candidatePairs, 'rows'));
    addedCount = nnz(~ismember(candidatePairs, registeredPairs, 'rows'));
    witnesses = detailsToWitnesses(details);
    score = aggregateV43Score(witnesses);
    candidate = struct('rankPairs', candidatePairs, ...
        'baseAdjacency', candidateBase, ...
        'removedCount', removedCount, 'addedCount', addedCount, ...
        'witnesses', witnesses, 'score', score);
    if ~bestFound || objectiveBetter(candidate, best)
        best = candidate;
        bestFound = true;
    end
end
assert(bestFound, ['Oracle found no feasible topology for ', fixture.name]);

forcedRemovedCount = nnz(~ismember( ...
    registeredPairs, quotientPairs, 'rows'));
expected = struct();
expected.projectedPairPhysicalUids = rankPairsToUidPairs( ...
    best.rankPairs, formation.uids);
expected.projectedBaseAdjacency = best.baseAdjacency;
expected.registeredRemovalCount = best.removedCount;
expected.nonregisteredAdditionCount = best.addedCount;
expected.forcedRegisteredRemovalCount = forcedRemovedCount;
expected.optionalRegisteredRemovalCount = ...
    best.removedCount - forcedRemovedCount;
expected.matchingWitnesses = best.witnesses;
expected.selectedScore = best.score;
end

function better = objectiveBetter(candidate, incumbent)
if candidate.removedCount ~= incumbent.removedCount
    better = candidate.removedCount < incumbent.removedCount;
elseif candidate.addedCount ~= incumbent.addedCount
    better = candidate.addedCount < incumbent.addedCount;
else
    better = scoreBetter(candidate.score, incumbent.score);
end
end

function witnesses = detailsToWitnesses(details)
count = numel(details.crossAssignmentDetails);
template = details.crossAssignmentDetails{1};
witnesses = repmat(template, 1, count);
for formationIdx = 1:count
    witnesses(formationIdx) = details.crossAssignmentDetails{formationIdx};
end
end

function score = aggregateV43Score(witnesses)
score = emptyScore();
score.bottleneckReliability = Inf;
score.totalLogReliability = 0;
score.totalDistance = 0;
edgeRows = zeros(0, 4);
for formationIdx = 1:numel(witnesses)
    witness = witnesses(formationIdx);
    score.bottleneckReliability = min( ...
        score.bottleneckReliability, ...
        witness.score.bottleneckReliability);
    score.totalLogReliability = score.totalLogReliability + ...
        witness.score.totalLogReliability;
    score.totalDistance = score.totalDistance + ...
        witness.score.totalDistance;
    edgeRows = [edgeRows; ... %#ok<AGROW>
        reshape(witness.senderFormationPhysicalUids, [], 1), ...
        repmat(witness.receiverFormationPhysicalUid, ...
            numel(witness.senderFormationPhysicalUids), 1), ...
        reshape(witness.senderSensorPhysicalUids, [], 1), ...
        reshape(witness.receiverSensorPhysicalUids, [], 1)];
end
edgeRows = sortrows(edgeRows, [1, 2, 3, 4]);
score.tieKey = reshape(edgeRows', 1, []);
end

function better = scoreBetter(candidate, incumbent)
tolerance = 1e-12;
if candidate.bottleneckReliability > ...
        incumbent.bottleneckReliability + tolerance
    better = true;
    return;
elseif candidate.bottleneckReliability < ...
        incumbent.bottleneckReliability - tolerance
    better = false;
    return;
end
if candidate.totalLogReliability > ...
        incumbent.totalLogReliability + tolerance
    better = true;
    return;
elseif candidate.totalLogReliability < ...
        incumbent.totalLogReliability - tolerance
    better = false;
    return;
end
if candidate.totalDistance < incumbent.totalDistance - tolerance
    better = true;
    return;
elseif candidate.totalDistance > incumbent.totalDistance + tolerance
    better = false;
    return;
end
better = lexicographicallyLess(candidate.tieKey, incumbent.tieKey);
end

function less = lexicographicallyLess(left, right)
if isempty(right)
    less = true;
    return;
end
count = min(numel(left), numel(right));
firstDifference = find(left(1:count) ~= right(1:count), 1, 'first');
if isempty(firstDifference)
    less = numel(left) < numel(right);
else
    less = left(firstDifference) < right(firstDifference);
end
end

function testLocalCycleBrokenCompositionBoundary()
fixture = buildF3NoOpFixture();
fixture.name = 'f3-cross-feasible-local-cycle-broken';
brokenMembers = find(fixture.groupIds == fixture.storageGroupLabels(1));
fixture.physicalAdjacency(brokenMembers, brokenMembers) = false;
fixture.physicalAdjacency(brokenMembers(1), brokenMembers(2)) = true;
fixture.physicalAdjacency(brokenMembers(2), brokenMembers(1)) = true;
[projectedBase, certificate] = ...
    projectCausalMinimalEditFormationBackbone( ...
        fixture.groupIds, fixture.sensorUids, ...
        fixture.formationUidsBySensor, fixture.positions, ...
        fixture.registeredBaseAdjacency, fixture.physicalAdjacency, ...
        fixture.pDropByEdge);

% The projector certificate is intentionally cross-formation only.  It
% certifies a repaired formation-pair registry and matching witnesses, not
% the local dual-cycle or full 2N route.  The composed runtime/V43 gate must
% still run and fail closed when the independent local-cycle assumption is
% violated.
assert(strcmp(certificate.contractVersion, ...
    'causal-minimal-edit-formation-backbone-projection-v1'));
assert(~isfield(certificate, 'fullRoutePassed'));
assert(~isempty(certificate.matchingWitnesses));
assertErrorId(@() buildIndexEquivariantFormationBackboneReference( ...
    fixture.groupIds, fixture.sensorUids, ...
    fixture.formationUidsBySensor, fixture.positions, projectedBase, ...
    fixture.physicalAdjacency, fixture.pDropByEdge), ...
    'IndexEquivariantFormationRoute:NoLocalCycle');
end

function fixture = buildF3NoOpFixture()
fixture = baseFixture('f3-no-op', [330, 110, 220], [7, 2, 9]);
fixture = setRegisteredPairs(fixture, [1, 2; 2, 3]);
fixture = addFullPhysicalPair(fixture, 1, 2);
fixture = addFullPhysicalPair(fixture, 2, 3);
fixture = addFullPhysicalPair(fixture, 1, 3);
fixture = finalizeLinkProbabilities(fixture);
end

function fixture = buildF4ForcedRepairFixture()
fixture = baseFixture('f4-forced-remove-add', ...
    [440, 120, 360, 250], [8, 3, 11, 5]);
fixture = setRegisteredPairs(fixture, [1, 2; 2, 3; 3, 4]);
fixture = addFullPhysicalPair(fixture, 1, 2);
fixture = addFullPhysicalPair(fixture, 3, 4);
fixture = addFullPhysicalPair(fixture, 1, 3);
fixture = addFullPhysicalPair(fixture, 2, 4);
fixture = finalizeLinkProbabilities(fixture);
end

function fixture = buildF5HallConflictFixture()
fixture = baseFixture('f5-hall-conflict', ...
    [510, 130, 470, 260, 390], [12, 4, 15, 6, 9]);
fixture = setRegisteredPairs(fixture, ...
    [1, 2; 2, 3; 3, 4; 4, 5]);
fixture = addSpecificPhysicalEdge(fixture, 1, 1, 2, 1); % active 1-2
fixture = addSpecificPhysicalEdge(fixture, 3, 1, 4, 1); % active 3-4
fixture = addSpecificPhysicalEdge(fixture, 4, 2, 5, 1); % active 4-5
fixture = addSpecificPhysicalEdge(fixture, 1, 1, 3, 2); % Hall-bad 1-3
fixture = addSpecificPhysicalEdge(fixture, 2, 2, 4, 3); % feasible 2-4
fixture = finalizeLinkProbabilities(fixture);
end

function fixture = buildF6TwoRepairFixture()
fixture = baseFixture('f6-two-forced-repairs', ...
    [620, 140, 570, 280, 450, 330], [14, 3, 12, 5, 10, 7]);
fixture = setRegisteredPairs(fixture, ...
    [1, 2; 2, 3; 3, 4; 4, 5; 5, 6]);
fixture = addFullPhysicalPair(fixture, 1, 2);
fixture = addFullPhysicalPair(fixture, 3, 4);
fixture = addFullPhysicalPair(fixture, 5, 6);
fixture = addFullPhysicalPair(fixture, 1, 3);
fixture = addFullPhysicalPair(fixture, 2, 4);
fixture = addFullPhysicalPair(fixture, 3, 5);
fixture = addFullPhysicalPair(fixture, 4, 6);
fixture = finalizeLinkProbabilities(fixture);
end

function fixture = baseFixture(name, formationUidsByStorageGroup, ...
        storageGroupLabels)
formationCount = numel(formationUidsByStorageGroup);
sensorsPerFormation = 3;
nodeCount = formationCount * sensorsPerFormation;
fixture = struct();
fixture.name = name;
fixture.sensorsPerFormation = sensorsPerFormation;
fixture.storageGroupLabels = storageGroupLabels;
fixture.groupIds = repelem(storageGroupLabels, sensorsPerFormation);
fixture.sensorUids = zeros(1, nodeCount);
fixture.formationUidsBySensor = zeros(1, nodeCount);
fixture.positions = zeros(2, nodeCount);
for storageGroup = 1:formationCount
    members = storageMembers(storageGroup, sensorsPerFormation);
    formationUid = formationUidsByStorageGroup(storageGroup);
    fixture.sensorUids(members) = ...
        formationUid * 100 + (1:sensorsPerFormation);
    fixture.formationUidsBySensor(members) = formationUid;
    fixture.positions(:, members) = [ ...
        formationUid + (1:sensorsPerFormation) / 10; ...
        mod(formationUid, 17) + (1:sensorsPerFormation) / 100];
end
fixture.registeredBaseAdjacency = false(nodeCount);
fixture.physicalAdjacency = false(nodeCount);
for storageGroup = 1:formationCount
    members = storageMembers(storageGroup, sensorsPerFormation);
    fixture.physicalAdjacency(members, members) = true;
end
fixture.physicalAdjacency(1:nodeCount+1:end) = false;
fixture.pDropByEdge = ones(nodeCount);
end

function fixture = setRegisteredPairs(fixture, storagePairs)
for pairIdx = 1:size(storagePairs, 1)
    leftMembers = storageMembers( ...
        storagePairs(pairIdx, 1), fixture.sensorsPerFormation);
    rightMembers = storageMembers( ...
        storagePairs(pairIdx, 2), fixture.sensorsPerFormation);
    [~, leftLocal] = min(fixture.sensorUids(leftMembers));
    [~, rightLocal] = min(fixture.sensorUids(rightMembers));
    left = leftMembers(leftLocal);
    right = rightMembers(rightLocal);
    fixture.registeredBaseAdjacency(left, right) = true;
    fixture.registeredBaseAdjacency(right, left) = true;
end
end

function fixture = addFullPhysicalPair(fixture, leftStorage, rightStorage)
left = storageMembers(leftStorage, fixture.sensorsPerFormation);
right = storageMembers(rightStorage, fixture.sensorsPerFormation);
fixture.physicalAdjacency(left, right) = true;
fixture.physicalAdjacency(right, left) = true;
end

function fixture = addSpecificPhysicalEdge(fixture, ...
        leftStorage, leftRole, rightStorage, rightRole)
leftMembers = storageMembers(leftStorage, fixture.sensorsPerFormation);
rightMembers = storageMembers(rightStorage, fixture.sensorsPerFormation);
left = leftMembers(leftRole);
right = rightMembers(rightRole);
fixture.physicalAdjacency(left, right) = true;
fixture.physicalAdjacency(right, left) = true;
end

function fixture = finalizeLinkProbabilities(fixture)
nodeCount = numel(fixture.sensorUids);
fixture.pDropByEdge = ones(nodeCount);
for sender = 1:nodeCount
    for receiver = 1:nodeCount
        if ~fixture.physicalAdjacency(receiver, sender)
            continue;
        end
        reliability = 0.65 + mod( ...
            3 * fixture.sensorUids(sender) + ...
            7 * fixture.sensorUids(receiver), 31) / 100;
        fixture.pDropByEdge(sender, receiver) = 1 - reliability;
    end
end
fixture.pDropByEdge(1:nodeCount+1:end) = 1;
end

function permuted = permutedAndRelabeled(fixture)
nodeCount = numel(fixture.sensorUids);
order = [2:2:nodeCount, 1:2:nodeCount];
permuted = fixture;
permuted.name = [fixture.name, '-permuted-relabeled'];
permuted.groupIds = fixture.groupIds(order);
originalGroups = permuted.groupIds;
for storageGroup = 1:numel(fixture.storageGroupLabels)
    oldLabel = fixture.storageGroupLabels(storageGroup);
    newLabel = 101 + 13 * storageGroup;
    permuted.groupIds(originalGroups == oldLabel) = newLabel;
end
permuted.sensorUids = fixture.sensorUids(order);
permuted.formationUidsBySensor = ...
    fixture.formationUidsBySensor(order);
permuted.positions = fixture.positions(:, order);
permuted.registeredBaseAdjacency = ...
    fixture.registeredBaseAdjacency(order, order);
permuted.physicalAdjacency = fixture.physicalAdjacency(order, order);
permuted.pDropByEdge = fixture.pDropByEdge(order, order);
end

function formation = canonicalMembership(groupIds, sensorUids, ...
        formationUidsBySensor)
labels = unique(groupIds);
uidsByLabel = zeros(1, numel(labels));
membersByLabel = cell(1, numel(labels));
for labelIdx = 1:numel(labels)
    members = find(groupIds == labels(labelIdx));
    formationUid = unique(formationUidsBySensor(members));
    assert(numel(formationUid) == 1);
    uidsByLabel(labelIdx) = formationUid;
    [~, order] = sort(sensorUids(members));
    membersByLabel{labelIdx} = members(order);
end
[uids, order] = sort(uidsByLabel);
formation = struct('count', numel(uids), 'uids', uids, ...
    'members', {membersByLabel(order)});
end

function adjacency = collapseRegistered(baseAdjacency, members)
count = numel(members);
adjacency = false(count);
for leftIdx = 1:count-1
    for rightIdx = leftIdx+1:count
        present = any(any(baseAdjacency( ...
            members{leftIdx}, members{rightIdx})));
        adjacency(leftIdx, rightIdx) = present;
        adjacency(rightIdx, leftIdx) = present;
    end
end
end

function adjacency = collapseCurrentPhysical(physical, members)
count = numel(members);
adjacency = false(count);
for leftIdx = 1:count-1
    for rightIdx = leftIdx+1:count
        forward = any(any(physical( ...
            members{leftIdx}, members{rightIdx})));
        reverse = any(any(physical( ...
            members{rightIdx}, members{leftIdx})));
        present = forward && reverse;
        adjacency(leftIdx, rightIdx) = present;
        adjacency(rightIdx, leftIdx) = present;
    end
end
end

function pairs = upperPairs(adjacency)
[left, right] = find(triu(adjacency, 1));
pairs = sortrows([left, right], [1, 2]);
end

function adjacency = pairsToAdjacency(pairs, count)
adjacency = false(count);
for pairIdx = 1:size(pairs, 1)
    left = pairs(pairIdx, 1);
    right = pairs(pairIdx, 2);
    adjacency(left, right) = true;
    adjacency(right, left) = true;
end
end

function base = materializePlaceholder(pairs, members, sensorUids, nodeCount)
base = false(nodeCount);
for pairIdx = 1:size(pairs, 1)
    [~, leftLocal] = min(sensorUids(members{pairs(pairIdx, 1)}));
    [~, rightLocal] = min(sensorUids(members{pairs(pairIdx, 2)}));
    left = members{pairs(pairIdx, 1)}(leftLocal);
    right = members{pairs(pairIdx, 2)}(rightLocal);
    base(left, right) = true;
    base(right, left) = true;
end
end

function uidPairs = rankPairsToUidPairs(rankPairs, formationUids)
if isempty(rankPairs)
    uidPairs = zeros(0, 2);
else
    uidPairs = [formationUids(rankPairs(:, 1))', ...
        formationUids(rankPairs(:, 2))'];
    uidPairs = sortrows(sort(uidPairs, 2), [1, 2]);
end
end

function members = storageMembers(storageGroup, sensorsPerFormation)
members = (storageGroup - 1) * sensorsPerFormation + ...
    (1:sensorsPerFormation);
end

function assertScoreEqual(actual, expected, message)
tolerance = 1e-12;
assert(abs(actual.bottleneckReliability - ...
    expected.bottleneckReliability) <= tolerance, message);
assert(abs(actual.totalLogReliability - ...
    expected.totalLogReliability) <= tolerance, message);
assert(abs(actual.totalDistance - expected.totalDistance) ...
    <= tolerance, message);
assert(isequal(actual.tieKey, expected.tieKey), message);
end

function assertWitnessesEqual(actual, expected, message)
assert(numel(actual) == numel(expected), message);
for formationIdx = 1:numel(actual)
    assert(actual(formationIdx).receiverFormationPhysicalUid == ...
        expected(formationIdx).receiverFormationPhysicalUid, message);
    assert(isequal(actual(formationIdx).senderFormationPhysicalUids, ...
        expected(formationIdx).senderFormationPhysicalUids), message);
    assert(isequal(actual(formationIdx).receiverSensorPhysicalUids, ...
        expected(formationIdx).receiverSensorPhysicalUids), message);
    assert(isequal(actual(formationIdx).senderSensorPhysicalUids, ...
        expected(formationIdx).senderSensorPhysicalUids), message);
    assertScoreEqual(actual(formationIdx).score, ...
        expected(formationIdx).score, message);
end
end

function score = emptyScore()
score = struct('bottleneckReliability', -Inf, ...
    'totalLogReliability', -Inf, 'totalDistance', Inf, ...
    'tieKey', zeros(1, 0));
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

function connected = isConnected(adjacency)
if isempty(adjacency)
    connected = false;
    return;
end
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
