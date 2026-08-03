function [dominantAdjacency, residualAdjacency, details] = ...
    buildIndexEquivariantFormationBackboneReference( ...
        groupIds, sensorPhysicalUids, formationPhysicalUidsBySensor, ...
        positions, registeredAdjacency, physicalAdjacency, ...
        pDropByEdge)
% BUILDINDEXEQUIVARIANTFORMATIONBACKBONEREFERENCE Physical-UID route.
%
% Matrices use receiver rows and sender columns, except pDropByEdge, whose
% repository contract uses sender rows and receiver columns.  The latter is
% transposed exactly once below.  Current array indices are never used to
% break a route tie: immutable sensor/formation physical UIDs are carried
% through coordinate permutations and are the final lexicographic key.
%
% The construction has two layers:
%   1. reliability-aware directed cycles inside every formation;
%   2. one residual arc in each direction for every registered formation
%      pair, assigned to distinct receivers in the receiving formation.
%
% The residual layer need not itself be a Hamiltonian cycle.  Strong
% connectivity follows from the local dominant cycles and the doubled,
% connected registered formation backbone.

[contract, formation] = validateInputs( ...
    groupIds, sensorPhysicalUids, formationPhysicalUidsBySensor, ...
    positions, registeredAdjacency, physicalAdjacency, pDropByEdge);
nodeCount = contract.nodeCount;
reliability = 1 - pDropByEdge';

dominantAdjacency = false(nodeCount);
localResidualAdjacency = false(nodeCount);
dominantCycleDetails = cell(1, formation.count);
localResidualCycleDetails = cell(1, formation.count);
for formationIdx = 1:formation.count
    members = formation.members{formationIdx};
    [dominantOrder, dominantScore] = selectBestCycle( ...
        members, [], sensorPhysicalUids, positions, ...
        physicalAdjacency, reliability);
    dominantSources = sourcesFromCycleOrder(dominantOrder, nodeCount);
    [residualOrder, residualScore] = selectBestCycle( ...
        members, dominantSources, sensorPhysicalUids, positions, ...
        physicalAdjacency, reliability);
    residualSources = sourcesFromCycleOrder(residualOrder, nodeCount);
    dominantAdjacency = addSources( ...
        dominantAdjacency, members, dominantSources);
    localResidualAdjacency = addSources( ...
        localResidualAdjacency, members, residualSources);
    dominantCycleDetails{formationIdx} = cycleDetails( ...
        dominantOrder, dominantScore, sensorPhysicalUids);
    localResidualCycleDetails{formationIdx} = cycleDetails( ...
        residualOrder, residualScore, sensorPhysicalUids);
end

residualAdjacency = localResidualAdjacency;
crossSenders = zeros(1, 0);
crossReceivers = zeros(1, 0);
crossSenderFormationUids = zeros(1, 0);
crossReceiverFormationUids = zeros(1, 0);
incomingCrossReceiversByFormation = cell(1, formation.count);
incomingCrossSendersByFormation = cell(1, formation.count);
crossAssignmentDetails = cell(1, formation.count);
for receiverFormationIdx = 1:formation.count
    neighborIndices = find(formation.adjacency( ...
        receiverFormationIdx, :));
    [~, neighborOrder] = sort(formation.uids(neighborIndices));
    neighborIndices = neighborIndices(neighborOrder);
    receivers = formation.members{receiverFormationIdx};
    [selectedReceivers, selectedSenders, assignmentScore] = ...
        selectBestCrossAssignment( ...
            neighborIndices, receivers, formation, ...
            sensorPhysicalUids, positions, physicalAdjacency, ...
            reliability);
    for slotIdx = 1:numel(neighborIndices)
        receiver = selectedReceivers(slotIdx);
        sender = selectedSenders(slotIdx);
        residualAdjacency(receiver, :) = false;
        residualAdjacency(receiver, sender) = true;
        crossReceivers(end + 1) = receiver; %#ok<AGROW>
        crossSenders(end + 1) = sender; %#ok<AGROW>
        crossSenderFormationUids(end + 1) = ...
            formation.uids(neighborIndices(slotIdx)); %#ok<AGROW>
        crossReceiverFormationUids(end + 1) = ...
            formation.uids(receiverFormationIdx); %#ok<AGROW>
    end
    incomingCrossReceiversByFormation{receiverFormationIdx} = ...
        selectedReceivers;
    incomingCrossSendersByFormation{receiverFormationIdx} = ...
        selectedSenders;
    crossAssignmentDetails{receiverFormationIdx} = struct( ...
        'receiverFormationPhysicalUid', ...
            formation.uids(receiverFormationIdx), ...
        'senderFormationPhysicalUids', ...
            formation.uids(neighborIndices), ...
        'receiverSensorPhysicalUids', ...
            sensorPhysicalUids(selectedReceivers), ...
        'senderSensorPhysicalUids', ...
            sensorPhysicalUids(selectedSenders), ...
        'score', assignmentScore);
end

dominantSources = findOneSourcePerReceiver(dominantAdjacency);
localResidualSources = findOneSourcePerReceiver( ...
    localResidualAdjacency);
residualSources = findOneSourcePerReceiver(residualAdjacency);
crossMask = formationPhysicalUidsBySensor(:) ~= ...
    formationPhysicalUidsBySensor(:)';
selectedCross = residualAdjacency & crossMask;
expectedCrossCount = nnz(formation.adjacency);

if any(sum(dominantAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 2) ~= 1) || ...
        any(dominantSources == residualSources) || ...
        any(dominantAdjacency(:) & ~physicalAdjacency(:)) || ...
        any(residualAdjacency(:) & ~physicalAdjacency(:)) || ...
        nnz(selectedCross) ~= expectedCrossCount || ...
        ~registeredPairMultiplicityPassed( ...
            crossSenderFormationUids, crossReceiverFormationUids, ...
            formation.adjacency, formation.uids) || ...
        ~isStronglyConnected(dominantAdjacency | residualAdjacency)
    error('IndexEquivariantFormationRoute:InternalContract', ...
        'The constructed physical reference violates its route contract.');
end

details = struct();
details.contractVersion = ...
    'index-equivariant-formation-backbone-reference-v1';
details.nodeCount = nodeCount;
details.formationCount = formation.count;
details.sensorPhysicalUids = sensorPhysicalUids;
details.formationPhysicalUidsBySensor = ...
    formationPhysicalUidsBySensor;
details.formationPhysicalUids = formation.uids;
details.groupIds = groupIds;
details.registeredFormationAdjacency = formation.adjacency;
details.dominantSourcesByReceiver = dominantSources;
details.localResidualSourcesByReceiver = localResidualSources;
details.residualSourcesByReceiver = residualSources;
details.dominantCycleDetails = dominantCycleDetails;
details.localResidualCycleDetails = localResidualCycleDetails;
details.crossAssignmentDetails = crossAssignmentDetails;
details.crossReceivers = crossReceivers;
details.crossSenders = crossSenders;
details.crossReceiverPhysicalUids = ...
    sensorPhysicalUids(crossReceivers);
details.crossSenderPhysicalUids = sensorPhysicalUids(crossSenders);
details.crossReceiverFormationPhysicalUids = ...
    crossReceiverFormationUids;
details.crossSenderFormationPhysicalUids = ...
    crossSenderFormationUids;
details.incomingCrossReceiversByFormation = ...
    incomingCrossReceiversByFormation;
details.incomingCrossSendersByFormation = ...
    incomingCrossSendersByFormation;
details.crossFormationMessageCount = nnz(selectedCross);
details.expectedCrossFormationMessageCount = expectedCrossCount;
details.maximumCrossSourceLoad = maximumLoad(crossSenders, nodeCount);
details.maximumCrossReceiverLoad = maximumLoad(crossReceivers, nodeCount);
details.maximumEnumeratedCandidates = ...
    formation.maximumEnumeratedCandidates;
details.localCycleCandidateCounts = formation.localCycleCandidateCounts;
details.crossAssignmentCandidateCounts = ...
    formation.crossAssignmentCandidateCounts;
details.oneDominantInputPerSensor = true;
details.oneResidualInputPerSensor = true;
details.distinctInputsPerSensor = true;
details.localDominantCyclesStronglyConnected = true;
details.residualSingleDirectedCycleRequired = false;
details.fullPositiveGraphStronglyConnected = true;
details.registeredFormationPairsOnly = true;
details.currentGeometryUsed = true;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.sensorPhysicalUidTieBreakUsed = true;
details.arrayIndexTieBreakUsed = false;
details.posteriorUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
end

function [contract, formation] = validateInputs( ...
        groupIds, sensorUids, formationUidsBySensor, positions, ...
        registeredAdjacency, physicalAdjacency, pDropByEdge)
groupIds = reshape(groupIds, 1, []);
sensorUids = reshape(sensorUids, 1, []);
formationUidsBySensor = reshape(formationUidsBySensor, 1, []);
nodeCount = numel(groupIds);
matrixSize = [nodeCount, nodeCount];
numericMatricesValid = (isnumeric(registeredAdjacency) || ...
    islogical(registeredAdjacency)) && ...
    isreal(registeredAdjacency) && all(isfinite(registeredAdjacency(:))) && ...
    (isnumeric(physicalAdjacency) || islogical(physicalAdjacency)) && ...
    isreal(physicalAdjacency) && ...
    all(isfinite(physicalAdjacency(:))) && isnumeric(pDropByEdge) && ...
    isreal(pDropByEdge) && all(isfinite(pDropByEdge(:)));
if nodeCount < 4 || ~isnumeric(groupIds) || ~isreal(groupIds) || ...
        any(~isfinite(groupIds)) || any(groupIds ~= round(groupIds)) || ...
        ~isnumeric(sensorUids) || ~isreal(sensorUids) || ...
        any(~isfinite(sensorUids)) || ...
        numel(sensorUids) ~= nodeCount || ...
        numel(unique(sensorUids)) ~= nodeCount || ...
        ~isnumeric(formationUidsBySensor) || ...
        ~isreal(formationUidsBySensor) || ...
        any(~isfinite(formationUidsBySensor)) || ...
        numel(formationUidsBySensor) ~= nodeCount || ...
        ~isnumeric(positions) || ~isreal(positions) || ...
        ~isequal(size(positions), [2, nodeCount]) || ...
        any(~isfinite(positions(:))) || ~numericMatricesValid || ...
        ~isequal(size(registeredAdjacency), matrixSize) || ...
        ~isequal(size(physicalAdjacency), matrixSize) || ...
        ~isequal(size(pDropByEdge), matrixSize) || ...
        any(pDropByEdge(:) < 0) || any(pDropByEdge(:) > 1)
    error('IndexEquivariantFormationRoute:InvalidContract', ...
        'The physical route inputs are malformed or lack unique UIDs.');
end

registeredAdjacency = logical(registeredAdjacency);
physicalAdjacency = logical(physicalAdjacency);
registeredAdjacency(1:nodeCount+1:end) = false;
physicalAdjacency(1:nodeCount+1:end) = false;
if ~isequal(registeredAdjacency, registeredAdjacency')
    error('IndexEquivariantFormationRoute:InvalidContract', ...
        'The registered sensor graph must be symmetric.');
end

groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
formationUids = zeros(1, formationCount);
members = cell(1, formationCount);
for groupIdx = 1:formationCount
    groupMembers = find(groupIds == groupLabels(groupIdx));
    groupFormationUids = unique(formationUidsBySensor(groupMembers));
    if numel(groupFormationUids) ~= 1 || numel(groupMembers) < 3
        error('IndexEquivariantFormationRoute:InvalidContract', ...
            'Each formation needs one UID and at least three sensors.');
    end
    formationUids(groupIdx) = groupFormationUids;
    members{groupIdx} = groupMembers;
end
if numel(unique(formationUids)) ~= formationCount
    error('IndexEquivariantFormationRoute:InvalidContract', ...
        'Formation physical UIDs must be unique across memberships.');
end
[formationUids, canonicalOrder] = sort(formationUids);
groupLabels = groupLabels(canonicalOrder);
members = members(canonicalOrder);

formationAdjacency = false(formationCount);
for receiverFormationIdx = 1:formationCount
    receivers = members{receiverFormationIdx};
    for senderFormationIdx = 1:formationCount
        senders = members{senderFormationIdx};
        formationAdjacency(receiverFormationIdx, senderFormationIdx) = ...
            any(any(registeredAdjacency(receivers, senders)));
    end
end
formationAdjacency = formationAdjacency | formationAdjacency';
formationAdjacency(1:formationCount+1:end) = false;
if ~isStronglyConnected(formationAdjacency) || ...
        any(sum(formationAdjacency, 2)' > cellfun(@numel, members))
    error('IndexEquivariantFormationRoute:InvalidBackbone', ...
        'The registered formation backbone is disconnected or overfull.');
end

maximumEnumeratedCandidates = 1e5;
localCycleCandidateCounts = zeros(1, formationCount);
crossAssignmentCandidateCounts = zeros(1, formationCount);
degrees = reshape(sum(formationAdjacency, 2), 1, []);
for formationIdx = 1:formationCount
    memberCount = numel(members{formationIdx});
    localCycleCandidateCounts(formationIdx) = ...
        boundedOrderedSelectionCount(memberCount - 1, ...
            memberCount - 1, maximumEnumeratedCandidates);
    crossAssignmentCandidateCounts(formationIdx) = ...
        boundedOrderedSelectionCount(memberCount, degrees(formationIdx), ...
            maximumEnumeratedCandidates);
end
if any(localCycleCandidateCounts > maximumEnumeratedCandidates) || ...
        any(crossAssignmentCandidateCounts > ...
            maximumEnumeratedCandidates)
    error('IndexEquivariantFormationRoute:EnumerationLimitExceeded', ...
        ['Exact v43 enumeration exceeds %d candidates for at least one ', ...
         'formation; use a registered DP/assignment implementation.'], ...
        maximumEnumeratedCandidates);
end

contract = struct('nodeCount', nodeCount);
formation = struct('count', formationCount, 'uids', formationUids, ...
    'groupLabels', groupLabels, 'members', {members}, ...
    'adjacency', formationAdjacency, ...
    'maximumEnumeratedCandidates', maximumEnumeratedCandidates, ...
    'localCycleCandidateCounts', localCycleCandidateCounts, ...
    'crossAssignmentCandidateCounts', ...
        crossAssignmentCandidateCounts);
end

function count = boundedOrderedSelectionCount(total, selected, cap)
count = 1;
for factor = (total - selected + 1):total
    count = count * factor;
    if count > cap
        count = cap + 1;
        return;
    end
end
end

function [bestOrder, bestScore] = selectBestCycle( ...
        members, forbiddenSources, sensorUids, positions, physical, ...
        reliability)
[~, uidOrder] = sort(sensorUids(members));
members = members(uidOrder);
root = members(1);
tail = members(2:end);
tailOrders = perms(tail);
bestOrder = zeros(1, 0);
bestScore = emptyScore();
for candidateIdx = 1:size(tailOrders, 1)
    order = [root, tailOrders(candidateIdx, :)];
    sources = sourcesFromCycleOrder(order, size(physical, 1));
    receivers = order;
    selectedSources = sources(receivers);
    linear = sub2ind(size(physical), receivers, selectedSources);
    if any(~physical(linear)) || ...
            (~isempty(forbiddenSources) && ...
             any(selectedSources == forbiddenSources(receivers)))
        continue;
    end
    score = scoreEdges(receivers, selectedSources, sensorUids, ...
        positions, reliability);
    score.tieKey = sensorUids(order);
    if isempty(bestOrder) || scoreBetter(score, bestScore)
        bestOrder = order;
        bestScore = score;
    end
end
if isempty(bestOrder)
    error('IndexEquivariantFormationRoute:NoLocalCycle', ...
        'No physical local cycle satisfies the distinct-source contract.');
end
end

function [bestReceivers, bestSenders, bestScore] = ...
        selectBestCrossAssignment(neighborFormationIndices, receivers, ...
            formation, sensorUids, positions, physical, reliability)
slotCount = numel(neighborFormationIndices);
if slotCount < 1
    error('IndexEquivariantFormationRoute:InvalidBackbone', ...
        'Every registered formation must have at least one neighbor.');
end
[~, receiverUidOrder] = sort(sensorUids(receivers));
receivers = receivers(receiverUidOrder);
receiverSelections = orderedSelections(receivers, slotCount);
bestReceivers = zeros(1, 0);
bestSenders = zeros(1, 0);
bestScore = emptyScore();
for selectionIdx = 1:size(receiverSelections, 1)
    candidateReceivers = receiverSelections(selectionIdx, :);
    candidateSenders = zeros(1, slotCount);
    feasible = true;
    for slotIdx = 1:slotCount
        senders = formation.members{neighborFormationIndices(slotIdx)};
        sender = selectBestSender(candidateReceivers(slotIdx), senders, ...
            sensorUids, positions, physical, reliability);
        if ~isfinite(sender)
            feasible = false;
            break;
        end
        candidateSenders(slotIdx) = sender;
    end
    if ~feasible
        continue;
    end
    score = scoreEdges(candidateReceivers, candidateSenders, ...
        sensorUids, positions, reliability);
    edgeKey = [ ...
        reshape(formation.uids(neighborFormationIndices), [], 1), ...
        repmat(formationPhysicalUidForReceiver( ...
            candidateReceivers(1), formation), slotCount, 1), ...
        reshape(sensorUids(candidateSenders), [], 1), ...
        reshape(sensorUids(candidateReceivers), [], 1)];
    edgeKey = sortrows(edgeKey, [1, 2, 3, 4]);
    score.tieKey = reshape(edgeKey', 1, []);
    if isempty(bestReceivers) || scoreBetter(score, bestScore)
        bestReceivers = candidateReceivers;
        bestSenders = candidateSenders;
        bestScore = score;
    end
end
if isempty(bestReceivers)
    error('IndexEquivariantFormationRoute:NoCrossAssignment', ...
        'No physical registered-backbone cross assignment exists.');
end
end

function uid = formationPhysicalUidForReceiver(receiver, formation)
uid = NaN;
for formationIdx = 1:formation.count
    if any(formation.members{formationIdx} == receiver)
        uid = formation.uids(formationIdx);
        return;
    end
end
error('IndexEquivariantFormationRoute:InternalContract', ...
    'A receiver has no physical formation UID.');
end

function sender = selectBestSender(receiver, senders, sensorUids, ...
        positions, physical, reliability)
[~, uidOrder] = sort(sensorUids(senders));
senders = senders(uidOrder);
sender = NaN;
bestScore = emptyScore();
for senderIdx = senders
    if ~physical(receiver, senderIdx)
        continue;
    end
    score = scoreEdges(receiver, senderIdx, sensorUids, ...
        positions, reliability);
    score.tieKey = sensorUids(senderIdx);
    if ~isfinite(sender) || scoreBetter(score, bestScore)
        sender = senderIdx;
        bestScore = score;
    end
end
end

function selections = orderedSelections(values, count)
if count > numel(values)
    selections = zeros(0, count);
    return;
end
combinations = nchoosek(values, count);
selections = zeros(0, count);
for combinationIdx = 1:size(combinations, 1)
    selections = [selections; perms(combinations(combinationIdx, :))]; ...
        %#ok<AGROW>
end
end

function score = scoreEdges(receivers, senders, sensorUids, ...
        positions, reliability)
linear = sub2ind(size(reliability), receivers, senders);
q = reliability(linear);
delta = positions(:, receivers) - positions(:, senders);
score = emptyScore();
score.bottleneckReliability = min(q);
score.totalLogReliability = sum(log(max(q, realmin)));
score.totalDistance = sum(sqrt(sum(delta.^2, 1)));
score.tieKey = reshape([sensorUids(senders); sensorUids(receivers)], 1, []);
end

function score = emptyScore()
score = struct('bottleneckReliability', -Inf, ...
    'totalLogReliability', -Inf, 'totalDistance', Inf, ...
    'tieKey', zeros(1, 0));
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

function sources = sourcesFromCycleOrder(order, nodeCount)
sources = zeros(1, nodeCount);
predecessors = order([end, 1:end-1]);
sources(order) = predecessors;
end

function adjacency = addSources(adjacency, receivers, sources)
for receiver = receivers
    adjacency(receiver, sources(receiver)) = true;
end
end

function details = cycleDetails(order, score, sensorUids)
details = struct('sensorPhysicalUidOrder', sensorUids(order), ...
    'bottleneckReliability', score.bottleneckReliability, ...
    'totalLogReliability', score.totalLogReliability, ...
    'totalDistance', score.totalDistance);
end

function sources = findOneSourcePerReceiver(adjacency)
nodeCount = size(adjacency, 1);
sources = zeros(1, nodeCount);
for receiver = 1:nodeCount
    selected = find(adjacency(receiver, :));
    if numel(selected) ~= 1
        error('IndexEquivariantFormationRoute:InternalContract', ...
            'Every receiver must have exactly one selected source.');
    end
    sources(receiver) = selected;
end
end

function passed = registeredPairMultiplicityPassed( ...
        senderFormationUids, receiverFormationUids, adjacency, uids)
passed = true;
for leftIdx = 1:numel(uids)-1
    for rightIdx = leftIdx+1:numel(uids)
        forward = nnz(senderFormationUids == uids(leftIdx) & ...
            receiverFormationUids == uids(rightIdx));
        reverse = nnz(senderFormationUids == uids(rightIdx) & ...
            receiverFormationUids == uids(leftIdx));
        expected = adjacency(leftIdx, rightIdx);
        if forward ~= expected || reverse ~= expected
            passed = false;
            return;
        end
    end
end
end

function value = maximumLoad(indices, nodeCount)
if isempty(indices)
    value = 0;
else
    value = max(accumarray(indices(:), 1, [nodeCount, 1]));
end
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function connected = reachableAll(adjacency)
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
