function registry = FormationB4V45BuildCausalRegisteredBaseGraph( ...
        backboneMode, sensorGroupIds, sensorPhysicalUids, ...
        formationPhysicalUidsBySensor, initialPositions, ...
        initialPhysicalAdjacency)
% FORMATIONB4V45BUILDCAUSALREGISTEREDBASEGRAPH Causal formation registry.
%
% The returned sensor-level graph is a UID-canonical placeholder whose
% only meaning is to register formation pairs for the V43 route builder.
% It is not a communication graph and is never claimed to be physically
% executable.  Runtime executability is decided from the current physical
% page supplied separately to V43.
%
% formation-ring:
%   Treat the named scene's sorted group-label cycle as an exogenous
%   registered topology, bind every group to its immutable formation UID,
%   and persist the resulting UID pairs.  This is invariant to a sensor-
%   array permutation when group IDs and physical UIDs move with sensors.
%   It does not claim invariance to relabeling the registered groups.
%
% initial-geometry-mst:
%   Register a Kruskal MST using only t=1 positions and the t=1 physical
%   page.  Candidate ties are resolved by immutable formation UIDs.

mode = lower(char(backboneMode));
groupIds = reshape(sensorGroupIds, 1, []);
sensorUids = reshape(sensorPhysicalUids, 1, []);
formationUidBySensor = reshape( ...
    formationPhysicalUidsBySensor, 1, []);
nodeCount = numel(groupIds);
if ~ismember(mode, {'formation-ring', 'ring', ...
        'initial-geometry-mst', 'initial-mst'}) || ...
        nodeCount < 4 || ~isnumeric(groupIds) || ...
        ~isreal(groupIds) || any(~isfinite(groupIds)) || ...
        any(groupIds < 1) || any(groupIds ~= round(groupIds)) || ...
        ~isnumeric(sensorUids) || ~isreal(sensorUids) || ...
        numel(sensorUids) ~= nodeCount || ...
        any(~isfinite(sensorUids)) || ...
        any(sensorUids < 1) || any(sensorUids ~= round(sensorUids)) || ...
        numel(unique(sensorUids)) ~= nodeCount || ...
        ~isnumeric(formationUidBySensor) || ...
        ~isreal(formationUidBySensor) || ...
        numel(formationUidBySensor) ~= nodeCount || ...
        any(~isfinite(formationUidBySensor)) || ...
        any(formationUidBySensor < 1) || ...
        any(formationUidBySensor ~= round(formationUidBySensor)) || ...
        ~isnumeric(initialPositions) || ~isreal(initialPositions) || ...
        ~isequal(size(initialPositions), [2, nodeCount]) || ...
        any(~isfinite(initialPositions(:))) || ...
        ~(islogical(initialPhysicalAdjacency) || ...
            isnumeric(initialPhysicalAdjacency)) || ...
        ~isreal(initialPhysicalAdjacency) || ...
        ~isequal(size(initialPhysicalAdjacency), ...
            [nodeCount, nodeCount]) || ...
        any(~isfinite(initialPhysicalAdjacency(:)))
    error('FormationB4V45BaseGraph:InvalidInput', ...
        'The causal formation-registry inputs are malformed.');
end
initialPhysical = logical(initialPhysicalAdjacency);
initialPhysical(1:nodeCount+1:end) = false;
if ~isequal(initialPhysical, initialPhysical')
    error('FormationB4V45BaseGraph:InvalidInput', ...
        'The t=1 physical adjacency must be symmetric.');
end

[formationUids, members, groupLabels, groupOrderUids] = validateMemberships( ...
    groupIds, formationUidBySensor);
formationCount = numel(formationUids);
switch mode
    case {'formation-ring', 'ring'}
        selectionMode = 'registered-group-order-uid-ring';
        groupRankPairs = buildRingPairs(formationCount);
        rankByGroup = zeros(1, formationCount);
        for groupIdx = 1:formationCount
            rankByGroup(groupIdx) = find( ...
                formationUids == groupOrderUids(groupIdx), 1);
        end
        rankPairs = [rankByGroup(groupRankPairs(:, 1))', ...
            rankByGroup(groupRankPairs(:, 2))'];
        formationGroupLabelPairs = [ ...
            groupLabels(groupRankPairs(:, 1))', ...
            groupLabels(groupRankPairs(:, 2))'];
        initialGeometryUsed = false;
        initialPhysicalPageUsed = false;
    case {'initial-geometry-mst', 'initial-mst'}
        selectionMode = ...
            't1-physical-minimum-distance-uid-tiebroken-mst';
        rankPairs = buildInitialMstPairs(formationUids, members, ...
            initialPositions, initialPhysical);
        formationGroupLabelPairs = rankPairsToGroupLabels( ...
            rankPairs, formationUids, groupLabels, groupOrderUids);
        initialGeometryUsed = true;
        initialPhysicalPageUsed = true;
end

formationAdjacency = false(formationCount);
for pairIdx = 1:size(rankPairs, 1)
    left = rankPairs(pairIdx, 1);
    right = rankPairs(pairIdx, 2);
    formationAdjacency(left, right) = true;
    formationAdjacency(right, left) = true;
end
if ~isStronglyConnected(formationAdjacency)
    error('FormationB4V45BaseGraph:Disconnected', ...
        'The registered causal formation graph is disconnected.');
end

baseAdjacency = false(nodeCount);
placeholderSensorUidPairs = zeros(size(rankPairs));
for pairIdx = 1:size(rankPairs, 1)
    leftMembers = members{rankPairs(pairIdx, 1)};
    rightMembers = members{rankPairs(pairIdx, 2)};
    [leftUid, leftLocal] = min(sensorUids(leftMembers));
    [rightUid, rightLocal] = min(sensorUids(rightMembers));
    leftSensor = leftMembers(leftLocal);
    rightSensor = rightMembers(rightLocal);
    baseAdjacency(leftSensor, rightSensor) = true;
    baseAdjacency(rightSensor, leftSensor) = true;
    placeholderSensorUidPairs(pairIdx, :) = [leftUid, rightUid];
end
sensorNeighborMap = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    sensorNeighborMap{sensorIdx} = unique( ...
        [sensorIdx, find(baseAdjacency(sensorIdx, :))]);
end
formationPairPhysicalUids = [ ...
    formationUids(rankPairs(:, 1))', ...
    formationUids(rankPairs(:, 2))'];
formationPairPhysicalUids = sort(formationPairPhysicalUids, 2);
formationPairPhysicalUids = sortrows(formationPairPhysicalUids, [1, 2]);
placeholderSensorUidPairs = sort(placeholderSensorUidPairs, 2);
placeholderSensorUidPairs = sortrows( ...
    placeholderSensorUidPairs, [1, 2]);

payload = struct();
payload.contractVersion = ...
    'formation-b4-v45-causal-registered-base-graph-v1';
payload.selectionMode = selectionMode;
payload.baseGraphSemantics = ...
    'formation-pair-registry-only-not-runtime-communication-graph';
payload.formationPhysicalUids = formationUids;
payload.registeredGroupLabels = groupLabels;
payload.registeredGroupOrderFormationPhysicalUids = groupOrderUids;
payload.formationPairPhysicalUids = formationPairPhysicalUids;
payload.formationGroupLabelPairs = ...
    sortrows(sort(formationGroupLabelPairs, 2), [1, 2]);
payload.formationAdjacency = formationAdjacency;
payload.baseAdjacency = baseAdjacency;
payload.sensorNeighborMap = sensorNeighborMap;
payload.placeholderSensorPhysicalUidPairs = ...
    placeholderSensorUidPairs;
payload.currentGeometryUsed = false;
payload.initialGeometryUsed = initialGeometryUsed;
payload.initialPhysicalPageUsed = initialPhysicalPageUsed;
payload.futureGeometryUsed = false;
payload.futurePhysicalPageUsed = false;
payload.linkProbabilityUsed = false;
payload.sensorArrayPermutationEquivarianceDesigned = true;
payload.formationNumericLabelEquivarianceClaimed = false;
payload.posteriorUsed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.realizedDeliveryUniformsUsed = false;
registry = payload;
identityPayload = rmfield(payload, {'baseAdjacency', ...
    'sensorNeighborMap'});
registry.canonicalSha256 = ...
    computeCanonicalValueSha256(identityPayload);
executionView = struct( ...
    'contractVersion', ...
        'formation-b4-v45-causal-base-graph-execution-view-v1', ...
    'sensorPhysicalUidsByArray', sensorUids, ...
    'baseAdjacency', baseAdjacency, ...
    'sensorNeighborMap', {sensorNeighborMap});
registry.executionViewCanonicalSha256 = ...
    computeCanonicalValueSha256(executionView);
end

function [uids, members, groupLabels, groupOrderUids] = ...
        validateMemberships(groupIds, uidBySensor)
groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
groupOrderUids = zeros(1, formationCount);
groupOrderMembers = cell(1, formationCount);
for groupIdx = 1:formationCount
    groupMembers = find(groupIds == groupLabels(groupIdx));
    groupUids = unique(uidBySensor(groupMembers));
    if numel(groupUids) ~= 1 || numel(groupMembers) < 3
        error('FormationB4V45BaseGraph:InvalidMembership', ...
            'Each formation needs one immutable UID and at least 3 sensors.');
    end
    groupOrderUids(groupIdx) = groupUids;
    groupOrderMembers{groupIdx} = groupMembers;
end
if numel(unique(groupOrderUids)) ~= formationCount
    error('FormationB4V45BaseGraph:InvalidMembership', ...
        'Formation physical UIDs must be globally unique.');
end
[uids, order] = sort(groupOrderUids);
members = groupOrderMembers(order);
end

function pairs = rankPairsToGroupLabels( ...
        rankPairs, sortedUids, groupLabels, groupOrderUids)
pairs = zeros(size(rankPairs));
for pairIdx = 1:size(rankPairs, 1)
    leftUid = sortedUids(rankPairs(pairIdx, 1));
    rightUid = sortedUids(rankPairs(pairIdx, 2));
    pairs(pairIdx, 1) = groupLabels( ...
        find(groupOrderUids == leftUid, 1));
    pairs(pairIdx, 2) = groupLabels( ...
        find(groupOrderUids == rightUid, 1));
end
end

function pairs = buildRingPairs(formationCount)
if formationCount < 2
    error('FormationB4V45BaseGraph:InvalidMembership', ...
        'A formation ring requires at least two formations.');
end
pairs = zeros(formationCount, 2);
for formationIdx = 1:formationCount
    pairs(formationIdx, :) = [formationIdx, ...
        mod(formationIdx, formationCount) + 1];
end
pairs = sort(pairs, 2);
pairs = unique(pairs, 'rows');
end

function pairs = buildInitialMstPairs(formationUids, members, ...
        positions, physical)
formationCount = numel(formationUids);
candidates = zeros(0, 5);
for leftIdx = 1:formationCount-1
    for rightIdx = leftIdx+1:formationCount
        leftMembers = members{leftIdx};
        rightMembers = members{rightIdx};
        allowed = physical(leftMembers, rightMembers);
        if ~any(allowed(:))
            continue;
        end
        distances = pairDistances( ...
            positions(:, leftMembers), positions(:, rightMembers));
        minimumDistance = min(distances(allowed));
        candidates(end + 1, :) = [ ... %#ok<AGROW>
            leftIdx, rightIdx, minimumDistance, ...
            formationUids(leftIdx), formationUids(rightIdx)];
    end
end
if isempty(candidates)
    error('FormationB4V45BaseGraph:NoInitialCrossEdge', ...
        'No t=1 physical inter-formation edge is available.');
end
[~, order] = sortrows(candidates(:, [3, 4, 5]), [1, 2, 3]);
candidates = candidates(order, :);
components = 1:formationCount;
pairs = zeros(formationCount - 1, 2);
pairCount = 0;
for candidateIdx = 1:size(candidates, 1)
    leftIdx = candidates(candidateIdx, 1);
    rightIdx = candidates(candidateIdx, 2);
    leftComponent = components(leftIdx);
    rightComponent = components(rightIdx);
    if leftComponent == rightComponent
        continue;
    end
    pairCount = pairCount + 1;
    pairs(pairCount, :) = [leftIdx, rightIdx];
    components(components == rightComponent) = leftComponent;
    if pairCount == formationCount - 1
        break;
    end
end
if pairCount ~= formationCount - 1
    error('FormationB4V45BaseGraph:InitialGraphDisconnected', ...
        'The t=1 physical formation graph has no spanning tree.');
end
end

function distances = pairDistances(leftPositions, rightPositions)
distances = zeros(size(leftPositions, 2), size(rightPositions, 2));
for leftIdx = 1:size(leftPositions, 2)
    delta = rightPositions - leftPositions(:, leftIdx);
    distances(leftIdx, :) = sqrt(sum(delta.^2, 1));
end
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
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
