function [projectedBaseAdjacency, certificate] = ...
    projectCausalMinimalEditFormationBackbone( ...
        sensorGroupIds, sensorPhysicalUids, ...
        formationPhysicalUidsBySensor, positions, ...
        registeredBaseAdjacency, physicalAdjacency, pDropByEdge)
% PROJECTCAUSALMINIMALEDITFORMATIONBACKBONE Current-page safe projection.
%
% The registered formation graph is a nominal anchor, not a promise that
% every registered pair remains physical.  This projector collapses the
% anchor and the current physical page to UID-canonical formation graphs,
% then searches exact edit layers.  It first minimizes removed registered
% pairs and then added nonregistered pairs.  Every returned neighbor slot
% has an exact distinct-receiver assignment using only the current physical
% and link-probability pages.  For each candidate topology, the matching
% witness exactly mirrors V43's per-receiving-formation deterministic
% selectBestCrossAssignment.  The aggregate score compares those V43-
% realized witnesses across topologies; it is not a joint reassignment of
% all sensor edges.
%
% The returned sensor matrix is only a formation-pair registry placeholder:
% each selected formation pair is represented by the minimum-UID sensor in
% each formation.  Runtime physical edges are selected later by the V43
% sensor-level route builder.

if nargin ~= 7
    error('CausalFormationProjection:InvalidInput', ...
        'Exactly seven frozen current-page inputs are required.');
end

contract = validateInputs(sensorGroupIds, sensorPhysicalUids, ...
    formationPhysicalUidsBySensor, positions, ...
    registeredBaseAdjacency, physicalAdjacency, pDropByEdge);
groupIds = contract.groupIds;
sensorUids = contract.sensorUids;
formationUidBySensor = contract.formationUidBySensor;
positions = contract.positions;
registered = contract.registered;
physical = contract.physical;
pDrop = contract.pDrop;
formation = contract.formation;

[registeredFormationAdjacency, physicalQuotientAdjacency] = ...
    collapseFormationGraphs(formation, registered, physical);
if ~isConnectedUndirected(registeredFormationAdjacency)
    error('CausalFormationProjection:InvalidInput', ...
        'The registered formation graph must be connected.');
end

registeredPairs = upperPairs(registeredFormationAdjacency);
quotientPairs = upperPairs(physicalQuotientAdjacency);
registeredInQuotient = ismember(registeredPairs, quotientPairs, 'rows');
activeRegisteredPairs = registeredPairs(registeredInQuotient, :);
forcedRemovedPairs = registeredPairs(~registeredInQuotient, :);
newPairMask = ~ismember(quotientPairs, registeredPairs, 'rows');
newPairs = quotientPairs(newPairMask, :);

searchCap = 1e6;
counts = emptyCounts();
selected = struct();
found = false;
for optionalRemovalCount = 0:size(activeRegisteredPairs, 1)
    removalSelectionCount = boundedCombinationCount( ...
        size(activeRegisteredPairs, 1), optionalRemovalCount, searchCap);
    if removalSelectionCount > searchCap - counts.workUnitCount
        error('CausalFormationProjection:SearchLimit', ...
            'The exact projection exceeded its 1e6-work-unit cap.');
    end
    removalSelections = chooseIndices( ...
        size(activeRegisteredPairs, 1), optionalRemovalCount);
    for additionCount = 0:size(newPairs, 1)
        additionSelectionCount = boundedCombinationCount( ...
            size(newPairs, 1), additionCount, searchCap);
        remainingWork = searchCap - counts.workUnitCount;
        if additionSelectionCount > 0 && ...
                removalSelectionCount > ...
                    floor(remainingWork / additionSelectionCount)
            error('CausalFormationProjection:SearchLimit', ...
                'The exact projection exceeded its 1e6-work-unit cap.');
        end
        layerTopologyCount = ...
            removalSelectionCount * additionSelectionCount;
        additionSelections = chooseIndices( ...
            size(newPairs, 1), additionCount);

        layerConnectedCount = 0;
        layerMatchingFeasibleCount = 0;
        layerBest = struct();
        layerFound = false;
        for removalIdx = 1:size(removalSelections, 1)
            retainedMask = true(size(activeRegisteredPairs, 1), 1);
            retainedMask(removalSelections(removalIdx, :)) = false;
            retainedPairs = activeRegisteredPairs(retainedMask, :);
            for additionIdx = 1:size(additionSelections, 1)
                counts.topologyCandidateCount = ...
                    counts.topologyCandidateCount + 1;
                counts.workUnitCount = counts.workUnitCount + 1;
                addedPairs = newPairs( ...
                    additionSelections(additionIdx, :), :);
                candidatePairs = sortPairRows([retainedPairs; addedPairs]);
                candidateAdjacency = pairsToAdjacency( ...
                    candidatePairs, formation.count);
                if ~isConnectedUndirected(candidateAdjacency)
                    continue;
                end
                layerConnectedCount = layerConnectedCount + 1;

                remainingWork = searchCap - counts.workUnitCount;
                [witnesses, score, matchingWork, feasible] = ...
                    evaluateCandidate(candidateAdjacency, formation, ...
                        sensorUids, positions, physical, pDrop, ...
                        remainingWork);
                counts.receiverAssignmentCandidateCount = ...
                    counts.receiverAssignmentCandidateCount + matchingWork;
                counts.workUnitCount = counts.workUnitCount + matchingWork;
                if counts.workUnitCount > searchCap
                    error('CausalFormationProjection:SearchLimit', ...
                        ['The exact projection exceeded its ', ...
                         '1e6-work-unit cap.']);
                end
                if ~feasible
                    continue;
                end
                layerMatchingFeasibleCount = ...
                    layerMatchingFeasibleCount + 1;
                candidate = struct('rankPairs', candidatePairs, ...
                    'adjacency', candidateAdjacency, ...
                    'witnesses', witnesses, 'score', score);
                if ~layerFound || scoreBetter(score, layerBest.score)
                    layerBest = candidate;
                    layerFound = true;
                end
            end
        end

        if layerFound
            selected = layerBest;
            counts.selectedLayerTopologyCount = layerTopologyCount;
            counts.selectedLayerConnectedCount = layerConnectedCount;
            counts.selectedLayerMatchingFeasibleCount = ...
                layerMatchingFeasibleCount;
            counts.selectedLayerOptionalRegisteredRemovalCount = ...
                optionalRemovalCount;
            counts.selectedLayerNonregisteredAdditionCount = additionCount;
            found = true;
            break;
        end
    end
    if found
        break;
    end
end
if ~found
    error('CausalFormationProjection:NoFeasibleBackbone', ...
        ['No connected current-physical formation graph has a full ', ...
         'distinct-receiver assignment.']);
end

projectedPairs = selected.rankPairs;
removedPairs = registeredPairs( ...
    ~ismember(registeredPairs, projectedPairs, 'rows'), :);
addedPairs = projectedPairs( ...
    ~ismember(projectedPairs, registeredPairs, 'rows'), :);
[projectedBaseAdjacency, placeholderUidPairs] = ...
    materializePlaceholder(projectedPairs, formation, sensorUids, ...
        contract.nodeCount);

registeredUidPairs = rankPairsToUidPairs(registeredPairs, formation.uids);
quotientUidPairs = rankPairsToUidPairs(quotientPairs, formation.uids);
projectedUidPairs = rankPairsToUidPairs(projectedPairs, formation.uids);
removedUidPairs = rankPairsToUidPairs(removedPairs, formation.uids);
addedUidPairs = rankPairsToUidPairs(addedPairs, formation.uids);

canonicalSensorOrder = uidOrder(sensorUids);
currentInputPayload = struct( ...
    'contractVersion', ...
        'causal-minimal-edit-formation-current-input-v1', ...
    'sensorPhysicalUids', sensorUids(canonicalSensorOrder), ...
    'formationPhysicalUidsBySensor', ...
        formationUidBySensor(canonicalSensorOrder), ...
    'positions', positions(:, canonicalSensorOrder), ...
    'registeredFormationPairPhysicalUids', registeredUidPairs, ...
    'physicalAdjacency', ...
        physical(canonicalSensorOrder, canonicalSensorOrder), ...
    'pDropByEdge', pDrop(canonicalSensorOrder, canonicalSensorOrder));

payload = struct();
payload.contractVersion = ...
    'causal-minimal-edit-formation-backbone-projection-v1';
payload.formationPhysicalUids = formation.uids;
payload.registeredPairPhysicalUids = registeredUidPairs;
payload.currentPhysicalQuotientPairPhysicalUids = quotientUidPairs;
payload.projectedPairPhysicalUids = projectedUidPairs;
payload.removedRegisteredPairPhysicalUids = removedUidPairs;
payload.addedNonregisteredPairPhysicalUids = addedUidPairs;
payload.placeholderSensorPhysicalUidPairs = placeholderUidPairs;
payload.registeredRemovalCount = size(removedUidPairs, 1);
payload.forcedRegisteredRemovalCount = size(forcedRemovedPairs, 1);
payload.optionalRegisteredRemovalCount = ...
    counts.selectedLayerOptionalRegisteredRemovalCount;
payload.nonregisteredAdditionCount = size(addedUidPairs, 1);
payload.matchingWitnesses = selected.witnesses;
payload.selectedScore = selected.score;
payload.searchCap = searchCap;
payload.searchCandidateCounts = counts;
payload.currentInputCanonicalSha256 = ...
    computeCanonicalValueSha256(currentInputPayload);
payload.routeInputBoundary = ...
    'current-page-only-no-runtime-overrides-or-score-functions';
payload.registeredBaseGraphUsed = true;
payload.currentGeometryUsed = true;
payload.currentPhysicalPageUsed = true;
payload.currentLinkProbabilityPageUsed = true;
payload.futureGeometryUsed = false;
payload.futurePhysicalPageUsed = false;
payload.posteriorUsed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.runtimeOverridesAccepted = false;
payload.scoreFunctionAccepted = false;
payload.sensorArrayPermutationEquivarianceDesigned = true;
payload.groupNumericRelabelEquivarianceDesigned = true;
certificate = payload;
certificate.canonicalSha256 = computeCanonicalValueSha256(payload);

executionView = struct( ...
    'contractVersion', ...
        'causal-minimal-edit-formation-execution-view-v1', ...
    'sensorPhysicalUidsByArray', sensorUids, ...
    'projectedBaseAdjacency', projectedBaseAdjacency);
certificate.arrayOrderExecutionCanonicalSha256 = ...
    computeCanonicalValueSha256(executionView);
end

function contract = validateInputs(groupIds, sensorUids, ...
        formationUidBySensor, positions, registered, physical, pDrop)
groupIds = reshape(groupIds, 1, []);
sensorUids = reshape(sensorUids, 1, []);
formationUidBySensor = reshape(formationUidBySensor, 1, []);
nodeCount = numel(groupIds);
matrixSize = [nodeCount, nodeCount];
matrixInputsValid = ...
    (islogical(registered) || isnumeric(registered)) && ...
    isreal(registered) && all(isfinite(registered(:))) && ...
    (islogical(physical) || isnumeric(physical)) && ...
    isreal(physical) && all(isfinite(physical(:))) && ...
    isnumeric(pDrop) && isreal(pDrop) && all(isfinite(pDrop(:)));
if nodeCount < 6 || ~isnumeric(groupIds) || ~isreal(groupIds) || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(groupIds ~= round(groupIds)) || ...
        ~isnumeric(sensorUids) || ~isreal(sensorUids) || ...
        numel(sensorUids) ~= nodeCount || ...
        any(~isfinite(sensorUids)) || any(sensorUids < 1) || ...
        any(sensorUids ~= round(sensorUids)) || ...
        numel(unique(sensorUids)) ~= nodeCount || ...
        ~isnumeric(formationUidBySensor) || ...
        ~isreal(formationUidBySensor) || ...
        numel(formationUidBySensor) ~= nodeCount || ...
        any(~isfinite(formationUidBySensor)) || ...
        any(formationUidBySensor < 1) || ...
        any(formationUidBySensor ~= round(formationUidBySensor)) || ...
        ~isnumeric(positions) || ~isreal(positions) || ...
        ~isequal(size(positions), [2, nodeCount]) || ...
        any(~isfinite(positions(:))) || ~matrixInputsValid || ...
        ~isequal(size(registered), matrixSize) || ...
        ~isequal(size(physical), matrixSize) || ...
        ~isequal(size(pDrop), matrixSize) || ...
        any(pDrop(:) < 0) || any(pDrop(:) > 1)
    error('CausalFormationProjection:InvalidInput', ...
        'The frozen current-page projection inputs are malformed.');
end
registered = logical(registered);
physical = logical(physical);
registered(1:nodeCount+1:end) = false;
physical(1:nodeCount+1:end) = false;
if ~isequal(registered, registered')
    error('CausalFormationProjection:InvalidInput', ...
        'The registered formation-pair placeholder must be symmetric.');
end

groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
if formationCount < 2 || formationCount > 8
    error('CausalFormationProjection:InvalidInput', ...
        'The frozen development scope requires 2 to 8 formations.');
end
formationUidsInGroupOrder = zeros(1, formationCount);
membersInGroupOrder = cell(1, formationCount);
for groupIdx = 1:formationCount
    members = find(groupIds == groupLabels(groupIdx));
    uids = unique(formationUidBySensor(members));
    if numel(members) < 3 || numel(uids) ~= 1
        error('CausalFormationProjection:InvalidInput', ...
            ['Every formation must contain at least three sensors and ', ...
             'exactly one immutable formation UID.']);
    end
    formationUidsInGroupOrder(groupIdx) = uids;
    membersInGroupOrder{groupIdx} = members;
end
if numel(unique(formationUidsInGroupOrder)) ~= formationCount
    error('CausalFormationProjection:InvalidInput', ...
        'Formation physical UIDs must be globally unique.');
end
[formationUids, canonicalFormationOrder] = ...
    sort(formationUidsInGroupOrder);
members = membersInGroupOrder(canonicalFormationOrder);
formation = struct('count', formationCount, 'uids', formationUids, ...
    'members', {members});
contract = struct('nodeCount', nodeCount, 'groupIds', groupIds, ...
    'sensorUids', sensorUids, ...
    'formationUidBySensor', formationUidBySensor, ...
    'positions', positions, 'registered', registered, ...
    'physical', physical, 'pDrop', pDrop, ...
    'formation', formation);
end

function [registeredFormation, physicalQuotient] = ...
        collapseFormationGraphs(formation, registered, physical)
formationCount = formation.count;
registeredFormation = false(formationCount);
physicalQuotient = false(formationCount);
for leftIdx = 1:formationCount-1
    left = formation.members{leftIdx};
    for rightIdx = leftIdx+1:formationCount
        right = formation.members{rightIdx};
        registeredPair = any(any(registered(left, right)));
        currentForward = any(any(physical(left, right)));
        currentReverse = any(any(physical(right, left)));
        registeredFormation(leftIdx, rightIdx) = registeredPair;
        registeredFormation(rightIdx, leftIdx) = registeredPair;
        currentPair = currentForward && currentReverse;
        physicalQuotient(leftIdx, rightIdx) = currentPair;
        physicalQuotient(rightIdx, leftIdx) = currentPair;
    end
end
end

function pairs = upperPairs(adjacency)
[left, right] = find(triu(logical(adjacency), 1));
pairs = sortPairRows([left, right]);
end

function pairs = sortPairRows(pairs)
if isempty(pairs)
    pairs = zeros(0, 2);
    return;
end
pairs = sort(pairs, 2);
pairs = unique(sortrows(pairs, [1, 2]), 'rows');
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

function selections = chooseIndices(total, selected)
if selected == 0
    selections = zeros(1, 0);
elseif selected > total
    selections = zeros(0, selected);
else
    selections = nchoosek(1:total, selected);
end
end

function count = boundedCombinationCount(total, selected, cap)
if selected < 0 || selected > total
    count = 0;
    return;
end
selected = min(selected, total - selected);
count = 1;
for idx = 1:selected
    numerator = total - selected + idx;
    if count > floor(cap * idx / numerator)
        count = cap + 1;
        return;
    end
    count = count * numerator / idx;
end
count = round(count);
end

function [witnesses, globalScore, workCount, feasible] = ...
        evaluateCandidate(adjacency, formation, sensorUids, positions, ...
            physical, pDrop, workBudget)
degrees = reshape(sum(adjacency, 2), 1, []);
memberCounts = cellfun(@numel, formation.members);
witnesses = repmat(emptyWitness(), 1, formation.count);
globalScore = emptyScore();
workCount = 0;
feasible = false;
if any(degrees < 1) || any(degrees > memberCounts)
    return;
end
reliability = 1 - pDrop';
for receiverFormationIdx = 1:formation.count
    neighbors = find(adjacency(receiverFormationIdx, :));
    [~, order] = sort(formation.uids(neighbors));
    neighbors = neighbors(order);
    receivers = formation.members{receiverFormationIdx};
    [~, receiverOrder] = sort(sensorUids(receivers));
    receivers = receivers(receiverOrder);
    selectionCount = orderedSelectionCount( ...
        numel(receivers), numel(neighbors));
    if selectionCount > workBudget - workCount
        error('CausalFormationProjection:SearchLimit', ...
            'The exact projection exceeded its 1e6-work-unit cap.');
    end
    receiverSelections = orderedSelections( ...
        receivers, numel(neighbors));
    workCount = workCount + size(receiverSelections, 1);
    [selectedReceivers, selectedSenders, localScore, localFeasible] = ...
        bestReceiverAssignment(neighbors, receiverSelections, ...
            receiverFormationIdx, formation, sensorUids, positions, ...
            physical, reliability);
    if ~localFeasible
        return;
    end
    witnesses(receiverFormationIdx) = struct( ...
        'receiverFormationPhysicalUid', ...
            formation.uids(receiverFormationIdx), ...
        'senderFormationPhysicalUids', formation.uids(neighbors), ...
        'receiverSensorPhysicalUids', ...
            sensorUids(selectedReceivers), ...
        'senderSensorPhysicalUids', sensorUids(selectedSenders), ...
        'score', localScore);
end
globalScore = aggregateScores(witnesses);
feasible = true;
end

function [bestReceivers, bestSenders, bestScore, feasible] = ...
        bestReceiverAssignment(neighbors, receiverSelections, ...
            receiverFormationIdx, formation, sensorUids, positions, ...
            physical, reliability)
slotCount = numel(neighbors);
bestReceivers = zeros(1, 0);
bestSenders = zeros(1, 0);
bestScore = emptyScore();
feasible = false;
for selectionIdx = 1:size(receiverSelections, 1)
    candidateReceivers = receiverSelections(selectionIdx, :);
    candidateSenders = zeros(1, slotCount);
    assignmentFeasible = true;
    for slotIdx = 1:slotCount
        senders = formation.members{neighbors(slotIdx)};
        sender = bestSender(candidateReceivers(slotIdx), senders, ...
            sensorUids, positions, physical, reliability);
        if ~isfinite(sender)
            assignmentFeasible = false;
            break;
        end
        candidateSenders(slotIdx) = sender;
    end
    if ~assignmentFeasible
        continue;
    end
    candidateScore = scoreEdges(candidateReceivers, candidateSenders, ...
        sensorUids, positions, reliability);
    edgeKey = [reshape(formation.uids(neighbors), [], 1), ...
        repmat(formation.uids(receiverFormationIdx), slotCount, 1), ...
        reshape(sensorUids(candidateSenders), [], 1), ...
        reshape(sensorUids(candidateReceivers), [], 1)];
    edgeKey = sortrows(edgeKey, [1, 2, 3, 4]);
    candidateScore.tieKey = reshape(edgeKey', 1, []);
    if ~feasible || scoreBetter(candidateScore, bestScore)
        bestReceivers = candidateReceivers;
        bestSenders = candidateSenders;
        bestScore = candidateScore;
        feasible = true;
    end
end
end

function sender = bestSender(receiver, senders, sensorUids, ...
        positions, physical, reliability)
[~, order] = sort(sensorUids(senders));
senders = senders(order);
sender = NaN;
bestScore = emptyScore();
for candidate = senders
    if ~physical(receiver, candidate)
        continue;
    end
    candidateScore = scoreEdges(receiver, candidate, sensorUids, ...
        positions, reliability);
    candidateScore.tieKey = sensorUids(candidate);
    if ~isfinite(sender) || scoreBetter(candidateScore, bestScore)
        sender = candidate;
        bestScore = candidateScore;
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
    selections = [selections; ...
        perms(combinations(combinationIdx, :))]; %#ok<AGROW>
end
end

function count = orderedSelectionCount(total, selected)
count = 1;
for value = (total - selected + 1):total
    count = count * value;
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
score.tieKey = reshape( ...
    [sensorUids(senders); sensorUids(receivers)], 1, []);
end

function score = aggregateScores(witnesses)
score = emptyScore();
score.bottleneckReliability = Inf;
score.totalLogReliability = 0;
score.totalDistance = 0;
edgeRows = zeros(0, 4);
for witnessIdx = 1:numel(witnesses)
    witness = witnesses(witnessIdx);
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

function [adjacency, placeholderUidPairs] = materializePlaceholder( ...
        pairs, formation, sensorUids, nodeCount)
adjacency = false(nodeCount);
placeholderUidPairs = zeros(size(pairs));
for pairIdx = 1:size(pairs, 1)
    leftMembers = formation.members{pairs(pairIdx, 1)};
    rightMembers = formation.members{pairs(pairIdx, 2)};
    [leftUid, leftLocal] = min(sensorUids(leftMembers));
    [rightUid, rightLocal] = min(sensorUids(rightMembers));
    leftSensor = leftMembers(leftLocal);
    rightSensor = rightMembers(rightLocal);
    adjacency(leftSensor, rightSensor) = true;
    adjacency(rightSensor, leftSensor) = true;
    placeholderUidPairs(pairIdx, :) = [leftUid, rightUid];
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

function order = uidOrder(sensorUids)
[~, order] = sort(sensorUids);
end

function counts = emptyCounts()
counts = struct( ...
    'topologyCandidateCount', 0, ...
    'receiverAssignmentCandidateCount', 0, ...
    'workUnitCount', 0, ...
    'selectedLayerTopologyCount', 0, ...
    'selectedLayerConnectedCount', 0, ...
    'selectedLayerMatchingFeasibleCount', 0, ...
    'selectedLayerOptionalRegisteredRemovalCount', NaN, ...
    'selectedLayerNonregisteredAdditionCount', NaN);
end

function witness = emptyWitness()
witness = struct( ...
    'receiverFormationPhysicalUid', NaN, ...
    'senderFormationPhysicalUids', zeros(1, 0), ...
    'receiverSensorPhysicalUids', zeros(1, 0), ...
    'senderSensorPhysicalUids', zeros(1, 0), ...
    'score', emptyScore());
end

function connected = isConnectedUndirected(adjacency)
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
