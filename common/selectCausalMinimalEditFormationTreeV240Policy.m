function [adjacency, details] = ...
        selectCausalMinimalEditFormationTreeV240Policy(context)
% SELECTCAUSALMINIMALEDITFORMATIONTREEV240POLICY Preserve, then repair.

protocol = getCausalMinimalEditFormationTreeV240Protocol();
[contract, formation] = validateContext(context, protocol);
physical = logical(context.physicalAdjacency);
physical(1:contract.nodeCount+1:end) = false;
reliability = 1 - context.commConfig.pDropByEdge';
previous = previousFormationTree(context, formation);
currentFormationAdjacency = formationAdjacencyFromPhysical( ...
    physical, formation.members);

previousAvailable = isTree(previous) && ...
    treeUsesOnlyAvailableEdges(previous, currentFormationAdjacency);
previousAssignmentFeasible = previousAvailable && ...
    treeAssignmentsFeasible(previous, physical, formation.members);
preserved = previousAvailable && previousAssignmentFeasible;
if preserved
    tree = previous;
    selection = scoreTree(tree, previous, physical, reliability, ...
        context.positions, formation);
    selection.candidateCount = 1;
    selection.feasibleCandidateCount = 1;
else
    [tree, selection] = selectCurrentTree( ...
        currentFormationAdjacency, previous, physical, reliability, ...
        context.positions, formation, ...
        protocol.maximumTreeEnumerationCount);
end

dynamicRegistration = sensorRegistrationForTree( ...
    tree, physical, formation.members);
[dominantAdjacency, residualAdjacency, construction] = ...
    buildIndexEquivariantFormationBackboneReference( ...
        formation.groupIds, context.sensorPhysicalUids, ...
        context.formationPhysicalUidsBySensor, context.positions, ...
        dynamicRegistration, physical, ...
        context.commConfig.pDropByEdge);
[adjacency, fusionWeights, route] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        protocol.dominantWeight, protocol.residualWeight);
if route.duplicateSourceFraction ~= 0 || ...
        nnz(adjacency) ~= ...
            protocol.directedInputsPerReceiver * contract.nodeCount || ...
        route.maximumMessagesPerReceiver ~= ...
            protocol.directedInputsPerReceiver || ...
        (isfield(context, 'directedMessageBudget') && ...
         nnz(adjacency) > context.directedMessageBudget)
    error('CausalMinimalEditFormationTreeV240:InvalidRoute', ...
        'The selected route violates the matched two-input contract.');
end

hadPreviousTree = isTree(previous);
details = route;
details.contractVersion = ...
    'causal-minimal-edit-formation-tree-v240-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'causal-minimal-edit-current-physical-formation-tree';
details.backboneMode = details.mode;
details.objective = selection.retainedEdgeCount;
details.routeObjectiveComponents = [selection.retainedEdgeCount, ...
    selection.bottleneckReliability, ...
    selection.totalLogReliability, -selection.totalDistance];
details.fusionWeightMatrix = fusionWeights;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.construction = construction;
details.currentFormationTreeAdjacency = tree;
details.previousFormationTreeAdjacency = previous;
details.currentFormationTreePairs = pairsFromTree( ...
    tree, formation.uids);
details.previousFormationTreePairs = pairsFromTree( ...
    previous, formation.uids);
details.formationPhysicalUids = formation.uids;
details.incumbentTreeAvailable = previousAvailable;
details.incumbentAssignmentFeasible = previousAssignmentFeasible;
details.incumbentTreePreserved = preserved;
details.treeReselectionUsed = hadPreviousTree && ~preserved;
details.initialTreeSelectionUsed = ~hadPreviousTree;
details.physicalInfeasibilityFallbackUsed = ...
    hadPreviousTree && ~previousAvailable;
details.assignmentInfeasibilityFallbackUsed = ...
    hadPreviousTree && previousAvailable && ...
        ~previousAssignmentFeasible;
details.formationEdgeReplacementCount = ...
    formationEdgeReplacementCount(previous, tree);
details.validCandidateCount = selection.feasibleCandidateCount;
details.enumeratedCandidateCount = selection.candidateCount;
details.referenceMessageCount = nnz(adjacency);
details.currentMessageCount = nnz(adjacency);
details.currentGeometryUsed = true;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.previousPolicyHistoryUsed = hadPreviousTree;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
details.referenceFallbackUsed = false;
details.scheduleCertificate = buildSchedule(context.currentTime, details);
end

function [contract, formation] = validateContext(context, protocol)
required = {'localPosteriorBySensor', 'model', 'physicalAdjacency', ...
    'positions', 'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor', 'commConfig', 'currentTime'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'presetName') || ...
        ~ismember(context.model.dynamicTopologyScenario.config.presetName, ...
            protocol.allowedPresets) || ...
        ~isfield(context.commConfig, 'pDropByEdge')
    error('CausalMinimalEditFormationTreeV240:InvalidContext', ...
        'The current physical route context is incomplete.');
end
nodeCount = numel(context.localPosteriorBySensor);
matrixSize = [nodeCount, nodeCount];
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
formationUidsBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
if nodeCount < 4 || numel(groupIds) ~= nodeCount || ...
        numel(sensorUids) ~= nodeCount || ...
        numel(unique(sensorUids)) ~= nodeCount || ...
        numel(formationUidsBySensor) ~= nodeCount || ...
        ~isequal(size(context.physicalAdjacency), matrixSize) || ...
        ~isequal(size(context.positions), [2, nodeCount]) || ...
        ~isequal(size(context.commConfig.pDropByEdge), matrixSize) || ...
        any(~isfinite(context.positions(:))) || ...
        any(~isfinite(context.commConfig.pDropByEdge(:))) || ...
        any(context.commConfig.pDropByEdge(:) < 0) || ...
        any(context.commConfig.pDropByEdge(:) > 1)
    error('CausalMinimalEditFormationTreeV240:InvalidContext', ...
        'The route matrices or immutable identities are malformed.');
end

groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
members = cell(1, formationCount);
formationUids = zeros(1, formationCount);
for formationIdx = 1:formationCount
    currentMembers = find(groupIds == groupLabels(formationIdx));
    currentUids = unique(formationUidsBySensor(currentMembers));
    if numel(currentMembers) < 3 || numel(currentUids) ~= 1
        error('CausalMinimalEditFormationTreeV240:InvalidContext', ...
            'Every formation needs one UID and at least three sensors.');
    end
    members{formationIdx} = currentMembers;
    formationUids(formationIdx) = currentUids;
end
[formationUids, order] = sort(formationUids);
members = members(order);
groupLabels = groupLabels(order);
canonicalGroupIds = zeros(1, nodeCount);
for formationIdx = 1:formationCount
    canonicalGroupIds(members{formationIdx}) = formationIdx;
end
if numel(unique(formationUids)) ~= formationCount
    error('CausalMinimalEditFormationTreeV240:InvalidContext', ...
        'Formation physical UIDs must be unique.');
end
contract = struct('nodeCount', nodeCount, ...
    'formationCount', formationCount);
formation = struct('count', formationCount, 'uids', formationUids, ...
    'members', {members}, 'groupLabels', groupLabels, ...
    'groupIds', canonicalGroupIds);
end

function previous = previousFormationTree(context, formation)
previous = false(formation.count);
if ~isfield(context, 'previousAdjacencyHistory') || ...
        isempty(context.previousAdjacencyHistory)
    return;
end
history = logical(context.previousAdjacencyHistory);
if size(history, 1) ~= numel(formation.groupIds) || ...
        size(history, 2) ~= numel(formation.groupIds)
    error('CausalMinimalEditFormationTreeV240:InvalidHistory', ...
        'Previous adjacency history has the wrong dimensions.');
end
previousSensor = history(:, :, end);
for left = 1:formation.count-1
    for right = left+1:formation.count
        cross = previousSensor(formation.members{left}, ...
            formation.members{right});
        reverse = previousSensor(formation.members{right}, ...
            formation.members{left});
        previous(left, right) = any(cross(:)) || any(reverse(:));
        previous(right, left) = previous(left, right);
    end
end
end

function adjacency = formationAdjacencyFromPhysical(physical, members)
count = numel(members);
adjacency = false(count);
for left = 1:count-1
    for right = left+1:count
        block = physical(members{left}, members{right});
        adjacency(left, right) = any(block(:));
        adjacency(right, left) = adjacency(left, right);
    end
end
end

function passed = treeUsesOnlyAvailableEdges(tree, available)
passed = ~any(tree(:) & ~available(:));
end

function passed = treeAssignmentsFeasible(tree, physical, members)
passed = true;
for receiverFormation = 1:numel(members)
    neighbors = find(tree(receiverFormation, :));
    receivers = members{receiverFormation};
    availability = false(numel(receivers), numel(neighbors));
    for slot = 1:numel(neighbors)
        senders = members{neighbors(slot)};
        availability(:, slot) = any( ...
            physical(receivers, senders), 2);
    end
    if ~hasInjectiveReceiverAssignment(availability)
        passed = false;
        return;
    end
end
end

function passed = hasInjectiveReceiverAssignment(availability)
slotCount = size(availability, 2);
matchedSlotByReceiver = zeros(1, size(availability, 1));
passed = true;
for slot = 1:slotCount
    visited = false(1, size(availability, 1));
    [assigned, matchedSlotByReceiver] = augmentAssignment( ...
        slot, availability, visited, matchedSlotByReceiver);
    if ~assigned
        passed = false;
        return;
    end
end
end

function [assigned, matching] = augmentAssignment( ...
        slot, availability, visited, matching)
assigned = false;
for receiver = reshape(find(availability(:, slot)), 1, [])
    if visited(receiver)
        continue;
    end
    visited(receiver) = true;
    if matching(receiver) == 0
        matching(receiver) = slot;
        assigned = true;
        return;
    end
    [reassigned, matching] = augmentAssignment( ...
        matching(receiver), availability, visited, matching);
    if reassigned
        matching(receiver) = slot;
        assigned = true;
        return;
    end
end
end

function [tree, best] = selectCurrentTree(available, previous, ...
        physical, reliability, positions, formation, maximumCount)
edgeList = zeros(0, 2);
for left = 1:formation.count-1
    for right = left+1:formation.count
        if available(left, right)
            edgeList(end + 1, :) = [left, right]; %#ok<AGROW>
        end
    end
end
if ~isConnected(available)
    error('CausalMinimalEditFormationTreeV240:DisconnectedPhysicalGraph', ...
        'The current physical formation graph has no spanning tree.');
end
selectionCount = combinationCount( ...
    size(edgeList, 1), formation.count - 1, maximumCount);
if selectionCount > maximumCount
    error('CausalMinimalEditFormationTreeV240:EnumerationLimitExceeded', ...
        'Current tree enumeration exceeds the registered limit.');
end
combinations = nchoosek(1:size(edgeList, 1), formation.count - 1);
best = emptyScore();
tree = false(formation.count);
feasibleCount = 0;
for candidateIdx = 1:size(combinations, 1)
    candidate = false(formation.count);
    selected = edgeList(combinations(candidateIdx, :), :);
    for edgeIdx = 1:size(selected, 1)
        candidate(selected(edgeIdx, 1), selected(edgeIdx, 2)) = true;
        candidate(selected(edgeIdx, 2), selected(edgeIdx, 1)) = true;
    end
    if ~isTree(candidate) || ...
            ~treeAssignmentsFeasible(candidate, physical, ...
                formation.members)
        continue;
    end
    feasibleCount = feasibleCount + 1;
    score = scoreTree(candidate, previous, physical, reliability, ...
        positions, formation);
    if isempty(best.tieKey) || scoreBetter(score, best)
        best = score;
        tree = candidate;
    end
end
if feasibleCount == 0
    error('CausalMinimalEditFormationTreeV240:NoFeasibleTree', ...
        'No current formation tree admits distinct incoming receivers.');
end
best.candidateCount = size(combinations, 1);
best.feasibleCandidateCount = feasibleCount;
end

function score = scoreTree(tree, previous, physical, reliability, ...
        positions, formation)
score = emptyScore();
score.retainedEdgeCount = nnz(triu(tree & previous, 1));
score.bottleneckReliability = inf;
score.totalLogReliability = 0;
score.totalDistance = 0;
uidPairs = zeros(0, 2);
for left = 1:formation.count-1
    for right = left+1:formation.count
        if ~tree(left, right)
            continue;
        end
        leftMembers = formation.members{left};
        rightMembers = formation.members{right};
        forwardMask = physical(leftMembers, rightMembers);
        reverseMask = physical(rightMembers, leftMembers);
        forwardReliability = reliability(leftMembers, rightMembers);
        reverseReliability = reliability(rightMembers, leftMembers);
        bestForward = max(forwardReliability(forwardMask));
        bestReverse = max(reverseReliability(reverseMask));
        pairReliability = min(bestForward, bestReverse);
        score.bottleneckReliability = min( ...
            score.bottleneckReliability, pairReliability);
        score.totalLogReliability = score.totalLogReliability + ...
            log(max(bestForward, realmin)) + ...
            log(max(bestReverse, realmin));
        delta = reshape(positions(:, leftMembers), 2, [], 1) - ...
            reshape(positions(:, rightMembers), 2, 1, []);
        distances = reshape(sqrt(sum(delta .^ 2, 1)), ...
            numel(leftMembers), numel(rightMembers));
        score.totalDistance = score.totalDistance + ...
            min(distances(forwardMask));
        uidPairs(end + 1, :) = sort([ ...
            formation.uids(left), formation.uids(right)]); %#ok<AGROW>
    end
end
uidPairs = sortrows(uidPairs, [1, 2]);
score.tieKey = reshape(uidPairs', 1, []);
end

function score = emptyScore()
score = struct('retainedEdgeCount', -Inf, ...
    'bottleneckReliability', -Inf, ...
    'totalLogReliability', -Inf, 'totalDistance', Inf, ...
    'tieKey', zeros(1, 0), 'candidateCount', 0, ...
    'feasibleCandidateCount', 0);
end

function better = scoreBetter(candidate, incumbent)
tolerance = 1e-12;
if candidate.retainedEdgeCount ~= incumbent.retainedEdgeCount
    better = candidate.retainedEdgeCount > incumbent.retainedEdgeCount;
    return;
end
if abs(candidate.bottleneckReliability - ...
        incumbent.bottleneckReliability) > tolerance
    better = candidate.bottleneckReliability > ...
        incumbent.bottleneckReliability;
    return;
end
if abs(candidate.totalLogReliability - ...
        incumbent.totalLogReliability) > tolerance
    better = candidate.totalLogReliability > ...
        incumbent.totalLogReliability;
    return;
end
if abs(candidate.totalDistance - incumbent.totalDistance) > tolerance
    better = candidate.totalDistance < incumbent.totalDistance;
    return;
end
better = lexicographicLess(candidate.tieKey, incumbent.tieKey);
end

function less = lexicographicLess(left, right)
less = false;
for idx = 1:min(numel(left), numel(right))
    if left(idx) < right(idx)
        less = true;
        return;
    elseif left(idx) > right(idx)
        return;
    end
end
less = numel(left) < numel(right);
end

function registration = sensorRegistrationForTree( ...
        tree, physical, members)
nodeCount = size(physical, 1);
registration = false(nodeCount);
for left = 1:numel(members)-1
    for right = left+1:numel(members)
        if ~tree(left, right)
            continue;
        end
        leftMembers = members{left};
        rightMembers = members{right};
        block = physical(leftMembers, rightMembers);
        registration(leftMembers, rightMembers) = block;
        registration(rightMembers, leftMembers) = block';
    end
end
end

function count = formationEdgeReplacementCount(previous, current)
if ~isTree(previous)
    count = 0;
else
    count = nnz(triu(current & ~previous, 1));
end
end

function pairs = pairsFromTree(tree, uids)
pairs = zeros(0, 2);
for left = 1:size(tree, 1)-1
    for right = left+1:size(tree, 1)
        if tree(left, right)
            pairs(end + 1, :) = sort([uids(left), uids(right)]); %#ok<AGROW>
        end
    end
end
pairs = sortrows(pairs, [1, 2]);
end

function passed = isTree(adjacency)
count = size(adjacency, 1);
passed = isequal(adjacency, adjacency') && ...
    nnz(triu(adjacency, 1)) == count - 1 && isConnected(adjacency);
end

function connected = isConnected(adjacency)
count = size(adjacency, 1);
visited = false(1, count);
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end

function count = combinationCount(total, selected, cap)
if selected < 0 || selected > total
    count = 0;
    return;
end
selected = min(selected, total - selected);
count = 1;
for idx = 1:selected
    count = count * (total - selected + idx) / idx;
    if count > cap
        count = cap + 1;
        return;
    end
end
count = round(count);
end

function schedule = buildSchedule(currentTime, details)
if details.initialTreeSelectionUsed
    phase = 'initial-current-physical-tree';
elseif details.treeReselectionUsed
    phase = 'physical-infeasibility-tree-repair';
else
    phase = 'incumbent-tree-preserved';
end
schedule = struct( ...
    'contractVersion', ...
        'causal-minimal-edit-formation-tree-v240-schedule-v1', ...
    'currentTime', currentTime, 'phase', phase, 'phaseIndex', 1, ...
    'gatewayIndices', zeros(1, 0), ...
    'formationIds', zeros(1, 0), ...
    'sourceIndices', zeros(1, 0), ...
    'proposalGatewayCount', 0, 'selectedGatewayCount', 0, ...
    'pendingAcquisitionTime', NaN, 'messageParityPassed', true, ...
    'rollingSensorStrong', true, 'rollingFormationStrong', true, ...
    'referenceFallbackUsed', false, 'cycleSelected', true, ...
    'treeReselectionUsed', details.treeReselectionUsed, ...
    'formationEdgeReplacementCount', ...
        details.formationEdgeReplacementCount, ...
    'truthReadAtRuntime', false, ...
    'futureOutcomeReadAtRuntime', false);
end
