function [residualAdjacency, details] = ...
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registeredAdjacency, physicalAdjacency)
% BUILDREGISTEREDFORMATIONBACKBONERESIDUALTOUR Deterministic graph tour.
%
% Every undirected edge in the registered formation quotient is doubled
% into one arc per direction.  A deterministic Euler circuit over those
% arcs is expanded into one sensor-level directed Hamiltonian cycle.
% Consequently every sensor has exactly one residual sender, every cross-
% formation edge lies on a registered formation pair, and every registered
% formation pair is represented once in each direction.  The particular
% sensor endpoints need only belong to the current physical action set;
% they need not be edges of the initial static sensor graph.  No posterior,
% link outcome, target truth, or future geometry is read.

groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
if ~isnumeric(groupIds) || ~isreal(groupIds) || ...
        nodeCount < 2 || any(~isfinite(groupIds)) || ...
        any(groupIds < 1) || any(groupIds ~= round(groupIds)) || ...
        ~(isnumeric(registeredAdjacency) || ...
          islogical(registeredAdjacency)) || ...
        ~isreal(registeredAdjacency) || ...
        any(~isfinite(registeredAdjacency(:))) || ...
        ~(isnumeric(physicalAdjacency) || ...
          islogical(physicalAdjacency)) || ...
        ~isreal(physicalAdjacency) || ...
        any(~isfinite(physicalAdjacency(:))) || ...
        ~isequal(size(registeredAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount])
    error('FormationBackboneTour:InvalidContract', ...
        'Formation-backbone residual-tour inputs are invalid.');
end
registered = logical(registeredAdjacency);
registered(1:nodeCount+1:end) = false;
if ~isequal(registered, registered')
    error('FormationBackboneTour:InvalidContract', ...
        'The registered static graph must be symmetric.');
end
physical = logical(physicalAdjacency);
physical(1:nodeCount+1:end) = false;
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
if formationCount < 2 || formationCount > 8
    error('FormationBackboneTour:InvalidContract', ...
        'The residual tour supports between two and eight formations.');
end

formationAdjacency = collapseToFormations(registered, groupIds, groups);
[backboneEdges, degrees] = validateRegisteredFormationBackbone( ...
    formationAdjacency);
membersByFormation = cell(1, formationCount);
for formationIdx = 1:formationCount
    members = reshape(find(groupIds == groups(formationIdx)), 1, []);
    if numel(members) < max(degrees(formationIdx), 3)
        error('FormationBackboneTour:InsufficientSensors', [ ...
            'Formation %d has %d sensors but needs %d Euler blocks.'], ...
            groups(formationIdx), numel(members), degrees(formationIdx));
    end
    % The fixed-index dominant route uses local member 1 as the source for
    % member 2.  Rotating the residual order makes that local edge a cut
    % boundary, so the residual and dominant sources are distinct at every
    % receiver after the cross-formation splice.
    membersByFormation{formationIdx} = [members(2:end), members(1)];
end

eulerVertices = buildDoubledFormationGraphEulerWalk( ...
    formationAdjacency);
blockFormationIndices = eulerVertices(1:end-1);
expectedBlockCount = nnz(formationAdjacency);
if numel(blockFormationIndices) ~= expectedBlockCount
    error('FormationBackboneTour:InternalError', ...
        'The doubled-backbone Euler walk has the wrong length.');
end

blocks = partitionSensorsAcrossEulerBlocks( ...
    membersByFormation, blockFormationIndices, degrees);
cycleOrder = [blocks{:}];
if numel(cycleOrder) ~= nodeCount || ...
        numel(unique(cycleOrder)) ~= nodeCount || ...
        ~isequal(sort(cycleOrder), 1:nodeCount)
    error('FormationBackboneTour:InternalError', ...
        'The sensor expansion is not a Hamiltonian ordering.');
end

residualAdjacency = false(nodeCount);
predecessors = cycleOrder([end, 1:end-1]);
for orderIdx = 1:nodeCount
    receiver = cycleOrder(orderIdx);
    sender = predecessors(orderIdx);
    residualAdjacency(receiver, sender) = true;
end
if any(residualAdjacency(:) & ~physical(:))
    error('FormationBackboneTour:NonphysicalTour', [ ...
        'The deterministic formation-backbone tour contains a currently ', ...
        'nonphysical sensor edge.']);
end

[receiverIndices, senderIndices] = find(residualAdjacency);
crossMask = groupIds(receiverIndices) ~= groupIds(senderIndices);
crossReceivers = reshape(receiverIndices(crossMask), 1, []);
crossSenders = reshape(senderIndices(crossMask), 1, []);
crossFormationPairs = zeros(numel(crossReceivers), 2);
incomingCrossReceiversByFormation = cell(1, formationCount);
incomingCrossSendersByFormation = cell(1, formationCount);
for crossIdx = 1:numel(crossReceivers)
    receiverFormation = find( ...
        groups == groupIds(crossReceivers(crossIdx)), 1);
    senderFormation = find( ...
        groups == groupIds(crossSenders(crossIdx)), 1);
    crossFormationPairs(crossIdx, :) = ...
        [senderFormation, receiverFormation];
    if ~formationAdjacency(receiverFormation, senderFormation)
        error('FormationBackboneTour:InternalError', ...
            'A cross edge escaped the registered formation backbone.');
    end
    incomingCrossReceiversByFormation{receiverFormation}(end + 1) = ...
        crossReceivers(crossIdx); %#ok<AGROW>
    incomingCrossSendersByFormation{receiverFormation}(end + 1) = ...
        crossSenders(crossIdx); %#ok<AGROW>
end
if numel(crossReceivers) ~= expectedBlockCount || ...
        any(cellfun(@numel, incomingCrossReceiversByFormation) ~= degrees) || ...
        any(accumarray(crossReceivers', 1, [nodeCount, 1]) > 1) || ...
        any(accumarray(crossSenders', 1, [nodeCount, 1]) > 1)
    error('FormationBackboneTour:InternalError', ...
        'The directed backbone-edge multiplicities are inconsistent.');
end
for edgeIdx = 1:size(backboneEdges, 1)
    pair = backboneEdges(edgeIdx, :);
    forward = all(crossFormationPairs == pair, 2);
    reverse = all(crossFormationPairs == fliplr(pair), 2);
    if nnz(forward) ~= 1 || nnz(reverse) ~= 1
        error('FormationBackboneTour:InternalError', ...
            'A registered formation pair lacks one arc per direction.');
    end
end
if any(sum(residualAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 1) ~= 1) || ...
        ~isStronglyConnected(residualAdjacency)
    error('FormationBackboneTour:InternalError', ...
        'The residual route is not one directed Hamiltonian cycle.');
end

details = struct();
details.contractVersion = ...
    'registered-formation-backbone-residual-tour-v1';
details.nodeCount = nodeCount;
details.formationCount = formationCount;
details.groups = groups;
details.groupIds = groupIds;
details.registeredFormationAdjacency = formationAdjacency;
details.backboneEdges = backboneEdges;
details.backboneDegrees = degrees;
details.eulerVertices = eulerVertices;
details.blockFormationIndices = blockFormationIndices;
details.blocks = blocks;
details.cycleOrder = cycleOrder;
details.residualSourcesByReceiver = ...
    reshape(findResidualSources(residualAdjacency), 1, []);
details.crossReceivers = crossReceivers;
details.crossSenders = crossSenders;
details.crossFormationPairs = crossFormationPairs;
details.incomingCrossReceiversByFormation = ...
    incomingCrossReceiversByFormation;
details.incomingCrossSendersByFormation = ...
    incomingCrossSendersByFormation;
details.crossFormationMessageCount = numel(crossReceivers);
details.expectedCrossFormationMessageCount = expectedBlockCount;
details.oneResidualInputPerSensor = true;
details.singleDirectedCycle = true;
details.registeredFormationPairsOnly = true;
details.maximumCrossSourceLoad = max(accumarray( ...
    crossSenders', 1, [nodeCount, 1]));
details.maximumCrossReceiverLoad = max(accumarray( ...
    crossReceivers', 1, [nodeCount, 1]));
details.currentPhysicalActionSetUsed = true;
details.posteriorUsed = false;
details.currentLinkReliabilityUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function formation = collapseToFormations(adjacency, groupIds, groups)
formation = false(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIdx);
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        formation(receiverGroupIdx, senderGroupIdx) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formation = formation | formation';
formation(1:numel(groups)+1:end) = false;
end

function [edges, degrees] = ...
    validateRegisteredFormationBackbone(adjacency)
formationCount = size(adjacency, 1);
visited = false(1, formationCount);
visited(1) = true;
queue = 1;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    neighbors = reshape(find(adjacency(current, :)), 1, []);
    for neighbor = neighbors
        if visited(neighbor)
            continue;
        end
        visited(neighbor) = true;
        queue(end + 1) = neighbor; %#ok<AGROW>
    end
end
if ~all(visited)
    error('FormationBackboneTour:DisconnectedRegisteredGraph', ...
        'The registered formation graph is not connected.');
end
[left, right] = find(triu(adjacency, 1));
edges = [left, right];
degrees = reshape(sum(adjacency, 2), 1, []);
end

function walk = buildDoubledFormationGraphEulerWalk(adjacency)
remainingArcs = logical(adjacency);
stack = 1;
reverseCircuit = zeros(1, 0);
while ~isempty(stack)
    current = stack(end);
    next = find(remainingArcs(current, :), 1, 'first');
    if isempty(next)
        reverseCircuit(end + 1) = current; %#ok<AGROW>
        stack(end) = [];
    else
        remainingArcs(current, next) = false;
        stack(end + 1) = next; %#ok<AGROW>
    end
end
walk = fliplr(reverseCircuit);
if any(remainingArcs(:)) || isempty(walk) || ...
        walk(1) ~= walk(end) || ...
        numel(walk) ~= nnz(adjacency) + 1
    error('FormationBackboneTour:InternalError', ...
        'The registered formation backbone has no valid Euler circuit.');
end
end

function blocks = partitionSensorsAcrossEulerBlocks( ...
        membersByFormation, blockFormationIndices, degrees)
blocks = cell(1, numel(blockFormationIndices));
occurrenceCount = zeros(1, numel(membersByFormation));
for blockIdx = 1:numel(blockFormationIndices)
    formationIdx = blockFormationIndices(blockIdx);
    occurrenceCount(formationIdx) = occurrenceCount(formationIdx) + 1;
    members = membersByFormation{formationIdx};
    degree = degrees(formationIdx);
    baseSize = floor(numel(members) / degree);
    remainder = mod(numel(members), degree);
    sizes = baseSize * ones(1, degree);
    sizes(1:remainder) = sizes(1:remainder) + 1;
    occurrenceIdx = occurrenceCount(formationIdx);
    first = 1 + sum(sizes(1:occurrenceIdx-1));
    last = first + sizes(occurrenceIdx) - 1;
    blocks{blockIdx} = members(first:last);
end
if any(occurrenceCount ~= degrees)
    error('FormationBackboneTour:InternalError', ...
        'Euler occurrence counts do not equal backbone degrees.');
end
end

function sources = findResidualSources(adjacency)
sources = zeros(1, size(adjacency, 1));
for receiverIdx = 1:size(adjacency, 1)
    sources(receiverIdx) = find(adjacency(receiverIdx, :), 1);
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
