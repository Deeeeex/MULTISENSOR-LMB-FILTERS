function selection = selectScaleAwareResidualGatewayLanes( ...
        groupIds, baselineSources, dominantSources, ...
        physicalAdjacency, scoreMatrix, formationOrder, offsets)
% SELECTSCALEAWARERESIDUALGATEWAYLANES Rewire equal-cost gateway lanes.
%
% Each lane cuts one nonduplicate edge from every local residual cycle and
% reconnects those senders across formations at a prescribed circular
% offset.  Receivers and senders remain one-to-one, so the residual route
% keeps exactly one attempted input per receiver and the combined message
% count is unchanged.  The high-weight dominant route is not modified.

groupIds = reshape(groupIds, 1, []);
baselineSources = reshape(baselineSources, 1, []);
dominantSources = reshape(dominantSources, 1, []);
formationOrder = reshape(formationOrder, 1, []);
offsets = reshape(offsets, 1, []);
nodeCount = numel(groupIds);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
if nodeCount < 2 || formationCount < 2 || ...
        numel(baselineSources) ~= nodeCount || ...
        numel(dominantSources) ~= nodeCount || ...
        ~isequal(size(physicalAdjacency), ...
            [nodeCount, nodeCount]) || ...
        ~isequal(size(scoreMatrix), [nodeCount, nodeCount]) || ...
        numel(formationOrder) ~= formationCount || ...
        ~isequal(sort(formationOrder), sort(groups)) || ...
        isempty(offsets) || any(~isfinite(offsets)) || ...
        any(mod(offsets, 1) ~= 0)
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware residual-lane inputs are invalid.');
end
if any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0) || ...
        any(~isfinite(baselineSources)) || ...
        any(~isfinite(dominantSources)) || ...
        any(mod(baselineSources, 1) ~= 0) || ...
        any(mod(dominantSources, 1) ~= 0) || ...
        any(baselineSources < 1 | baselineSources > nodeCount) || ...
        any(dominantSources < 1 | dominantSources > nodeCount)
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware source metadata are invalid.');
end

physical = logical(physicalAdjacency);
physical(1:nodeCount+1:end) = false;
baselineAdjacency = sourceMapToAdjacency( ...
    baselineSources, nodeCount);
if any(sum(baselineAdjacency, 2) ~= 1) || ...
        any(sum(baselineAdjacency, 1) ~= 1) || ...
        any(baselineAdjacency(:) & ~physical(:))
    error('ScaleMixing:InvalidContract', ...
        'The local residual baseline must be a physical permutation.');
end

offsetsModulo = mod(offsets, formationCount);
if any(offsetsModulo == 0)
    error('ScaleMixing:InvalidContract', ...
        'Every gateway lane must cross to another formation.');
end
laneCount = numel(offsetsModulo);
candidateReceivers = cell(1, formationCount);
for position = 1:formationCount
    members = reshape(find( ...
        groupIds == formationOrder(position)), 1, []);
    candidateReceivers{position} = members( ...
        baselineSources(members) ~= dominantSources(members));
    if numel(candidateReceivers{position}) < laneCount
        error('ScaleMixing:Infeasible', [ ...
            'Formation %d has fewer nonduplicate residual cuts than ', ...
            'the requested gateway-lane count.'], ...
            formationOrder(position));
    end
end

usedCuts = false(1, nodeCount);
cutReceiversByLane = zeros(laneCount, formationCount);
cutSendersByLane = zeros(laneCount, formationCount);
laneObjectives = zeros(1, laneCount);
residualSources = baselineSources;
for laneIdx = 1:laneCount
    offset = offsetsModulo(laneIdx);
    permutationCycles = decomposeOffsetPermutation( ...
        formationCount, offset);
    selectedCuts = zeros(1, formationCount);
    objective = 0;
    for cycleIdx = 1:numel(permutationCycles)
        positions = permutationCycles{cycleIdx};
        available = cell(1, numel(positions));
        for cursor = 1:numel(positions)
            available{cursor} = candidateReceivers{positions(cursor)}( ...
                ~usedCuts(candidateReceivers{positions(cursor)}));
            if isempty(available{cursor})
                error('ScaleMixing:Infeasible', ...
                    'No unused gateway cut remains for lane %d.', ...
                    laneIdx);
            end
        end
        [cycleCuts, cycleObjective] = solveCutCycle( ...
            available, baselineSources, dominantSources, ...
            physical, scoreMatrix);
        if isempty(cycleCuts)
            error('ScaleMixing:Infeasible', [ ...
                'No physical realization exists for gateway lane %d ', ...
                'at offset %d.'], laneIdx, offset);
        end
        selectedCuts(positions) = cycleCuts;
        objective = objective + cycleObjective;
    end
    if any(selectedCuts == 0) || ...
            numel(unique(selectedCuts)) ~= formationCount
        error('ScaleMixing:Infeasible', ...
            'Gateway-lane projection did not select one cut per formation.');
    end
    usedCuts(selectedCuts) = true;
    cutReceiversByLane(laneIdx, :) = selectedCuts;
    cutSendersByLane(laneIdx, :) = ...
        baselineSources(selectedCuts);
    laneObjectives(laneIdx) = objective;
    for sourcePosition = 1:formationCount
        targetPosition = 1 + mod( ...
            sourcePosition - 1 + offset, formationCount);
        sender = baselineSources(selectedCuts(sourcePosition));
        receiver = selectedCuts(targetPosition);
        if ~physical(receiver, sender) || ...
                sender == dominantSources(receiver)
            error('ScaleMixing:Infeasible', ...
                'Projected gateway lane contains an invalid cross edge.');
        end
        residualSources(receiver) = sender;
    end
end

residualAdjacency = sourceMapToAdjacency( ...
    residualSources, nodeCount);
crossMask = groupIds ~= groupIds(residualSources);
expectedCrossCount = formationCount * laneCount;
if any(sum(residualAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 1) ~= 1) || ...
        any(residualAdjacency(:) & ~physical(:)) || ...
        nnz(crossMask) ~= expectedCrossCount || ...
        nnz(baselineSources == dominantSources) ~= ...
            nnz(residualSources == dominantSources)
    error('ScaleMixing:Infeasible', [ ...
        'Projected residual lanes violated permutation, physical, ', ...
        'cross-count, or duplicate-count invariants.']);
end

formationAdjacency = false(formationCount);
for receiver = find(crossMask)
    receiverPosition = find(formationOrder == ...
        groupIds(receiver), 1);
    senderPosition = find(formationOrder == ...
        groupIds(residualSources(receiver)), 1);
    formationAdjacency(receiverPosition, senderPosition) = true;
end
if ~isStronglyConnected(formationAdjacency)
    error('ScaleMixing:Infeasible', ...
        'Projected gateway lanes are not formation-strongly-connected.');
end

selection = struct();
selection.contractVersion = ...
    'scale-aware-residual-gateway-lanes-v1';
selection.formationOrder = formationOrder;
selection.requestedOffsets = offsets;
selection.offsetsModulo = offsetsModulo;
selection.gatewayLaneCount = laneCount;
selection.cutReceiversByLane = cutReceiversByLane;
selection.cutSendersByLane = cutSendersByLane;
selection.laneObjectives = laneObjectives;
selection.objective = sum(laneObjectives);
selection.residualSourcesByReceiver = residualSources;
selection.residualAdjacency = residualAdjacency;
selection.crossReceiverMask = crossMask;
selection.crossFormationEdgeCount = nnz(crossMask);
selection.expectedCrossFormationEdgeCount = expectedCrossCount;
selection.residualSensorStrongConnected = ...
    isStronglyConnected(residualAdjacency);
selection.residualFormationStrongConnected = true;
selection.residualPermutationPreserved = true;
selection.dominantResidualDuplicateCount = ...
    nnz(residualSources == dominantSources);
selection.truthUsed = false;
selection.futureOutcomeUsed = false;
end

function adjacency = sourceMapToAdjacency(sources, nodeCount)
adjacency = false(nodeCount);
for receiver = 1:nodeCount
    adjacency(receiver, sources(receiver)) = true;
end
end

function cycles = decomposeOffsetPermutation(count, offset)
visited = false(1, count);
cycles = cell(1, 0);
for start = 1:count
    if visited(start)
        continue;
    end
    positions = zeros(1, 0);
    cursor = start;
    while ~visited(cursor)
        visited(cursor) = true;
        positions(end + 1) = cursor; %#ok<AGROW>
        cursor = 1 + mod(cursor - 1 + offset, count);
    end
    cycles{end + 1} = positions; %#ok<AGROW>
end
end

function [bestCuts, bestObjective] = solveCutCycle( ...
        candidates, baselineSources, dominantSources, ...
        physical, scoreMatrix)
bestCuts = [];
bestObjective = -inf;
firstCandidates = reshape(candidates{1}, 1, []);
for firstIdx = 1:numel(firstCandidates)
    firstCut = firstCandidates(firstIdx);
    previousCandidates = firstCandidates;
    pathScores = -inf(1, numel(previousCandidates));
    pathScores(firstIdx) = 0;
    paths = cell(1, numel(previousCandidates));
    paths{firstIdx} = firstCut;
    for position = 2:numel(candidates)
        currentCandidates = reshape(candidates{position}, 1, []);
        nextScores = -inf(1, numel(currentCandidates));
        nextPaths = cell(1, numel(currentCandidates));
        for currentIdx = 1:numel(currentCandidates)
            currentCut = currentCandidates(currentIdx);
            for previousIdx = 1:numel(previousCandidates)
                if ~isfinite(pathScores(previousIdx))
                    continue;
                end
                transition = transitionScore( ...
                    previousCandidates(previousIdx), currentCut, ...
                    baselineSources, dominantSources, ...
                    physical, scoreMatrix);
                candidateScore = pathScores(previousIdx) + transition;
                if candidateScore > nextScores(currentIdx)
                    nextScores(currentIdx) = candidateScore;
                    nextPaths{currentIdx} = [ ...
                        paths{previousIdx}, currentCut];
                end
            end
        end
        previousCandidates = currentCandidates;
        pathScores = nextScores;
        paths = nextPaths;
    end
    for lastIdx = 1:numel(previousCandidates)
        if ~isfinite(pathScores(lastIdx))
            continue;
        end
        closure = transitionScore( ...
            previousCandidates(lastIdx), firstCut, ...
            baselineSources, dominantSources, physical, scoreMatrix);
        objective = pathScores(lastIdx) + closure;
        if objective > bestObjective
            bestObjective = objective;
            bestCuts = paths{lastIdx};
        end
    end
end
end

function value = transitionScore( ...
        previousCut, currentCut, baselineSources, ...
        dominantSources, physical, scoreMatrix)
sender = baselineSources(previousCut);
if sender == dominantSources(currentCut) || ...
        ~physical(currentCut, sender)
    value = -inf;
else
    value = scoreMatrix(currentCut, sender);
    if ~isfinite(value)
        value = -inf;
    end
end
end

function valid = isStronglyConnected(policyAdjacency)
senderAdjacency = logical(policyAdjacency');
valid = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function valid = reachableAll(adjacency)
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
valid = all(visited);
end
