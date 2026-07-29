function selection = selectSplicedResidualFormationCycle( ...
        groupIds, baselineSources, dominantSources, ...
        physicalAdjacency, scoreMatrix, orientation)
% SELECTSPLICEDRESIDUALFORMATIONCYCLE Exact maximum-score cycle splice.
%
% One edge is cut from each intra-formation residual cycle.  The cut
% sender in every formation is redirected to the cut receiver in the next
% formation.  The resulting residual route is one sensor-level directed
% Hamiltonian cycle, not merely a strongly connected formation quotient.

groupIds = reshape(groupIds, 1, []);
baselineSources = reshape(baselineSources, 1, []);
dominantSources = reshape(dominantSources, 1, []);
nodeCount = numel(groupIds);
if numel(baselineSources) ~= nodeCount || ...
        numel(dominantSources) ~= nodeCount || ...
        ~isequal(size(physicalAdjacency), ...
            [nodeCount, nodeCount]) || ...
        ~isequal(size(scoreMatrix), [nodeCount, nodeCount])
    error('Spliced-cycle inputs have incompatible dimensions.');
end
if any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0) || ...
        any(~isfinite(baselineSources)) || ...
        any(~isfinite(dominantSources)) || ...
        any(mod(baselineSources, 1) ~= 0) || ...
        any(mod(dominantSources, 1) ~= 0) || ...
        any(baselineSources < 1 | baselineSources > nodeCount) || ...
        any(dominantSources < 1 | dominantSources > nodeCount)
    error('Spliced-cycle group/source metadata is invalid.');
end
physical = logical(physicalAdjacency);
physical(1:nodeCount+1:end) = false;
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('Spliced-cycle projection needs at least two formations.');
end

orientation = lower(strrep(char(orientation), '_', '-'));
switch orientation
    case 'clockwise'
        formationOrder = 1:groupCount;
    case {'counter-clockwise', 'counterclockwise'}
        orientation = 'counter-clockwise';
        formationOrder = [1, groupCount:-1:2];
    otherwise
        error('Unknown spliced-cycle orientation: %s.', orientation);
end

candidateReceivers = cell(1, groupCount);
for position = 1:groupCount
    groupCursor = formationOrder(position);
    members = reshape(find( ...
        groupIds == groups(groupCursor)), 1, []);
    allowed = members( ...
        baselineSources(members) ~= ...
            dominantSources(members));
    if isempty(allowed)
        error('ResidualCycle:Infeasible', [ ...
            'Formation %d has no message-count-preserving cut.'], ...
            groups(groupCursor));
    end
    candidateReceivers{position} = allowed;
end

bestObjective = -inf;
bestCuts = zeros(1, groupCount);
firstCandidates = candidateReceivers{1};
for firstCursor = 1:numel(firstCandidates)
    firstReceiver = firstCandidates(firstCursor);
    pathScores = zeros(1, numel(firstCandidates));
    pathScores(:) = -inf;
    pathScores(firstCursor) = 0;
    paths = cell(1, numel(firstCandidates));
    paths{firstCursor} = firstReceiver;
    previousCandidates = firstCandidates;
    for position = 2:groupCount
        currentCandidates = candidateReceivers{position};
        nextScores = -inf(1, numel(currentCandidates));
        nextPaths = cell(1, numel(currentCandidates));
        for currentCursor = 1:numel(currentCandidates)
            currentReceiver = ...
                currentCandidates(currentCursor);
            for previousCursor = 1:numel(previousCandidates)
                if ~isfinite(pathScores(previousCursor))
                    continue;
                end
                previousReceiver = ...
                    previousCandidates(previousCursor);
                transition = transitionScore( ...
                    previousReceiver, currentReceiver, ...
                    baselineSources, dominantSources, ...
                    physical, scoreMatrix);
                candidateScore = ...
                    pathScores(previousCursor) + transition;
                if candidateScore > ...
                        nextScores(currentCursor)
                    nextScores(currentCursor) = candidateScore;
                    nextPaths{currentCursor} = [ ...
                        paths{previousCursor}, ...
                        currentReceiver];
                end
            end
        end
        previousCandidates = currentCandidates;
        pathScores = nextScores;
        paths = nextPaths;
    end
    for lastCursor = 1:numel(previousCandidates)
        if ~isfinite(pathScores(lastCursor))
            continue;
        end
        closure = transitionScore( ...
            previousCandidates(lastCursor), ...
            firstReceiver, baselineSources, ...
            dominantSources, physical, scoreMatrix);
        objective = pathScores(lastCursor) + closure;
        if objective > bestObjective
            bestObjective = objective;
            bestCuts = paths{lastCursor};
        end
    end
end
if ~isfinite(bestObjective) || any(bestCuts == 0)
    error('ResidualCycle:Infeasible', ...
        'No feasible message-count-preserving residual splice exists.');
end

residualSources = baselineSources;
crossReceivers = zeros(1, groupCount);
crossSenders = zeros(1, groupCount);
for position = 1:groupCount
    nextPosition = 1 + mod(position, groupCount);
    sender = baselineSources(bestCuts(position));
    receiver = bestCuts(nextPosition);
    residualSources(receiver) = sender;
    crossReceivers(position) = receiver;
    crossSenders(position) = sender;
end
residualAdjacency = false(nodeCount);
for receiver = 1:nodeCount
    residualAdjacency(receiver, ...
        residualSources(receiver)) = true;
end
if any(sum(residualAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 1) ~= 1) || ...
        ~isStronglyConnected(residualAdjacency)
    error(['Spliced residual construction failed its independent ', ...
        'sensor-cycle invariant.']);
end

selection = struct();
selection.contractVersion = ...
    'spliced-residual-formation-cycle-v1';
selection.orientation = orientation;
selection.formationOrder = ...
    reshape(groups(formationOrder), 1, []);
selection.cutReceivers = bestCuts;
selection.cutSenders = baselineSources(bestCuts);
selection.crossReceivers = crossReceivers;
selection.crossSenders = crossSenders;
selection.residualSourcesByReceiver = ...
    residualSources;
selection.residualAdjacency = residualAdjacency;
selection.predictedObjective = bestObjective;
selection.crossFormationEdgeCount = groupCount;
selection.residualSensorStrongConnected = true;
selection.residualFormationStrongConnected = true;
selection.maximumCrossSourceLoad = 1;
selection.maximumCrossReceiverLoad = 1;
end

function score = transitionScore( ...
        previousReceiver, currentReceiver, ...
        baselineSources, dominantSources, physical, scoreMatrix)
sender = baselineSources(previousReceiver);
if sender == dominantSources(currentReceiver) || ...
        ~physical(currentReceiver, sender)
    score = -inf;
else
    score = scoreMatrix(currentReceiver, sender);
    if ~isfinite(score)
        score = -inf;
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
