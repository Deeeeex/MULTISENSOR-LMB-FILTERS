function selections = enumerateTopKSplicedResidualFormationCycles( ...
        groupIds, baselineSources, dominantSources, ...
        physicalAdjacency, scoreMatrix, orientation, maximumCount, ...
        referenceCuts, maximumCutChanges)
% ENUMERATETOPKSPLICEDRESIDUALFORMATIONCYCLES K-best gateway cycles.
%
% Each candidate cuts one non-dominant residual edge per formation and
% reconnects the cut endpoints along a fixed formation-level orientation.
% A layered K-best dynamic program ranks complete sensor-level residual
% cycles without enumerating the Cartesian product of formation gateways.
% Optional reference cuts and a Hamming-radius bound restrict the search to
% a topology trust region while preserving the same polynomial algorithm.

if nargin < 8 || isempty(referenceCuts)
    referenceCuts = zeros(1, 0);
    maximumCutChanges = 0;
    trustRegionEnabled = false;
elseif nargin < 9 || isempty(maximumCutChanges)
    maximumCutChanges = 1;
    trustRegionEnabled = true;
else
    trustRegionEnabled = true;
end

groupIds = reshape(groupIds, 1, []);
baselineSources = reshape(baselineSources, 1, []);
dominantSources = reshape(dominantSources, 1, []);
nodeCount = numel(groupIds);
if numel(baselineSources) ~= nodeCount || ...
        numel(dominantSources) ~= nodeCount || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(scoreMatrix), [nodeCount, nodeCount]) || ...
        ~isscalar(maximumCount) || ~isfinite(maximumCount) || ...
        maximumCount < 1 || mod(maximumCount, 1) ~= 0
    error('ResidualCycle:InvalidTopKInput', ...
        'Top-K residual-cycle inputs are invalid.');
end
if any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0) || ...
        any(~isfinite(baselineSources)) || ...
        any(~isfinite(dominantSources)) || ...
        any(mod(baselineSources, 1) ~= 0) || ...
        any(mod(dominantSources, 1) ~= 0) || ...
        any(baselineSources < 1 | baselineSources > nodeCount) || ...
        any(dominantSources < 1 | dominantSources > nodeCount)
    error('ResidualCycle:InvalidTopKInput', ...
        'Top-K residual-cycle metadata is invalid.');
end

physical = logical(physicalAdjacency);
physical(1:nodeCount+1:end) = false;
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('ResidualCycle:Infeasible', ...
        'Top-K residual cycles need at least two formations.');
end
if trustRegionEnabled
    referenceCuts = reshape(referenceCuts, 1, []);
    if numel(referenceCuts) ~= groupCount || ...
            ~isscalar(maximumCutChanges) || ...
            ~isfinite(maximumCutChanges) || ...
            maximumCutChanges < 0 || ...
            maximumCutChanges > groupCount || ...
            mod(maximumCutChanges, 1) ~= 0
        error('ResidualCycle:InvalidTopKInput', ...
            'Residual-cycle trust-region inputs are invalid.');
    end
else
    referenceCuts = nan(1, groupCount);
end

orientation = lower(strrep(char(orientation), '_', '-'));
switch orientation
    case 'clockwise'
        formationOrderIndices = 1:groupCount;
    case {'counter-clockwise', 'counterclockwise'}
        orientation = 'counter-clockwise';
        formationOrderIndices = [1, groupCount:-1:2];
    otherwise
        error('ResidualCycle:InvalidTopKInput', ...
            'Unknown residual-cycle orientation: %s.', orientation);
end

candidateReceivers = cell(1, groupCount);
for position = 1:groupCount
    groupIdx = formationOrderIndices(position);
    members = reshape(find(groupIds == groups(groupIdx)), 1, []);
    candidateReceivers{position} = members( ...
        baselineSources(members) ~= dominantSources(members));
    if isempty(candidateReceivers{position})
        error('ResidualCycle:Infeasible', ...
            'Formation %d has no message-count-preserving gateway.', ...
            groups(groupIdx));
    end
    if trustRegionEnabled && ...
            ~ismember(referenceCuts(position), ...
                candidateReceivers{position})
        error('ResidualCycle:InvalidTopKInput', ...
            'Reference cut %d is invalid at formation position %d.', ...
            referenceCuts(position), position);
    end
end

completeScores = zeros(0, 1);
completeCuts = zeros(0, groupCount);
firstCandidates = candidateReceivers{1};
for firstIdx = 1:numel(firstCandidates)
    firstReceiver = firstCandidates(firstIdx);
    initialChangeCount = double(trustRegionEnabled && ...
        firstReceiver ~= referenceCuts(1));
    if initialChangeCount > maximumCutChanges
        continue;
    end
    previousCandidates = firstCandidates(firstIdx);
    stateScores = cell(1, maximumCutChanges + 1);
    statePaths = cell(1, maximumCutChanges + 1);
    stateScores{1, initialChangeCount + 1} = 0;
    statePaths{1, initialChangeCount + 1} = firstReceiver;
    for position = 2:groupCount
        currentCandidates = candidateReceivers{position};
        nextScores = cell(numel(currentCandidates), ...
            maximumCutChanges + 1);
        nextPaths = cell(numel(currentCandidates), ...
            maximumCutChanges + 1);
        for currentIdx = 1:numel(currentCandidates)
            currentReceiver = currentCandidates(currentIdx);
            changeIncrement = double(trustRegionEnabled && ...
                currentReceiver ~= referenceCuts(position));
            gatheredScores = cell(1, maximumCutChanges + 1);
            gatheredPaths = cell(1, maximumCutChanges + 1);
            for previousIdx = 1:numel(previousCandidates)
                transition = transitionScore( ...
                    previousCandidates(previousIdx), currentReceiver, ...
                    baselineSources, dominantSources, physical, ...
                    scoreMatrix);
                if ~isfinite(transition)
                    continue;
                end
                for previousChangeCount = 0:maximumCutChanges
                    nextChangeCount = previousChangeCount + ...
                        changeIncrement;
                    if nextChangeCount > maximumCutChanges || ...
                            isempty(stateScores{ ...
                                previousIdx, ...
                                previousChangeCount + 1})
                        continue;
                    end
                    priorScores = reshape(stateScores{ ...
                        previousIdx, previousChangeCount + 1}, [], 1);
                    priorPaths = statePaths{ ...
                        previousIdx, previousChangeCount + 1};
                    gatheredScores{nextChangeCount + 1} = [ ...
                        gatheredScores{nextChangeCount + 1}; ...
                        priorScores + transition]; %#ok<AGROW>
                    gatheredPaths{nextChangeCount + 1} = [ ...
                        gatheredPaths{nextChangeCount + 1}; ...
                        priorPaths, repmat(currentReceiver, ...
                            size(priorPaths, 1), 1)]; %#ok<AGROW>
                end
            end
            for changeCount = 0:maximumCutChanges
                [nextScores{currentIdx, changeCount + 1}, ...
                    nextPaths{currentIdx, changeCount + 1}] = ...
                    retainTopK( ...
                        gatheredScores{changeCount + 1}, ...
                        gatheredPaths{changeCount + 1}, ...
                        maximumCount);
            end
        end
        previousCandidates = currentCandidates;
        stateScores = nextScores;
        statePaths = nextPaths;
    end

    for lastIdx = 1:numel(previousCandidates)
        closure = transitionScore( ...
            previousCandidates(lastIdx), firstReceiver, ...
            baselineSources, dominantSources, physical, scoreMatrix);
        if ~isfinite(closure)
            continue;
        end
        for changeCount = 0:maximumCutChanges
            if isempty(stateScores{lastIdx, changeCount + 1})
                continue;
            end
            completeScores = [completeScores; ...
                reshape(stateScores{ ...
                    lastIdx, changeCount + 1}, [], 1) + ...
                    closure]; %#ok<AGROW>
            completeCuts = [completeCuts; ...
                statePaths{lastIdx, changeCount + 1}]; %#ok<AGROW>
        end
    end
end

[completeScores, completeCuts] = retainTopK( ...
    completeScores, completeCuts, maximumCount);
if isempty(completeScores)
    error('ResidualCycle:Infeasible', ...
        'No feasible top-K residual cycle exists.');
end

selectionTemplate = struct( ...
    'contractVersion', '', ...
    'orientation', '', ...
    'formationOrder', zeros(1, 0), ...
    'cutReceivers', zeros(1, 0), ...
    'cutSenders', zeros(1, 0), ...
    'crossReceivers', zeros(1, 0), ...
    'crossSenders', zeros(1, 0), ...
    'residualSourcesByReceiver', zeros(1, 0), ...
    'residualAdjacency', false(0), ...
    'predictedObjective', NaN, ...
    'rank', NaN, ...
    'trustRegionEnabled', false, ...
    'referenceCutChangeCount', NaN, ...
    'crossFormationEdgeCount', NaN, ...
    'residualSensorStrongConnected', false, ...
    'residualFormationStrongConnected', false, ...
    'maximumCrossSourceLoad', NaN, ...
    'maximumCrossReceiverLoad', NaN);
selections = repmat(selectionTemplate, 1, numel(completeScores));
for candidateIdx = 1:numel(completeScores)
    selections(candidateIdx) = buildSelection( ...
        groupIds, groups, formationOrderIndices, baselineSources, ...
        completeCuts(candidateIdx, :), completeScores(candidateIdx), ...
        orientation, candidateIdx, trustRegionEnabled, referenceCuts);
end
end

function [scores, paths] = retainTopK(scores, paths, maximumCount)
scores = reshape(scores, [], 1);
if isempty(scores)
    paths = zeros(0, size(paths, 2));
    return;
end
finiteMask = isfinite(scores);
scores = scores(finiteMask);
paths = paths(finiteMask, :);
if isempty(scores)
    return;
end
ranking = [-scores, paths];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
keepCount = min(maximumCount, numel(order));
order = order(1:keepCount);
scores = scores(order);
paths = paths(order, :);
end

function selection = buildSelection( ...
        groupIds, groups, formationOrderIndices, baselineSources, ...
        cuts, objective, orientation, rank, trustRegionEnabled, ...
        referenceCuts)
nodeCount = numel(groupIds);
groupCount = numel(groups);
residualSources = baselineSources;
crossReceivers = zeros(1, groupCount);
crossSenders = zeros(1, groupCount);
for position = 1:groupCount
    nextPosition = 1 + mod(position, groupCount);
    sender = baselineSources(cuts(position));
    receiver = cuts(nextPosition);
    residualSources(receiver) = sender;
    crossReceivers(position) = receiver;
    crossSenders(position) = sender;
end
residualAdjacency = false(nodeCount);
for receiver = 1:nodeCount
    residualAdjacency(receiver, residualSources(receiver)) = true;
end
formationAdjacency = collapseToFormations( ...
    residualAdjacency, groupIds);
sensorStrong = isStronglyConnected(residualAdjacency);
formationStrong = isStronglyConnected(formationAdjacency);
if any(sum(residualAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 1) ~= 1) || ...
        ~sensorStrong || ~formationStrong
    error('ResidualCycle:InvalidTopKResult', ...
        'A ranked residual cycle violates its permutation invariant.');
end
selection = struct();
selection.contractVersion = ...
    'top-k-spliced-residual-formation-cycle-v1';
selection.orientation = orientation;
selection.formationOrder = ...
    reshape(groups(formationOrderIndices), 1, []);
selection.cutReceivers = reshape(cuts, 1, []);
selection.cutSenders = baselineSources(cuts);
selection.crossReceivers = crossReceivers;
selection.crossSenders = crossSenders;
selection.residualSourcesByReceiver = residualSources;
selection.residualAdjacency = residualAdjacency;
selection.predictedObjective = objective;
selection.rank = rank;
selection.trustRegionEnabled = trustRegionEnabled;
if trustRegionEnabled
    selection.referenceCutChangeCount = nnz(cuts ~= referenceCuts);
else
    selection.referenceCutChangeCount = NaN;
end
selection.crossFormationEdgeCount = groupCount;
selection.residualSensorStrongConnected = sensorStrong;
selection.residualFormationStrongConnected = formationStrong;
crossAdjacency = residualAdjacency & ...
    (reshape(groupIds, [], 1) ~= reshape(groupIds, 1, []));
selection.maximumCrossSourceLoad = max(sum(crossAdjacency, 1));
selection.maximumCrossReceiverLoad = 1;
end

function score = transitionScore( ...
        previousReceiver, currentReceiver, baselineSources, ...
        dominantSources, physical, scoreMatrix)
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

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(groupIds, 'stable');
formation = false(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIdx);
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        formation(receiverGroupIdx, senderGroupIdx) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formation(1:numel(groups)+1:end) = false;
end

function valid = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
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
