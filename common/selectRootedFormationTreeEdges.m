function selection = selectRootedFormationTreeEdges( ...
        groupIds, receiverIndices, senderIndices, scores, options)
% SELECTROOTEDFORMATIONTREEEDGES Project edge scores onto a rooted tree.
%
% Every non-root formation receives exactly one directed cross-formation
% edge. The selected sensor sender is used at most once. The output is a
% rooted spanning arborescence and therefore weakly connected by design.
% An optional previous formation adjacency can require the two-graph union
% to be strongly connected.

if nargin < 5 || isempty(options)
    options = struct();
end

groupIds = reshape(groupIds, 1, []);
receiverIndices = reshape(receiverIndices, [], 1);
senderIndices = reshape(senderIndices, [], 1);
scores = reshape(scores, [], 1);
if numel(receiverIndices) ~= numel(senderIndices) || ...
        numel(receiverIndices) ~= numel(scores)
    error('Formation-tree edge arrays must have the same length.');
end
nodeCount = numel(groupIds);
if any(receiverIndices < 1 | receiverIndices > nodeCount | ...
        senderIndices < 1 | senderIndices > nodeCount | ...
        mod(receiverIndices, 1) ~= 0 | ...
        mod(senderIndices, 1) ~= 0)
    error('Formation-tree edge indices are invalid.');
end
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('Formation-tree projection needs at least two formations.');
end
requiredUnion = getField( ...
    options, 'requiredUnionFormationAdjacency', []);
if ~isempty(requiredUnion)
    requiredUnion = logical(requiredUnion);
    if ~isequal(size(requiredUnion), [groupCount, groupCount])
        error('Required formation-union adjacency must be G-by-G.');
    end
    requiredUnion(1:groupCount+1:end) = false;
end

candidatesByFormationPair = cell(groupCount);
for exampleIdx = 1:numel(scores)
    if ~isfinite(scores(exampleIdx))
        continue;
    end
    receiverGroup = find(groups == ...
        groupIds(receiverIndices(exampleIdx)), 1);
    senderGroup = find(groups == ...
        groupIds(senderIndices(exampleIdx)), 1);
    if receiverGroup == senderGroup
        continue;
    end
    candidatesByFormationPair{senderGroup, receiverGroup}(end + 1) = ...
        exampleIdx;
end
sourceAssignmentCache = buildSourceAssignmentCache( ...
    groupCount, candidatesByFormationPair, senderIndices, scores);

bestScore = -inf;
bestParent = [];
bestExamples = [];
for root = 1:groupCount
    nonroot = setdiff(1:groupCount, root, 'stable');
    choices = cell(1, numel(nonroot));
    for cursor = 1:numel(nonroot)
        choices{cursor} = setdiff( ...
            1:groupCount, nonroot(cursor), 'stable');
    end
    combinationCount = prod(cellfun(@numel, choices));
    for combinationIdx = 0:(combinationCount - 1)
        code = combinationIdx;
        parent = zeros(1, groupCount);
        for cursor = 1:numel(nonroot)
            radix = numel(choices{cursor});
            choiceIdx = 1 + mod(code, radix);
            code = floor(code / radix);
            parent(nonroot(cursor)) = choices{cursor}(choiceIdx);
        end
        if ~isRootedArborescence(parent, root)
            continue;
        end
        if ~isempty(requiredUnion)
            currentFormationAdjacency = false(groupCount);
            for receiverGroup = nonroot
                currentFormationAdjacency( ...
                    parent(receiverGroup), receiverGroup) = true;
            end
            if ~isStronglyConnected( ...
                    requiredUnion | currentFormationAdjacency)
                continue;
            end
        end
        exampleByReceiverGroup = zeros(1, groupCount);
        score = 0;
        valid = true;
        for senderGroup = 1:groupCount
            children = find(parent == senderGroup);
            if isempty(children)
                continue;
            end
            childMask = sum(2 .^ (children - 1));
            assignment = sourceAssignmentCache{ ...
                senderGroup, childMask + 1};
            if isempty(assignment) || ~assignment.valid
                valid = false;
                break;
            end
            exampleByReceiverGroup(children) = ...
                assignment.exampleByReceiverGroup(children);
            score = score + assignment.score;
        end
        if ~valid
            continue;
        end
        exampleIndices = exampleByReceiverGroup(nonroot);
        if score > bestScore
            bestScore = score;
            bestParent = parent;
            bestExamples = exampleIndices;
        end
    end
end
if isempty(bestExamples)
    error('No feasible rooted formation tree can be projected.');
end

selection = struct();
selection.rootFormation = find(bestParent == 0, 1);
selection.formationParents = bestParent;
selection.exampleIndices = bestExamples;
selection.receiverIndices = ...
    reshape(receiverIndices(bestExamples), 1, []);
selection.senderIndices = ...
    reshape(senderIndices(bestExamples), 1, []);
selection.predictedObjective = bestScore;
selection.formationWeakConnected = true;
selection.formationRooted = true;
selection.formationUnionStrongConnected = ~isempty(requiredUnion);
selection.maximumSourceLoad = maximumSourceLoad( ...
    selection.senderIndices, nodeCount);
end

function cache = buildSourceAssignmentCache( ...
        groupCount, candidatesByFormationPair, senderIndices, scores)
cache = cell(groupCount, 2 ^ groupCount);
for senderGroup = 1:groupCount
    for childMask = 1:(2 ^ groupCount - 1)
        if bitget(childMask, senderGroup)
            continue;
        end
        children = find(bitget(childMask, 1:groupCount));
        cache{senderGroup, childMask + 1} = ...
            solveDistinctSourceAssignment( ...
                senderGroup, children, ...
                candidatesByFormationPair, ...
                senderIndices, scores, groupCount);
    end
end
end

function assignment = solveDistinctSourceAssignment( ...
        senderGroup, children, candidatesByFormationPair, ...
        senderIndices, scores, groupCount)
assignment = struct( ...
    'valid', false, ...
    'score', -inf, ...
    'exampleByReceiverGroup', zeros(1, groupCount));
candidateByChild = cell(1, numel(children));
for childCursor = 1:numel(children)
    childGroup = children(childCursor);
    rawCandidates = candidatesByFormationPair{ ...
        senderGroup, childGroup};
    if isempty(rawCandidates)
        return;
    end
    candidateSenders = unique( ...
        senderIndices(rawCandidates), 'stable');
    bestForSender = zeros(1, numel(candidateSenders));
    for sourceCursor = 1:numel(candidateSenders)
        source = candidateSenders(sourceCursor);
        sourceCandidates = rawCandidates( ...
            senderIndices(rawCandidates) == source);
        [~, bestCursor] = max(scores(sourceCandidates));
        bestForSender(sourceCursor) = ...
            sourceCandidates(bestCursor);
    end
    [~, order] = sort(scores(bestForSender), 'descend');
    candidateByChild{childCursor} = bestForSender(order);
end
[~, childOrder] = sort(cellfun(@numel, candidateByChild));
[bestExamples, bestScore] = searchDistinctSourceAssignments( ...
    1, childOrder, candidateByChild, senderIndices, scores, ...
    false(1, max(senderIndices)), zeros(1, numel(children)), 0);
if ~isfinite(bestScore)
    return;
end
assignment.valid = true;
assignment.score = bestScore;
assignment.exampleByReceiverGroup(children) = bestExamples;
end

function [bestExamples, bestScore] = searchDistinctSourceAssignments( ...
        cursor, childOrder, candidateByChild, senderIndices, scores, ...
        usedSenders, currentExamples, currentScore)
if cursor > numel(childOrder)
    bestExamples = currentExamples;
    bestScore = currentScore;
    return;
end
bestExamples = zeros(size(currentExamples));
bestScore = -inf;
childCursor = childOrder(cursor);
for exampleIdx = reshape(candidateByChild{childCursor}, 1, [])
    senderIdx = senderIndices(exampleIdx);
    if usedSenders(senderIdx)
        continue;
    end
    nextUsed = usedSenders;
    nextUsed(senderIdx) = true;
    nextExamples = currentExamples;
    nextExamples(childCursor) = exampleIdx;
    [candidateExamples, candidateScore] = ...
        searchDistinctSourceAssignments( ...
            cursor + 1, childOrder, candidateByChild, ...
            senderIndices, scores, nextUsed, nextExamples, ...
            currentScore + scores(exampleIdx));
    if candidateScore > bestScore
        bestExamples = candidateExamples;
        bestScore = candidateScore;
    end
end
end

function valid = isRootedArborescence(parent, root)
nodeCount = numel(parent);
valid = parent(root) == 0;
if ~valid
    return;
end
for nodeIdx = setdiff(1:nodeCount, root, 'stable')
    current = nodeIdx;
    seen = false(1, nodeCount);
    while current ~= root
        if current < 1 || current > nodeCount || ...
                seen(current) || parent(current) == 0
            valid = false;
            return;
        end
        seen(current) = true;
        current = parent(current);
    end
end
end

function valid = isStronglyConnected(adjacency)
nodeCount = size(adjacency, 1);
valid = all(reachableFrom(adjacency, 1)) && ...
    all(reachableFrom(adjacency', 1));
end

function reached = reachableFrom(adjacency, startNode)
reached = false(1, size(adjacency, 1));
frontier = startNode;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if reached(node)
        continue;
    end
    reached(node) = true;
    next = find(adjacency(node, :) & ~reached);
    frontier = [frontier, reshape(next, 1, [])]; %#ok<AGROW>
end
end

function value = maximumSourceLoad(senders, nodeCount)
counts = accumarray( ...
    reshape(senders, [], 1), 1, [nodeCount, 1]);
value = max(counts);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
