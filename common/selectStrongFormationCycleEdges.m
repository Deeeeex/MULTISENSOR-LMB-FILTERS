function selection = selectStrongFormationCycleEdges( ...
        groupIds, receiverIndices, senderIndices, scores)
% SELECTSTRONGFORMATIONCYCLEEDGES Maximum-score directed formation cycle.
%
% The selected cross-formation routes form one Hamiltonian directed cycle.
% Every formation has exactly one incoming and one outgoing formation edge,
% so the instantaneous formation graph is strongly connected. Dynamic
% programming solves the formation-level cycle exactly; each formation pair
% uses its best sensor-level endpoint.

groupIds = reshape(groupIds, 1, []);
receiverIndices = reshape(receiverIndices, [], 1);
senderIndices = reshape(senderIndices, [], 1);
scores = reshape(scores, [], 1);
if numel(receiverIndices) ~= numel(senderIndices) || ...
        numel(receiverIndices) ~= numel(scores)
    error('Formation-cycle edge arrays must have the same length.');
end
nodeCount = numel(groupIds);
if any(receiverIndices < 1 | receiverIndices > nodeCount | ...
        senderIndices < 1 | senderIndices > nodeCount | ...
        mod(receiverIndices, 1) ~= 0 | ...
        mod(senderIndices, 1) ~= 0)
    error('Formation-cycle edge indices are invalid.');
end
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('Formation-cycle projection needs at least two formations.');
end
if groupCount > 20
    error('Formation-cycle exact projection supports at most 20 groups.');
end

bestByFormationPair = nan(groupCount);
scoreByFormationPair = -inf(groupCount);
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
    if scores(exampleIdx) > ...
            scoreByFormationPair(senderGroup, receiverGroup)
        scoreByFormationPair(senderGroup, receiverGroup) = ...
            scores(exampleIdx);
        bestByFormationPair(senderGroup, receiverGroup) = ...
            exampleIdx;
    end
end

stateCount = 2 ^ groupCount;
startGroup = 1;
startMask = 2 ^ (startGroup - 1);
pathScore = -inf(stateCount, groupCount);
predecessor = zeros(stateCount, groupCount);
pathScore(startMask + 1, startGroup) = 0;
for mask = 0:(stateCount - 1)
    if bitand(mask, startMask) == 0
        continue;
    end
    for lastGroup = 1:groupCount
        if bitget(mask, lastGroup) == 0 || ...
                ~isfinite(pathScore(mask + 1, lastGroup))
            continue;
        end
        for nextGroup = 2:groupCount
            if bitget(mask, nextGroup) || ...
                    ~isfinite(scoreByFormationPair( ...
                        lastGroup, nextGroup))
                continue;
            end
            nextMask = mask + 2 ^ (nextGroup - 1);
            candidateScore = ...
                pathScore(mask + 1, lastGroup) + ...
                scoreByFormationPair(lastGroup, nextGroup);
            if candidateScore > ...
                    pathScore(nextMask + 1, nextGroup)
                pathScore(nextMask + 1, nextGroup) = ...
                    candidateScore;
                predecessor(nextMask + 1, nextGroup) = ...
                    lastGroup;
            end
        end
    end
end

fullMask = stateCount - 1;
bestScore = -inf;
bestLastGroup = 0;
for lastGroup = 2:groupCount
    if ~isfinite(pathScore(fullMask + 1, lastGroup)) || ...
            ~isfinite(scoreByFormationPair( ...
                lastGroup, startGroup))
        continue;
    end
    candidateScore = ...
        pathScore(fullMask + 1, lastGroup) + ...
        scoreByFormationPair(lastGroup, startGroup);
    if candidateScore > bestScore
        bestScore = candidateScore;
        bestLastGroup = lastGroup;
    end
end
if bestLastGroup == 0
    error('No feasible strongly connected formation cycle exists.');
end

cycle = zeros(1, groupCount);
cycle(1) = startGroup;
mask = fullMask;
lastGroup = bestLastGroup;
for position = groupCount:-1:2
    cycle(position) = lastGroup;
    previousGroup = predecessor(mask + 1, lastGroup);
    mask = mask - 2 ^ (lastGroup - 1);
    lastGroup = previousGroup;
end
exampleIndices = zeros(1, groupCount);
for position = 1:groupCount
    senderGroup = cycle(position);
    receiverGroup = cycle(1 + mod(position, groupCount));
    exampleIndices(position) = ...
        bestByFormationPair(senderGroup, receiverGroup);
end

selection = struct();
selection.formationCycle = cycle;
selection.exampleIndices = exampleIndices;
selection.receiverIndices = ...
    reshape(receiverIndices(exampleIndices), 1, []);
selection.senderIndices = ...
    reshape(senderIndices(exampleIndices), 1, []);
selection.predictedObjective = bestScore;
selection.formationWeakConnected = true;
selection.formationStrongConnected = true;
selection.maximumSourceLoad = maximumSourceLoad( ...
    selection.senderIndices, nodeCount);
end

function value = maximumSourceLoad(senders, nodeCount)
counts = accumarray( ...
    reshape(senders, [], 1), 1, [nodeCount, 1]);
value = max(counts);
end
