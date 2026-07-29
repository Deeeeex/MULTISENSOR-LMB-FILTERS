function assignment = assignExchangeableProposalHeads( ...
        graphScores, targetGraphIndices)
% ASSIGNEXCHANGEABLEPROPOSALHEADS Match unordered targets to distinct heads.
%
% graphScores(candidate, head) contains complete-graph scores. Each target
% is assigned to a different exchangeable head by maximum total score.

targetGraphIndices = round(reshape(targetGraphIndices, 1, []));
candidateCount = size(graphScores, 1);
headCount = size(graphScores, 2);
targetCount = numel(targetGraphIndices);
if isempty(graphScores) || targetCount < 1 || ...
        targetCount > headCount || ...
        any(targetGraphIndices < 1) || ...
        any(targetGraphIndices > candidateCount) || ...
        numel(unique(targetGraphIndices)) ~= targetCount || ...
        any(~isfinite(graphScores(:)))
    error('Exchangeable proposal-head assignment input is invalid.');
end

headTuples = orderedHeadTuples(headCount, targetCount);
totalScores = zeros(size(headTuples, 1), 1);
for tupleIdx = 1:size(headTuples, 1)
    keys = sub2ind(size(graphScores), ...
        targetGraphIndices, headTuples(tupleIdx, :));
    totalScores(tupleIdx) = sum(graphScores(keys));
end
[bestScore, bestIdx] = max(totalScores);
targetHeads = headTuples(bestIdx, :);
headTargets = zeros(1, headCount);
headTargets(targetHeads) = targetGraphIndices;
assignment = struct( ...
    'targetGraphIndices', targetGraphIndices, ...
    'targetHeads', targetHeads, ...
    'headTargets', headTargets, ...
    'supervisedHeadMask', headTargets > 0, ...
    'totalTargetScore', bestScore);
end

function tuples = orderedHeadTuples(headCount, targetCount)
if targetCount == headCount
    tuples = sortrows(perms(1:headCount));
    return
end
subsets = nchoosek(1:headCount, targetCount);
tuples = zeros(0, targetCount);
for subsetIdx = 1:size(subsets, 1)
    tuples = [tuples; perms(subsets(subsetIdx, :))]; %#ok<AGROW>
end
tuples = sortrows(tuples);
end
