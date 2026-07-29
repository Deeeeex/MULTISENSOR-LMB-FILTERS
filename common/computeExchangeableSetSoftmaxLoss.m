function [loss, edgeGradient, assignment, supervisedCount] = ...
    computeExchangeableSetSoftmaxLoss( ...
        edgeScores, candidateIncidence, targetCandidateIndices)
% COMPUTEEXCHANGEABLESETSOFTMAXLOSS Complete-graph set loss and gradient.

if size(candidateIncidence, 1) ~= size(edgeScores, 1) || ...
        any(~isfinite(edgeScores(:))) || ...
        any(~isfinite(candidateIncidence(:))) || ...
        any(candidateIncidence(:) < 0)
    error('Exchangeable set-softmax input is invalid.');
end
graphScores = candidateIncidence' * edgeScores;
logProbability = stableLogSoftmax(graphScores);
assignment = assignExchangeableProposalHeads( ...
    logProbability, targetCandidateIndices);
dGraph = zeros(size(graphScores));
loss = 0;
supervisedCount = numel(assignment.targetGraphIndices);
for targetIdx = 1:supervisedCount
    headIdx = assignment.targetHeads(targetIdx);
    positiveIdx = assignment.targetGraphIndices(targetIdx);
    probability = exp(logProbability(:, headIdx));
    loss = loss - logProbability(positiveIdx, headIdx);
    dGraph(:, headIdx) = dGraph(:, headIdx) + probability;
    dGraph(positiveIdx, headIdx) = ...
        dGraph(positiveIdx, headIdx) - 1;
end
edgeGradient = candidateIncidence * dGraph;
end

function values = stableLogSoftmax(scores)
maximum = max(scores, [], 1);
shifted = bsxfun(@minus, scores, maximum);
normalizer = log(sum(exp(shifted), 1));
values = bsxfun(@minus, shifted, normalizer);
end
