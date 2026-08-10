function details = buildInterventionLocalSoftReturnWeightsV81( ...
        referenceAdjacency, referenceWeights, appliedTriples, beta)
% BUILDINTERVENTIONLOCALSOFTRETURNWEIGHTSV81 Fractional incumbent return.

nodeCount = size(referenceWeights, 1);
referenceAdjacency = logical(referenceAdjacency);
if ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        size(appliedTriples, 2) ~= 3 || isempty(appliedTriples) || ...
        ~isscalar(beta) || ~isfinite(beta) || beta <= 0 || beta >= 1
    error('SoftReturnV81:InvalidInput', ...
        'The soft-return route or beta is invalid.');
end
weights = referenceWeights;
receivers = reshape(appliedTriples(:, 1), 1, []);
incumbents = reshape(appliedTriples(:, 2), 1, []);
if numel(unique(receivers)) ~= numel(receivers)
    error('SoftReturnV81:DuplicateReceiver', ...
        'Each intervention-local receiver may appear only once.');
end
referenceResidualWeights = zeros(1, numel(receivers));
for rowIdx = 1:numel(receivers)
    receiverIdx = receivers(rowIdx);
    incumbentIdx = incumbents(rowIdx);
    sourceWeight = referenceWeights(receiverIdx, incumbentIdx);
    if sourceWeight <= 0 || ...
            ~referenceAdjacency(receiverIdx, incumbentIdx) || ...
            receiverIdx == incumbentIdx
        error('SoftReturnV81:InvalidIncumbent', ...
            'An intervention row has no positive incumbent input.');
    end
    referenceResidualWeights(rowIdx) = sourceWeight;
    returnedToSelf = (1 - beta) * sourceWeight;
    weights(receiverIdx, incumbentIdx) = beta * sourceWeight;
    weights(receiverIdx, receiverIdx) = ...
        weights(receiverIdx, receiverIdx) + returnedToSelf;
end
support = referenceAdjacency | logical(eye(nodeCount));
if any(weights(support) <= 0) || ...
        any(weights(:) > 0 & ~support(:)) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12)
    error('SoftReturnV81:InvalidWeights', ...
        'Soft-return weights violate reference support or row sums.');
end
details = struct();
details.contractVersion = ...
    'intervention-local-soft-return-weights-v81-v1';
details.beta = beta;
details.adjacency = referenceAdjacency;
details.fusionWeights = weights;
details.receivers = receivers;
details.incumbentSenders = incumbents;
details.referenceResidualWeights = referenceResidualWeights;
details.softResidualWeights = beta * referenceResidualWeights;
details.returnedSelfWeightByReceiver = ...
    (1 - beta) * referenceResidualWeights;
details.messageCount = nnz(referenceAdjacency);
details.exactTopologyParity = true;
details.exactMessageCountParity = true;
details.posteriorUsed = false;
details.currentLinkReliabilityUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end
