function details = buildReceiverDominantDampingWeightsV82( ...
        referenceAdjacency, referenceWeights, receiverIndices, factor)
% BUILDRECEIVERDOMINANTDAMPINGWEIGHTSV82 Sparse star-input damping.

nodeCount = size(referenceWeights, 1);
referenceAdjacency = logical(referenceAdjacency);
receiverIndices = reshape(receiverIndices, 1, []);
if ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        isempty(receiverIndices) || ...
        numel(unique(receiverIndices)) ~= numel(receiverIndices) || ...
        any(receiverIndices < 1) || any(receiverIndices > nodeCount) || ...
        ~isscalar(factor) || ~isfinite(factor) || ...
        factor <= 0 || factor >= 1
    error('DominantDampingV82:InvalidInput', ...
        'Receiver indices or damping factor are invalid.');
end
weights = referenceWeights;
dominantSenders = zeros(1, numel(receiverIndices));
referenceDominantWeights = zeros(1, numel(receiverIndices));
for receiverCursor = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(receiverCursor);
    senders = reshape(find(referenceAdjacency(receiverIdx, :)), 1, []);
    senderWeights = referenceWeights(receiverIdx, senders);
    [dominantWeight, localIdx] = max(senderWeights);
    dominantSender = senders(localIdx);
    if dominantWeight <= 0 || ...
            nnz(abs(senderWeights - dominantWeight) <= 1e-12) ~= 1
        error('DominantDampingV82:AmbiguousDominant', ...
            'A nominated receiver has no unique dominant input.');
    end
    dominantSenders(receiverCursor) = dominantSender;
    referenceDominantWeights(receiverCursor) = dominantWeight;
    returnedToSelf = (1 - factor) * dominantWeight;
    weights(receiverIdx, dominantSender) = factor * dominantWeight;
    weights(receiverIdx, receiverIdx) = ...
        weights(receiverIdx, receiverIdx) + returnedToSelf;
end
support = referenceAdjacency | logical(eye(nodeCount));
if any(weights(support) <= 0) || ...
        any(weights(:) > 0 & ~support(:)) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12)
    error('DominantDampingV82:InvalidWeights', ...
        'Dominant damping violates support or row sums.');
end
details = struct();
details.contractVersion = ...
    'receiver-dominant-damping-weights-v82-v1';
details.factor = factor;
details.adjacency = referenceAdjacency;
details.fusionWeights = weights;
details.receiverIndices = receiverIndices;
details.dominantSenders = dominantSenders;
details.referenceDominantWeights = referenceDominantWeights;
details.dampedDominantWeights = ...
    factor * referenceDominantWeights;
details.returnedSelfWeightByReceiver = ...
    (1 - factor) * referenceDominantWeights;
details.messageCount = nnz(referenceAdjacency);
details.exactTopologyParity = true;
details.exactMessageCountParity = true;
details.posteriorUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end
