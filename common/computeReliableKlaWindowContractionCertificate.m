function certificate = ...
    computeReliableKlaWindowContractionCertificate( ...
        adjacencySequence, fusionWeightSequence, ...
        receiverLinkReliabilitySequence, options)
% COMPUTERELIABLEKLAWINDOWCONTRACTIONCERTIFICATE
% Reliability-aware finite-window mixing certificate.
%
% Matrices use receiver rows and sender columns.  The fusion-weight page is
% the all-selected-input row-stochastic matrix.  The reliability page has
% the same orientation and contains the current delivery probability of
% every selected sender-to-receiver link.
%
% On an event where a registered subset of edges is delivered, every actual
% effective mixing page M_t lower-bounds a substochastic page L_t containing
% the raw self weights and the raw weights of those required edges.  This is
% valid for both `renormalize` and `self` missing-message modes.  Therefore
% the product of the L_t pages lower-bounds the realized product entrywise.
% Its minimum row overlap beta gives delta(P) <= 1-beta on that event.
% Under independent required-edge deliveries with probability q,
%
%   E[delta(P)] <= 1 - q beta.
%
% Later row-stochastic pages cannot increase the Dobrushin coefficient, so
% the strongest certified prefix also certifies the complete supplied
% window.  The bound applies directly to linear consensus and to exact KLA
% set-density log ratios under common positive support.  Marginal Bernoulli
% existence log odds also contain the spatial-overlap normalizer and are not
% linear unless that disturbance is controlled separately.  Local Bayes
% updates, mixture approximation, pruning and label-support loss likewise
% require explicit disturbance terms and are not silently covered here.

if nargin < 4 || isempty(options)
    options = struct();
end
allowedOptionFields = { ...
    'missingNeighborWeightMode', ...
    'maximumIncomingCount', ...
    'requiredDeliveryMask', ...
    'rootCandidates'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedOptionFields))
    error('KlaWindowContraction:InvalidOptions', ...
        'The window-certificate options are malformed.');
end
missingMode = lower(char(getField( ...
    options, 'missingNeighborWeightMode', 'renormalize')));
maximumIncomingCount = getField(options, 'maximumIncomingCount', 4);
if ~ismember(missingMode, {'renormalize', 'self'}) || ...
        ~isscalar(maximumIncomingCount) || ...
        ~isfinite(maximumIncomingCount) || ...
        maximumIncomingCount < 1 || ...
        maximumIncomingCount ~= round(maximumIncomingCount)
    error('KlaWindowContraction:InvalidOptions', ...
        'The missing-message mode or incoming-count limit is invalid.');
end

[adjacency, weights, reliability, nodeCount, horizon] = ...
    validateSequenceInputs( ...
        adjacencySequence, fusionWeightSequence, ...
        receiverLinkReliabilitySequence, maximumIncomingCount);

if isfield(options, 'requiredDeliveryMask')
    requiredMask = validateRequiredMask( ...
        options.requiredDeliveryMask, adjacency, weights);
    scheduleSource = 'caller-supplied-required-delivery-mask';
    rootIndex = nan;
    reachedMask = false(1, nodeCount);
else
    roots = getField(options, 'rootCandidates', 1:nodeCount);
    roots = validateRootCandidates(roots, nodeCount);
    [requiredMask, rootIndex, reachedMask] = ...
        buildBestGreedyBroadcastSchedule( ...
            adjacency, weights, reliability, roots);
    scheduleSource = 'best-greedy-temporal-broadcast-root';
end

schedule = evaluateRequiredSchedule( ...
    requiredMask, weights, reliability);
allDeliveredProduct = eye(nodeCount);
meanMixingProduct = eye(nodeCount);
meanMixingSequence = zeros(nodeCount, nodeCount, horizon);
for timeIdx = 1:horizon
    allDeliveredProduct = ...
        weights(:, :, timeIdx) * allDeliveredProduct;
    meanPage = computeExpectedEffectiveMixingPageMoments( ...
        adjacency(:, :, timeIdx), ...
        weights(:, :, timeIdx), ...
        reliability(:, :, timeIdx), missingMode, ...
        maximumIncomingCount);
    meanMixingSequence(:, :, timeIdx) = meanPage;
    meanMixingProduct = meanPage * meanMixingProduct;
end
[allDeliveredDobrushin, allDeliveredDetails] = ...
    computeDobrushinErgodicityCoefficient(allDeliveredProduct);
[meanMixingDobrushin, meanMixingDetails] = ...
    computeDobrushinErgodicityCoefficient(meanMixingProduct);

certificate = struct();
certificate.contractVersion = ...
    'reliable-kla-window-contraction-certificate-v1';
certificate.nodeCount = nodeCount;
certificate.suppliedHorizon = horizon;
certificate.certificatePrefixLength = schedule.bestPrefixLength;
certificate.missingNeighborWeightMode = missingMode;
certificate.maximumIncomingCount = maximumIncomingCount;
certificate.scheduleSource = scheduleSource;
certificate.rootIndex = rootIndex;
certificate.reachedMask = reachedMask;
certificate.allNodesTemporallyReached = all(reachedMask) || ...
    schedule.eventOverlapLowerBound > 0;
certificate.requiredDeliveryMask = schedule.selectedRequiredMask;
certificate.requiredDeliveryCount = ...
    nnz(schedule.selectedRequiredMask);
certificate.requiredDeliveryEventProbability = ...
    schedule.eventProbability;
certificate.requiredDeliveryEventLogProbability = ...
    schedule.eventLogProbability;
certificate.eventProductLowerBound = ...
    schedule.productLowerBound;
certificate.eventOverlapLowerBound = ...
    schedule.eventOverlapLowerBound;
certificate.eventDobrushinUpperBound = ...
    1 - schedule.eventOverlapLowerBound;
certificate.expectedDobrushinUpperBound = ...
    1 - schedule.expectedContractionGainLowerBound;
certificate.expectedContractionGainLowerBound = ...
    schedule.expectedContractionGainLowerBound;
certificate.allDeliveredMixingProduct = allDeliveredProduct;
certificate.allDeliveredDobrushin = allDeliveredDobrushin;
certificate.allDeliveredDobrushinDetails = allDeliveredDetails;
certificate.meanEffectiveMixingSequence = meanMixingSequence;
certificate.meanEffectiveMixingProduct = meanMixingProduct;
certificate.meanMixingDobrushin = meanMixingDobrushin;
certificate.meanMixingDobrushinDetails = meanMixingDetails;
certificate.meanMixingDobrushinIsExpectedRiskUpperBound = false;
certificate.requiredDeliveryIndependenceAssumed = true;
certificate.laterPagesCannotWeakenCertifiedPrefix = true;
certificate.linearConsensusDiameterBound = true;
certificate.exactKlaLogRatioBoundUnderCommonPositiveSupport = true;
certificate.bernoulliSetDensityLogRatioBoundUnderCommonPositiveSupport = ...
    true;
certificate.marginalBernoulliExistenceLogOddsBound = false;
certificate.spatialOverlapNormalizerDisturbanceAccounted = false;
certificate.localBayesUpdateDisturbanceAccounted = false;
certificate.labelSupportLossAccounted = false;
certificate.mixtureApproximationDisturbanceAccounted = false;
certificate.pruningDisturbanceAccounted = false;
certificate.posteriorUsed = false;
certificate.truthUsed = false;
certificate.futureOutcomeUsed = false;
end

function [adjacency, weights, reliability, nodeCount, horizon] = ...
        validateSequenceInputs( ...
            adjacencyInput, weightInput, reliabilityInput, ...
            maximumIncomingCount)
if ~(isnumeric(adjacencyInput) || islogical(adjacencyInput)) || ...
        ~isreal(adjacencyInput) || any(~isfinite(adjacencyInput(:))) || ...
        any(adjacencyInput(:) ~= 0 & adjacencyInput(:) ~= 1) || ...
        ndims(adjacencyInput) > 3 || ...
        ~isnumeric(weightInput) || ~isreal(weightInput) || ...
        any(~isfinite(weightInput(:))) || ...
        ndims(weightInput) > 3 || ...
        ~isnumeric(reliabilityInput) || ~isreal(reliabilityInput) || ...
        any(~isfinite(reliabilityInput(:))) || ...
        ndims(reliabilityInput) > 3
    error('KlaWindowContraction:InvalidSequence', ...
        'A window-certificate input sequence is malformed.');
end
adjacency = logical(adjacencyInput);
weights = weightInput;
reliability = reliabilityInput;
if ndims(adjacency) == 2
    adjacency = reshape(adjacency, ...
        size(adjacency, 1), size(adjacency, 2), 1);
end
if ndims(weights) == 2
    weights = reshape(weights, ...
        size(weights, 1), size(weights, 2), 1);
end
if ndims(reliability) == 2
    reliability = reshape(reliability, ...
        size(reliability, 1), size(reliability, 2), 1);
end
nodeCount = size(adjacency, 1);
horizon = size(adjacency, 3);
if nodeCount < 2 || size(adjacency, 2) ~= nodeCount || ...
        horizon < 1 || ...
        ~isequal(size(weights), size(adjacency)) || ...
        ~isequal(size(reliability), size(adjacency)) || ...
        any(reliability(:) < 0) || any(reliability(:) > 1) || ...
        any(weights(:) < 0)
    error('KlaWindowContraction:InvalidSequence', ...
        'Window-certificate sequence dimensions or values are invalid.');
end
for timeIdx = 1:horizon
    adjacencyPage = adjacency(:, :, timeIdx);
    weightPage = weights(:, :, timeIdx);
    weightSupport = adjacencyPage | logical(eye(nodeCount));
    if any(diag(adjacencyPage)) || ...
            any(diag(weightPage) <= 0) || ...
            any(abs(sum(weightPage, 2) - 1) > 1e-12) || ...
            any(weightPage(:) > 1e-12 & ~weightSupport(:)) || ...
            any(sum(adjacencyPage, 2) > maximumIncomingCount)
        error('KlaWindowContraction:InvalidSequence', ...
            'A routing page violates its stochastic support contract.');
    end
end
end

function mask = validateRequiredMask(maskInput, adjacency, weights)
if ~(isnumeric(maskInput) || islogical(maskInput)) || ...
        ~isreal(maskInput) || any(~isfinite(maskInput(:))) || ...
        any(maskInput(:) ~= 0 & maskInput(:) ~= 1) || ...
        ndims(maskInput) > 3
    error('KlaWindowContraction:InvalidRequiredSchedule', ...
        'The required delivery mask is malformed.');
end
mask = logical(maskInput);
if ndims(mask) == 2
    mask = reshape(mask, size(mask, 1), size(mask, 2), 1);
end
nodeCount = size(adjacency, 1);
if ~isequal(size(mask), size(adjacency))
    error('KlaWindowContraction:InvalidRequiredSchedule', ...
        'The required delivery mask has the wrong shape.');
end
for timeIdx = 1:size(mask, 3)
    page = mask(:, :, timeIdx);
    adjacencyPage = adjacency(:, :, timeIdx);
    weightPage = weights(:, :, timeIdx);
    if any(diag(page)) || ...
            any(page(:) & ~adjacencyPage(:)) || ...
            any(page(:) & weightPage(:) <= 0)
        error('KlaWindowContraction:InvalidRequiredSchedule', ...
            'A required delivery is absent from the weighted route.');
    end
end
end

function roots = validateRootCandidates(rootsInput, nodeCount)
if ~isnumeric(rootsInput) || ~isreal(rootsInput) || ...
        isempty(rootsInput) || any(~isfinite(rootsInput(:)))
    error('KlaWindowContraction:InvalidRootCandidates', ...
        'The temporal broadcast roots are malformed.');
end
roots = reshape(rootsInput, 1, []);
if any(roots < 1) || any(roots > nodeCount) || ...
        any(roots ~= round(roots)) || ...
        numel(unique(roots)) ~= numel(roots)
    error('KlaWindowContraction:InvalidRootCandidates', ...
        'The temporal broadcast roots are outside the node set.');
end
end

function [bestMask, bestRoot, bestReached] = ...
        buildBestGreedyBroadcastSchedule( ...
            adjacency, weights, reliability, roots)
nodeCount = size(adjacency, 1);
horizon = size(adjacency, 3);
bestGain = -1;
bestCount = inf;
bestRoot = roots(1);
bestMask = false(size(adjacency));
bestReached = false(1, nodeCount);
for rootCursor = 1:numel(roots)
    root = roots(rootCursor);
    required = false(size(adjacency));
    reached = false(1, nodeCount);
    reached(root) = true;
    rootInfluence = zeros(nodeCount, 1);
    rootInfluence(root) = 1;
    for timeIdx = 1:horizon
        currentReached = reached;
        pageRequired = false(nodeCount);
        for receiverIdx = 1:nodeCount
            if currentReached(receiverIdx)
                continue;
            end
            senders = find( ...
                adjacency(receiverIdx, :, timeIdx) & ...
                weights(receiverIdx, :, timeIdx) > 0 & ...
                reliability(receiverIdx, :, timeIdx) > 0 & ...
                reshape(currentReached, 1, []));
            if isempty(senders)
                continue;
            end
            value = weights(receiverIdx, senders, timeIdx) .* ...
                reliability(receiverIdx, senders, timeIdx) .* ...
                reshape(rootInfluence(senders), 1, []);
            maximum = max(value);
            sender = senders(find(value == maximum, 1, 'first'));
            pageRequired(receiverIdx, sender) = true;
        end
        required(:, :, timeIdx) = pageRequired;
        lowerPage = diag(diag(weights(:, :, timeIdx)));
        pageWeights = weights(:, :, timeIdx);
        lowerPage(pageRequired) = pageWeights(pageRequired);
        rootInfluence = lowerPage * rootInfluence;
        reached = reached | reshape(rootInfluence > 0, 1, []);
        if all(reached)
            break;
        end
    end
    evaluated = evaluateRequiredSchedule( ...
        required, weights, reliability);
    gain = evaluated.expectedContractionGainLowerBound;
    count = nnz(evaluated.selectedRequiredMask);
    if gain > bestGain || ...
            (gain == bestGain && count < bestCount) || ...
            (gain == bestGain && count == bestCount && root < bestRoot)
        bestGain = gain;
        bestCount = count;
        bestRoot = root;
        bestMask = evaluated.selectedRequiredMask;
        bestReached = reached;
    end
end
end

function schedule = evaluateRequiredSchedule(mask, weights, reliability)
nodeCount = size(weights, 1);
horizon = size(weights, 3);
productLowerBound = eye(nodeCount);
eventLogProbability = 0;
bestGain = 0;
bestPrefixLength = horizon;
bestProbability = 1;
bestLogProbability = 0;
bestProduct = zeros(nodeCount);
bestOverlap = 0;
for timeIdx = 1:horizon
    pageMask = mask(:, :, timeIdx);
    lowerPage = diag(diag(weights(:, :, timeIdx)));
    pageWeights = weights(:, :, timeIdx);
    lowerPage(pageMask) = pageWeights(pageMask);
    productLowerBound = lowerPage * productLowerBound;
    requiredReliability = reliability(:, :, timeIdx);
    requiredReliability = requiredReliability(pageMask);
    if any(requiredReliability <= 0)
        eventLogProbability = -inf;
    elseif isfinite(eventLogProbability)
        eventLogProbability = eventLogProbability + ...
            sum(log(requiredReliability));
    end
    eventProbability = exp(eventLogProbability);
    overlap = minimumRowOverlap(productLowerBound);
    gain = eventProbability * overlap;
    if gain > bestGain
        bestGain = gain;
        bestPrefixLength = timeIdx;
        bestProbability = eventProbability;
        bestLogProbability = eventLogProbability;
        bestProduct = productLowerBound;
        bestOverlap = overlap;
    end
end
if bestGain == 0
    bestPrefixLength = 0;
    bestProbability = 1;
    bestLogProbability = 0;
    bestProduct = eye(nodeCount);
    bestOverlap = 0;
end
selectedMask = false(size(mask));
if bestPrefixLength > 0
    selectedMask(:, :, 1:bestPrefixLength) = ...
        mask(:, :, 1:bestPrefixLength);
end
schedule = struct( ...
    'bestPrefixLength', bestPrefixLength, ...
    'selectedRequiredMask', selectedMask, ...
    'eventProbability', bestProbability, ...
    'eventLogProbability', bestLogProbability, ...
    'productLowerBound', bestProduct, ...
    'eventOverlapLowerBound', bestOverlap, ...
    'expectedContractionGainLowerBound', bestGain);
end

function overlap = minimumRowOverlap(matrix)
nodeCount = size(matrix, 1);
overlap = inf;
for leftIdx = 1:nodeCount
    % For a realized row-stochastic product, a row compared with itself
    % has overlap one and can never define the Dobrushin minimum when
    % N >= 2.  The lower-bound product is only substochastic; including
    % its diagonal row pair would replace that exact value one by the
    % potentially much smaller lower-bound row sum and introduce a purely
    % artificial bottleneck.
    for rightIdx = (leftIdx + 1):nodeCount
        overlap = min(overlap, sum(min( ...
            matrix(leftIdx, :), matrix(rightIdx, :))));
    end
end
if isempty(overlap) || ~isfinite(overlap)
    overlap = 0;
end
overlap = min(max(overlap, 0), 1);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
