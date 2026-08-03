function test_reliable_kla_window_contraction_certificate()
% TEST_RELIABLEKLAWINDOWCONTRACTIONCERTIFICATE Exact small-network audit.

protocol = getReliableKlaWindowContractionProtocol();
assert(strcmp(protocol.contractVersion, ...
    'reliable-kla-window-contraction-protocol-v1'));
assert(strcmp(protocol.certificateContractVersion, ...
    'reliable-kla-window-contraction-certificate-v1'));
assert(strcmp(protocol.meanSquareCertificateContractVersion, ...
    'reliable-kla-window-mean-square-contraction-certificate-v1'));
assert(strcmp(protocol.runtimeSemanticsContractVersion, ...
    'reliable-kla-linear-mixing-runtime-semantics-v1'));
assert(protocol.adaptiveReferenceSquaredContractionTarget == 0.90);
assert(protocol.adaptiveMaximumHorizonMultiplier == 4);
assert(~protocol.graphCertificateIsStandaloneTrackingSafetyTest);
assert(~protocol.graphCertificateIsStandaloneSelectionObjective);
assert(~protocol.marginalExistenceLogOddsClaimAllowed);
assert(protocol.spatialOverlapNormalizerRequiresDisturbanceTerm);
assert(~protocol.m24TrackingAuthorized && ...
    ~protocol.x36TrackingAuthorized && ...
    ~protocol.validationClaimAllowed);

[identityDelta, identityDetails] = ...
    computeDobrushinErgodicityCoefficient(eye(3));
assert(identityDelta == 1);
assert(identityDetails.minimumRowOverlap == 0);
[averageDelta, averageDetails] = ...
    computeDobrushinErgodicityCoefficient(ones(3) / 3);
assert(abs(averageDelta) < 1e-12);
assert(abs(averageDetails.minimumRowOverlap - 1) < 1e-12);
[halfDelta, ~] = computeDobrushinErgodicityCoefficient( ...
    [0.75, 0.25; 0.25, 0.75]);
assert(abs(halfDelta - 0.5) < 1e-12);

[adjacency, weights, reliability] = buildTemporalChain();
certificate = computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, reliability);
assert(strcmp(certificate.contractVersion, ...
    'reliable-kla-window-contraction-certificate-v1'));
assert(certificate.rootIndex == 1);
assert(certificate.allNodesTemporallyReached);
assert(certificate.certificatePrefixLength == 2);
assert(certificate.requiredDeliveryCount == 2);
assert(abs(certificate.requiredDeliveryEventProbability - 0.56) < 1e-12);
assert(abs(certificate.eventOverlapLowerBound - 0.25) < 1e-12);
assert(abs(certificate.eventDobrushinUpperBound - 0.75) < 1e-12);
assert(abs(certificate.expectedContractionGainLowerBound - 0.14) < 1e-12);
assert(abs(certificate.expectedDobrushinUpperBound - 0.86) < 1e-12);
assert(~certificate.meanMixingDobrushinIsExpectedRiskUpperBound);
assert(certificate.linearConsensusDiameterBound);
assert(certificate.exactKlaLogRatioBoundUnderCommonPositiveSupport);
assert(certificate. ...
    bernoulliSetDensityLogRatioBoundUnderCommonPositiveSupport);
assert(~certificate.marginalBernoulliExistenceLogOddsBound);
assert(~certificate.spatialOverlapNormalizerDisturbanceAccounted);
assert(~certificate.localBayesUpdateDisturbanceAccounted);
assert(~certificate.mixtureApproximationDisturbanceAccounted);
assert(~certificate.posteriorUsed && ~certificate.truthUsed && ...
    ~certificate.futureOutcomeUsed);

expectedDelta = enumerateExpectedProductDobrushin( ...
    adjacency, weights, reliability, 'renormalize');
assert(expectedDelta <= certificate.expectedDobrushinUpperBound + 1e-12);
requiredProduct = realizedProduct( ...
    adjacency, weights, true(size(adjacency)), 'renormalize');
[requiredDelta, ~] = ...
    computeDobrushinErgodicityCoefficient(requiredProduct);
assert(requiredDelta <= certificate.eventDobrushinUpperBound + 1e-12);
x = [-2; 1; 5];
assert(vectorDiameter(requiredProduct * x) <= ...
    requiredDelta * vectorDiameter(x) + 1e-12);

% A later arbitrary row-stochastic page cannot invalidate the certified
% prefix.  Identity is used here so the exact diagnostic also stays fixed.
adjacency3 = cat(3, adjacency, false(3));
weights3 = cat(3, weights, eye(3));
reliability3 = cat(3, reliability, zeros(3));
certificate3 = computeReliableKlaWindowContractionCertificate( ...
    adjacency3, weights3, reliability3);
assert(certificate3.certificatePrefixLength == 2);
assert(abs(certificate3.expectedDobrushinUpperBound - ...
    certificate.expectedDobrushinUpperBound) < 1e-12);

requiredMask = false(size(adjacency));
requiredMask(2, 1, 1) = true;
requiredMask(3, 2, 2) = true;
supplied = computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, reliability, struct( ...
        'requiredDeliveryMask', requiredMask));
assert(isnan(supplied.rootIndex));
assert(isequal(supplied.requiredDeliveryMask, requiredMask));
assert(abs(supplied.expectedDobrushinUpperBound - 0.86) < 1e-12);

selfMode = computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, reliability, struct( ...
        'missingNeighborWeightMode', 'self'));
selfExpectedDelta = enumerateExpectedProductDobrushin( ...
    adjacency, weights, reliability, 'self');
assert(selfExpectedDelta <= selfMode.expectedDobrushinUpperBound + 1e-12);

% Same-row sums of a substochastic lower product are not Dobrushin row
% pairs.  Requiring only the two useful cross edges leaves row 2 with
% omitted mass, but the distinct-row overlap remains informative.
multiAdjacency = false(3);
multiWeights = eye(3);
multiReliability = zeros(3);
multiAdjacency(1, 2) = true;
multiAdjacency(2, 1) = true;
multiAdjacency(2, 3) = true;
multiAdjacency(3, 2) = true;
multiWeights(1, :) = [0.5, 0.5, 0];
multiWeights(2, :) = [0.2, 0.6, 0.2];
multiWeights(3, :) = [0, 0.5, 0.5];
multiReliability(multiAdjacency) = 1;
multiRequired = false(3);
multiRequired(1, 2) = true;
multiRequired(3, 2) = true;
multi = computeReliableKlaWindowContractionCertificate( ...
    multiAdjacency, multiWeights, multiReliability, struct( ...
        'requiredDeliveryMask', multiRequired));
assert(abs(multi.eventOverlapLowerBound - 0.5) < 1e-12);
assert(abs(multi.expectedDobrushinUpperBound - 0.5) < 1e-12);

disconnected = computeReliableKlaWindowContractionCertificate( ...
    false(3), eye(3), zeros(3));
assert(~disconnected.allNodesTemporallyReached);
assert(disconnected.certificatePrefixLength == 0);
assert(disconnected.requiredDeliveryCount == 0);
assert(disconnected.eventOverlapLowerBound == 0);
assert(disconnected.expectedDobrushinUpperBound == 1);

% The automatic schedule is intentionally only a greedy diagnostic.  A
% weak direct shortcut can make every node immediately reachable while a
% high-quality multi-hop temporal schedule gives a much stronger rigorous
% event bound.
[shortcutAdjacency, shortcutWeights, shortcutReliability, ...
    highQualitySchedule] = buildWeakShortcutCounterexample();
shortcutGreedy = computeReliableKlaWindowContractionCertificate( ...
    shortcutAdjacency, shortcutWeights, shortcutReliability);
shortcutCaller = computeReliableKlaWindowContractionCertificate( ...
    shortcutAdjacency, shortcutWeights, shortcutReliability, struct( ...
        'requiredDeliveryMask', highQualitySchedule));
assert(abs(shortcutGreedy.expectedContractionGainLowerBound - ...
    9.9e-8) < 1e-14);
assert(abs(shortcutCaller.expectedContractionGainLowerBound - ...
    0.026198073) < 1e-12);
assert(shortcutCaller.expectedContractionGainLowerBound / ...
    shortcutGreedy.expectedContractionGainLowerBound > 2.6e5);

assertRandomExactBounds();
assertErrorId(@() computeDobrushinErgodicityCoefficient( ...
    [0.6, 0.6; 0.4, 0.6]), ...
    'KlaWindowContraction:InvalidStochasticMatrix');
badReliability = reliability;
badReliability(2, 1, 1) = 1.1;
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, badReliability), ...
    'KlaWindowContraction:InvalidSequence');
badMask = requiredMask;
badMask(1, 3, 1) = true;
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, reliability, struct( ...
        'requiredDeliveryMask', badMask)), ...
    'KlaWindowContraction:InvalidRequiredSchedule');
bad4dAdjacency = repmat(adjacency, 1, 1, 1, 2);
bad4dWeights = repmat(weights, 1, 1, 1, 2);
bad4dReliability = repmat(reliability, 1, 1, 1, 2);
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    bad4dAdjacency, bad4dWeights, bad4dReliability), ...
    'KlaWindowContraction:InvalidSequence');
nonbinaryAdjacency = double(adjacency);
nonbinaryAdjacency(2, 1, 1) = 0.5;
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    nonbinaryAdjacency, weights, reliability), ...
    'KlaWindowContraction:InvalidSequence');
negativeWeights = weights;
negativeWeights(1, 2, 1) = -1e-14;
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    adjacency, negativeWeights, reliability), ...
    'KlaWindowContraction:InvalidSequence');
nonbinaryMask = double(requiredMask);
nonbinaryMask(2, 1, 1) = 0.5;
assertErrorId(@() computeReliableKlaWindowContractionCertificate( ...
    adjacency, weights, reliability, struct( ...
        'requiredDeliveryMask', nonbinaryMask)), ...
    'KlaWindowContraction:InvalidRequiredSchedule');

fprintf('PASS: reliable KLA window-contraction certificate tests\n');
end

function [adjacency, weights, reliability, required] = ...
        buildWeakShortcutCounterexample()
nodeCount = 4;
horizon = 3;
pageAdjacency = logical(ones(nodeCount) - eye(nodeCount));
pageWeights = 0.001 * double(pageAdjacency);
pageReliability = 0.01 * double(pageAdjacency);
for receiver = 1:nodeCount
    ringSender = mod(receiver - 2, nodeCount) + 1;
    pageWeights(receiver, ringSender) = 0.30;
    pageReliability(receiver, ringSender) = 0.99;
    pageWeights(receiver, receiver) = 0.698;
end
adjacency = repmat(pageAdjacency, 1, 1, horizon);
weights = repmat(pageWeights, 1, 1, horizon);
reliability = repmat(pageReliability, 1, 1, horizon);
required = false(nodeCount, nodeCount, horizon);
required(2, 1, 1) = true;
required(3, 2, 2) = true;
required(4, 3, 3) = true;
end

function [adjacency, weights, reliability] = buildTemporalChain()
adjacency = false(3, 3, 2);
weights = repmat(eye(3), 1, 1, 2);
reliability = zeros(3, 3, 2);
adjacency(2, 1, 1) = true;
weights(2, 2, 1) = 0.5;
weights(2, 1, 1) = 0.5;
reliability(2, 1, 1) = 0.8;
adjacency(3, 2, 2) = true;
weights(3, 3, 2) = 0.5;
weights(3, 2, 2) = 0.5;
reliability(3, 2, 2) = 0.7;
end

function assertRandomExactBounds()
rng(4117, 'twister');
for trialIdx = 1:24
    nodeCount = 3;
    horizon = 2;
    adjacency = false(nodeCount, nodeCount, horizon);
    weights = zeros(nodeCount, nodeCount, horizon);
    reliability = zeros(nodeCount, nodeCount, horizon);
    for timeIdx = 1:horizon
        for receiverIdx = 1:nodeCount
            weights(receiverIdx, receiverIdx, timeIdx) = 1;
            availableSenders = setdiff(1:nodeCount, receiverIdx);
            senderCount = randi([0, numel(availableSenders)]);
            if senderCount > 0
                order = randperm(numel(availableSenders), senderCount);
                senders = availableSenders(order);
                sourceMass = 0.15 + 0.70 * rand();
                shares = rand(1, senderCount);
                shares = sourceMass * shares / sum(shares);
                adjacency(receiverIdx, senders, timeIdx) = true;
                weights(receiverIdx, receiverIdx, timeIdx) = ...
                    1 - sourceMass;
                weights(receiverIdx, senders, timeIdx) = shares;
                reliability(receiverIdx, senders, timeIdx) = ...
                    0.2 + 0.75 * rand(1, senderCount);
            end
        end
    end
    for modeCell = {'renormalize', 'self'}
        mode = modeCell{1};
        certificate = computeReliableKlaWindowContractionCertificate( ...
            adjacency, weights, reliability, struct( ...
                'missingNeighborWeightMode', mode));
        exactExpected = enumerateExpectedProductDobrushin( ...
            adjacency, weights, reliability, mode);
        assert(exactExpected <= ...
            certificate.expectedDobrushinUpperBound + 2e-12);
        assert(certificate.expectedDobrushinUpperBound >= -1e-12 && ...
            certificate.expectedDobrushinUpperBound <= 1 + 1e-12);
    end
end
end

function expected = enumerateExpectedProductDobrushin( ...
        adjacency, weights, reliability, missingMode)
[receiverIndex, senderIndex, timeIndex] = ind2sub( ...
    size(adjacency), find(adjacency));
edgeCount = numel(receiverIndex);
expected = 0;
probabilitySum = 0;
for outcomeIdx = 0:(2^edgeCount - 1)
    delivered = logical(bitget(uint64(outcomeIdx), 1:edgeCount));
    deliveryMask = false(size(adjacency));
    probability = 1;
    for edgeIdx = 1:edgeCount
        receiver = receiverIndex(edgeIdx);
        sender = senderIndex(edgeIdx);
        time = timeIndex(edgeIdx);
        deliveryMask(receiver, sender, time) = delivered(edgeIdx);
        edgeProbability = reliability(receiver, sender, time);
        if delivered(edgeIdx)
            probability = probability * edgeProbability;
        else
            probability = probability * (1 - edgeProbability);
        end
    end
    product = realizedProduct( ...
        adjacency, weights, deliveryMask, missingMode);
    [delta, ~] = computeDobrushinErgodicityCoefficient(product);
    expected = expected + probability * delta;
    probabilitySum = probabilitySum + probability;
end
assert(abs(probabilitySum - 1) < 1e-12);
end

function product = realizedProduct( ...
        adjacency, weights, deliveryMask, missingMode)
nodeCount = size(adjacency, 1);
horizon = size(adjacency, 3);
product = eye(nodeCount);
for timeIdx = 1:horizon
    page = zeros(nodeCount);
    for receiverIdx = 1:nodeCount
        senders = reshape(find(adjacency(receiverIdx, :, timeIdx)), 1, []);
        delivered = deliveryMask(receiverIdx, senders, timeIdx);
        deliveredSenders = senders(delivered);
        row = zeros(1, nodeCount);
        row(receiverIdx) = weights(receiverIdx, receiverIdx, timeIdx);
        row(deliveredSenders) = ...
            weights(receiverIdx, deliveredSenders, timeIdx);
        if strcmp(missingMode, 'self')
            row(receiverIdx) = row(receiverIdx) + sum( ...
                weights(receiverIdx, senders(~delivered), timeIdx));
        end
        row = row / sum(row);
        page(receiverIdx, :) = row;
    end
    product = page * product;
end
end

function value = vectorDiameter(vector)
value = max(vector) - min(vector);
end

function assertErrorId(callback, expectedIdentifier)
failed = false;
try
    callback();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expectedIdentifier);
end
assert(failed);
end
