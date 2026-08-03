function test_reliable_kla_window_mean_square_contraction_certificate()
% TEST_RELIABLEKLAWINDOWMEANSQUARECONTRACTIONCERTIFICATE Exact audit.

[adjacency, weights, reliability] = buildTwoPageNetwork();
for modeCell = {'renormalize', 'self'}
    mode = modeCell{1};
    certificate = ...
        computeReliableKlaWindowMeanSquareContractionCertificate( ...
            adjacency, weights, reliability, struct( ...
                'missingNeighborWeightMode', mode));
    assert(strcmp(certificate.contractVersion, ...
        'reliable-kla-window-mean-square-contraction-certificate-v1'));
    exactQuadratic = enumerateExpectedCenteredQuadratic( ...
        adjacency, weights, reliability, mode);
    assert(norm(certificate.expectedCenteredQuadraticForm - ...
        exactQuadratic, 'fro') < 2e-12);
    centering = eye(3) - ones(3) / 3;
    exactEigenvalues = eig(centering * exactQuadratic * centering);
    exactFactor = max(max(real(exactEigenvalues)), 0);
    assert(abs(certificate. ...
        worstCaseExpectedSquaredContractionFactor - ...
        exactFactor) < 2e-12);
    assert(abs(certificate. ...
        worstCaseExpectedRmsContractionFactor^2 - ...
        exactFactor) < 2e-12);
    assert(abs(certificate. ...
        suffixWorstCaseExpectedSquaredContractionFactors(1) - ...
        exactFactor) < 2e-12);
    assert(abs(certificate. ...
        suffixWorstCaseExpectedSquaredContractionFactors(end) - 1) ...
        < 2e-12);
    assert(numel(certificate. ...
        preMixingDisturbanceRmsPropagationCoefficients) == 2);
    assert(numel(certificate. ...
        postMixingDisturbanceRmsPropagationCoefficients) == 2);
    assert(abs(certificate. ...
        postMixingDisturbanceRmsPropagationCoefficients(end) - 1) ...
        < 2e-12);
    assert(certificate.meanProductSquaredContractionFactor <= ...
        exactFactor + 2e-12);
    assert(certificate.exactExpectedQuadraticFormUnderIndependence);
    assert(certificate. ...
        exactKlaSetDensityLogRatioBoundUnderCommonPositiveSupport);
    assert(~certificate.marginalBernoulliExistenceLogOddsBound);
    assert(~certificate.localBayesUpdateDisturbanceAccounted);
    assert(~certificate.posteriorUsed && ~certificate.truthUsed && ...
        ~certificate.futureOutcomeUsed);
    rng(902, 'twister');
    for vectorIdx = 1:20
        value = randn(3, 1);
        initialEnergy = value' * centering * value;
        expectedEnergy = value' * exactQuadratic * value;
        assert(expectedEnergy <= exactFactor * initialEnergy + 2e-12);
    end
end

deterministicReliability = double(adjacency);
deterministic = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, weights, deterministicReliability);
product = weights(:, :, 2) * weights(:, :, 1);
centering = eye(3) - ones(3) / 3;
expectedDeterministicFactor = ...
    norm(centering * product * centering, 2)^2;
assert(abs(deterministic. ...
    worstCaseExpectedSquaredContractionFactor - ...
    expectedDeterministicFactor) < 2e-12);
assert(abs(deterministic.allDeliveredSquaredContractionFactor - ...
    expectedDeterministicFactor) < 2e-12);

identity = computeReliableKlaWindowMeanSquareContractionCertificate( ...
    false(3), eye(3), zeros(3));
assert(abs(identity.worstCaseExpectedSquaredContractionFactor - 1) ...
    < 2e-12);
assert(~identity.strictMeanSquareContractionCertified);
assert(identity.nonexpansiveMeanSquarePropagationCertified);

staticAdjacency = adjacency(:, :, 1);
staticWeights = weights(:, :, 1);
staticReliability = reliability(:, :, 1);
profile = computeStaticReliableKlaMeanSquareHorizonProfile( ...
    staticAdjacency, staticWeights, staticReliability, 8, struct( ...
        'missingNeighborWeightMode', 'renormalize'));
assert(strcmp(profile.contractVersion, ...
    'static-reliable-kla-mean-square-horizon-profile-v1'));
assert(isequal(profile.horizons, 0:8));
assert(abs(profile. ...
    worstCaseExpectedSquaredFactorByHorizon(1) - 1) < 2e-12);
for horizon = 1:8
    direct = computeReliableKlaWindowMeanSquareContractionCertificate( ...
        repmat(staticAdjacency, 1, 1, horizon), ...
        repmat(staticWeights, 1, 1, horizon), ...
        repmat(staticReliability, 1, 1, horizon));
    assert(abs(profile. ...
        worstCaseExpectedSquaredFactorByHorizon(horizon + 1) - ...
        direct.worstCaseExpectedSquaredContractionFactor) < 3e-12);
end

assertFiniteHypothesisKlaLogRatioIdentity();
assertBernoulliExistenceNormalizerCounterexample();
assertCorrelatedDeliveryCounterexample();
assertRandomExactQuadratics();
assertErrorId(@() ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, weights, reliability, struct( ...
            'maximumIncomingCount', 1)), ...
    'KlaMixingMoments:InvalidPage');
badReliability = reliability;
badReliability(1, 2, 1) = 1.1;
assertErrorId(@() ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, weights, badReliability), ...
    'KlaMixingMoments:InvalidPage');
bad4dAdjacency = repmat(adjacency, 1, 1, 1, 2);
bad4dWeights = repmat(weights, 1, 1, 1, 2);
bad4dReliability = repmat(reliability, 1, 1, 1, 2);
assertErrorId(@() ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        bad4dAdjacency, bad4dWeights, bad4dReliability), ...
    'KlaMeanSquareContraction:InvalidSequence');
nonbinaryAdjacency = double(adjacency);
nonbinaryAdjacency(1, 2, 1) = 0.5;
assertErrorId(@() ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        nonbinaryAdjacency, weights, reliability), ...
    'KlaMeanSquareContraction:InvalidSequence');
negativeWeights = weights;
negativeWeights(1, 2, 1) = -1e-14;
assertErrorId(@() ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, negativeWeights, reliability), ...
    'KlaMixingMoments:InvalidPage');

fprintf('PASS: reliable KLA window mean-square certificate tests\n');
end

function assertFiniteHypothesisKlaLogRatioIdentity()
nodeDensities = [ ...
    0.60, 0.25, 0.15; ...
    0.20, 0.30, 0.50];
fusionWeights = [0.35; 0.65];
logUnnormalized = fusionWeights' * log(nodeDensities);
fusedDensity = exp(logUnnormalized);
fusedDensity = fusedDensity / sum(fusedDensity);
left = log(fusedDensity(1)) - log(fusedDensity(3));
right = fusionWeights' * ( ...
    log(nodeDensities(:, 1)) - log(nodeDensities(:, 3)));
assert(abs(left - right) < 2e-12);
end

function assertBernoulliExistenceNormalizerCounterexample()
existence = [0.70; 0.40];
fusionWeights = [0.35; 0.65];
spatialDensity = [0.90, 0.10; 0.15, 0.85];
logSpatialProduct = fusionWeights' * log(spatialDensity);
eta = sum(exp(logSpatialProduct));
absentMass = exp(fusionWeights' * log(1 - existence));
presentStateMass = exp(fusionWeights' * ...
    bsxfun(@plus, log(existence), log(spatialDensity)));
presentMass = sum(presentStateMass);
fusedExistence = presentMass / (absentMass + presentMass);
fusedLogOdds = log(fusedExistence / (1 - fusedExistence));
weightedInputLogOdds = fusionWeights' * ...
    log(existence ./ (1 - existence));
assert(abs(log(eta)) > 1e-3);
assert(abs(fusedLogOdds - ...
    (weightedInputLogOdds + log(eta))) < 2e-12);
assert(abs(fusedLogOdds - weightedInputLogOdds) > 1e-3);
end

function assertCorrelatedDeliveryCounterexample()
adjacency = logical([0, 1; 1, 0]);
weights = 0.5 * ones(2);
reliability = 0.5 * double(adjacency);
independent = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, weights, reliability);
centering = eye(2) - ones(2) / 2;
noneDelivered = eye(2);
bothDelivered = 0.5 * ones(2);
correlatedQuadratic = 0.5 * ( ...
    noneDelivered' * centering * noneDelivered + ...
    bothDelivered' * centering * bothDelivered);
correlatedFactor = max(real(eig( ...
    centering * correlatedQuadratic * centering)));
assert(abs(independent. ...
    worstCaseExpectedSquaredContractionFactor - 0.375) < 2e-12);
assert(abs(correlatedFactor - 0.5) < 2e-12);
assert(correlatedFactor > independent. ...
    worstCaseExpectedSquaredContractionFactor + 0.1);
end

function [adjacency, weights, reliability] = buildTwoPageNetwork()
adjacency = false(3, 3, 2);
weights = zeros(3, 3, 2);
reliability = zeros(3, 3, 2);

adjacency(:, :, 1) = logical([ ...
    0, 1, 1; ...
    1, 0, 0; ...
    0, 1, 0]);
weights(:, :, 1) = [ ...
    0.35, 0.40, 0.25; ...
    0.55, 0.45, 0; ...
    0, 0.30, 0.70];
reliability(:, :, 1) = [ ...
    0, 0.75, 0.45; ...
    0.65, 0, 0; ...
    0, 0.80, 0];

adjacency(:, :, 2) = logical([ ...
    0, 0, 1; ...
    1, 0, 1; ...
    1, 0, 0]);
weights(:, :, 2) = [ ...
    0.60, 0, 0.40; ...
    0.25, 0.50, 0.25; ...
    0.35, 0, 0.65];
reliability(:, :, 2) = [ ...
    0, 0, 0.70; ...
    0.55, 0, 0.85; ...
    0.60, 0, 0];
end

function assertRandomExactQuadratics()
rng(4207, 'twister');
for trialIdx = 1:12
    nodeCount = 3;
    horizon = 2;
    adjacency = false(nodeCount, nodeCount, horizon);
    weights = zeros(nodeCount, nodeCount, horizon);
    reliability = zeros(nodeCount, nodeCount, horizon);
    for timeIdx = 1:horizon
        for receiverIdx = 1:nodeCount
            available = setdiff(1:nodeCount, receiverIdx);
            senderCount = randi([0, numel(available)]);
            senders = available(randperm(numel(available), senderCount));
            sourceMass = 0;
            if senderCount > 0
                sourceMass = 0.1 + 0.75 * rand();
                shares = rand(1, senderCount);
                shares = sourceMass * shares / sum(shares);
                adjacency(receiverIdx, senders, timeIdx) = true;
                weights(receiverIdx, senders, timeIdx) = shares;
                reliability(receiverIdx, senders, timeIdx) = ...
                    0.15 + 0.80 * rand(1, senderCount);
            end
            weights(receiverIdx, receiverIdx, timeIdx) = ...
                1 - sourceMass;
        end
    end
    for modeCell = {'renormalize', 'self'}
        mode = modeCell{1};
        certificate = ...
            computeReliableKlaWindowMeanSquareContractionCertificate( ...
                adjacency, weights, reliability, struct( ...
                    'missingNeighborWeightMode', mode));
        exact = enumerateExpectedCenteredQuadratic( ...
            adjacency, weights, reliability, mode);
        assert(norm(certificate.expectedCenteredQuadraticForm - ...
            exact, 'fro') < 3e-12);
    end
end
end

function expected = enumerateExpectedCenteredQuadratic( ...
        adjacency, weights, reliability, missingMode)
nodeCount = size(adjacency, 1);
centering = eye(nodeCount) - ones(nodeCount) / nodeCount;
[receiverIndex, senderIndex, timeIndex] = ind2sub( ...
    size(adjacency), find(adjacency));
edgeCount = numel(receiverIndex);
expected = zeros(nodeCount);
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
    expected = expected + probability * (product' * centering * product);
    probabilitySum = probabilitySum + probability;
end
assert(abs(probabilitySum - 1) < 2e-12);
expected = 0.5 * (expected + expected');
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

function assertErrorId(callback, identifier)
failed = false;
try
    callback();
catch errorInfo
    failed = strcmp(errorInfo.identifier, identifier);
end
assert(failed);
end
