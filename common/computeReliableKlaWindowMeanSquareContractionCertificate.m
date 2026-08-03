function certificate = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacencySequence, fusionWeightSequence, ...
        receiverLinkReliabilitySequence, options)
% COMPUTERELIABLEKLAWINDOWMEANSQUARECONTRACTIONCERTIFICATE
% Exact expected centered-L2 propagation for a fixed causal route window.
%
% Let Pi = I - 11'/N and P = M_H ... M_1.  Under independent delivery
% variables across receiver-sender-time tuples, this routine computes
%
%   Q = E[P' Pi P]
%
% exactly from per-receiver row moments, without enumerating joint network
% delivery outcomes.  For every deterministic node value vector x,
%
%   E[||Pi P x||_2^2] <= rho ||Pi x||_2^2,
%
% where rho is the largest eigenvalue of Pi Q Pi on the disagreement
% subspace.  This is a propagation certificate, not a bound on local Bayes
% update, spatial-overlap, label-support or approximation disturbances.

if nargin < 4 || isempty(options)
    options = struct();
end
allowedFields = { ...
    'missingNeighborWeightMode', 'maximumIncomingCount'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('KlaMeanSquareContraction:InvalidOptions', ...
        'The mean-square certificate options are malformed.');
end
missingMode = lower(char(getField( ...
    options, 'missingNeighborWeightMode', 'renormalize')));
maximumIncomingCount = getField(options, 'maximumIncomingCount', 4);
if ~ismember(missingMode, {'renormalize', 'self'}) || ...
        ~isscalar(maximumIncomingCount) || ...
        ~isfinite(maximumIncomingCount) || ...
        maximumIncomingCount < 1 || ...
        maximumIncomingCount ~= round(maximumIncomingCount)
    error('KlaMeanSquareContraction:InvalidOptions', ...
        'The mean-square certificate options are invalid.');
end

[adjacency, weights, reliability, nodeCount, horizon] = ...
    normalizeSequenceInputs(adjacencySequence, ...
        fusionWeightSequence, receiverLinkReliabilitySequence);
centering = eye(nodeCount) - ones(nodeCount) / nodeCount;
meanSequence = zeros(nodeCount, nodeCount, horizon);
rowSecondMoments = cell(1, horizon);
momentDetails = cell(1, horizon);
allDeliveredProduct = eye(nodeCount);
meanProduct = eye(nodeCount);
for timeIdx = 1:horizon
    [meanPage, secondMoments, pageDetails] = ...
        computeExpectedEffectiveMixingPageMoments( ...
            adjacency(:, :, timeIdx), weights(:, :, timeIdx), ...
            reliability(:, :, timeIdx), missingMode, ...
            maximumIncomingCount);
    meanSequence(:, :, timeIdx) = meanPage;
    rowSecondMoments{timeIdx} = secondMoments;
    momentDetails{timeIdx} = pageDetails;
    allDeliveredProduct = ...
        weights(:, :, timeIdx) * allDeliveredProduct;
    meanProduct = meanPage * meanProduct;
end

% Backward dynamic programming is exact because the current delivery page
% is independent of all later pages already integrated into quadratic.
quadratic = centering;
backwardQuadraticSequence = zeros( ...
    nodeCount, nodeCount, horizon + 1);
backwardQuadraticSequence(:, :, horizon + 1) = quadratic;
for timeIdx = horizon:-1:1
    meanPage = meanSequence(:, :, timeIdx);
    secondMoments = rowSecondMoments{timeIdx};
    previous = meanPage' * quadratic * meanPage;
    for receiverIdx = 1:nodeCount
        meanRow = meanPage(receiverIdx, :);
        rowCovariance = secondMoments(:, :, receiverIdx) - ...
            meanRow' * meanRow;
        previous = previous + ...
            quadratic(receiverIdx, receiverIdx) * rowCovariance;
    end
    quadratic = 0.5 * (previous + previous');
    backwardQuadraticSequence(:, :, timeIdx) = quadratic;
end

centeredQuadratic = centering * quadratic * centering;
centeredQuadratic = 0.5 * ...
    (centeredQuadratic + centeredQuadratic');
eigenvalues = eig(centeredQuadratic);
if min(eigenvalues) < -1e-9
    error('KlaMeanSquareContraction:InternalMomentError', ...
        'The expected centered quadratic form is not semidefinite.');
end
rho = max(max(real(eigenvalues)), 0);
rmsFactor = sqrt(rho);

suffixSquaredFactors = zeros(1, horizon + 1);
for suffixIdx = 1:(horizon + 1)
    suffixQuadratic = centering * ...
        backwardQuadraticSequence(:, :, suffixIdx) * centering;
    suffixQuadratic = 0.5 * ...
        (suffixQuadratic + suffixQuadratic');
    suffixSquaredFactors(suffixIdx) = max( ...
        max(real(eig(suffixQuadratic))), 0);
end
if abs(suffixSquaredFactors(1) - rho) > 1e-10 || ...
        abs(suffixSquaredFactors(end) - 1) > 1e-10
    error('KlaMeanSquareContraction:InternalMomentError', ...
        'The suffix mean-square factors are internally inconsistent.');
end

allDeliveredCentered = ...
    centering * allDeliveredProduct * centering;
allDeliveredSquaredFactor = norm(allDeliveredCentered, 2)^2;
meanCentered = centering * meanProduct * centering;
meanProductSquaredFactor = norm(meanCentered, 2)^2;

annihilationError = max([ ...
    norm(quadratic * ones(nodeCount, 1), inf), ...
    norm(ones(1, nodeCount) * quadratic, inf)]);
if annihilationError > 1e-8
    error('KlaMeanSquareContraction:InternalMomentError', ...
        'The expected centered quadratic form retained consensus mass.');
end

certificate = struct();
certificate.contractVersion = ...
    'reliable-kla-window-mean-square-contraction-certificate-v1';
certificate.nodeCount = nodeCount;
certificate.horizon = horizon;
certificate.missingNeighborWeightMode = missingMode;
certificate.maximumIncomingCount = maximumIncomingCount;
certificate.centeredEnergyDefinition = ...
    'squared-l2-distance-to-nodewise-consensus-subspace';
certificate.pairwiseIdentity = ...
    'centered-energy-equals-one-over-n-sum-i-less-j-squared-difference';
certificate.expectedCenteredQuadraticForm = quadratic;
certificate.centeredExpectedQuadraticForm = centeredQuadratic;
certificate.backwardQuadraticSequence = ...
    backwardQuadraticSequence;
certificate.worstCaseExpectedSquaredContractionFactor = rho;
certificate.worstCaseExpectedRmsContractionFactor = rmsFactor;
certificate.suffixWorstCaseExpectedSquaredContractionFactors = ...
    suffixSquaredFactors;
certificate.suffixWorstCaseExpectedRmsContractionFactors = ...
    sqrt(suffixSquaredFactors);
certificate.preMixingDisturbanceRmsPropagationCoefficients = ...
    sqrt(suffixSquaredFactors(1:horizon));
certificate.postMixingDisturbanceRmsPropagationCoefficients = ...
    sqrt(suffixSquaredFactors(2:horizon + 1));
certificate.additiveDisturbanceBoundForm = [ ...
    'minkowski-rms-sum-conditioned-on-fixed-route-and-', ...
    'future-link-independence'];
certificate.strictMeanSquareContractionCertified = rho < 1 - 1e-12;
certificate.nonexpansiveMeanSquarePropagationCertified = rho <= 1 + 1e-12;
certificate.meanEffectiveMixingSequence = meanSequence;
certificate.meanEffectiveMixingProduct = meanProduct;
certificate.meanProductSquaredContractionFactor = ...
    meanProductSquaredFactor;
certificate.meanProductFactorIsExactExpectedEnergyFactor = false;
certificate.allDeliveredMixingProduct = allDeliveredProduct;
certificate.allDeliveredSquaredContractionFactor = ...
    allDeliveredSquaredFactor;
certificate.pageMomentDetails = momentDetails;
certificate.receiverSenderTimeDeliveriesIndependentAssumed = true;
certificate.fixedRouteSequenceConditionedOnCurrentInformation = true;
certificate.futureDeliveriesIndependentOfCurrentStateAndDisturbanceAssumed = ...
    true;
certificate.exactExpectedQuadraticFormUnderIndependence = true;
certificate.posteriorUsed = false;
certificate.truthUsed = false;
certificate.futureOutcomeUsed = false;
certificate.realizedDeliveryUniformsUsed = false;
certificate.exactKlaSetDensityLogRatioBoundUnderCommonPositiveSupport = ...
    true;
certificate.marginalBernoulliExistenceLogOddsBound = false;
certificate.spatialOverlapNormalizerDisturbanceAccounted = false;
certificate.localBayesUpdateDisturbanceAccounted = false;
certificate.labelSupportLossAccounted = false;
certificate.mixtureApproximationDisturbanceAccounted = false;
certificate.pruningDisturbanceAccounted = false;
end

function [adjacency, weights, reliability, nodeCount, horizon] = ...
        normalizeSequenceInputs(adjacencyInput, weightInput, ...
            reliabilityInput)
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
    error('KlaMeanSquareContraction:InvalidSequence', ...
        'A mean-square certificate input sequence is malformed.');
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
        size(weights, 1) ~= nodeCount || ...
        size(weights, 2) ~= nodeCount || ...
        size(weights, 3) ~= horizon || ...
        size(reliability, 1) ~= nodeCount || ...
        size(reliability, 2) ~= nodeCount || ...
        size(reliability, 3) ~= horizon
    error('KlaMeanSquareContraction:InvalidSequence', ...
        'Mean-square certificate sequence dimensions are invalid.');
end
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
