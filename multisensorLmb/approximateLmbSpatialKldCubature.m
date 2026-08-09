function [spatialKld, details] = approximateLmbSpatialKldCubature( ...
        referenceObject, candidateObject)
% APPROXIMATELMBSPATIALKLDCUBATURE Deterministic GM KLD approximation.
%
% Approximates D_KL(p_reference || p_candidate) by applying a positive-
% weight spherical-radial cubature rule to every Gaussian component of the
% reference mixture. This is a runtime teacher metric, not an exact GM KLD.

reference = validateAndNormalizeObject(referenceObject, 'reference');
candidate = validateAndNormalizeObject(candidateObject, 'candidate');
if reference.dimension ~= candidate.dimension
    error('LmbSpatialKld:DimensionMismatch', ...
        'Reference and candidate state dimensions must match.');
end

dimension = reference.dimension;
componentContributions = zeros(1, reference.componentCount);
pointCount = 2 * dimension;
for componentIdx = 1:reference.componentCount
    covariance = reference.covariances{componentIdx};
    factor = chol(covariance, 'lower');
    points = reference.means{componentIdx} + ...
        sqrt(dimension) * [factor, -factor];
    logReference = logGaussianMixtureDensity(reference, points);
    logCandidate = logGaussianMixtureDensity(candidate, points);
    componentContributions(componentIdx) = ...
        mean(logReference - logCandidate);
end

rawKld = sum(reference.weights .* componentContributions);
spatialKld = max(rawKld, 0);
details = struct();
details.contractVersion = 'lmb-spatial-kld-cubature-v1';
details.referenceToCandidate = true;
details.isExactGaussianMixtureKld = false;
details.integrationRule = ...
    'componentwise-positive-weight-spherical-radial-cubature';
details.stateDimension = dimension;
details.pointsPerReferenceComponent = pointCount;
details.componentContributions = componentContributions;
details.rawKld = rawKld;
details.nonnegativeKld = spatialKld;
details.wasClampedToZero = rawKld < 0;
end

function normalized = validateAndNormalizeObject(object, role)
required = {'numberOfGmComponents', 'w', 'mu', 'Sigma'};
if ~isstruct(object) || ~isscalar(object) || ...
        ~all(isfield(object, required)) || ...
        object.numberOfGmComponents < 1 || ...
        object.numberOfGmComponents ~= round(object.numberOfGmComponents) || ...
        numel(object.w) ~= object.numberOfGmComponents || ...
        numel(object.mu) ~= object.numberOfGmComponents || ...
        numel(object.Sigma) ~= object.numberOfGmComponents
    error('LmbSpatialKld:InvalidObject', ...
        'The %s object is not a valid Gaussian mixture.', role);
end

weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    error('LmbSpatialKld:InvalidWeights', ...
        'The %s mixture has no positive component weight.', role);
end
weights = weights / sum(weights);
dimension = numel(object.mu{1});
if dimension < 1
    error('LmbSpatialKld:InvalidDimension', ...
        'The %s mixture has an empty state.', role);
end
means = cell(1, object.numberOfGmComponents);
covariances = cell(1, object.numberOfGmComponents);
precisions = cell(1, object.numberOfGmComponents);
logNormalizers = zeros(1, object.numberOfGmComponents);
for componentIdx = 1:object.numberOfGmComponents
    mean = reshape(object.mu{componentIdx}, [], 1);
    covariance = object.Sigma{componentIdx};
    if numel(mean) ~= dimension || ...
            ~isequal(size(covariance), [dimension, dimension]) || ...
            any(~isfinite(mean)) || any(~isfinite(covariance(:)))
        error('LmbSpatialKld:InvalidComponent', ...
            'The %s mixture contains an invalid component.', role);
    end
    covariance = regularizeCovariance(covariance);
    factor = chol(covariance, 'lower');
    means{componentIdx} = mean;
    covariances{componentIdx} = covariance;
    precisions{componentIdx} = factor' \ (factor \ eye(dimension));
    logNormalizers(componentIdx) = -0.5 * ( ...
        dimension * log(2 * pi) + 2 * sum(log(diag(factor))));
end

normalized = struct( ...
    'componentCount', object.numberOfGmComponents, ...
    'dimension', dimension, ...
    'weights', weights, ...
    'means', {means}, ...
    'covariances', {covariances}, ...
    'precisions', {precisions}, ...
    'logNormalizers', logNormalizers);
end

function logDensity = logGaussianMixtureDensity(object, points)
componentLogDensity = zeros(object.componentCount, size(points, 2));
for componentIdx = 1:object.componentCount
    residual = points - object.means{componentIdx};
    quadratic = sum(residual .* ...
        (object.precisions{componentIdx} * residual), 1);
    componentLogDensity(componentIdx, :) = ...
        log(max(object.weights(componentIdx), realmin)) + ...
        object.logNormalizers(componentIdx) - 0.5 * quadratic;
end
maximum = max(componentLogDensity, [], 1);
logDensity = maximum + log(sum(exp( ...
    componentLogDensity - maximum), 1));
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:9
    [~, flag] = chol(covariance + jitter * eye(size(covariance)));
    if flag == 0
        covariance = covariance + jitter * eye(size(covariance));
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('LmbSpatialKld:InvalidCovariance', ...
    'A Gaussian component covariance is not positive definite.');
end
