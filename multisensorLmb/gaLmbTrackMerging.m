function objects = gaLmbTrackMerging(measurementUpdatedDistributions, model)
% GALMBTRACKMERGING -- Merge the objects' measurement-updated distributions together using the GA-fusion rule.
%   objects = gaLmbTrackMerging(measurementUpdatedDistributions, model)
%
%   Merge the objects' measurement-updated distributions together using the GA-fusion rule.
%   This makes use of a very crude merging algorithm that is actually
%   reasonably accurate. It might be possible to extend this to Gaussian
%   mixtures, or, using the well-space mixture assumptions, apply
%   expectation propagation.
%   File guide:
%       Geometric-average fusion for local sensor posteriors. It moment
%       matches each sensor's mixture, combines Gaussian canonical
%       parameters with spatial weights, then combines existence
%       probabilities with existence weights.
%
%   See also generateMultisensorModel, loopyBeliefPropagation, lmbGibbsSampling, lmbMurtysAlgorithm
%
%   Inputs
%       measurementUpdatedDistributions - (1, numberOfSensors) cell array.
%           Each is an object struct produced by computePosteriorLmbSpatialDistributions.
%       model - struct. A struct with the fields declared in generateMultisensorModel.
%
%   Output
%       objects - struct. A struct containing the posterior LMB's Bernoulli
%           components.

objects = measurementUpdatedDistributions{1};
for i = 1:numel(objects)
    spatialWeights = resolveObjectWeightVector( ...
        model, 'gaTargetWiseWeights', 'gaSpatialWeights', model.gaSensorWeights, i);
    existenceWeights = resolveObjectWeightVector( ...
        model, 'gaTargetWiseWeights', 'gaExistenceWeights', model.gaSensorWeights, i);
    %% Moment match and determine geometric average
    K = zeros(model.xDimension, model.xDimension);
    h = zeros(model.xDimension, 1);
    g = 0;
    for s = 1:model.numberOfSensors
        [nu, T] = mprojection(model.xDimension, measurementUpdatedDistributions{s}(i));
        % Convert to canonical form and exponentiate
        KMatched = spatialWeights(s) * inv(T);
        hMatched = KMatched * nu;
        gMatched = -0.5 * nu' * KMatched * nu - 0.5 * spatialWeights(s) * log(det(2*pi*T));
        % Throw it on the pile
        K = K + KMatched;
        h = h + hMatched;
        g = g + gMatched;
    end
    %% Convert to covariance form and normalise
    SigmaGa = inv(K);
    muGa = SigmaGa * h;
    eta = exp(g + 0.5 * muGa' * K * muGa + 0.5 * log(det(2*pi*SigmaGa)));
    %% Determine existence probability
    numerator = eta;
    partialDenominator = 1;
    for s = 1:model.numberOfSensors
        rS = measurementUpdatedDistributions{s}(i).r;
        numerator = numerator * (rS^(existenceWeights(s)));
        partialDenominator = partialDenominator *  ((1-rS)^(existenceWeights(s)));
    end
    %% Update Bernoulli component
    objects(i).r = numerator / (numerator + partialDenominator);
    objects(i).numberOfGmComponents = 1;
    objects(i).w = 1;
    objects(i).mu = {muGa};
    objects(i).Sigma = {SigmaGa};
end
end

function weights = resolveObjectWeightVector(model, targetWiseFieldName, fieldName, fallback, objectIdx)
weights = [];
if isfield(model, targetWiseFieldName)
    targetWiseWeights = model.(targetWiseFieldName);
    if size(targetWiseWeights, 1) >= objectIdx && size(targetWiseWeights, 2) == numel(fallback)
        weights = targetWiseWeights(objectIdx, :);
    end
end
if isempty(weights)
    weights = resolveWeightVector(model, fieldName, fallback);
end
weights = normalizeWeightVector(weights, fallback);
end

function weights = resolveWeightVector(model, fieldName, fallback)
if isfield(model, fieldName)
    weights = model.(fieldName);
else
    weights = fallback;
end
end

function weights = normalizeWeightVector(weights, fallback)
weights = reshape(weights, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= numel(fallback) || sum(weights) <= 0
    weights = reshape(fallback, 1, []);
    weights(~isfinite(weights)) = 0;
    weights = max(weights, 0);
end
if sum(weights) <= 0
    weights = ones(1, numel(fallback)) / numel(fallback);
else
    weights = weights / sum(weights);
end
end

%% M-projection
function [nu, T] = mprojection(n, measurementUpdatedDistribution)
% Determine m-projected mean
nu = zeros(n, 1);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    nu = nu + measurementUpdatedDistribution.w(j) * measurementUpdatedDistribution.mu{j};
end
% Determine m-projected covariance
T = zeros(n, n);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    w = measurementUpdatedDistribution.w(j);
    mu = measurementUpdatedDistribution.mu{j} - nu;
    Sigma = measurementUpdatedDistribution.Sigma{j};
    T = T + w * (Sigma + mu * mu');
end
end
