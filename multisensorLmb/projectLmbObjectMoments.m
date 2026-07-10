function [mu, covariance, normalizedWeights] = ...
    projectLmbObjectMoments(object, stateDimension)
% PROJECTLMBOBJECTMOMENTS Canonical per-label Gaussian moment projection.
%
% The projection sanitizes mixture weights, canonicalizes every component
% covariance, and returns the mixture mean and total covariance. Numerical
% jitter belongs to the downstream solve and is deliberately excluded here.

if ~isstruct(object) || numel(object) ~= 1
    error('projectLmbObjectMoments:InvalidObject', ...
        'Expected one scalar LMB object.');
end
if ~isscalar(stateDimension) || ~isfinite(stateDimension) || ...
        stateDimension < 1 || stateDimension ~= floor(stateDimension)
    error('projectLmbObjectMoments:InvalidStateDimension', ...
        'stateDimension must be a positive integer.');
end

requiredFields = {'numberOfGmComponents', 'w', 'mu', 'Sigma'};
for fieldIdx = 1:numel(requiredFields)
    if ~isfield(object, requiredFields{fieldIdx})
        error('projectLmbObjectMoments:MissingField', ...
            'Object is missing field %s.', requiredFields{fieldIdx});
    end
end

componentCount = object.numberOfGmComponents;
if ~isscalar(componentCount) || ~isfinite(componentCount) || ...
        componentCount < 1 || componentCount ~= floor(componentCount)
    error('projectLmbObjectMoments:InvalidComponentCount', ...
        'numberOfGmComponents must be a positive integer.');
end
if numel(object.w) ~= componentCount || ...
        numel(object.mu) ~= componentCount || ...
        numel(object.Sigma) ~= componentCount
    error('projectLmbObjectMoments:InconsistentComponents', ...
        'w, mu, and Sigma must match numberOfGmComponents.');
end

normalizedWeights = reshape(double(object.w), 1, []);
normalizedWeights(~isfinite(normalizedWeights)) = 0;
normalizedWeights = max(normalizedWeights, 0);
weightSum = sum(normalizedWeights);
if weightSum <= 0
    normalizedWeights = ones(1, componentCount) / componentCount;
elseif isfinite(weightSum)
    normalizedWeights = normalizedWeights / weightSum;
else
    weightScale = max(normalizedWeights);
    normalizedWeights = normalizedWeights / weightScale;
    normalizedWeights = normalizedWeights / sum(normalizedWeights);
end

means = cell(1, componentCount);
covariances = cell(1, componentCount);
for componentIdx = 1:componentCount
    componentMean = object.mu{componentIdx};
    componentCovariance = object.Sigma{componentIdx};
    if ~isnumeric(componentMean) || ~isreal(componentMean) || ...
            numel(componentMean) ~= stateDimension || ...
            any(~isfinite(componentMean(:)))
        error('projectLmbObjectMoments:InvalidMean', ...
            'Each component mean must be a finite d-vector.');
    end
    if ~isnumeric(componentCovariance) || ...
            ~isreal(componentCovariance) || ...
            ~isequal(size(componentCovariance), ...
                [stateDimension, stateDimension]) || ...
            any(~isfinite(componentCovariance(:)))
        error('projectLmbObjectMoments:InvalidCovariance', ...
            'Each component covariance must be a finite real d-by-d matrix.');
    end
    means{componentIdx} = reshape(double(componentMean), stateDimension, 1);
    componentCovariance = double(componentCovariance);
    covariances{componentIdx} = ...
        (componentCovariance + componentCovariance') / 2;
end

mu = zeros(stateDimension, 1);
for componentIdx = 1:componentCount
    mu = mu + normalizedWeights(componentIdx) * means{componentIdx};
end

covariance = zeros(stateDimension);
for componentIdx = 1:componentCount
    delta = means{componentIdx} - mu;
    covariance = covariance + normalizedWeights(componentIdx) * ...
        (covariances{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end
