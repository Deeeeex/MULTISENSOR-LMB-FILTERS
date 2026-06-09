function compressedObjects = compressLmbPosterior(objects, model, existenceThreshold)
% COMPRESSLMBPOSTERIOR Moment-match each active Bernoulli to one Gaussian.

if nargin < 3 || isempty(existenceThreshold)
    existenceThreshold = getField(model, 'existenceThreshold', 0);
end
if isempty(objects)
    compressedObjects = objects;
    return;
end

active = [objects.r] > existenceThreshold & ...
    [objects.numberOfGmComponents] > 0;
compressedObjects = objects(active);
for idx = 1:numel(compressedObjects)
    [mu, Sigma] = momentMatchObject(compressedObjects(idx), model.xDimension);
    compressedObjects(idx).numberOfGmComponents = 1;
    compressedObjects(idx).w = 1;
    compressedObjects(idx).mu = {mu};
    compressedObjects(idx).Sigma = {Sigma};
end
end

function [mu, Sigma] = momentMatchObject(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(size(weights)) / max(numel(weights), 1);
else
    weights = weights / sum(weights);
end

mu = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    mu = mu + weights(componentIdx) * object.mu{componentIdx};
end

Sigma = zeros(stateDimension, stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - mu;
    Sigma = Sigma + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
Sigma = regularizeCovariance(Sigma);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
if isempty(covariance)
    return;
end
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
