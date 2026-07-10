function compressedObjects = compressLmbPosterior( ...
    objects, model, existenceThreshold, compressionConfig, diagnostics)
% COMPRESSLMBPOSTERIOR Moment-match each active Bernoulli to one Gaussian.

if nargin < 3 || isempty(existenceThreshold)
    existenceThreshold = getField(model, 'existenceThreshold', 0);
end
if nargin < 4 || isempty(compressionConfig)
    compressionConfig = struct();
end
if nargin < 5 || isempty(diagnostics)
    diagnostics = struct();
end
if isempty(objects)
    compressedObjects = objects;
    return;
end

active = [objects.r] > existenceThreshold & ...
    [objects.numberOfGmComponents] > 0;
compressedObjects = objects(active);
for idx = 1:numel(compressedObjects)
    [mu, Sigma] = projectLmbObjectMoments( ...
        compressedObjects(idx), model.xDimension);
    Sigma = inflateLightCovariance( ...
        Sigma, compressedObjects(idx), compressionConfig, diagnostics);
    compressedObjects(idx).numberOfGmComponents = 1;
    compressedObjects(idx).w = 1;
    compressedObjects(idx).mu = {mu};
    compressedObjects(idx).Sigma = {Sigma};
end
end

function Sigma = inflateLightCovariance( ...
    Sigma, object, compressionConfig, diagnostics)
if ~getField(compressionConfig, 'lightCovarianceInflationEnabled', false)
    return;
end
associationConfidence = clamp01(getField( ...
    diagnostics, 'associationConfidence', 1));
associationPenalty = 1 - associationConfidence;
mixtureEntropy = normalizedMixtureEntropy(object);
baseInflation = max(getField( ...
    compressionConfig, 'lightCovarianceInflationBase', 0), 0);
associationScale = max(getField( ...
    compressionConfig, 'lightCovarianceAssociationScale', 0), 0);
mixtureScale = max(getField( ...
    compressionConfig, 'lightCovarianceMixtureScale', 0), 0);
inflation = baseInflation + ...
    associationScale * associationPenalty + ...
    mixtureScale * mixtureEntropy;
if inflation <= 0
    return;
end
Sigma = Sigma + inflation * eye(size(Sigma));
Sigma = (Sigma + Sigma') / 2;
end

function value = normalizedMixtureEntropy(object)
componentCount = max(1, object.numberOfGmComponents);
if componentCount <= 1
    value = 0;
    return;
end
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(1, componentCount) / componentCount;
else
    weights = weights / sum(weights);
end
weights = weights(weights > 0);
entropy = -sum(weights .* log(weights));
value = min(max(entropy / log(componentCount), 0), 1);
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
