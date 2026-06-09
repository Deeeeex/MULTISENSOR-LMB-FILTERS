function fusedObjects = fuseLmbPosteriorsByLabel( ...
    posteriorDistributions, spatialWeights, model, existenceWeights)
% FUSELMBPOSTERIORSBYLABEL GA/KLA fusion for possibly different label sets.
%
% Sources missing a label do not participate in that label's fusion. The
% remaining source weights are renormalized, avoiding a zero-existence veto
% when independently pruned nodes have different active label sets.

if isempty(posteriorDistributions)
    fusedObjects = model.object;
    return;
end
if nargin < 4 || isempty(existenceWeights)
    existenceWeights = spatialWeights;
end
spatialWeights = normalizeWeights( ...
    spatialWeights, numel(posteriorDistributions));
existenceWeights = normalizeWeights( ...
    existenceWeights, numel(posteriorDistributions));

labels = collectLabels(posteriorDistributions);
if isempty(labels)
    fusedObjects = posteriorDistributions{1};
    return;
end

fusedObjects = posteriorDistributions{1}([]);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    localObjects = cell(1, numel(posteriorDistributions));
    present = false(1, numel(posteriorDistributions));
    for sourceIdx = 1:numel(posteriorDistributions)
        localObjects{sourceIdx} = findObject( ...
            posteriorDistributions{sourceIdx}, label);
        present(sourceIdx) = ~isempty(localObjects{sourceIdx});
    end
    activeSpatialWeights = spatialWeights .* present;
    activeExistenceWeights = existenceWeights .* present;
    if sum(activeSpatialWeights) <= 0 || sum(activeExistenceWeights) <= 0
        continue;
    end
    activeSpatialWeights = activeSpatialWeights / sum(activeSpatialWeights);
    activeExistenceWeights = ...
        activeExistenceWeights / sum(activeExistenceWeights);
    templateIdx = find(present, 1, 'first');
    fusedObject = localObjects{templateIdx};

    canonicalK = zeros(model.xDimension);
    canonicalH = zeros(model.xDimension, 1);
    canonicalG = 0;
    for sourceIdx = find(present)
        [mu, covariance] = momentMatch( ...
            localObjects{sourceIdx}, model.xDimension);
        covariance = regularizeCovariance(covariance);
        precision = activeSpatialWeights(sourceIdx) * inv(covariance);
        canonicalK = canonicalK + precision;
        canonicalH = canonicalH + precision * mu;
        canonicalG = canonicalG - ...
            0.5 * mu' * precision * mu - ...
            0.5 * activeSpatialWeights(sourceIdx) * ...
            logDet(2 * pi * covariance);
    end
    fusedCovariance = regularizeCovariance(inv(canonicalK));
    fusedMean = fusedCovariance * canonicalH;
    eta = exp(canonicalG + ...
        0.5 * fusedMean' * canonicalK * fusedMean + ...
        0.5 * logDet(2 * pi * fusedCovariance));

    numerator = eta;
    denominatorAbsent = 1;
    for sourceIdx = find(present)
        probability = clampProbability(localObjects{sourceIdx}.r);
        numerator = numerator * probability^activeExistenceWeights(sourceIdx);
        denominatorAbsent = denominatorAbsent * ...
            (1 - probability)^activeExistenceWeights(sourceIdx);
    end
    fusedObject.r = numerator / max(numerator + denominatorAbsent, eps);
    fusedObject.numberOfGmComponents = 1;
    fusedObject.w = 1;
    fusedObject.mu = {fusedMean};
    fusedObject.Sigma = {fusedCovariance};
    fusedObjects(end+1) = fusedObject; %#ok<AGROW>
end

function weights = normalizeWeights(weights, sourceCount)
weights = reshape(weights, 1, []);
if numel(weights) ~= sourceCount
    weights = ones(1, sourceCount);
end
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(1, sourceCount);
end
weights = weights / sum(weights);
end
end

function labels = collectLabels(distributions)
labels = zeros(2, 0);
for sourceIdx = 1:numel(distributions)
    objects = distributions{sourceIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents < 1
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ~any(all(labels == label, 1))
            labels(:, end+1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function object = findObject(objects, label)
object = [];
for idx = 1:numel(objects)
    if objects(idx).numberOfGmComponents > 0 && ...
            objects(idx).birthTime == label(1) && ...
            objects(idx).birthLocation == label(2)
        object = objects(idx);
        return;
    end
end
end

function [mu, covariance] = momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
weights = weights / max(sum(weights), eps);
mu = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    mu = mu + weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - mu;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end

function value = logDet(matrix)
matrix = regularizeCovariance(matrix);
[factor, flag] = chol(matrix);
if flag == 0
    value = 2 * sum(log(diag(factor)));
else
    value = log(max(det(matrix), realmin));
end
end

function probability = clampProbability(probability)
if ~isfinite(probability)
    probability = 0.5;
end
probability = min(max(probability, 1e-9), 1 - 1e-9);
end
