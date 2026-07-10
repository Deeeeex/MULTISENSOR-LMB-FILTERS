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
        [mu, covariance] = projectLmbObjectMoments( ...
            localObjects{sourceIdx}, model.xDimension);
        [covariance, ~, covarianceFactor] = ...
            regularizeCovarianceForSolve(covariance);
        inverseCovariance = solveFromCholesky( ...
            covarianceFactor, eye(model.xDimension));
        precision = activeSpatialWeights(sourceIdx) * inverseCovariance;
        canonicalK = canonicalK + precision;
        canonicalH = canonicalH + precision * mu;
        canonicalG = canonicalG - ...
            0.5 * mu' * precision * mu - ...
            0.5 * activeSpatialWeights(sourceIdx) * ...
            gaussianLogNormalizer(covarianceFactor);
    end
    [canonicalK, ~, canonicalFactor] = ...
        regularizeCovarianceForSolve(canonicalK);
    fusedCovariance = solveFromCholesky( ...
        canonicalFactor, eye(model.xDimension));
    [fusedCovariance, ~, fusedCovarianceFactor] = ...
        regularizeCovarianceForSolve(fusedCovariance);
    fusedMean = solveFromCholesky(canonicalFactor, canonicalH);
    eta = exp(canonicalG + ...
        0.5 * fusedMean' * canonicalK * fusedMean + ...
        0.5 * gaussianLogNormalizer(fusedCovarianceFactor));

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

function solution = solveFromCholesky(choleskyFactor, rightHandSide)
solution = choleskyFactor \ (choleskyFactor' \ rightHandSide);
end

function value = gaussianLogNormalizer(choleskyFactor)
dimension = size(choleskyFactor, 1);
value = dimension * log(2 * pi) + ...
    2 * sum(log(diag(choleskyFactor)));
end

function probability = clampProbability(probability)
if ~isfinite(probability)
    probability = 0.5;
end
probability = min(max(probability, 1e-9), 1 - 1e-9);
end
