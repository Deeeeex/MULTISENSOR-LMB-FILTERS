function [scoreMatrix, details] = ...
    computeLmbKlaCompatibilityMatrix(context, options)
% COMPUTELMBKLACOMPATIBILITYMATRIX Truth-free directed KLA compatibility.
%
% For sender j -> receiver i, the spatial term is the Chernoff overlap
%
%   eta = integral p_i(x)^(1-alpha) p_j(x)^alpha dx.
%
% It is evaluated on moment-matched label densities with the same
% receiver/source weight used by the candidate KLA action. The Bernoulli
% coefficient also includes existence probabilities. A small coefficient
% identifies a conflicting posterior pair whose KLA normalization can
% suppress label existence. The score is deployment-observable and never
% reads target truth.

if nargin < 2 || isempty(options)
    options = struct();
end
posteriors = reshape(context.localPosteriorBySensor, 1, []);
nodeCount = numel(posteriors);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;

sourceWeight = getField(options, 'sourceWeight', 0.50);
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('sourceWeight must be a finite scalar strictly between 0 and 1.');
end
activeThreshold = getField(options, 'activeExistenceThreshold', 0.01);
noveltyWeight = max(getField(options, 'noveltyWeight', 0.10), 0);
precisionPenaltyWeight = max(getField( ...
    options, 'precisionPenaltyWeight', 0.10), 0);

prepared = cell(1, nodeCount);
for nodeIdx = 1:nodeCount
    prepared{nodeIdx} = preparePosterior( ...
        posteriors{nodeIdx}, context.model, activeThreshold);
end

scoreMatrix = -inf(nodeCount);
compatibilityMatrix = nan(nodeCount);
spatialOverlapMatrix = nan(nodeCount);
noveltyMatrix = nan(nodeCount);
precisionPenaltyMatrix = nan(nodeCount);
reliabilityMatrix = nan(nodeCount);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        reliability = 1 - edgeDrop( ...
            context.commConfig, senderIdx, receiverIdx, ...
            context.currentTime);
        pair = comparePreparedPosteriors( ...
            prepared{receiverIdx}, prepared{senderIdx}, ...
            sourceWeight);
        score = reliability * ...
            (pair.bernoulliCompatibility + ...
             noveltyWeight * pair.sourceNovelty) - ...
            precisionPenaltyWeight * pair.sourceWorsePrecision;
        scoreMatrix(receiverIdx, senderIdx) = score;
        compatibilityMatrix(receiverIdx, senderIdx) = ...
            pair.bernoulliCompatibility;
        spatialOverlapMatrix(receiverIdx, senderIdx) = ...
            pair.spatialOverlap;
        noveltyMatrix(receiverIdx, senderIdx) = pair.sourceNovelty;
        precisionPenaltyMatrix(receiverIdx, senderIdx) = ...
            pair.sourceWorsePrecision;
        reliabilityMatrix(receiverIdx, senderIdx) = reliability;
    end
end

details = struct();
details.sourceWeight = sourceWeight;
details.scoreMatrix = scoreMatrix;
details.bernoulliCompatibilityMatrix = compatibilityMatrix;
details.spatialOverlapMatrix = spatialOverlapMatrix;
details.sourceNoveltyMatrix = noveltyMatrix;
details.sourceWorsePrecisionMatrix = precisionPenaltyMatrix;
details.reliabilityMatrix = reliabilityMatrix;
details.noveltyWeight = noveltyWeight;
details.precisionPenaltyWeight = precisionPenaltyWeight;
end

function prepared = preparePosterior(objects, model, activeThreshold)
prepared = struct( ...
    'labels', zeros(2, 0), ...
    'existence', zeros(1, 0), ...
    'means', {{}}, ...
    'covariances', {{}}, ...
    'positionTrace', zeros(1, 0), ...
    'quality', zeros(1, 0));
if isempty(objects)
    return;
end
active = [objects.numberOfGmComponents] > 0 & ...
    [objects.r] >= activeThreshold;
objects = reshape(objects(active), 1, []);
objectCount = numel(objects);
if objectCount == 0
    return;
end
prepared.labels = zeros(2, objectCount);
prepared.existence = zeros(1, objectCount);
prepared.means = cell(1, objectCount);
prepared.covariances = cell(1, objectCount);
prepared.positionTrace = zeros(1, objectCount);
prepared.quality = zeros(1, objectCount);
positionDimension = min(2, model.xDimension);
for objectIdx = 1:objectCount
    object = objects(objectIdx);
    [meanVector, covariance] = momentMatch( ...
        object, model.xDimension);
    prepared.labels(:, objectIdx) = [ ...
        object.birthTime; object.birthLocation];
    prepared.existence(objectIdx) = clampProbability(object.r);
    prepared.means{objectIdx} = meanVector;
    prepared.covariances{objectIdx} = covariance;
    prepared.positionTrace(objectIdx) = trace( ...
        covariance(1:positionDimension, 1:positionDimension));
    association = clamp01(getScalarField( ...
        object, 'associationConfidence', 0));
    detection = clamp01(getScalarField( ...
        object, 'detectionAssociationMass', 0));
    prepared.quality(objectIdx) = 0.5 * association + ...
        0.5 * detection;
end
end

function pair = comparePreparedPosteriors(receiver, source, sourceWeight)
receiverCount = size(receiver.labels, 2);
sourceCount = size(source.labels, 2);
sharedWeight = 0;
bernoulliSum = 0;
spatialSum = 0;
precisionPenaltySum = 0;
sourceNoveltyMass = 0;
sourceMass = sum(source.existence);

for receiverIdx = 1:receiverCount
    sourceIdx = findLabelIndex( ...
        source.labels, receiver.labels(:, receiverIdx));
    if isempty(sourceIdx)
        continue;
    end
    receiverExistence = receiver.existence(receiverIdx);
    sourceExistence = source.existence(sourceIdx);
    salience = max(receiverExistence, sourceExistence);
    logSpatialOverlap = gaussianChernoffLogOverlap( ...
        receiver.means{receiverIdx}, ...
        receiver.covariances{receiverIdx}, ...
        source.means{sourceIdx}, ...
        source.covariances{sourceIdx}, sourceWeight);
    spatialOverlap = exp(min(logSpatialOverlap, 0));
    logAbsent = (1 - sourceWeight) * ...
        log(max(1 - receiverExistence, realmin)) + ...
        sourceWeight * log(max(1 - sourceExistence, realmin));
    logPresent = (1 - sourceWeight) * ...
        log(max(receiverExistence, realmin)) + ...
        sourceWeight * log(max(sourceExistence, realmin)) + ...
        logSpatialOverlap;
    bernoulliCompatibility = exp(min( ...
        logSumExp([logAbsent, logPresent]), 0));
    receiverTrace = max(receiver.positionTrace(receiverIdx), eps);
    sourceTrace = max(source.positionTrace(sourceIdx), eps);
    sourceWorsePrecision = max(log(sourceTrace / receiverTrace), 0);

    sharedWeight = sharedWeight + salience;
    bernoulliSum = bernoulliSum + ...
        salience * bernoulliCompatibility;
    spatialSum = spatialSum + salience * spatialOverlap;
    precisionPenaltySum = precisionPenaltySum + ...
        salience * sourceWorsePrecision;
end

for sourceIdx = 1:sourceCount
    if isempty(findLabelIndex( ...
            receiver.labels, source.labels(:, sourceIdx)))
        sourceNoveltyMass = sourceNoveltyMass + ...
            source.existence(sourceIdx) * source.quality(sourceIdx);
    end
end

if sharedWeight > eps
    bernoulliCompatibility = bernoulliSum / sharedWeight;
    spatialOverlap = spatialSum / sharedWeight;
    sourceWorsePrecision = precisionPenaltySum / sharedWeight;
else
    bernoulliCompatibility = 0;
    spatialOverlap = 0;
    sourceWorsePrecision = 0;
end
pair = struct( ...
    'bernoulliCompatibility', clamp01(bernoulliCompatibility), ...
    'spatialOverlap', clamp01(spatialOverlap), ...
    'sourceNovelty', clamp01( ...
        sourceNoveltyMass / max(sourceMass, 1)), ...
    'sourceWorsePrecision', max(sourceWorsePrecision, 0));
end

function idx = findLabelIndex(labels, label)
idx = [];
if isempty(labels)
    return;
end
idx = find(all(bsxfun(@eq, labels, label), 1), 1);
end

function logOverlap = gaussianChernoffLogOverlap( ...
    receiverMean, receiverCovariance, ...
    sourceMean, sourceCovariance, sourceWeight)
selfWeight = 1 - sourceWeight;
receiverCovariance = regularizeCovariance(receiverCovariance);
sourceCovariance = regularizeCovariance(sourceCovariance);
receiverPrecision = inv(receiverCovariance);
sourcePrecision = inv(sourceCovariance);
canonicalK = selfWeight * receiverPrecision + ...
    sourceWeight * sourcePrecision;
canonicalH = selfWeight * receiverPrecision * receiverMean + ...
    sourceWeight * sourcePrecision * sourceMean;
canonicalG = -0.5 * selfWeight * ...
    (receiverMean' * receiverPrecision * receiverMean) - ...
    0.5 * selfWeight * logDet(2 * pi * receiverCovariance) - ...
    0.5 * sourceWeight * ...
    (sourceMean' * sourcePrecision * sourceMean) - ...
    0.5 * sourceWeight * logDet(2 * pi * sourceCovariance);
fusedCovariance = regularizeCovariance(inv(canonicalK));
fusedMean = fusedCovariance * canonicalH;
logOverlap = canonicalG + ...
    0.5 * fusedMean' * canonicalK * fusedMean + ...
    0.5 * logDet(2 * pi * fusedCovariance);
logOverlap = min(real(logOverlap), 0);
end

function [meanVector, covariance] = ...
    momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
componentCount = object.numberOfGmComponents;
if numel(weights) ~= componentCount || sum(weights) <= 0
    weights = ones(1, componentCount);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:componentCount
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:componentCount
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
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

function value = logDet(matrix)
matrix = regularizeCovariance(matrix);
[factor, flag] = chol(matrix);
if flag == 0
    value = 2 * sum(log(diag(factor)));
else
    value = log(max(real(det(matrix)), realmin));
end
end

function value = logSumExp(values)
values = reshape(values(isfinite(values)), 1, []);
if isempty(values)
    value = -inf;
    return;
end
maximum = max(values);
value = maximum + log(sum(exp(values - maximum)));
end

function probability = edgeDrop( ...
    config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function value = getScalarField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName) && ...
        isscalar(structure.(fieldName)) && ...
        isfinite(structure.(fieldName))
    value = structure.(fieldName);
else
    value = defaultValue;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end

function value = clampProbability(value)
if ~isfinite(value)
    value = 0.5;
end
value = min(max(value, 1e-9), 1 - 1e-9);
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
