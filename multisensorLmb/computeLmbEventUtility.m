function [utility, details] = computeLmbEventUtility( ...
    currentObjects, referenceObjects, neighborObjects, predictedObjects, ...
    updateDiagnostics, model, config)
% COMPUTELMBEVENTUTILITY Compute target-wise multi-indicator trigger utility.

if nargin < 7 || isempty(config)
    config = struct();
end
weights = getField(config, 'criterionWeights', [0.20, 0.20, 0.35, 0.25]);
weights = reshape(weights, 1, []);
if numel(weights) ~= 4 || sum(max(weights, 0)) <= 0
    weights = [0.20, 0.20, 0.35, 0.25];
end
weights = max(weights, 0) / sum(max(weights, 0));
activeThreshold = getField(config, 'activeExistenceThreshold', ...
    getField(model, 'existenceThreshold', 0));

labels = unionLabels(currentObjects, referenceObjects, ...
    neighborObjects, predictedObjects, activeThreshold);
numLabels = size(labels, 2);
perLabel = zeros(1, numLabels);
stateScore = zeros(1, numLabels);
informationGain = zeros(1, numLabels);
neighborDisagreement = zeros(1, numLabels);
localQuality = zeros(1, numLabels);
innovation = clamp01(getField( ...
    updateDiagnostics, 'innovationNovelty', 0));

for labelIdx = 1:numLabels
    label = labels(:, labelIdx);
    current = findObject(currentObjects, label, activeThreshold);
    reference = findObject(referenceObjects, label, activeThreshold);
    neighbor = findObject(neighborObjects, label, activeThreshold);
    predicted = findObject(predictedObjects, label, activeThreshold);

    stateScore(labelIdx) = compareObjects(current, reference, model);
    informationGain(labelIdx) = computeInformationGain( ...
        predicted, current, model);
    neighborDisagreement(labelIdx) = compareObjects( ...
        current, neighbor, model);
    localQuality(labelIdx) = computeLocalQuality( ...
        current, updateDiagnostics, model, config);
    perLabel(labelIdx) = localQuality(labelIdx) * ...
        (weights(1) * stateScore(labelIdx) + ...
         weights(2) * innovation + ...
         weights(3) * informationGain(labelIdx) + ...
         weights(4) * neighborDisagreement(labelIdx));
end

if isempty(perLabel)
    utility = 0;
else
    utility = max(perLabel);
end

details = struct();
details.labels = labels;
details.perLabelUtility = perLabel;
details.stateScore = stateScore;
details.innovation = innovation;
details.informationGain = informationGain;
details.neighborDisagreement = neighborDisagreement;
details.localQuality = localQuality;
details.maxInformationGain = maxOrZero(informationGain);
forceLabelThreshold = getField( ...
    config, 'forceLabelExistenceThreshold', 0.5);
details.labelChanged = labelsChanged( ...
    currentObjects, referenceObjects, forceLabelThreshold);
details.referenceEmpty = isemptyActive(referenceObjects, activeThreshold);
details.currentEmpty = isemptyActive(currentObjects, activeThreshold);
end

function score = compareObjects(left, right, model)
if isempty(left) && isempty(right)
    score = 0;
    return;
end
if isempty(left) || isempty(right)
    score = 1;
    return;
end

[leftMu, leftCov] = momentMatch(left, model.xDimension);
[rightMu, rightCov] = momentMatch(right, model.xDimension);
positionDimension = min(2, model.xDimension);
delta = leftMu(1:positionDimension) - rightMu(1:positionDimension);
scaleCov = leftCov(1:positionDimension, 1:positionDimension) + ...
    rightCov(1:positionDimension, 1:positionDimension);
scaleCov = regularizeCovariance(scaleCov);
mahalanobis = delta' * (scaleCov \ delta);
positionScore = 1 - exp(-0.5 * max(mahalanobis, 0));
existenceScore = abs(clamp01(left.r) - clamp01(right.r));
traceScore = abs(log((trace(leftCov) + eps) / (trace(rightCov) + eps)));
traceScore = 1 - exp(-traceScore);
score = clamp01(0.55 * positionScore + ...
    0.30 * existenceScore + 0.15 * traceScore);
end

function score = computeInformationGain(predicted, current, model)
if isempty(predicted) || isempty(current)
    score = double(~isempty(current));
    return;
end
[~, predictedCov] = momentMatch(predicted, model.xDimension);
[~, currentCov] = momentMatch(current, model.xDimension);
logDetGain = max(logDet(predictedCov) - logDet(currentCov), 0);
covarianceGain = 1 - exp(-0.25 * logDetGain);

predictedEntropy = bernoulliEntropy(predicted.r);
currentEntropy = bernoulliEntropy(current.r);
entropyGain = max(predictedEntropy - currentEntropy, 0) / log(2);
score = clamp01(0.65 * covarianceGain + 0.35 * entropyGain);
end

function score = computeLocalQuality(object, diagnostics, model, config)
if isempty(object)
    score = 0;
    return;
end
[~, covariance] = momentMatch(object, model.xDimension);
positionDimension = min(2, model.xDimension);
positionTrace = trace(covariance(1:positionDimension, 1:positionDimension));
traceScale = max(getField(config, 'localQualityTraceScale', 25), eps);
covarianceQuality = 1 / (1 + positionTrace / traceScale);
existenceQuality = clamp01(object.r);
associationQuality = clamp01(getField( ...
    diagnostics, 'associationConfidence', 1));
score = clamp01(0.50 * covarianceQuality + ...
    0.30 * existenceQuality + 0.20 * associationQuality);
end

function labels = unionLabels(varargin)
activeThreshold = varargin{end};
objectLists = varargin(1:end-1);
labels = zeros(2, 0);
for listIdx = 1:numel(objectLists)
    objects = objectLists{listIdx};
    if isempty(objects)
        continue;
    end
    active = [objects.r] > activeThreshold & ...
        [objects.numberOfGmComponents] > 0;
    objects = objects(active);
    for objectIdx = 1:numel(objects)
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

function object = findObject(objects, label, activeThreshold)
object = [];
if isempty(objects)
    return;
end
for idx = 1:numel(objects)
    if objects(idx).r > activeThreshold && ...
            objects(idx).numberOfGmComponents > 0 && ...
            objects(idx).birthTime == label(1) && ...
            objects(idx).birthLocation == label(2)
        object = objects(idx);
        return;
    end
end
end

function changed = labelsChanged(currentObjects, referenceObjects, threshold)
currentLabels = unionLabels(currentObjects, threshold);
referenceLabels = unionLabels(referenceObjects, threshold);
changed = ~isequal(currentLabels, referenceLabels);
end

function tf = isemptyActive(objects, threshold)
tf = isempty(objects) || ~any([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
end

function [mu, covariance] = momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
weights = weights / max(sum(weights), eps);
mu = zeros(stateDimension, 1);
for idx = 1:object.numberOfGmComponents
    mu = mu + weights(idx) * object.mu{idx};
end
covariance = zeros(stateDimension, stateDimension);
for idx = 1:object.numberOfGmComponents
    delta = object.mu{idx} - mu;
    covariance = covariance + weights(idx) * ...
        (object.Sigma{idx} + delta * delta');
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

function value = bernoulliEntropy(probability)
probability = min(max(probability, eps), 1 - eps);
value = -probability * log(probability) - ...
    (1 - probability) * log(1 - probability);
end

function value = maxOrZero(values)
if isempty(values)
    value = 0;
else
    value = max(values);
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
