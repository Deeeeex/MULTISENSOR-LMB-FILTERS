function metrics = computeLmbPosteriorNetworkDisagreement( ...
    posteriorsBySensor, model)
% COMPUTELMBPOSTERIORNETWORKDISAGREEMENT Label-wise posterior dispersion.
%
% The metric separates existence disagreement from normalized spatial
% moment disagreement. It operates on the Bernoulli posteriors rather than
% MAP set estimates, avoiding cardinality-extraction artifacts.

pairExistence = [];
pairSpatial = [];
pairCombined = [];
sensorCount = numel(posteriorsBySensor);
for leftIdx = 1:sensorCount-1
    for rightIdx = leftIdx+1:sensorCount
        [existenceValue, spatialValue, combinedValue] = ...
            pairDisagreement( ...
                posteriorsBySensor{leftIdx}, ...
                posteriorsBySensor{rightIdx}, model);
        pairExistence(end+1) = existenceValue; %#ok<AGROW>
        pairSpatial(end+1) = spatialValue; %#ok<AGROW>
        pairCombined(end+1) = combinedValue; %#ok<AGROW>
    end
end
metrics = struct( ...
    'existence', meanOrZero(pairExistence), ...
    'spatial', meanOrZero(pairSpatial), ...
    'combined', meanOrZero(pairCombined), ...
    'pairCount', numel(pairCombined));
end

function [existenceValue, spatialValue, combinedValue] = ...
    pairDisagreement(leftObjects, rightObjects, model)
labels = collectLabels(leftObjects, rightObjects);
if isempty(labels)
    existenceValue = 0;
    spatialValue = 0;
    combinedValue = 0;
    return;
end
existenceTerms = zeros(1, size(labels, 2));
spatialTerms = zeros(1, size(labels, 2));
combinedTerms = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    left = findObject(leftObjects, labels(:, labelIdx));
    right = findObject(rightObjects, labels(:, labelIdx));
    leftExistence = existence(left);
    rightExistence = existence(right);
    existenceTerms(labelIdx) = abs(leftExistence - rightExistence);
    if ~isempty(left) && ~isempty(right)
        [leftMean, leftCovariance] = moments(left, model.xDimension);
        [rightMean, rightCovariance] = moments(right, model.xDimension);
        positionScale = sqrt(max(trace( ...
            leftCovariance(1:2, 1:2) + ...
            rightCovariance(1:2, 1:2)), 1));
        spatialTerms(labelIdx) = min(norm( ...
            leftMean(1:2) - rightMean(1:2)) / positionScale, 5);
    end
    combinedTerms(labelIdx) = existenceTerms(labelIdx) + ...
        min(leftExistence, rightExistence) * spatialTerms(labelIdx);
end
existenceValue = mean(existenceTerms);
spatialValue = mean(spatialTerms);
combinedValue = mean(combinedTerms);
end

function labels = collectLabels(leftObjects, rightObjects)
labels = zeros(2, 0);
collections = {leftObjects, rightObjects};
for collectionIdx = 1:numel(collections)
    objects = collections{collectionIdx};
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
end

function object = findObject(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function value = existence(object)
if isempty(object)
    value = 0;
else
    value = min(max(object.r, 0), 1);
end
end

function [meanVector, covariance] = moments(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function value = meanOrZero(values)
if isempty(values)
    value = 0;
else
    value = mean(values);
end
end
