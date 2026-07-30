function result = ...
    evaluateNestedOneDimensionalGateLoso( ...
        features, targetPositive, featureNames)
% EVALUATENESTEDONEDIMENSIONALGATELOSO Honest seed-level threshold audit.
%
% Each outer fold searches every scalar feature, direction and threshold
% using only the remaining rows, then applies the selected rule once to the
% held-out row. The optimistic all-row fit is reported separately and must
% not be interpreted as generalization evidence.

if nargin < 3 || isempty(featureNames)
    featureNames = arrayfun(@(idx) ...
        sprintf('feature_%d', idx), ...
        1:size(features, 2), 'UniformOutput', false);
end
features = double(features);
targetPositive = logical(reshape(targetPositive, [], 1));
rowCount = size(features, 1);
featureCount = size(features, 2);
if rowCount < 3 || featureCount < 1 || ...
        numel(targetPositive) ~= rowCount || ...
        numel(featureNames) ~= featureCount || ...
        any(~isfinite(features(:))) || ...
        numel(unique(targetPositive)) < 2
    error('Nested scalar gate inputs are invalid.');
end

prediction = false(rowCount, 1);
selectedFeatureIndex = zeros(rowCount, 1);
selectedThreshold = nan(rowCount, 1);
selectedDirection = zeros(rowCount, 1);
trainingAccuracy = nan(rowCount, 1);
for heldoutIdx = 1:rowCount
    trainingRows = setdiff(1:rowCount, heldoutIdx);
    fit = fitBestScalarThreshold( ...
        features(trainingRows, :), ...
        targetPositive(trainingRows));
    prediction(heldoutIdx) = ...
        fit.direction * ...
            (features(heldoutIdx, fit.featureIndex) - ...
             fit.threshold) >= 0;
    selectedFeatureIndex(heldoutIdx) = ...
        fit.featureIndex;
    selectedThreshold(heldoutIdx) = fit.threshold;
    selectedDirection(heldoutIdx) = fit.direction;
    trainingAccuracy(heldoutIdx) = fit.accuracy;
end
optimistic = fitBestScalarThreshold( ...
    features, targetPositive);

result = struct();
result.contractVersion = ...
    'nested-one-dimensional-gate-loso-v1';
result.prediction = prediction;
result.targetPositive = targetPositive;
result.correct = prediction == targetPositive;
result.accuracy = mean(result.correct);
result.selectedFeatureIndex = ...
    selectedFeatureIndex;
result.selectedFeatureNames = ...
    featureNames(selectedFeatureIndex);
result.selectedThreshold = selectedThreshold;
result.selectedDirection = selectedDirection;
result.trainingAccuracy = trainingAccuracy;
result.optimisticInSampleAccuracy = ...
    optimistic.accuracy;
result.optimisticFeatureIndex = ...
    optimistic.featureIndex;
result.optimisticFeatureName = ...
    featureNames{optimistic.featureIndex};
result.optimisticThreshold = ...
    optimistic.threshold;
result.optimisticDirection = ...
    optimistic.direction;
result.wholeRowHeldOut = true;
result.featureSelectionNestedWithinTrainingRows = true;
end

function fit = fitBestScalarThreshold(features, target)
featureCount = size(features, 2);
bestAccuracy = -inf;
bestFeature = 1;
bestThreshold = 0;
bestDirection = 1;
for featureIdx = 1:featureCount
    values = sort(unique(features(:, featureIdx)));
    if numel(values) == 1
        thresholds = values;
    else
        padding = max(1, max(abs(values)));
        thresholds = [ ...
            values(1) - padding; ...
            (values(1:end-1) + values(2:end)) / 2; ...
            values(end) + padding];
    end
    for direction = [-1, 1]
        for thresholdIdx = 1:numel(thresholds)
            threshold = thresholds(thresholdIdx);
            prediction = direction * ...
                (features(:, featureIdx) - threshold) >= 0;
            accuracy = mean(prediction == target);
            if accuracy > bestAccuracy + 1e-12
                bestAccuracy = accuracy;
                bestFeature = featureIdx;
                bestThreshold = threshold;
                bestDirection = direction;
            end
        end
    end
end
fit = struct( ...
    'accuracy', bestAccuracy, ...
    'featureIndex', bestFeature, ...
    'threshold', bestThreshold, ...
    'direction', bestDirection);
end
