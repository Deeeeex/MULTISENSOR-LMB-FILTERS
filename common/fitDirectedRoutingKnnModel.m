function [model, summary] = ...
    fitDirectedRoutingKnnModel(datasetPath, options)
% FITDIRECTEDROUTINGKNNMODEL Receiver-held-out nonlinear edge regressor.
%
% A small standardized k-nearest-neighbor regressor is intentionally used
% instead of a heavyweight toolbox model. It captures nonlinear feature
% interactions, remains inspectable, and can run in Octave at M24/X36.

if nargin < 2 || isempty(options)
    options = struct();
end
loaded = load(datasetPath);
[X, y, weightTarget, receivers, featureNames] = unpackDataset(loaded);
foldCount = min(max(2, round(getField( ...
    options, 'foldCount', 4))), numel(unique(receivers)));
neighborCounts = reshape(getField(options, ...
    'neighborCounts', [3, 5, 9, 15, 25, 40]), 1, []);
thresholds = reshape(getField(options, ...
    'gainThresholds', [0, 0.01, 0.02, 0.04, 0.06, 0.10]), ...
    1, []);
positiveNeighborFractions = reshape(getField(options, ...
    'positiveNeighborFractions', [0.50, 0.60, 0.70, 0.80, 0.90]), ...
    1, []);
maximumNegativeSelectionFraction = getField( ...
    options, 'maximumNegativeSelectionFraction', 0.10);
minimumSelectedGainFraction = getField( ...
    options, 'minimumSelectedGainFraction', -0.05);

featureMean = mean(X, 1);
featureScale = std(X, 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
Z = bsxfun(@rdivide, bsxfun(@minus, X, featureMean), featureScale);

uniqueReceivers = unique(receivers);
foldByReceiver = mod(uniqueReceivers - 1, foldCount) + 1;
foldAssignment = zeros(size(receivers));
for receiverCursor = 1:numel(uniqueReceivers)
    foldAssignment(receivers == uniqueReceivers(receiverCursor)) = ...
        foldByReceiver(receiverCursor);
end

bestScore = -inf;
bestNeighborCount = NaN;
bestThreshold = NaN;
bestCv = struct();
for neighborCount = neighborCounts
    predictions = nan(size(y));
    positiveConfidence = nan(size(y));
    for foldIdx = 1:foldCount
        training = foldAssignment ~= foldIdx;
        validationIndices = find(foldAssignment == foldIdx);
        [predictions(validationIndices), ...
            positiveConfidence(validationIndices)] = knnPredict( ...
            Z(training, :), y(training), ...
            Z(validationIndices, :), neighborCount);
    end
    for threshold = thresholds
        for positiveFraction = positiveNeighborFractions
            cv = evaluateReceiverRanking( ...
                predictions, positiveConfidence, y, receivers, ...
                threshold, positiveFraction);
            if cv.negativeSelectionFraction > ...
                    maximumNegativeSelectionFraction || ...
                    cv.minimumSelectedGainFraction < ...
                    minimumSelectedGainFraction
                continue;
            end
            score = cv.meanSelectedGainFraction - ...
                0.5 * cv.negativeSelectionFraction;
            if score > bestScore
                bestScore = score;
                bestNeighborCount = neighborCount;
                bestThreshold = threshold;
                bestCv = cv;
            end
        end
    end
end
if ~isfinite(bestNeighborCount)
    error('No kNN model passed the receiver-held-out safety gate.');
end

model = struct();
model.kind = 'receiver-held-out-knn-directed-routing';
model.featureNames = featureNames;
model.featureMean = featureMean;
model.featureScale = featureScale;
model.trainingFeatures = Z;
model.trainingGainTargets = y;
model.trainingWeightTargets = weightTarget;
model.neighborCount = bestNeighborCount;
model.gainThreshold = bestThreshold;
model.minimumPositiveNeighborFraction = ...
    bestCv.minimumPositiveNeighborFraction;
model.minimumSourceWeight = getField( ...
    options, 'minimumSourceWeight', 0.15);
model.maximumSourceWeight = getField( ...
    options, 'maximumSourceWeight', 0.70);
model.maxMessagesPerReceiver = 1;
model.trainingDatasetPath = datasetPath;

summary = struct();
summary.exampleCount = numel(y);
summary.receiverCount = numel(uniqueReceivers);
summary.selectedNeighborCount = bestNeighborCount;
summary.selectedGainThreshold = bestThreshold;
summary.crossValidation = bestCv;
summary.meanOracleEdgeGainFraction = ...
    mean(maxGainByReceiver(y, receivers));
summary.positiveEdgeFraction = mean(y > 0);
summary.featureNames = featureNames;
end

function [predictions, positiveConfidence] = knnPredict( ...
    trainingFeatures, trainingTargets, queryFeatures, neighborCount)
queryCount = size(queryFeatures, 1);
predictions = zeros(queryCount, 1);
positiveConfidence = zeros(queryCount, 1);
neighborCount = min(max(1, round(neighborCount)), ...
    size(trainingFeatures, 1));
for queryIdx = 1:queryCount
    delta = bsxfun(@minus, ...
        trainingFeatures, queryFeatures(queryIdx, :));
    distanceSquared = sum(delta.^2, 2);
    [sortedDistance, order] = sort(distanceSquared, 'ascend');
    order = order(1:neighborCount);
    sortedDistance = sortedDistance(1:neighborCount);
    weights = 1 ./ max(sortedDistance, 1e-6);
    weights = weights / sum(weights);
    predictions(queryIdx) = ...
        sum(weights .* trainingTargets(order));
    positiveConfidence(queryIdx) = ...
        sum(weights .* (trainingTargets(order) > 0));
end
end

function [X, y, weightTarget, receivers, featureNames] = ...
    unpackDataset(loaded)
required = {'features', 'featureNames', 'teacherDetails'};
if ~all(isfield(loaded, required))
    error('Directed-routing dataset is missing required fields.');
end
features = loaded.features;
teacher = loaded.teacherDetails;
nodeCount = size(features, 1);
featureNames = reshape(loaded.featureNames, 1, []);
biasIdx = find(strcmp(featureNames, 'bias'));
keptFeatureIdx = setdiff(1:numel(featureNames), biasIdx);
featureNames = featureNames(keptFeatureIdx);
X = zeros(0, numel(keptFeatureIdx));
y = zeros(0, 1);
weightTarget = zeros(0, 1);
receivers = zeros(0, 1);
for receiverIdx = 1:nodeCount
    for senderIdx = 1:nodeCount
        candidateRisk = ...
            teacher.firstStepCandidateRisk(receiverIdx, senderIdx);
        row = reshape(features( ...
            receiverIdx, senderIdx, keptFeatureIdx), 1, []);
        if ~isfinite(candidateRisk) || any(~isfinite(row))
            continue;
        end
        baselineRisk = teacher.nodeRiskBefore(receiverIdx);
        X(end+1, :) = row; %#ok<AGROW>
        y(end+1, 1) = ...
            (baselineRisk - candidateRisk) / max(baselineRisk, eps); %#ok<AGROW>
        weightTarget(end+1, 1) = ...
            teacher.firstStepCandidateSourceWeight( ...
                receiverIdx, senderIdx); %#ok<AGROW>
        receivers(end+1, 1) = receiverIdx; %#ok<AGROW>
    end
end
end

function summary = evaluateReceiverRanking( ...
    predictions, positiveConfidence, actualGain, receivers, ...
    threshold, minimumPositiveNeighborFraction)
uniqueReceivers = unique(receivers);
selectedGain = zeros(1, numel(uniqueReceivers));
selected = false(1, numel(uniqueReceivers));
oracleGain = zeros(1, numel(uniqueReceivers));
for receiverCursor = 1:numel(uniqueReceivers)
    mask = receivers == uniqueReceivers(receiverCursor);
    receiverPrediction = predictions(mask);
    receiverConfidence = positiveConfidence(mask);
    receiverGain = actualGain(mask);
    eligible = receiverPrediction >= threshold & ...
        receiverConfidence >= minimumPositiveNeighborFraction;
    oracleGain(receiverCursor) = max([0; receiverGain]);
    if any(eligible)
        eligibleIndices = find(eligible);
        [~, bestCursor] = max(receiverPrediction(eligible));
        bestIdx = eligibleIndices(bestCursor);
        selected(receiverCursor) = true;
        selectedGain(receiverCursor) = receiverGain(bestIdx);
    end
end
summary = struct();
summary.threshold = threshold;
summary.minimumPositiveNeighborFraction = ...
    minimumPositiveNeighborFraction;
summary.meanSelectedGainFraction = mean(selectedGain);
summary.medianSelectedGainFraction = median(selectedGain);
summary.minimumSelectedGainFraction = min(selectedGain);
if any(selected)
    summary.negativeSelectionFraction = ...
        mean(selectedGain(selected) < 0);
else
    summary.negativeSelectionFraction = 0;
end
summary.receiverSelectionFraction = mean(selected);
summary.oracleCaptureFraction = sum(max(selectedGain, 0)) / ...
    max(sum(oracleGain), eps);
summary.selectedGainByReceiver = selectedGain;
summary.oracleGainByReceiver = oracleGain;
end

function gains = maxGainByReceiver(y, receivers)
uniqueReceivers = unique(receivers);
gains = zeros(1, numel(uniqueReceivers));
for receiverCursor = 1:numel(uniqueReceivers)
    gains(receiverCursor) = max([ ...
        0; y(receivers == uniqueReceivers(receiverCursor))]);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
