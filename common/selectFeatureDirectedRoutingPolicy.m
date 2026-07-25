function [adjacency, details] = ...
    selectFeatureDirectedRoutingPolicy(context, modelOrPath)
% SELECTFEATUREDIRECTEDROUTINGPOLICY Deployment-observable directed routing.
%
% The policy predicts the value of sender -> receiver fusion from local LMB,
% geometry, formation, and link features. It never reads ground truth. Each
% receiver accepts at most one source, and can reject all communication when
% the predicted gain does not clear the receiver-held-out threshold.

timerId = tic;
if nargin < 2 || isempty(modelOrPath)
    modelOrPath = getField( ...
        context.triggerConfig, 'topologyDirectedRoutingModel', []);
    if isempty(modelOrPath)
        modelOrPath = getField( ...
            context.triggerConfig, ...
            'topologyDirectedRoutingModelPath', '');
    end
end
model = resolveModel(modelOrPath);
if ~strcmp(model.kind, ...
        'receiver-held-out-knn-directed-routing')
    error('Unsupported directed-routing model kind: %s', model.kind);
end

[featureTensor, availableNames] = ...
    computeDirectedRoutingFeatures(context);
featureIndices = resolveFeatureIndices( ...
    availableNames, model.featureNames);
nodeCount = size(featureTensor, 1);
physical = logical(context.physicalAdjacency);
predictedGain = nan(nodeCount);
predictedWeight = nan(nodeCount);
positiveNeighborConfidence = nan(nodeCount);
[receiverIndices, senderIndices] = find(physical);
candidateCount = numel(receiverIndices);
candidateFeatures = zeros( ...
    candidateCount, numel(featureIndices));
for candidateIdx = 1:candidateCount
    candidateFeatures(candidateIdx, :) = reshape(featureTensor( ...
        receiverIndices(candidateIdx), senderIndices(candidateIdx), ...
        featureIndices), 1, []);
end
standardized = bsxfun(@rdivide, bsxfun(@minus, ...
    candidateFeatures, model.featureMean), model.featureScale);
[candidateGain, candidateWeight, candidateConfidence] = ...
    predictKnnBatch(model, standardized);
candidateLinear = sub2ind( ...
    [nodeCount, nodeCount], receiverIndices, senderIndices);
predictedGain(candidateLinear) = candidateGain;
predictedWeight(candidateLinear) = candidateWeight;
positiveNeighborConfidence(candidateLinear) = candidateConfidence;
if any(~isfinite(candidateGain)) || ...
        any(~isfinite(candidateConfidence))
    error('Directed-routing model produced a non-finite prediction.');
end

selectedSources = nan(1, nodeCount);
selectedWeights = zeros(1, nodeCount);
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
for receiverIdx = 1:nodeCount
    eligible = predictedGain(receiverIdx, :) >= ...
        model.gainThreshold & ...
        positiveNeighborConfidence(receiverIdx, :) >= ...
        model.minimumPositiveNeighborFraction;
    if ~any(eligible)
        continue;
    end
    eligibleSources = find(eligible);
    [bestGain, bestCursor] = max( ...
        predictedGain(receiverIdx, eligible));
    senderIdx = eligibleSources(bestCursor);
    sourceWeight = predictedWeight(receiverIdx, senderIdx);
    sourceWeight = min(max(sourceWeight, ...
        model.minimumSourceWeight), model.maximumSourceWeight);
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedSources(receiverIdx) = senderIdx;
    selectedWeights(receiverIdx) = sourceWeight;
end

messageBudget = getField( ...
    context, 'directedMessageBudget', inf);
selectedReceivers = find(isfinite(selectedSources));
if numel(selectedReceivers) > messageBudget
    selectedGains = zeros(1, numel(selectedReceivers));
    for receiverCursor = 1:numel(selectedReceivers)
        receiverIdx = selectedReceivers(receiverCursor);
        selectedGains(receiverCursor) = ...
            predictedGain(receiverIdx, selectedSources(receiverIdx));
    end
    [~, keepOrder] = sort(selectedGains, 'descend');
    dropped = selectedReceivers( ...
        keepOrder((messageBudget + 1):end));
    for receiverIdx = reshape(dropped, 1, [])
        senderIdx = selectedSources(receiverIdx);
        adjacency(receiverIdx, senderIdx) = false;
        fusionWeights(receiverIdx, :) = 0;
        fusionWeights(receiverIdx, receiverIdx) = 1;
        selectedSources(receiverIdx) = NaN;
        selectedWeights(receiverIdx) = 0;
    end
end

selectedReceivers = reshape( ...
    find(isfinite(selectedSources)), 1, []);
selectedSenders = selectedSources(selectedReceivers);
selectedLinear = sub2ind( ...
    [nodeCount, nodeCount], selectedReceivers, selectedSenders);
selectedPredictedGain = predictedGain(selectedLinear);
details = struct();
details.mode = 'feature-directed-routing';
details.objective = -sum(selectedPredictedGain);
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = nnz(adjacency);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.predictedGainMatrix = predictedGain;
details.predictedWeightMatrix = predictedWeight;
details.positiveNeighborConfidenceMatrix = ...
    positiveNeighborConfidence;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.selectedPredictedGain = selectedPredictedGain;
details.messageCount = nnz(adjacency);
details.modelKind = model.kind;
details.modelGainThreshold = model.gainThreshold;
details.modelMinimumPositiveNeighborFraction = ...
    model.minimumPositiveNeighborFraction;
end

function model = resolveModel(modelOrPath)
if isstruct(modelOrPath)
    model = modelOrPath;
    return;
end
if isempty(modelOrPath) || ~ischar(modelOrPath)
    error('A directed-routing model struct or MAT path is required.');
end
loaded = load(modelOrPath, 'model');
if ~isfield(loaded, 'model')
    error('Directed-routing model file does not contain model.');
end
model = loaded.model;
end

function indices = resolveFeatureIndices(availableNames, requiredNames)
indices = zeros(1, numel(requiredNames));
for requiredIdx = 1:numel(requiredNames)
    match = find(strcmp( ...
        availableNames, requiredNames{requiredIdx}), 1);
    if isempty(match)
        error('Missing directed-routing feature: %s', ...
            requiredNames{requiredIdx});
    end
    indices(requiredIdx) = match;
end
end

function [gain, sourceWeight, positiveConfidence] = ...
    predictKnnBatch(model, standardizedRows)
training = model.trainingFeatures;
queryNorm = sum(standardizedRows.^2, 2);
trainingNorm = sum(training.^2, 2)';
distanceSquared = bsxfun(@plus, queryNorm, trainingNorm) - ...
    2 * (standardizedRows * training');
distanceSquared = max(distanceSquared, 0);
[sortedDistance, order] = sort(distanceSquared, 2, 'ascend');
neighborCount = min(model.neighborCount, size(order, 2));
order = order(:, 1:neighborCount);
sortedDistance = sortedDistance(:, 1:neighborCount);
weights = 1 ./ max(sortedDistance, 1e-6);
weights = bsxfun(@rdivide, weights, sum(weights, 2));
gainTargets = reshape(model.trainingGainTargets, [], 1);
weightTargets = reshape(model.trainingWeightTargets, [], 1);
localGain = reshape(gainTargets(order), size(order));
gain = sum(weights .* localGain, 2);
positiveConfidence = sum(weights .* (localGain > 0), 2);
localWeight = reshape(weightTargets(order), size(order));
validWeight = isfinite(localWeight);
localWeight(~validWeight) = 0;
validWeightedMass = sum(weights .* validWeight, 2);
sourceWeight = 0.5 * ( ...
    model.minimumSourceWeight + model.maximumSourceWeight) * ...
    ones(size(gain));
hasWeight = validWeightedMass > eps;
sourceWeight(hasWeight) = sum( ...
    weights(hasWeight, :) .* ...
    localWeight(hasWeight, :), 2) ./ ...
    validWeightedMass(hasWeight);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
