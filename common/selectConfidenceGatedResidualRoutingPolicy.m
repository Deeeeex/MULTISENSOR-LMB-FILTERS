function [adjacency, details] = ...
    selectConfidenceGatedResidualRoutingPolicy(context, modelOrPath)
% SELECTCONFIDENCEGATEDRESIDUALROUTINGPOLICY Support-gated joint policy.
%
% Every receiver starts from the registered reliability + equal-weight KLA
% action. A learned joint sender-weight action may replace it only when its
% calibrated lower confidence bound is positive and the feature vector is
% inside the training support box.

timerId = tic;
model = resolveModel(modelOrPath);
if ~strcmp(model.kind, ...
        'confidence-gated-residual-directed-kla')
    error('Unsupported residual-routing model kind: %s', model.kind);
end
[actionFeatures, featureNames, metadata] = ...
    computeScaleInvariantDirectedActionFeatures( ...
        context, model.sourceWeightGrid);
if ~isequal(featureNames, reshape(model.featureNames, 1, []))
    error('Residual-routing feature contract mismatch.');
end
if abs(metadata.backboneWeight - model.backboneWeight) > 1e-12
    error('Residual-routing backbone weight mismatch.');
end

nodeCount = size(actionFeatures, 1);
physical = logical(context.physicalAdjacency);
weightCount = numel(model.sourceWeightGrid);
featureCount = numel(model.featureNames);
predictedResidual = nan(nodeCount, nodeCount, weightCount);
lowerBound = nan(nodeCount, nodeCount, weightCount);
inSupport = false(nodeCount, nodeCount, weightCount);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        for weightIdx = 1:weightCount
            row = reshape(actionFeatures( ...
                receiverIdx, senderIdx, weightIdx, :), 1, []);
            if any(~isfinite(row))
                continue;
            end
            standardized = (row - model.featureMean) ./ ...
                model.featureScale;
            supported = all(standardized >= model.supportLower & ...
                standardized <= model.supportUpper);
            prediction = model.intercept + ...
                standardized * model.coefficient;
            predictedResidual(receiverIdx, senderIdx, weightIdx) = ...
                prediction;
            lowerBound(receiverIdx, senderIdx, weightIdx) = ...
                prediction - model.overestimateQuantile;
            inSupport(receiverIdx, senderIdx, weightIdx) = supported;
        end
    end
end

backboneFeatureIdx = find(strcmp( ...
    featureNames, 'backbone_action'), 1);
[~, backboneWeightIdx] = min(abs( ...
    model.sourceWeightGrid - model.backboneWeight));
selectedSources = nan(1, nodeCount);
selectedWeightIndices = nan(1, nodeCount);
selectedLowerBounds = zeros(1, nodeCount);
overrodeBackbone = false(1, nodeCount);
for receiverIdx = 1:nodeCount
    backboneSlice = reshape(actionFeatures( ...
        receiverIdx, :, backboneWeightIdx, ...
        backboneFeatureIdx), 1, []);
    backboneSender = find(backboneSlice > 0.5, 1);
    if isempty(backboneSender)
        continue;
    end
    selectedSources(receiverIdx) = backboneSender;
    selectedWeightIndices(receiverIdx) = backboneWeightIdx;

    localBound = reshape(lowerBound(receiverIdx, :, :), ...
        nodeCount, weightCount);
    localSupport = reshape(inSupport(receiverIdx, :, :), ...
        nodeCount, weightCount);
    localBackbone = reshape(actionFeatures( ...
        receiverIdx, :, :, backboneFeatureIdx), ...
        nodeCount, weightCount) > 0.5;
    eligible = localSupport & ~localBackbone & ...
        isfinite(localBound) & ...
        localBound > model.minimumResidual;
    if ~any(eligible(:))
        continue;
    end
    eligibleBound = localBound;
    eligibleBound(~eligible) = -inf;
    [bestBound, linearIdx] = max(eligibleBound(:));
    [senderIdx, weightIdx] = ind2sub( ...
        [nodeCount, weightCount], linearIdx);
    selectedSources(receiverIdx) = senderIdx;
    selectedWeightIndices(receiverIdx) = weightIdx;
    selectedLowerBounds(receiverIdx) = bestBound;
    overrodeBackbone(receiverIdx) = ...
        senderIdx ~= backboneSender || ...
        weightIdx ~= backboneWeightIdx;
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
activeReceivers = find(isfinite(selectedSources));
reliabilityFeatureIdx = find(strcmp( ...
    featureNames, 'link_reliability'), 1);
selectedPriority = -inf(1, nodeCount);
for receiverIdx = reshape(activeReceivers, 1, [])
    senderIdx = selectedSources(receiverIdx);
    weightIdx = selectedWeightIndices(receiverIdx);
    selectedPriority(receiverIdx) = actionFeatures( ...
        receiverIdx, senderIdx, weightIdx, reliabilityFeatureIdx);
end
if numel(activeReceivers) > messageBudget
    [~, order] = sort( ...
        selectedPriority(activeReceivers), 'descend');
    dropped = activeReceivers(order((messageBudget + 1):end));
    selectedSources(dropped) = NaN;
    selectedWeightIndices(dropped) = NaN;
    selectedLowerBounds(dropped) = -inf;
    selectedPriority(dropped) = -inf;
    overrodeBackbone(dropped) = false;
end

adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = reshape(find(isfinite(selectedSources)), 1, [])
    senderIdx = selectedSources(receiverIdx);
    weightIdx = selectedWeightIndices(receiverIdx);
    sourceWeight = model.sourceWeightGrid(weightIdx);
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

candidateMask = isfinite(predictedResidual);
details = struct();
details.mode = 'confidence-gated-residual-directed-kla';
details.objective = -sum(selectedLowerBounds( ...
    isfinite(selectedLowerBounds)));
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = nnz(candidateMask);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.predictedResidualTensor = predictedResidual;
details.lowerConfidenceBoundTensor = lowerBound;
details.inSupportTensor = inSupport;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.selectedLowerBoundsByReceiver = selectedLowerBounds;
details.selectedBudgetPriorityByReceiver = selectedPriority;
details.overrodeBackboneByReceiver = overrodeBackbone;
details.overrideFraction = mean(overrodeBackbone);
details.supportedCandidateFraction = ...
    nnz(inSupport & candidateMask) / max(nnz(candidateMask), 1);
details.messageCount = nnz(adjacency);
details.modelKind = model.kind;
details.overestimateQuantile = model.overestimateQuantile;
details.minimumResidual = model.minimumResidual;
details.calibrationMode = getField( ...
    model, 'calibrationMode', 'legacy-simultaneous-action');
end

function model = resolveModel(modelOrPath)
if isstruct(modelOrPath)
    model = modelOrPath;
    return;
end
if isempty(modelOrPath) || ~ischar(modelOrPath)
    error('A residual-routing model struct or MAT path is required.');
end
loaded = load(modelOrPath, 'model');
if ~isfield(loaded, 'model')
    error('Residual-routing model file does not contain model.');
end
model = loaded.model;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
