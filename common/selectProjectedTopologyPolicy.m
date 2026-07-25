function [adjacency, details] = ...
    selectProjectedTopologyPolicy(context, mode)
% SELECTPROJECTEDTOPOLOGYPOLICY Scalable analytic candidate-pool baseline.
%
% reliability  - maximize expected bidirectional delivery
% discrepancy  - maximize reliable posterior discrepancy across active edges
%
% This policy is intended for projected M24/X36 pools. D12 keeps its exact
% registered selector for backward-compatible comparisons.

if nargin < 2 || isempty(mode)
    mode = 'discrepancy';
end
mode = lower(char(mode));
if ~any(strcmp(mode, {'reliability', 'discrepancy'}))
    error('Unknown projected topology policy mode: %s', mode);
end
timerId = tic;
[candidates, metadata] = ...
    buildDynamicTopologyCandidatePool(context);
candidateCount = size(candidates, 3);
objectives = inf(1, candidateCount);
valid = false(1, candidateCount);
switchCosts = inf(1, candidateCount);
byteMismatch = inf(1, candidateCount);
nodePayloadBytes = estimateNodePayloadBytes( ...
    context.localPosteriorBySensor, context.model);
referenceBytes = topologyAttemptedBytes( ...
    context.baseAdjacency, nodePayloadBytes);
scenarioConfig = context.model.dynamicTopologyScenario.config;
byteTolerance = getField(scenarioConfig, ...
    'attemptedByteToleranceFraction', 0.02);
maxReplacements = getField(scenarioConfig, ...
    'maxEdgeReplacementsPerStep', inf);

for candidateIdx = 1:candidateCount
    candidate = candidates(:, :, candidateIdx);
    if any(candidate(:) & ~context.physicalAdjacency(:)) || ...
            countEdges(candidate) > context.edgeBudget || ...
            ~isConnected(candidate)
        continue;
    end
    switchCosts(candidateIdx) = countRemovedEdges( ...
        context.previousAdjacency, candidate);
    if any(context.previousAdjacency(:)) && ...
            switchCosts(candidateIdx) > maxReplacements
        continue;
    end
    candidateBytes = topologyAttemptedBytes( ...
        candidate, nodePayloadBytes);
    byteMismatch(candidateIdx) = abs( ...
        candidateBytes - referenceBytes) / max(referenceBytes, 1);
    if byteMismatch(candidateIdx) > byteTolerance + 1e-12
        continue;
    end
    valid(candidateIdx) = true;
    switch mode
        case 'reliability'
            value = topologyReliability( ...
                candidate, context.commConfig, context.currentTime);
        case 'discrepancy'
            value = topologyDiscrepancyValue( ...
                candidate, context.localPosteriorBySensor, ...
                context.model, context.commConfig, ...
                context.currentTime);
    end
    objectives(candidateIdx) = -value - ...
        0.05 * algebraicConnectivity(candidate) + ...
        0.02 * switchCosts(candidateIdx);
end

if ~any(valid)
    adjacency = context.baseAdjacency & context.physicalAdjacency;
    details = struct( ...
        'objective', inf, ...
        'candidateIndex', NaN, ...
        'mode', mode, ...
        'validCandidateCount', 0, ...
        'candidateSource', metadata.source, ...
        'selectionSeconds', toc(timerId));
    return;
end
[objective, selectedIdx] = min(objectives);
adjacency = candidates(:, :, selectedIdx);
details = struct( ...
    'objective', objective, ...
    'candidateIndex', selectedIdx, ...
    'mode', mode, ...
    'validCandidateCount', sum(valid), ...
    'candidateSource', metadata.source, ...
    'switchCost', switchCosts(selectedIdx), ...
    'attemptedByteMismatchFraction', byteMismatch(selectedIdx), ...
    'selectionSeconds', toc(timerId));
end

function value = topologyReliability(adjacency, commConfig, currentTime)
edges = find(triu(adjacency, 1));
value = 0;
for edgeCursor = 1:numel(edges)
    [leftIdx, rightIdx] = ind2sub(size(adjacency), edges(edgeCursor));
    value = value + 1 - 0.5 * ( ...
        edgeDrop(commConfig, leftIdx, rightIdx, currentTime) + ...
        edgeDrop(commConfig, rightIdx, leftIdx, currentTime));
end
end

function value = topologyDiscrepancyValue( ...
    adjacency, posteriors, model, commConfig, currentTime)
edges = find(triu(adjacency, 1));
value = 0;
for edgeCursor = 1:numel(edges)
    [leftIdx, rightIdx] = ind2sub(size(adjacency), edges(edgeCursor));
    reliability = 1 - 0.5 * ( ...
        edgeDrop(commConfig, leftIdx, rightIdx, currentTime) + ...
        edgeDrop(commConfig, rightIdx, leftIdx, currentTime));
    value = value + reliability * posteriorDisagreement( ...
        posteriors{leftIdx}, posteriors{rightIdx}, model);
end
end

function value = posteriorDisagreement(leftObjects, rightObjects, model)
labels = collectLabels(leftObjects, rightObjects);
if isempty(labels)
    value = 0;
    return;
end
terms = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    left = findLabel(leftObjects, labels(:, labelIdx));
    right = findLabel(rightObjects, labels(:, labelIdx));
    leftExistence = objectExistence(left);
    rightExistence = objectExistence(right);
    existenceTerm = abs(leftExistence - rightExistence);
    spatialTerm = 0;
    if ~isempty(left) && ~isempty(right)
        [leftMean, leftCovariance] = objectMoments( ...
            left, model.xDimension);
        [rightMean, rightCovariance] = objectMoments( ...
            right, model.xDimension);
        scale = sqrt(max(trace( ...
            leftCovariance(1:2, 1:2) + ...
            rightCovariance(1:2, 1:2)), 1));
        spatialTerm = min(norm( ...
            leftMean(1:2) - rightMean(1:2)) / scale, 5);
    end
    terms(labelIdx) = existenceTerm + ...
        min(leftExistence, rightExistence) * spatialTerm;
end
value = mean(terms);
end

function labels = collectLabels(leftObjects, rightObjects)
labels = zeros(2, 0);
collections = {leftObjects, rightObjects};
for collectionIdx = 1:2
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

function object = findLabel(objects, label)
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

function value = objectExistence(object)
if isempty(object)
    value = 0;
else
    value = min(max(object.r, 0), 1);
end
end

function [meanVector, covariance] = objectMoments( ...
    object, stateDimension)
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

function value = algebraicConnectivity(adjacency)
degree = sum(adjacency, 2);
eigenvalues = sort(real(eig(diag(degree) - double(adjacency))));
if numel(eigenvalues) < 2
    value = 0;
else
    value = max(eigenvalues(2), 0);
end
end

function bytes = estimateNodePayloadBytes(posteriors, model)
bytes = zeros(1, numel(posteriors));
for sensorIdx = 1:numel(posteriors)
    stats = estimateLmbPayloadSize( ...
        posteriors{sensorIdx}, model, 2);
    bytes(sensorIdx) = stats.estimatedBytes;
end
end

function bytes = topologyAttemptedBytes(adjacency, nodePayloadBytes)
bytes = sum(sum(adjacency, 2)' .* nodePayloadBytes);
end

function probability = edgeDrop(config, senderIdx, receiverIdx, currentTime)
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

function count = countRemovedEdges(previous, current)
if isempty(previous) || ~any(previous(:))
    count = 0;
else
    count = nnz(triu(previous & ~current, 1));
end
end

function count = countEdges(adjacency)
count = nnz(triu(adjacency, 1));
end

function connected = isConnected(adjacency)
nodeCount = size(adjacency, 1);
if nodeCount <= 1
    connected = true;
    return;
end
visited = false(1, nodeCount);
queue = 1;
visited(1) = true;
while ~isempty(queue)
    node = queue(1);
    queue(1) = [];
    neighbors = find(adjacency(node, :) & ~visited);
    visited(neighbors) = true;
    queue = [queue, neighbors]; %#ok<AGROW>
end
connected = all(visited);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
