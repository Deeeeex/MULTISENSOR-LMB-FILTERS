function preflight = preflightStaticVsDynamicRoutingV91(inputs)
% PREFLIGHTSTATICVSDYNAMICROUTINGV91 Structural baseline comparison.

protocol = getStaticVsDynamicRoutingV91Protocol();
requiredFields = {'config', 'model', 'graphData'};
if ~isstruct(inputs) || ~isscalar(inputs) || ...
        ~all(isfield(inputs, requiredFields)) || ...
        ~isfield(inputs.graphData, 'staticAdjacency') || ...
        ~isfield(inputs.graphData, 'physicalAdjacency')
    error('StaticVsDynamicV91:InvalidPreflightInput', ...
        'V91 preflight requires one generated dynamic-topology scene.');
end
nodeCount = inputs.config.numberOfSensors;
timeCount = inputs.config.simulationLength;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
context.directedMessageBudget = 2 * nodeCount;
dynamicPages = false(nodeCount, nodeCount, timeCount);
staticPages = false(nodeCount, nodeCount, timeCount);
staticWeights = [];
dynamicWeights = [];
for currentTime = 1:timeCount
    context.currentTime = currentTime;
    context.physicalAdjacency = logical( ...
        inputs.graphData.physicalAdjacency(:, :, currentTime));
    [dynamicPages(:, :, currentTime), dynamicDetails] = ...
        selectStaticVsDynamicRoutingV91DynamicPolicy(context);
    [staticPages(:, :, currentTime), currentStaticDetails] = ...
        selectStaticVsDynamicRoutingV91StaticPolicy(context);
    if currentTime == 1
        staticWeights = currentStaticDetails.fusionWeightMatrix;
        dynamicWeights = dynamicDetails.fusionWeightMatrix;
    elseif ~isequal(staticPages(:, :, currentTime), ...
            staticPages(:, :, 1)) || ...
            ~isequal(currentStaticDetails.fusionWeightMatrix, staticWeights)
        error('StaticVsDynamicV91:StaticRouteChanged', ...
            'The registered V91 static route changed during the episode.');
    end
    assertWeightMultisetParity( ...
        dynamicDetails.fusionWeightMatrix, ...
        currentStaticDetails.fusionWeightMatrix);
end
dynamicCounts = pageCounts(dynamicPages);
staticCounts = pageCounts(staticPages);
if any(dynamicCounts ~= 2 * nodeCount) || ...
        any(staticCounts ~= 2 * nodeCount) || ...
        any(dynamicCounts ~= staticCounts)
    error('StaticVsDynamicV91:MessageParityFailed', ...
        'The V91 routes do not preserve the matched message budget.');
end
if ~isequal(dynamicPages(:, :, 1), staticPages(:, :, 1)) || ...
        ~isequal(dynamicWeights, staticWeights)
    error('StaticVsDynamicV91:InitialRouteMismatch', [ ...
        'The static control is not the exact frozen first-round ', ...
        'dynamic route.']);
end
preflight = struct();
preflight.contractVersion = ...
    'static-vs-dynamic-routing-v91-preflight-v1';
preflight.staticAllTimePhysical = true;
preflight.staticRouteFixed = true;
preflight.messageParityPassed = true;
preflight.weightMultisetParityPassed = true;
preflight.initialRouteAndWeightMatched = true;
preflight.messageCountPerRound = staticCounts(1);
preflight.staticUniqueTopologyCount = uniquePageCount(staticPages);
preflight.dynamicUniqueTopologyCount = uniquePageCount(dynamicPages);
preflight.dynamicTopologyChangeCount = pageChangeCount(dynamicPages);
preflight.staticTopologyChangeCount = pageChangeCount(staticPages);
preflight.dynamicEquivalentToFrozenStatic = ...
    isequal(dynamicPages, staticPages);
preflight.staticAdjacency = staticPages(:, :, 1);
preflight.staticFusionWeightMatrix = staticWeights;
preflight.dynamicInitialFusionWeightMatrix = dynamicWeights;
preflight.meanDynamicStaticEdgeJaccard = ...
    meanPageJaccard(dynamicPages, staticPages);
preflight.receiverMode = protocol.receiverMode;
end

function assertWeightMultisetParity(left, right)
if ~isequal(size(left), size(right))
    error('StaticVsDynamicV91:WeightParityFailed', ...
        'The V91 fusion-weight matrices have different dimensions.');
end
for receiverIdx = 1:size(left, 1)
    leftValues = sort(left(receiverIdx, left(receiverIdx, :) > 0));
    rightValues = sort(right(receiverIdx, right(receiverIdx, :) > 0));
    if ~isequal(size(leftValues), size(rightValues)) || ...
            any(abs(leftValues - rightValues) > 1e-12)
        error('StaticVsDynamicV91:WeightParityFailed', ...
            'A V91 receiver changed its positive fusion-weight multiset.');
    end
end
end

function count = uniquePageCount(pages)
flat = reshape(logical(pages), [], size(pages, 3))';
count = size(unique(flat, 'rows'), 1);
end

function count = pageChangeCount(pages)
count = 0;
for timeIdx = 2:size(pages, 3)
    count = count + ~isequal( ...
        pages(:, :, timeIdx), pages(:, :, timeIdx - 1));
end
end

function counts = pageCounts(pages)
counts = zeros(1, size(pages, 3));
for timeIdx = 1:size(pages, 3)
    counts(timeIdx) = nnz(pages(:, :, timeIdx));
end
end

function value = meanPageJaccard(left, right)
values = zeros(1, size(left, 3));
for timeIdx = 1:size(left, 3)
    union = left(:, :, timeIdx) | right(:, :, timeIdx);
    overlap = left(:, :, timeIdx) & right(:, :, timeIdx);
    values(timeIdx) = nnz(overlap) / max(nnz(union), 1);
end
value = mean(values);
end
