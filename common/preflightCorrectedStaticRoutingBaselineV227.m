function preflight = ...
        preflightCorrectedStaticRoutingBaselineV227(inputs)
% PREFLIGHTCORRECTEDSTATICROUTINGBASELINEV227 Structural matched baseline.

if ~isstruct(inputs) || ~isscalar(inputs) || ...
        ~all(isfield(inputs, {'config', 'model', 'graphData', ...
            'commConfig', 'seed'})) || ...
        ~all(isfield(inputs.graphData, {'staticAdjacency', ...
            'physicalAdjacency', 'positions'})) || ...
        ~isfield(inputs.commConfig, 'pDropByEdge')
    error('CorrectedStaticRoutingV227:InvalidPreflightInput', ...
        'V227 preflight requires one generated multistyle scene.');
end
protocol = getCorrectedStaticRoutingBaselineV227Protocol();
nodeCount = inputs.config.numberOfSensors;
timeCount = inputs.config.simulationLength;
registered = logical(inputs.graphData.staticAdjacency);
physicalPages = logical(inputs.graphData.physicalAdjacency);
registeredPages = repmat(registered, 1, 1, timeCount);
if ~isequal(size(registered), [nodeCount, nodeCount]) || ...
        size(physicalPages, 1) ~= nodeCount || ...
        size(physicalPages, 2) ~= nodeCount || ...
        size(physicalPages, 3) ~= timeCount || ...
        any(registeredPages(:) & ~physicalPages(:))
    error('CorrectedStaticRoutingV227:StaticGraphNotRobust', ...
        'The registered static graph is not all-time physical.');
end

identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
context = baseContext(inputs, identity, registered, protocol);
routeOptions = struct( ...
    'dominantWeight', protocol.dominantWeight, ...
    'residualWeight', protocol.residualWeight);

staticContext = context;
staticContext.currentTime = 1;
staticContext.positions = inputs.graphData.positions(:, :, 1);
staticContext.physicalAdjacency = registered;
staticContext.commConfig = struct( ...
    'pDropByEdge', inputs.commConfig.pDropByEdge(:, :, 1));
[staticAdjacency, staticDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy( ...
        staticContext, routeOptions);
staticWeights = staticDetails.fusionWeightMatrix;

dynamicPages = false(nodeCount, nodeCount, timeCount);
dynamicWeights = cell(1, timeCount);
for currentTime = 1:timeCount
    context.currentTime = currentTime;
    context.positions = inputs.graphData.positions(:, :, currentTime);
    context.physicalAdjacency = physicalPages(:, :, currentTime);
    context.commConfig = struct( ...
        'pDropByEdge', inputs.commConfig.pDropByEdge(:, :, currentTime));
    [dynamicPages(:, :, currentTime), dynamicDetails] = ...
        selectIndexEquivariantFormationBackbonePolicy( ...
            context, routeOptions);
    dynamicWeights{currentTime} = dynamicDetails.fusionWeightMatrix;
    assertWeightMultisetParity( ...
        dynamicDetails.fusionWeightMatrix, staticWeights);
end

expectedMessages = protocol.directedInputsPerReceiver * nodeCount;
dynamicCounts = pageCounts(dynamicPages);
if any(dynamicCounts ~= expectedMessages) || ...
        nnz(staticAdjacency) ~= expectedMessages
    error('CorrectedStaticRoutingV227:MessageParityFailed', ...
        'The V227 arms do not preserve the matched message budget.');
end

registration = struct();
registration.contractVersion = ...
    'corrected-static-routing-v227-registration-v1';
registration.adjacency = staticAdjacency;
registration.fusionWeightMatrix = staticWeights;
registration.selectionDetails = staticDetails;
registration.messageCount = nnz(staticAdjacency);
registration.selectionTime = 1;
registration.registeredStaticGraphUsed = true;
registration.currentPageLinkReliabilityUsed = true;
registration.targetTruthUsed = false;
registration.futurePhysicalPageUsed = false;

staticPages = repmat(staticAdjacency, 1, 1, timeCount);
preflight = struct();
preflight.contractVersion = ...
    'corrected-static-routing-v227-preflight-v1';
preflight.staticRegistration = registration;
preflight.staticAllTimePhysical = true;
preflight.staticRouteFixed = true;
preflight.messageParityPassed = true;
preflight.weightMultisetParityPassed = true;
preflight.messageCountPerRound = expectedMessages;
preflight.staticUniqueTopologyCount = 1;
preflight.dynamicUniqueTopologyCount = uniquePageCount(dynamicPages);
preflight.staticTopologyChangeCount = 0;
preflight.dynamicTopologyChangeCount = pageChangeCount(dynamicPages);
preflight.dynamicEquivalentToStatic = isequal(dynamicPages, staticPages);
preflight.meanDynamicStaticEdgeJaccard = ...
    meanPageJaccard(dynamicPages, staticPages);
preflight.staticAdjacency = staticAdjacency;
preflight.staticFusionWeightMatrix = staticWeights;
preflight.dynamicFusionWeightMatrixByTime = dynamicWeights;
preflight.receiverMode = protocol.receiverMode;
preflight.presetName = inputs.config.presetName;
preflight.seed = inputs.seed;
end

function context = baseContext(inputs, identity, registered, protocol)
nodeCount = inputs.config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.baseAdjacency = registered;
context.directedMessageBudget = ...
    protocol.directedInputsPerReceiver * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
end

function assertWeightMultisetParity(left, right)
if ~isequal(size(left), size(right))
    error('CorrectedStaticRoutingV227:WeightParityFailed', ...
        'The fusion-weight matrices have different dimensions.');
end
for receiverIdx = 1:size(left, 1)
    leftValues = sort(left(receiverIdx, left(receiverIdx, :) > 0));
    rightValues = sort(right(receiverIdx, right(receiverIdx, :) > 0));
    if ~isequal(size(leftValues), size(rightValues)) || ...
            any(abs(leftValues - rightValues) > 1e-12)
        error('CorrectedStaticRoutingV227:WeightParityFailed', ...
            'A receiver changed its positive fusion-weight multiset.');
    end
end
end

function counts = pageCounts(pages)
counts = zeros(1, size(pages, 3));
for timeIdx = 1:size(pages, 3)
    counts(timeIdx) = nnz(pages(:, :, timeIdx));
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

function value = meanPageJaccard(left, right)
values = zeros(1, size(left, 3));
for timeIdx = 1:size(left, 3)
    union = left(:, :, timeIdx) | right(:, :, timeIdx);
    overlap = left(:, :, timeIdx) & right(:, :, timeIdx);
    values(timeIdx) = nnz(overlap) / max(nnz(union), 1);
end
value = mean(values);
end
