function route = buildShieldBroadcastV102RouteSequence( ...
        referenceAdjacency, referenceWeights, referenceDetails, ...
        physicalAdjacency, groupIds, broadcastFormationIdsByTime, options)
% BUILDSHIELDBROADCASTV102ROUTESEQUENCE Direct protected-gateway transport.

if nargin < 7 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
referenceAdjacency = logical(referenceAdjacency);
physicalAdjacency = logical(physicalAdjacency);
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
horizonSteps = numel(broadcastFormationIdsByTime);
if ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(referenceWeights), [nodeCount, nodeCount]) || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        ~isstruct(referenceDetails) || ...
        ~isfield(referenceDetails, 'dominantSourcesByReceiver') || ...
        ~isfield(referenceDetails, 'residualSourcesByReceiver') || ...
        horizonSteps < 1
    error('ShieldBroadcastV102:InvalidRouteInput', ...
        'The reference route, physical graph or schedule is invalid.');
end

groups = unique(groupIds, 'stable');
dominantSources = reshape( ...
    referenceDetails.dominantSourcesByReceiver, 1, []);
residualSources = reshape( ...
    referenceDetails.residualSourcesByReceiver, 1, []);
if numel(dominantSources) ~= nodeCount || ...
        numel(residualSources) ~= nodeCount
    error('ShieldBroadcastV102:ReferenceDecompositionDrift', ...
        'The reference dominant/residual decomposition is invalid.');
end
gateways = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    members = reshape(find(groupIds == groups(groupIdx)), 1, []);
    candidates = zeros(1, 0);
    for receiverIdx = members
        residualSender = residualSources(receiverIdx);
        if groupIds(residualSender) ~= groups(groupIdx)
            candidates(end + 1) = receiverIdx; %#ok<AGROW>
        end
    end
    if numel(candidates) ~= 1
        error('ShieldBroadcastV102:GatewayDrift', ...
            'Every formation must expose exactly one cross-input gateway.');
    end
    gateways(groupIdx) = candidates(1);
end

adjacencySequence = repmat( ...
    referenceAdjacency, 1, 1, horizonSteps);
weightSequence = repmat(referenceWeights, 1, 1, horizonSteps);
changedReceiversByTime = cell(1, horizonSteps);
gatewayIndicesByTime = cell(1, horizonSteps);
for timeIdx = 1:horizonSteps
    formationIds = reshape( ...
        broadcastFormationIdsByTime{timeIdx}, 1, []);
    changed = zeros(1, 0);
    activeGateways = zeros(1, 0);
    for formationId = formationIds
        groupIdx = find(groups == formationId, 1);
        if isempty(groupIdx)
            error('ShieldBroadcastV102:UnknownFormation', ...
                'The broadcast schedule contains an unknown formation.');
        end
        gatewayIdx = gateways(groupIdx);
        activeGateways(end + 1) = gatewayIdx; %#ok<AGROW>
        members = reshape(find(groupIds == formationId), 1, []);
        for receiverIdx = members
            if receiverIdx == gatewayIdx
                continue;
            end
            dominantSender = dominantSources(receiverIdx);
            residualSender = residualSources(receiverIdx);
            if groupIds(dominantSender) ~= formationId || ...
                    ~physicalAdjacency(receiverIdx, gatewayIdx)
                error('ShieldBroadcastV102:NonphysicalBroadcast', ...
                    'A protected gateway cannot serve a formation peer.');
            end
            if dominantSender == gatewayIdx
                continue;
            end
            if dominantSender == residualSender
                combinedWeight = ...
                    referenceWeights(receiverIdx, dominantSender);
                adjacencySequence(receiverIdx, dominantSender, timeIdx) = ...
                    false;
                weightSequence(receiverIdx, dominantSender, timeIdx) = 0;
                adjacencySequence(receiverIdx, gatewayIdx, timeIdx) = true;
                weightSequence(receiverIdx, gatewayIdx, timeIdx) = ...
                    combinedWeight;
            elseif residualSender == gatewayIdx
                weightSequence(receiverIdx, dominantSender, timeIdx) = ...
                    residualWeight;
                weightSequence(receiverIdx, gatewayIdx, timeIdx) = ...
                    dominantWeight;
            else
                adjacencySequence(receiverIdx, residualSender, timeIdx) = ...
                    false;
                weightSequence(receiverIdx, residualSender, timeIdx) = 0;
                weightSequence(receiverIdx, dominantSender, timeIdx) = ...
                    residualWeight;
                adjacencySequence(receiverIdx, gatewayIdx, timeIdx) = true;
                weightSequence(receiverIdx, gatewayIdx, timeIdx) = ...
                    dominantWeight;
            end
            changed(end + 1) = receiverIdx; %#ok<AGROW>
        end
    end
    validateRouteParity(adjacencySequence(:, :, timeIdx), ...
        weightSequence(:, :, timeIdx), referenceAdjacency, ...
        referenceWeights, physicalAdjacency);
    changedReceiversByTime{timeIdx} = unique(changed, 'stable');
    gatewayIndicesByTime{timeIdx} = activeGateways;
end

route = struct();
route.contractVersion = ...
    'shield-broadcast-v102-route-sequence-v1';
route.referenceAdjacency = referenceAdjacency;
route.referenceFusionWeights = referenceWeights;
route.adjacencyByTime = adjacencySequence;
route.fusionWeightsByTime = weightSequence;
route.gatewayIndicesByFormation = gateways;
route.gatewayIndicesByTime = gatewayIndicesByTime;
route.changedReceiverIndicesByTime = changedReceiversByTime;
route.broadcastFormationIdsByTime = broadcastFormationIdsByTime;
route.truthUsed = false;
route.futureOutcomeUsed = false;
end

function validateRouteParity( ...
        adjacency, weights, referenceAdjacency, referenceWeights, physical)
support = adjacency | logical(eye(size(adjacency, 1)));
physicalPass = ~any(adjacency(:) & ~physical(:));
messagePass = nnz(adjacency) == nnz(referenceAdjacency);
rowCountPass = isequal(sum(adjacency, 2), sum(referenceAdjacency, 2));
multisetPass = rowWeightMultisetParity(weights, referenceWeights);
supportPass = ~any(weights(:) > 1e-12 & ~support(:));
finitePass = all(isfinite(weights(:))) && all(weights(:) >= -1e-12);
sumPass = all(abs(sum(weights, 2) - 1) <= 1e-12);
valid = physicalPass && messagePass && rowCountPass && multisetPass && ...
    supportPass && finitePass && sumPass;
if ~valid
    error('ShieldBroadcastV102:RouteParityFailure', ...
        ['The broadcast route violates its row contract ', ...
         '(physical=%d message=%d rows=%d weights=%d support=%d ', ...
         'finite=%d sum=%d).'], physicalPass, messagePass, ...
        rowCountPass, multisetPass, supportPass, finitePass, sumPass);
end
end

function pass = rowWeightMultisetParity(first, second)
pass = true;
for receiverIdx = 1:size(first, 1)
    a = sort(first(receiverIdx, first(receiverIdx, :) > 1e-12));
    b = sort(second(receiverIdx, second(receiverIdx, :) > 1e-12));
    if numel(a) ~= numel(b) || any(abs(a - b) > 1e-12)
        pass = false;
        return;
    end
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
