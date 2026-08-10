function [adjacency, details] = ...
    selectRepeatedMultiGatewayHandoverV89RuntimePolicy(context)
% SELECTREPEATEDMULTIGATEWAYHANDOVERV89RUNTIMEPOLICY Causal three-phase route.

persistent pendingGateways pendingAcquisitionTime lastTime lastNodeCount
protocol = getRepeatedMultiGatewayHandoverV89Protocol();
timer = tic;
[referenceAdjacency, referenceDetails] = ...
    selectPhysicalFormationTreeResidualTourPolicy(context, struct( ...
        'dominantWeight', protocol.dominantWeight, ...
        'residualWeight', protocol.sourceWeight));
referenceWeights = referenceDetails.fusionWeightMatrix;
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
currentTime = context.currentTime;

if isempty(lastTime) || currentTime <= lastTime || ...
        isempty(lastNodeCount) || nodeCount ~= lastNodeCount
    pendingGateways = repmat(emptyGateway(), 1, 0);
    pendingAcquisitionTime = NaN;
end
lastTime = currentTime;
lastNodeCount = nodeCount;

adjacency = referenceAdjacency;
weights = referenceWeights;
phase = 'reference';
phaseIndex = 2;
proposalCount = 0;
selected = repmat(emptyGateway(), 1, 0);
sourceIndices = zeros(1, 0);
objective = 0;
posteriorUsed = false;
currentLinkReliabilityUsed = false;
referenceFallbackUsed = false;
messageParityPassed = true;
[rollingSensorStrong, rollingFormationStrong] = ...
    rollingB3Pass(context.previousAdjacencyHistory, adjacency, groupIds);

active = currentTime >= protocol.activationStartTime && ...
    currentTime <= protocol.activationEndTime;
if active
    phaseIndex = mod(currentTime - protocol.activationStartTime, ...
        protocol.period);
    if phaseIndex == 0
        phase = 'acquire';
        posteriorUsed = true;
        currentLinkReliabilityUsed = true;
        metrics = computeRepeatedMultiGatewayHandoverNominationsV89( ...
            context, groupIds, protocol);
        proposed = metrics.selectedGateways;
        proposalCount = numel(proposed);
        [adjacency, weights, selected, messageParityPassed, ...
            rollingSensorStrong, rollingFormationStrong] = ...
            projectAcquireCandidates( ...
                proposed, referenceAdjacency, referenceWeights, ...
                context, groupIds);
        pendingGateways = selected;
        pendingAcquisitionTime = currentTime;
        objective = sumOrZero([selected.senderNoveltyFraction]);
        sourceIndices = reshape([selected.candidateSenderIdx], 1, []);
        referenceFallbackUsed = isempty(selected) && ~isempty(proposed);
    elseif phaseIndex == 1
        phase = 'broadcast';
        proposed = pendingGateways;
        proposalCount = numel(proposed);
        [adjacency, weights, selected, messageParityPassed, ...
            rollingSensorStrong, rollingFormationStrong] = ...
            projectBroadcastCandidates( ...
                proposed, referenceAdjacency, referenceWeights, ...
                context, groupIds, protocol);
        objective = sumOrZero([selected.senderNoveltyFraction]);
        sourceIndices = reshape([selected.candidateSenderIdx], 1, []);
        referenceFallbackUsed = isempty(selected) && ~isempty(proposed);
        pendingGateways = repmat(emptyGateway(), 1, 0);
    else
        pendingGateways = repmat(emptyGateway(), 1, 0);
        pendingAcquisitionTime = NaN;
    end
else
    pendingGateways = repmat(emptyGateway(), 1, 0);
    pendingAcquisitionTime = NaN;
end

gatewayIndices = reshape([selected.receiverIdx], 1, []);
formationIds = reshape([selected.receiverFormationId], 1, []);
crossMask = groupIds(:) ~= groupIds(:)';
details = referenceDetails;
details.contractVersion = ...
    'repeated-multi-gateway-handover-v89-runtime-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.candidateArmId;
details.objective = objective;
details.candidateIndex = 1;
if strcmp(phase, 'acquire') && ~isempty(selected)
    details.candidateIndex = 2;
elseif strcmp(phase, 'broadcast') && ~isempty(selected)
    details.candidateIndex = 3;
end
details.selectionSeconds = toc(timer);
details.fusionWeightMatrix = weights;
details.proposalCrossCount = proposalCount;
details.crossFormationMessageCount = nnz(adjacency & crossMask);
details.sourceWeight = protocol.sourceWeight;
details.posteriorUsed = posteriorUsed;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = currentLinkReliabilityUsed;
details.backboneMode = ...
    'current-physical-tree-with-repeated-multi-gateway-handover';
details.sensorWindowMature = ...
    size(context.previousAdjacencyHistory, 3) >= 2;
details.formationWindowMature = details.sensorWindowMature;
details.sensorWindowStrongConnected = rollingSensorStrong;
details.formationWindowStrongConnected = rollingFormationStrong;
details.referenceMessageCount = nnz(referenceAdjacency);
details.currentMessageCount = nnz(adjacency);
details.referenceFallbackUsed = referenceFallbackUsed;
details.scheduleCertificate = struct( ...
    'contractVersion', ...
        'repeated-multi-gateway-handover-v89-schedule-v1', ...
    'currentTime', currentTime, ...
    'phase', phase, ...
    'phaseIndex', phaseIndex, ...
    'gatewayIndices', gatewayIndices, ...
    'formationIds', formationIds, ...
    'sourceIndices', sourceIndices, ...
    'proposalGatewayCount', proposalCount, ...
    'selectedGatewayCount', numel(selected), ...
    'pendingAcquisitionTime', pendingAcquisitionTime, ...
    'messageParityPassed', messageParityPassed, ...
    'rollingSensorStrong', rollingSensorStrong, ...
    'rollingFormationStrong', rollingFormationStrong, ...
    'referenceFallbackUsed', referenceFallbackUsed, ...
    'cycleSelected', ~isempty(selected));
end

function [adjacency, weights, selected, parity, sensorStrong, ...
        formationStrong] = projectAcquireCandidates( ...
        proposed, referenceAdjacency, referenceWeights, context, groupIds)
adjacency = referenceAdjacency;
weights = referenceWeights;
selected = repmat(emptyGateway(), 1, 0);
parity = true;
[sensorStrong, formationStrong] = ...
    rollingB3Pass(context.previousAdjacencyHistory, adjacency, groupIds);
for candidate = proposed
    trialAdjacency = adjacency;
    trialWeights = weights;
    receiver = candidate.receiverIdx;
    incumbent = candidate.incumbentSenderIdx;
    source = candidate.candidateSenderIdx;
    if ~trialAdjacency(receiver, incumbent) || ...
            trialAdjacency(receiver, source) || ...
            ~context.physicalAdjacency(receiver, source)
        continue;
    end
    sourceWeight = trialWeights(receiver, incumbent);
    trialAdjacency(receiver, incumbent) = false;
    trialWeights(receiver, incumbent) = 0;
    trialAdjacency(receiver, source) = true;
    trialWeights(receiver, source) = sourceWeight;
    [valid, trialParity, trialSensorStrong, trialFormationStrong] = ...
        validateCandidateRoute( ...
            trialAdjacency, trialWeights, referenceAdjacency, ...
            referenceWeights, context.physicalAdjacency, ...
            context.previousAdjacencyHistory, groupIds);
    if valid
        adjacency = trialAdjacency;
        weights = trialWeights;
        selected(end + 1) = candidate; %#ok<AGROW>
        parity = trialParity;
        sensorStrong = trialSensorStrong;
        formationStrong = trialFormationStrong;
    end
end
end

function [adjacency, weights, selected, parity, sensorStrong, ...
        formationStrong] = projectBroadcastCandidates( ...
        proposed, referenceAdjacency, referenceWeights, context, ...
        groupIds, protocol)
adjacency = referenceAdjacency;
weights = referenceWeights;
selected = repmat(emptyGateway(), 1, 0);
parity = true;
[sensorStrong, formationStrong] = ...
    rollingB3Pass(context.previousAdjacencyHistory, adjacency, groupIds);
for candidate = proposed
    gateway = candidate.receiverIdx;
    formationId = candidate.receiverFormationId;
    members = reshape(find(groupIds == formationId), 1, []);
    receivers = setdiff(members, gateway, 'stable');
    if numel(members) ~= protocol.expectedFormationSize || ...
            ~all(context.physicalAdjacency(receivers, gateway))
        continue;
    end
    trialAdjacency = adjacency;
    trialWeights = weights;
    rowValid = true;
    for receiver = receivers
        dominant = find(trialAdjacency(receiver, :) & ...
            groupIds == formationId & ...
            abs(trialWeights(receiver, :) - ...
                protocol.dominantWeight) <= 1e-12);
        residual = find(trialAdjacency(receiver, :) & ...
            abs(trialWeights(receiver, :) - ...
                protocol.sourceWeight) <= 1e-12);
        if numel(dominant) ~= 1 || numel(residual) ~= 1
            rowValid = false;
            break;
        end
        dominant = dominant(1);
        residual = residual(1);
        if gateway == dominant
            continue;
        elseif gateway == residual
            trialWeights(receiver, dominant) = protocol.sourceWeight;
            trialWeights(receiver, gateway) = protocol.dominantWeight;
        else
            trialAdjacency(receiver, residual) = false;
            trialWeights(receiver, residual) = 0;
            trialWeights(receiver, dominant) = protocol.sourceWeight;
            trialAdjacency(receiver, gateway) = true;
            trialWeights(receiver, gateway) = protocol.dominantWeight;
        end
    end
    if ~rowValid
        continue;
    end
    [valid, trialParity, trialSensorStrong, trialFormationStrong] = ...
        validateCandidateRoute( ...
            trialAdjacency, trialWeights, referenceAdjacency, ...
            referenceWeights, context.physicalAdjacency, ...
            context.previousAdjacencyHistory, groupIds);
    if valid
        adjacency = trialAdjacency;
        weights = trialWeights;
        selected(end + 1) = candidate; %#ok<AGROW>
        parity = trialParity;
        sensorStrong = trialSensorStrong;
        formationStrong = trialFormationStrong;
    end
end
end

function [valid, parity, sensorStrong, formationStrong] = ...
        validateCandidateRoute( ...
        adjacency, weights, referenceAdjacency, referenceWeights, ...
        physical, previousHistory, groupIds)
support = adjacency | logical(eye(size(adjacency, 1)));
parity = nnz(adjacency) == nnz(referenceAdjacency) && ...
    isequal(sum(adjacency, 2), sum(referenceAdjacency, 2)) && ...
    rowWeightMultisetParity(weights, referenceWeights) && ...
    ~any(adjacency(:) & ~logical(physical(:))) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(isfinite(weights(:))) && all(weights(:) >= -1e-12) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
[sensorStrong, formationStrong] = ...
    rollingB3Pass(previousHistory, adjacency, groupIds);
valid = parity && sensorStrong && formationStrong;
end

function pass = rowWeightMultisetParity(first, second)
pass = true;
for receiver = 1:size(first, 1)
    a = sort(first(receiver, first(receiver, :) > 1e-12));
    b = sort(second(receiver, second(receiver, :) > 1e-12));
    if numel(a) ~= numel(b) || any(abs(a - b) > 1e-12)
        pass = false;
        return;
    end
end
end

function [sensorStrong, formationStrong] = ...
        rollingB3Pass(previousHistory, adjacency, groupIds)
history = logical(previousHistory);
if size(history, 3) > 2
    history = history(:, :, end-1:end);
end
pages = cat(3, history, logical(adjacency));
window = any(pages(:, :, max(1, end-2):end), 3);
sensorStrong = isStronglyConnected(window);
formationStrong = isStronglyConnected( ...
    collapseToFormations(window, groupIds));
end

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(groupIds, 'stable');
formation = false(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIdx);
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        formation(receiverGroupIdx, senderGroupIdx) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formation(1:numel(groups)+1:end) = false;
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function connected = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
connected = all(visited);
end

function value = sumOrZero(values)
values = reshape(values, 1, []);
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = sum(values);
end
end

function value = emptyGateway()
value = struct( ...
    'receiverIdx', NaN, ...
    'receiverFormationId', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'incumbentFormationId', NaN, ...
    'candidateSenderIdx', NaN, ...
    'candidateFormationId', NaN, ...
    'linkReliability', NaN, ...
    'senderNoveltyFraction', NaN, ...
    'protectedSupportDeficitFraction', NaN, ...
    'maximumNovelAssociationSupport', NaN, ...
    'broadcastReceiverIndices', zeros(1, 0), ...
    'fullFormationBroadcastPhysical', false, ...
    'actionEnabled', false, ...
    'rankingScore', 0);
end
