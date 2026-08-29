function [adjacency, details] = ...
    selectSpatialLabelSupportDebtRoutingV147Policy( ...
        context, anchorTime, referenceCarrierMode, actionSpec)
% SELECTSPATIALLABELSUPPORTDEBTROUTINGV147POLICY Frozen safe graph pulses.

protocol = getSpatialLabelSupportDebtRoutingV147Protocol();
timerId = tic;
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, referenceCarrierMode, protocol.routeWeights);
referenceWeights = referenceDetails.fusionWeightMatrix;
relativeTime = context.currentTime - anchorTime + 1;
activeRequested = ismember(relativeTime, actionSpec.activeOffsets);
adjacency = logical(referenceAdjacency);
weights = referenceWeights;
fallbackReason = '';
candidateBytes = NaN;
referenceBytes = NaN;
if activeRequested
    trialAdjacency = logical(actionSpec.adjacency);
    trialWeights = actionSpec.fusionWeights;
    senderBytes = estimateSenderPayloadBytes( ...
        context.localPosteriorBySensor, context.model);
    referenceBytes = topologyPayloadBytes( ...
        referenceAdjacency, senderBytes);
    candidateBytes = topologyPayloadBytes(trialAdjacency, senderBytes);
    valid = isequal(size(trialAdjacency), ...
            size(context.physicalAdjacency)) && ...
        ~any(trialAdjacency(:) & ...
            ~logical(context.physicalAdjacency(:))) && ...
        nnz(trialAdjacency) == nnz(referenceAdjacency) && ...
        validWeights(trialAdjacency, trialWeights) && ...
        isStronglyConnected(trialAdjacency) && ...
        candidateBytes <= referenceBytes + 1e-9;
    if valid
        adjacency = trialAdjacency;
        weights = trialWeights;
    else
        fallbackReason = 'runtime-physical-weight-connectivity-or-byte-gate';
    end
end
details = referenceDetails;
details.contractVersion = ...
    'v147-spatial-label-support-debt-routing-policy-v1';
details.protocolId = protocol.id;
details.armId = actionSpec.name;
details.fusionWeightMatrix = weights;
details.objective = actionSpec.objective;
details.candidateIndex = actionSpec.index;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = activeRequested;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = false;
details.referenceMessageCount = nnz(referenceAdjacency);
details.currentMessageCount = nnz(adjacency);
details.referenceFallbackUsed = ...
    activeRequested && ~isempty(fallbackReason);
details.backboneMode = ...
    'fixed-carrier-with-spatial-label-support-debt-row-pulses';
details.selectionSeconds = toc(timerId);
details.scheduleCertificate = struct( ...
    'contractVersion', 'v147-spatial-debt-schedule-v1', ...
    'currentTime', context.currentTime, ...
    'relativeTime', relativeTime, ...
    'activeRequested', activeRequested, ...
    'activeExecuted', activeRequested && isempty(fallbackReason), ...
    'selectedFormationIds', actionSpec.selectedFormationIds, ...
    'referencePayloadBytes', referenceBytes, ...
    'candidatePayloadBytes', candidateBytes, ...
    'fallbackReason', fallbackReason);
end

function bytes = estimateSenderPayloadBytes(posteriors, model)
bytes = zeros(1, numel(posteriors));
for sensorIdx = 1:numel(posteriors)
    stats = estimateLmbPayloadSize( ...
        posteriors{sensorIdx}, model, 2, struct());
    bytes(sensorIdx) = stats.estimatedBytes;
end
end

function bytes = topologyPayloadBytes(adjacency, senderPayloadBytes)
bytes = sum(sum(logical(adjacency), 1) .* senderPayloadBytes);
end

function valid = validWeights(adjacency, weights)
support = logical(adjacency) | logical(eye(size(adjacency, 1)));
valid = isequal(size(weights), size(adjacency)) && ...
    all(isfinite(weights(:))) && all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachesAll(senderAdjacency) && reachesAll(senderAdjacency');
end

function connected = reachesAll(adjacency)
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
