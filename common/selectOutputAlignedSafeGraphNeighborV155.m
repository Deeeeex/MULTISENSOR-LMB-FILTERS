function [adjacency, details] = ...
    selectOutputAlignedSafeGraphNeighborV155( ...
        context, candidateOrdinal, options)
% SELECTOUTPUTALIGNEDSAFEGRAPHNEIGHBORV155 Hold one canonical neighbor graph.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getOutputAlignedSafeGraphNeighborhoodV155Protocol());
candidateOrdinal = round(candidateOrdinal);
nodeCount = numel(context.localPosteriorBySensor);
presetIdx = find(protocol.nodeCounts == nodeCount);
if numel(presetIdx) ~= 1 || ...
        ~isscalar(candidateOrdinal) || ...
        ~isfinite(candidateOrdinal) || ...
        candidateOrdinal < 1 || ...
        candidateOrdinal > protocol.candidateCounts(presetIdx)
    error('V155 graph-neighborhood candidate is outside the bank.');
end
anchorTime = protocol.anchorTimes(presetIdx);
if context.currentTime < anchorTime || ...
        context.currentTime >= anchorTime + protocol.horizon
    error('V155 graph-neighborhood action is outside its continuation.');
end

fallbackUsed = false;
if context.currentTime == anchorTime
    bank = enumerateOutputAlignedSafeGraphNeighborhoodV155( ...
        context, struct('protocol', protocol));
    adjacency = bank.candidateAdjacency(:, :, candidateOrdinal);
    details = bank.candidateDetails{candidateOrdinal};
    heldGraphReused = false;
else
    try
        [adjacency, details] = rebuildHeldGraph( ...
            context, protocol, presetIdx);
        heldGraphReused = true;
    catch errorInfo
        if ~strcmp(errorInfo.identifier, ...
                'SafeGraphNeighborhoodV155:UnsafeHold')
            rethrow(errorInfo);
        end
        [adjacency, details] = ...
            selectBackboneResidualSplicedCyclePolicy( ...
                context, protocol.fallbackMode, struct( ...
                    'dominantWeight', protocol.dominantWeight, ...
                    'residualWeight', protocol.residualWeight));
        details.v155FallbackReason = errorInfo.message;
        heldGraphReused = false;
        fallbackUsed = true;
    end
end

if nnz(adjacency) ~= ...
        protocol.exactSelectedMessageCount(presetIdx)
    error('V155 graph-neighborhood changed the exact message count.');
end
details.mode = 'output-aligned-safe-graph-neighborhood-v155';
details.v155ProtocolId = protocol.id;
details.v155ContractVersion = protocol.contractVersion;
details.v155CandidateOrdinal = candidateOrdinal;
details.v155AnchorTime = anchorTime;
details.v155HeldGraphReused = heldGraphReused;
details.v155FallbackUsed = fallbackUsed;
details.v155ExpectedSelectedMessageCount = ...
    protocol.exactSelectedMessageCount(presetIdx);
details.repairTriggered = fallbackUsed;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureOutcomeUsed = false;
end

function [adjacency, details] = ...
    rebuildHeldGraph(context, protocol, presetIdx)
nodeCount = numel(context.localPosteriorBySensor);
previous = logical(context.previousAdjacency);
physical = logical(context.physicalAdjacency);
if ~isequal(size(previous), [nodeCount, nodeCount]) || ...
        nnz(previous) ~= ...
            protocol.exactSelectedMessageCount(presetIdx) || ...
        any(previous(:) & ~physical(:))
    error('SafeGraphNeighborhoodV155:UnsafeHold', ...
        'The selected neighbor graph is no longer physical.');
end
[~, ~, candidates] = enumerateBackboneResidualSpliceCandidates( ...
    context, struct( ...
        'dominantWeight', protocol.dominantWeight, ...
        'residualWeight', protocol.residualWeight));
dominantAdjacency = candidates.dominantAdjacency;
dominantSources = candidates.dominantSourcesByReceiver;
baselineResidualSources = ...
    candidates.residualBaselineSourcesByReceiver;
if any(dominantAdjacency(:) & ~previous(:))
    error('SafeGraphNeighborhoodV155:UnsafeHold', ...
        'The selected graph lost the dominant backbone.');
end
residualAdjacency = false(nodeCount);
residualSources = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    nonDominant = find(previous(receiverIdx, :) & ...
        (1:nodeCount) ~= dominantSources(receiverIdx));
    if numel(nonDominant) == 1
        residualSources(receiverIdx) = nonDominant;
    elseif isempty(nonDominant) && ...
            baselineResidualSources(receiverIdx) == ...
                dominantSources(receiverIdx)
        residualSources(receiverIdx) = dominantSources(receiverIdx);
    else
        error('SafeGraphNeighborhoodV155:UnsafeHold', ...
            'The selected graph cannot be decomposed into two routes.');
    end
    residualAdjacency(receiverIdx, ...
        residualSources(receiverIdx)) = true;
end
if ~isStronglyConnected(residualAdjacency)
    error('SafeGraphNeighborhoodV155:UnsafeHold', ...
        'The held residual route is not strongly connected.');
end
[rebuilt, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        protocol.dominantWeight, protocol.residualWeight);
if ~isequal(rebuilt, previous)
    error('SafeGraphNeighborhoodV155:UnsafeHold', ...
        'The held graph failed exact reconstruction.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
crossMask = groupIds ~= groupIds(residualSources);
if nnz(crossMask) ~= protocol.formationCounts(presetIdx)
    error('SafeGraphNeighborhoodV155:UnsafeHold', ...
        'The held graph changed its formation-ring cross count.');
end

adjacency = rebuilt;
details = routeDetails;
details.objective = NaN;
details.candidateIndex = NaN;
details.selectionSeconds = 0;
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = NaN;
details.fusionWeightMatrix = fusionWeights;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.residualBaselineAdjacency = ...
    candidates.residualBaselineAdjacency;
details.selectedSourcesByReceiver = residualSources;
details.overrideMask = ...
    residualSources ~= baselineResidualSources;
details.overrideFraction = mean(details.overrideMask);
details.crossFormationMessageCount = nnz(crossMask);
details.maximumCrossEdges = nnz(crossMask);
details.maximumCrossSourceLoad = 1;
details.maximumCrossReceiverLoad = 1;
details.proposalCrossCount = nnz(crossMask);
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'held-radius-one-spliced-residual-cycle';
details.sensorWindowMature = true;
details.sensorWindowStrongConnected = true;
details.formationWindowMature = true;
details.formationWindowStrongConnected = true;
details.successorSensorStrongConnected = NaN;
details.oneStepTopologyReserveChecked = false;
details.oneStepTopologyReservePassed = NaN;
details.oneStepJointProjectionUsed = false;
details.recursiveSafetyClaimed = false;
details.topologyInfeasible = false;
details.sourceWeight = protocol.residualWeight;
end

function valid = isStronglyConnected(policyAdjacency)
senderAdjacency = logical(policyAdjacency');
valid = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function valid = reachableAll(adjacency)
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
valid = all(visited);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
