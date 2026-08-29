function [adjacency, details] = ...
    selectSafeGraphOptionDwellRankV154( ...
        context, model, rankIndex, options)
% SELECTSAFEGRAPHOPTIONDWELLRANKV154 Replan only at option boundaries.

if nargin < 4 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getSafeGraphOptionDwellV154Protocol());
generator = getField(options, 'generatorProtocol', ...
    getSafeGraphCodebookOracleV152Protocol());
labelProtocol = getField(options, 'labelProtocol', ...
    getLabelSetSimulatorPolicyProtocol());
rankIndex = round(rankIndex);
if ~isscalar(rankIndex) || ~isfinite(rankIndex) || ...
        ~ismember(rankIndex, protocol.optionRanks)
    error('V154 graph-option rank is outside the frozen bank.');
end
nodeCount = numel(context.localPosteriorBySensor);
presetIdx = find(protocol.nodeCounts == nodeCount);
if numel(presetIdx) ~= 1 || ...
        numel(unique(context.model.dynamicTopologyScenario. ...
            config.sensorGroupIds)) ~= ...
                protocol.formationCounts(presetIdx)
    error('V154 graph option scale is outside the frozen protocol.');
end
anchorTime = protocol.anchorTimes(presetIdx);
dwellPages = protocol.optionDwellPages(presetIdx);
if context.currentTime < anchorTime || ...
        context.currentTime >= anchorTime + protocol.horizon
    error('V154 graph option is outside its frozen continuation window.');
end
optionAge = mod(context.currentTime - anchorTime, dwellPages);
optionIndex = floor((context.currentTime - anchorTime) / ...
    dwellPages) + 1;
optionBoundary = optionAge == 0;
fallbackUsed = false;

if optionBoundary
    [adjacency, details] = selectSafeGraphCodebookRankV152( ...
        context, model, rankIndex, struct( ...
            'protocol', generator, ...
            'labelProtocol', labelProtocol));
    heldGraphReused = false;
else
    try
        [adjacency, details] = rebuildHeldGraph( ...
            context, protocol, presetIdx);
        heldGraphReused = true;
    catch errorInfo
        if ~isSafeHoldFailure(errorInfo)
            rethrow(errorInfo);
        end
        [adjacency, details] = ...
            selectBackboneResidualSplicedCyclePolicy( ...
                context, protocol.fallbackMode, struct( ...
                    'dominantWeight', protocol.dominantWeight, ...
                    'residualWeight', protocol.residualWeight));
        heldGraphReused = false;
        fallbackUsed = true;
        details.v154FallbackReason = errorInfo.message;
    end
end

if nnz(adjacency) ~= ...
        protocol.exactSelectedMessageCount(presetIdx)
    error('V154 graph option changed the exact message count.');
end
details.mode = 'safe-graph-option-dwell-rank-v154';
details.v154ProtocolId = protocol.id;
details.v154ContractVersion = protocol.contractVersion;
details.v154Rank = rankIndex;
details.v154OptionBoundary = optionBoundary;
details.v154OptionIndex = optionIndex;
details.v154OptionAge = optionAge;
details.v154OptionDwellPages = dwellPages;
details.v154HeldGraphReused = heldGraphReused;
details.v154FallbackUsed = fallbackUsed;
details.v154ExpectedSelectedMessageCount = ...
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
    error('SafeGraphOptionDwellV154:UnsafeHold', ...
        'The incumbent graph is no longer physically executable.');
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
    error('SafeGraphOptionDwellV154:UnsafeHold', ...
        'The incumbent graph does not preserve the dominant backbone.');
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
        error('SafeGraphOptionDwellV154:UnsafeHold', ...
            'The incumbent graph cannot be decomposed into two routes.');
    end
    residualAdjacency(receiverIdx, ...
        residualSources(receiverIdx)) = true;
end
if ~isStronglyConnected(residualAdjacency)
    error('SafeGraphOptionDwellV154:UnsafeHold', ...
        'The incumbent residual route is not strongly connected.');
end
[rebuilt, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        protocol.dominantWeight, protocol.residualWeight);
if ~isequal(rebuilt, previous)
    error('SafeGraphOptionDwellV154:UnsafeHold', ...
        'The held graph failed exact route reconstruction.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
crossMask = groupIds ~= groupIds(residualSources);
if nnz(crossMask) ~= protocol.formationCounts(presetIdx)
    error('SafeGraphOptionDwellV154:UnsafeHold', ...
        'The held graph changed its formation-cycle cross count.');
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
details.repairTriggered = false;
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'held-fixed-index-plus-spliced-strong-residual-cycle';
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

function value = isSafeHoldFailure(errorInfo)
value = strcmp(errorInfo.identifier, ...
    'SafeGraphOptionDwellV154:UnsafeHold');
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
