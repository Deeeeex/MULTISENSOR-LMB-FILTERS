function [adjacency, details] = ...
    selectScaleAwareFormationMixingPolicy(context, mode, options)
% SELECTSCALEAWAREFORMATIONMIXINGPOLICY Equal-cost multi-gateway routing.
%
% A fixed high-weight intra-formation backbone is retained.  Low-weight
% residual edges are rewired into one or more formation-level gateway
% lanes.  Additional lanes replace local residual edges rather than adding
% messages, while an optional cross-edge weight changes mixing strength
% without changing graph support.  Every admitted route is physical,
% row-stochastic, message-count matched to the registered reference, and
% one-step strongly connected in the combined sensor graph.

if nargin < 2 || isempty(mode)
    mode = 'reference-cycle';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
requestedMode = normalizeMode(mode);
orientation = normalizeOrientation(getField( ...
    options, 'orientation', 'counter-clockwise'));
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
crossResidualWeight = getField( ...
    options, 'crossResidualWeight', residualWeight);
fallbackMode = normalizeMode(getField( ...
    options, 'fallbackMode', 'reference'));
if strcmp(fallbackMode, 'reference-cycle')
    fallbackMode = 'reference';
end
if ~ismember(fallbackMode, {'reference', 'error'}) || ...
        ~isscalar(dominantWeight) || ...
        ~isscalar(residualWeight) || ...
        ~isscalar(crossResidualWeight) || ...
        any(~isfinite([dominantWeight, residualWeight, ...
            crossResidualWeight])) || ...
        dominantWeight <= 0 || residualWeight <= 0 || ...
        crossResidualWeight <= 0 || ...
        dominantWeight + residualWeight >= 1 || ...
        dominantWeight + crossResidualWeight >= 1
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware mixing weights or fallback mode are invalid.');
end

nodeCount = numel(context.localPosteriorBySensor);
if nodeCount < 2 || ...
        ~isfield(context, 'physicalAdjacency') || ...
        ~isequal(size(context.physicalAdjacency), ...
            [nodeCount, nodeCount])
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware mixing requires a valid physical context.');
end
groupIds = resolveGroupIds(context.model, nodeCount);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
formationOrder = resolveFormationOrder( ...
    groups, orientation);

[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, ['fixed-', orientation], struct( ...
            'dominantWeight', dominantWeight, ...
            'residualWeight', residualWeight));
if strcmp(requestedMode, 'reference-cycle') && ...
        abs(crossResidualWeight - residualWeight) <= 1e-12
    adjacency = referenceAdjacency;
    details = decorateReference( ...
        referenceAdjacency, referenceDetails, groupIds, ...
        requestedMode, orientation, residualWeight, ...
        dominantWeight, false, '', '', toc(timerId));
    return;
end

try
    offsets = resolveOffsets(requestedMode, formationCount);
    [adjacency, details] = buildCandidate( ...
        context, groupIds, formationOrder, offsets, ...
        requestedMode, orientation, dominantWeight, ...
        residualWeight, crossResidualWeight, ...
        referenceAdjacency, referenceDetails, timerId, options);
catch errorInfo
    if ~strcmp(fallbackMode, 'reference') || ...
            ~isScaleMixingInfeasible(errorInfo)
        rethrow(errorInfo);
    end
    adjacency = referenceAdjacency;
    details = decorateReference( ...
        referenceAdjacency, referenceDetails, groupIds, ...
        requestedMode, orientation, residualWeight, ...
        dominantWeight, true, getErrorField(errorInfo, 'identifier'), ...
        getErrorField(errorInfo, 'message'), toc(timerId));
end
end

function [adjacency, details] = buildCandidate( ...
        context, groupIds, formationOrder, offsets, requestedMode, ...
        orientation, dominantWeight, residualWeight, ...
        crossResidualWeight, referenceAdjacency, ...
        referenceDetails, timerId, options)
nodeCount = numel(groupIds);
physical = logical(context.physicalAdjacency);
physical(1:nodeCount+1:end) = false;
[dominantAdjacency, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
[residualBaselineAdjacency, residualBaselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-balanced-cycle', ...
        struct('sourceWeight', residualWeight, 'phase', 1));
dominantSources = reshape( ...
    dominantDetails.selectedSourcesByReceiver, 1, []);
residualBaselineSources = reshape( ...
    residualBaselineDetails.selectedSourcesByReceiver, 1, []);

scoreMatrix = getField(options, 'edgeScoreMatrix', []);
if isempty(scoreMatrix)
    scoreMatrix = -inf(nodeCount);
    [receiverIndices, senderIndices] = find(physical);
    crossMask = groupIds(receiverIndices) ~= ...
        groupIds(senderIndices);
    receiverIndices = receiverIndices(crossMask);
    senderIndices = senderIndices(crossMask);
    for edgeIdx = 1:numel(receiverIndices)
        scoreMatrix(receiverIndices(edgeIdx), ...
            senderIndices(edgeIdx)) = -( ...
                receiverIndices(edgeIdx) * (nodeCount + 1) + ...
                senderIndices(edgeIdx));
    end
else
    if ~isequal(size(scoreMatrix), [nodeCount, nodeCount]) || ...
            any(isnan(scoreMatrix(:)))
        error('ScaleMixing:InvalidContract', ...
            'edgeScoreMatrix must be S-by-S and contain no NaNs.');
    end
end

selection = selectScaleAwareResidualGatewayLanes( ...
    groupIds, residualBaselineSources, dominantSources, ...
    physical, scoreMatrix, formationOrder, offsets);
residualAdjacency = selection.residualAdjacency;
residualWeightsByReceiver = ...
    residualWeight * ones(1, nodeCount);
residualWeightsByReceiver(selection.crossReceiverMask) = ...
    crossResidualWeight;
[adjacency, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        dominantWeight, residualWeightsByReceiver);

referenceMessageCount = nnz(referenceAdjacency);
directedMessageBudget = floor(getField( ...
    context, 'directedMessageBudget', inf));
combinedStrong = isStronglyConnected(adjacency);
formationMixing = collapseFusionWeightsByGroup( ...
    fusionWeights, groupIds);
formationSupport = formationMixing > 1e-12;
formationSupport(1:size(formationSupport, 1)+1:end) = false;
formationStrong = isStronglyConnected(formationSupport);
duplicateCount = nnz( ...
    dominantSources == selection.residualSourcesByReceiver);
referenceDuplicateCount = nnz( ...
    referenceDetails.dominantSourcesByReceiver == ...
        referenceDetails.residualSourcesByReceiver);
if nnz(adjacency) ~= referenceMessageCount || ...
        nnz(adjacency) > directedMessageBudget || ...
        duplicateCount ~= referenceDuplicateCount || ...
        any(adjacency(:) & ~physical(:)) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        any(fusionWeights(:) < -1e-12) || ...
        ~combinedStrong || ~formationStrong
    error('ScaleMixing:Infeasible', [ ...
        'Candidate failed message parity, budget, physical, weight, ', ...
        'or strong-connectivity projection.']);
end

mixingDiagnostics = ...
    computeStochasticMixingDiagnostics(formationMixing);
details = routeDetails;
details.contractVersion = ...
    'scale-aware-multi-gateway-residual-mixing-v1';
details.mode = ['scale-aware-formation-mixing-', requestedMode];
details.requestedMode = requestedMode;
details.realizedMode = requestedMode;
details.orientation = orientation;
details.objective = -selection.objective;
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = finiteSpread(scoreMatrix);
details.validCandidateCount = nnz(isfinite(scoreMatrix));
details.fusionWeightMatrix = fusionWeights;
details.baselineAdjacency = referenceAdjacency;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.residualBaselineAdjacency = residualBaselineAdjacency;
details.dominantPolicyDetails = dominantDetails;
details.residualBaselinePolicyDetails = ...
    residualBaselineDetails;
details.gatewaySelection = selection;
details.formationOrder = formationOrder;
details.formationOffsets = offsets;
details.formationOffsetsModulo = selection.offsetsModulo;
details.gatewayLaneCount = selection.gatewayLaneCount;
details.dominantSourcesByReceiver = dominantSources;
details.residualSourcesByReceiver = ...
    selection.residualSourcesByReceiver;
details.selectedSourcesByReceiver = ...
    selection.residualSourcesByReceiver;
details.selectedSourceWeightsByReceiver = ...
    residualWeightsByReceiver;
details.residualWeightsByReceiver = ...
    residualWeightsByReceiver;
details.crossReceiverMask = selection.crossReceiverMask;
details.crossResidualWeights = ...
    residualWeightsByReceiver(selection.crossReceiverMask);
details.minimumCrossResidualWeight = min( ...
    details.crossResidualWeights);
details.meanCrossResidualWeight = mean( ...
    details.crossResidualWeights);
details.maximumCrossResidualWeight = max( ...
    details.crossResidualWeights);
details.sourceWeight = details.meanCrossResidualWeight;
details.crossFormationMessageCount = ...
    selection.crossFormationEdgeCount;
details.maximumCrossEdges = ...
    selection.crossFormationEdgeCount;
details.proposalCrossCount = ...
    selection.crossFormationEdgeCount;
details.dominantResidualDuplicateCount = duplicateCount;
details.referenceDominantResidualDuplicateCount = ...
    referenceDuplicateCount;
details.messageCount = nnz(adjacency);
details.referenceMessageCount = referenceMessageCount;
details.messageCountParityWithReference = true;
details.directedMessageBudget = directedMessageBudget;
details.residualSensorStrongConnected = ...
    selection.residualSensorStrongConnected;
details.combinedSensorStrongConnected = combinedStrong;
details.formationStrongConnected = formationStrong;
details.formationMixingMatrix = formationMixing;
details.formationMixingDiagnostics = mixingDiagnostics;
details.formationMixingSpectralGapProxy = ...
    1 - mixingDiagnostics.centeredSpectralNorm;
details.repairTriggered = false;
details.projectionFallbackUsed = false;
details.projectionFailureIdentifier = '';
details.projectionFailureMessage = '';
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = logical(getField( ...
    options, 'posteriorUsed', false));
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = logical(getField( ...
    options, 'truthUsed', false));
details.groundTruthUsed = details.truthUsed;
details.futureOutcomeUsed = logical(getField( ...
    options, 'futureOutcomeUsed', false));
if details.truthUsed || details.futureOutcomeUsed
    error('ScaleMixing:InvalidContract', [ ...
        'The registered scale-aware probe action must be truth-free ', ...
        'and future-free.']);
end
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'fixed-dominant-plus-scale-aware-residual-gateway-lanes';
details.sensorWindowMature = true;
details.sensorWindowStrongConnected = true;
details.formationWindowMature = true;
details.formationWindowStrongConnected = true;
details.oneStepTopologyReserveChecked = true;
details.oneStepTopologyReservePassed = true;
details.oneStepJointProjectionUsed = true;
details.recursiveSafetyClaimed = false;
details.topologyInfeasible = false;
details.safeProjectorUsed = true;
end

function details = decorateReference( ...
        adjacency, baseDetails, groupIds, requestedMode, ...
        orientation, residualWeight, dominantWeight, fallbackUsed, ...
        failureIdentifier, failureMessage, selectionSeconds)
weights = baseDetails.fusionWeightMatrix;
formationMixing = collapseFusionWeightsByGroup(weights, groupIds);
mixingDiagnostics = ...
    computeStochasticMixingDiagnostics(formationMixing);
details = baseDetails;
details.basePolicyMode = baseDetails.mode;
details.contractVersion = ...
    'scale-aware-multi-gateway-residual-mixing-v1';
details.mode = 'scale-aware-formation-mixing-reference-cycle';
details.requestedMode = requestedMode;
details.realizedMode = 'reference-cycle';
details.orientation = orientation;
details.selectionSeconds = selectionSeconds;
details.formationOrder = ...
    baseDetails.spliceSelection.formationOrder;
details.formationOffsets = 1;
details.formationOffsetsModulo = 1;
details.gatewayLaneCount = 1;
details.gatewaySelection = baseDetails.spliceSelection;
details.messageCount = nnz(adjacency);
details.referenceMessageCount = nnz(adjacency);
details.messageCountParityWithReference = true;
details.dominantResidualDuplicateCount = nnz( ...
    baseDetails.dominantSourcesByReceiver == ...
        baseDetails.residualSourcesByReceiver);
details.referenceDominantResidualDuplicateCount = ...
    details.dominantResidualDuplicateCount;
details.crossReceiverMask = groupIds ~= ...
    groupIds(baseDetails.residualSourcesByReceiver);
details.residualSensorStrongConnected = true;
details.combinedSensorStrongConnected = ...
    isStronglyConnected(adjacency);
details.formationStrongConnected = true;
details.formationMixingMatrix = formationMixing;
details.formationMixingDiagnostics = mixingDiagnostics;
details.formationMixingSpectralGapProxy = ...
    1 - mixingDiagnostics.centeredSpectralNorm;
details.residualWeightsByReceiver = ...
    residualWeight * ones(1, numel(groupIds));
details.crossResidualWeights = ...
    residualWeight * ones(1, nnz(details.crossReceiverMask));
details.minimumCrossResidualWeight = residualWeight;
details.meanCrossResidualWeight = residualWeight;
details.maximumCrossResidualWeight = residualWeight;
details.dominantWeight = dominantWeight;
details.projectionFallbackUsed = fallbackUsed;
details.projectionFailureIdentifier = failureIdentifier;
details.projectionFailureMessage = failureMessage;
details.oneStepTopologyReserveChecked = true;
details.oneStepTopologyReservePassed = ...
    details.combinedSensorStrongConnected;
details.oneStepJointProjectionUsed = true;
details.safeProjectorUsed = true;
details.recursiveSafetyClaimed = false;
details.topologyInfeasible = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureOutcomeUsed = false;
end

function offsets = resolveOffsets(mode, formationCount)
switch mode
    case {'reference-cycle', 'cycle', 'single-gateway-cycle'}
        offsets = 1;
    case {'dual-cycle', 'dual-gateway-cycle'}
        offsets = [1, 1];
    case 'bidirectional-cycle'
        offsets = [1, -1];
    case {'cycle-antipodal', 'cycle-chord'}
        offsets = [1, floor(formationCount / 2)];
    case {'bidirectional-antipodal', 'bidirectional-chord'}
        offsets = [1, -1, floor(formationCount / 2)];
    otherwise
        error('ScaleMixing:InvalidContract', ...
            'Unknown scale-aware formation-mixing mode: %s.', mode);
end
if any(mod(offsets, formationCount) == 0)
    error('ScaleMixing:Infeasible', ...
        'Requested chord collapses to a same-formation route.');
end
end

function order = resolveFormationOrder(groups, orientation)
switch orientation
    case 'clockwise'
        order = groups;
    case 'counter-clockwise'
        order = groups([1, numel(groups):-1:2]);
    otherwise
        error('ScaleMixing:InvalidContract', ...
            'Unknown formation orientation.');
end
end

function mode = normalizeMode(mode)
mode = lower(strrep(char(mode), '_', '-'));
end

function orientation = normalizeOrientation(value)
orientation = normalizeMode(value);
if strcmp(orientation, 'counterclockwise')
    orientation = 'counter-clockwise';
end
if ~ismember(orientation, {'clockwise', 'counter-clockwise'})
    error('ScaleMixing:InvalidContract', ...
        'Unknown scale-aware mixing orientation.');
end
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware mixing requires sensorGroupIds metadata.');
end
groupIds = reshape(model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0)
    error('ScaleMixing:InvalidContract', ...
        'sensorGroupIds metadata are invalid.');
end
end

function value = finiteSpread(matrix)
finiteValues = matrix(isfinite(matrix));
if isempty(finiteValues)
    value = NaN;
else
    value = max(finiteValues) - min(finiteValues);
end
end

function tf = isScaleMixingInfeasible(errorInfo)
tf = strcmp(getErrorField(errorInfo, 'identifier'), ...
    'ScaleMixing:Infeasible');
end

function value = getErrorField(errorInfo, fieldName)
value = '';
try
    value = errorInfo.(fieldName);
catch
    value = '';
end
if isempty(value)
    value = '';
end
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
