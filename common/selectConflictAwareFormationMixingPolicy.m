function [adjacency, details] = ...
    selectConflictAwareFormationMixingPolicy( ...
        context, action, options)
% SELECTCONFLICTAWAREFORMATIONMIXINGPOLICY Selective v28 residual action.
%
% The registered reference cycle is always retained.  A candidate either
% attenuates one reference cross-formation input or swaps two local
% residual sources to create a reciprocal formation-pair exchange.  The
% compensated pair mode redistributes, rather than increases, total
% cross-formation residual trust.  Gateway endpoints are selected from
% current-posterior KLA compatibility without truth or future outcomes.

if nargin < 2 || isempty(action)
    action = struct('mode', 'reference');
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = normalizeMode(getField(action, 'mode', 'reference'));
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
pairResidualWeight = getField(options, ...
    'pairResidualWeight', 0.025);
dampedReferenceWeight = getField(options, ...
    'dampedReferenceWeight', 0.025);
if ~ismember(mode, { ...
        'reference', 'damp-reference-edge', ...
        'pair-redistribute', 'pair-add-low', ...
        'pair-add-reference'}) || ...
        any(~isfinite([dominantWeight, residualWeight, ...
            pairResidualWeight, dampedReferenceWeight])) || ...
        any([dominantWeight, residualWeight, ...
            pairResidualWeight, dampedReferenceWeight] <= 0) || ...
        dominantWeight + max([residualWeight, ...
            pairResidualWeight, dampedReferenceWeight]) >= 1 || ...
        dampedReferenceWeight > residualWeight
    error('ConflictMixing:InvalidContract', ...
        'Conflict-aware action or weights are invalid.');
end

nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context, nodeCount);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', dominantWeight, ...
            'residualWeight', residualWeight));
referenceResidualSources = reshape( ...
    referenceDetails.residualSourcesByReceiver, 1, []);
dominantSources = reshape( ...
    referenceDetails.dominantSourcesByReceiver, 1, []);
referenceCrossMask = groupIds ~= ...
    groupIds(referenceResidualSources);
if nnz(referenceCrossMask) ~= formationCount
    error('ConflictMixing:InvalidContract', ...
        'Registered reference must have one cross edge per formation.');
end

if strcmp(mode, 'reference')
    adjacency = referenceAdjacency;
    details = decorateDetails( ...
        referenceDetails, mode, groupIds, ...
        referenceResidualSources, referenceCrossMask, ...
        referenceAdjacency, residualWeight, [], [], ...
        false, struct(), toc(timerId));
    return;
end

residualSources = referenceResidualSources;
residualWeightsByReceiver = ...
    residualWeight * ones(1, nodeCount);
selectedPairReceivers = zeros(1, 0);
selectedPairSenders = zeros(1, 0);
posteriorUsed = false;
compatibilityDetails = struct();
actionFormationIds = zeros(1, 0);
switch mode
    case 'damp-reference-edge'
        formationId = getField(action, 'formationId', NaN);
        validateFormationIds(formationId, groups, 1);
        actionFormationIds = formationId;
        crossReceivers = find(referenceCrossMask & ...
            groupIds == formationId);
        if numel(crossReceivers) ~= 1
            error('ConflictMixing:InvalidContract', ...
                'Reference cross receiver is ambiguous.');
        end
        residualWeightsByReceiver(crossReceivers) = ...
            dampedReferenceWeight;
    otherwise
        formationPair = reshape(getField( ...
            action, 'formationPair', []), 1, []);
        validateFormationIds(formationPair, groups, 2);
        actionFormationIds = formationPair;
        if strcmp(mode, 'pair-add-reference')
            selectedPairWeight = residualWeight;
        else
            selectedPairWeight = pairResidualWeight;
        end
        [scoreMatrix, compatibilityDetails] = ...
            resolveCompatibilityScores( ...
                context, selectedPairWeight, options);
        [selectedPairReceivers, selectedPairSenders] = ...
            selectReciprocalPairSwap( ...
                formationPair, groupIds, ...
                referenceResidualSources, dominantSources, ...
                logical(context.physicalAdjacency), scoreMatrix);
        residualSources(selectedPairReceivers) = ...
            selectedPairSenders;
        residualWeightsByReceiver(selectedPairReceivers) = ...
            selectedPairWeight;
        posteriorUsed = true;
        if strcmp(mode, 'pair-redistribute')
            if abs(pairResidualWeight - ...
                    (residualWeight - dampedReferenceWeight)) > 1e-12
                error('ConflictMixing:InvalidContract', [ ...
                    'Compensated pair mode must preserve total ', ...
                    'cross-formation residual trust.']);
            end
            dampedCrossReceivers = find(referenceCrossMask & ...
                ismember(groupIds, formationPair));
            if numel(dampedCrossReceivers) ~= 2
                error('ConflictMixing:InvalidContract', ...
                    'Compensated reference receivers are ambiguous.');
            end
            residualWeightsByReceiver(dampedCrossReceivers) = ...
                dampedReferenceWeight;
        end
end

residualAdjacency = sourceMapToAdjacency( ...
    residualSources, nodeCount);
[adjacency, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, referenceDetails.dominantAdjacency, ...
        residualAdjacency, dominantWeight, ...
        residualWeightsByReceiver);
physical = logical(context.physicalAdjacency);
candidateCrossMask = groupIds ~= groupIds(residualSources);
formationMixing = collapseFusionWeightsByGroup( ...
    fusionWeights, groupIds);
formationSupport = formationMixing > 1e-12;
formationSupport(1:formationCount+1:end) = false;
combinedStrong = isStronglyConnected(adjacency);
formationStrong = isStronglyConnected(formationSupport);
referenceDuplicateCount = nnz( ...
    referenceResidualSources == dominantSources);
candidateDuplicateCount = nnz(residualSources == dominantSources);
if any(sum(residualAdjacency, 2) ~= 1) || ...
        any(sum(residualAdjacency, 1) ~= 1) || ...
        any(adjacency(:) & ~physical(:)) || ...
        nnz(adjacency) ~= nnz(referenceAdjacency) || ...
        candidateDuplicateCount ~= referenceDuplicateCount || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        any(fusionWeights(:) < -1e-12) || ...
        ~combinedStrong || ~formationStrong
    error('ConflictMixing:Infeasible', [ ...
        'Candidate failed permutation, physical, message, weight, ', ...
        'duplicate, or connectivity invariants.']);
end

if strcmp(mode, 'pair-redistribute')
    referenceCrossTrust = residualWeight * nnz(referenceCrossMask);
    candidateCrossTrust = sum( ...
        residualWeightsByReceiver(candidateCrossMask));
    if abs(candidateCrossTrust - referenceCrossTrust) > 1e-12
        error('ConflictMixing:InvalidContract', ...
            'Compensated pair action changed total cross trust.');
    end
end

details = routeDetails;
details.contractVersion = ...
    'conflict-aware-selective-formation-mixing-v1';
details.mode = ['conflict-aware-', mode];
details.actionMode = mode;
details.actionFormationIds = actionFormationIds;
details.actionName = buildActionName(mode, actionFormationIds);
details.selectionSeconds = toc(timerId);
details.referenceAdjacency = referenceAdjacency;
details.referenceFusionWeights = ...
    referenceDetails.fusionWeightMatrix;
details.dominantAdjacency = referenceDetails.dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.referenceResidualAdjacency = ...
    referenceDetails.residualAdjacency;
details.referenceResidualSourcesByReceiver = ...
    referenceResidualSources;
details.residualSourcesByReceiver = residualSources;
details.referenceCrossReceiverMask = referenceCrossMask;
details.crossReceiverMask = candidateCrossMask;
details.selectedPairReceivers = selectedPairReceivers;
details.selectedPairSenders = selectedPairSenders;
details.residualWeightsByReceiver = ...
    residualWeightsByReceiver;
details.referenceCrossResidualTrust = ...
    residualWeight * nnz(referenceCrossMask);
details.candidateCrossResidualTrust = sum( ...
    residualWeightsByReceiver(candidateCrossMask));
details.crossResidualTrustChange = ...
    details.candidateCrossResidualTrust - ...
        details.referenceCrossResidualTrust;
details.crossFormationMessageCount = nnz(candidateCrossMask);
details.referenceCrossFormationMessageCount = ...
    nnz(referenceCrossMask);
details.messageCount = nnz(adjacency);
details.referenceMessageCount = nnz(referenceAdjacency);
details.messageCountParityWithReference = true;
details.combinedSensorStrongConnected = combinedStrong;
details.formationStrongConnected = formationStrong;
details.formationMixingMatrix = formationMixing;
details.formationMixingDiagnostics = ...
    computeStochasticMixingDiagnostics(formationMixing);
details.formationMixingSpectralGapProxy = 1 - ...
    details.formationMixingDiagnostics.centeredSpectralNorm;
details.posteriorUsed = posteriorUsed;
details.currentLinkReliabilityUsed = posteriorUsed;
details.compatibilityDetails = compatibilityDetails;
details.repairTriggered = false;
details.projectionFallbackUsed = false;
details.safeProjectorUsed = true;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureMeasurementUsed = false;
details.futureOutcomeUsed = false;
end

function [scoreMatrix, details] = ...
        resolveCompatibilityScores(context, sourceWeight, options)
if isfield(options, 'edgeScoreMatrix') && ...
        ~isempty(options.edgeScoreMatrix)
    scoreMatrix = options.edgeScoreMatrix;
    nodeCount = numel(context.localPosteriorBySensor);
    if ~isequal(size(scoreMatrix), [nodeCount, nodeCount]) || ...
            any(isnan(scoreMatrix(:)))
        error('ConflictMixing:InvalidContract', ...
            'Injected compatibility scores are invalid.');
    end
    details = struct( ...
        'scoreMatrix', scoreMatrix, ...
        'sourceWeight', sourceWeight, ...
        'injected', true, ...
        'truthUsed', logical(getField( ...
            options, 'edgeScoresUseTruth', false)), ...
        'futureOutcomeUsed', logical(getField( ...
            options, 'edgeScoresUseFutureOutcome', false)));
    if details.truthUsed || details.futureOutcomeUsed
        error('ConflictMixing:InvalidContract', ...
            'Conflict-aware gateway scores must be causal.');
    end
else
    [scoreMatrix, details] = computeLmbKlaCompatibilityMatrix( ...
        context, struct('sourceWeight', sourceWeight));
    details.injected = false;
end
end

function [receivers, senders] = selectReciprocalPairSwap( ...
        formationPair, groupIds, referenceSources, ...
        dominantSources, physicalAdjacency, scoreMatrix)
firstCandidates = find(groupIds == formationPair(1) & ...
    groupIds(referenceSources) == formationPair(1) & ...
    referenceSources ~= dominantSources);
secondCandidates = find(groupIds == formationPair(2) & ...
    groupIds(referenceSources) == formationPair(2) & ...
    referenceSources ~= dominantSources);
bestScore = -inf;
receivers = zeros(1, 0);
senders = zeros(1, 0);
for firstIdx = 1:numel(firstCandidates)
    firstReceiver = firstCandidates(firstIdx);
    firstSender = referenceSources(firstReceiver);
    for secondIdx = 1:numel(secondCandidates)
        secondReceiver = secondCandidates(secondIdx);
        secondSender = referenceSources(secondReceiver);
        if ~physicalAdjacency(firstReceiver, secondSender) || ...
                ~physicalAdjacency(secondReceiver, firstSender) || ...
                secondSender == dominantSources(firstReceiver) || ...
                firstSender == dominantSources(secondReceiver)
            continue;
        end
        score = scoreMatrix(firstReceiver, secondSender) + ...
            scoreMatrix(secondReceiver, firstSender);
        if ~isfinite(score)
            continue;
        end
        candidateKey = [firstReceiver, secondReceiver, ...
            secondSender, firstSender];
        currentKey = [receivers, senders];
        if score > bestScore + 1e-12 || ...
                (abs(score - bestScore) <= 1e-12 && ...
                 (isempty(currentKey) || ...
                  lexicographicallyLess(candidateKey, currentKey)))
            bestScore = score;
            receivers = [firstReceiver, secondReceiver];
            senders = [secondSender, firstSender];
        end
    end
end
if isempty(receivers)
    error('ConflictMixing:Infeasible', ...
        'No physical reciprocal residual swap exists for the pair.');
end
end

function value = lexicographicallyLess(left, right)
value = false;
for idx = 1:min(numel(left), numel(right))
    if left(idx) < right(idx)
        value = true;
        return;
    elseif left(idx) > right(idx)
        return;
    end
end
value = numel(left) < numel(right);
end

function details = decorateDetails( ...
        referenceDetails, mode, groupIds, residualSources, ...
        crossMask, referenceAdjacency, residualWeight, ...
        pairReceivers, pairSenders, posteriorUsed, ...
        compatibilityDetails, elapsedSeconds)
formationMixing = collapseFusionWeightsByGroup( ...
    referenceDetails.fusionWeightMatrix, groupIds);
details = referenceDetails;
details.contractVersion = ...
    'conflict-aware-selective-formation-mixing-v1';
details.mode = ['conflict-aware-', mode];
details.actionMode = mode;
details.actionFormationIds = zeros(1, 0);
details.actionName = 'reference';
details.selectionSeconds = elapsedSeconds;
details.referenceAdjacency = referenceAdjacency;
details.referenceFusionWeights = ...
    referenceDetails.fusionWeightMatrix;
details.referenceResidualAdjacency = ...
    referenceDetails.residualAdjacency;
details.referenceResidualSourcesByReceiver = residualSources;
details.residualSourcesByReceiver = residualSources;
details.referenceCrossReceiverMask = crossMask;
details.crossReceiverMask = crossMask;
details.selectedPairReceivers = pairReceivers;
details.selectedPairSenders = pairSenders;
details.residualWeightsByReceiver = ...
    residualWeight * ones(1, numel(groupIds));
details.referenceCrossResidualTrust = ...
    residualWeight * nnz(crossMask);
details.candidateCrossResidualTrust = ...
    details.referenceCrossResidualTrust;
details.crossResidualTrustChange = 0;
details.crossFormationMessageCount = nnz(crossMask);
details.referenceCrossFormationMessageCount = nnz(crossMask);
details.messageCount = nnz(referenceAdjacency);
details.referenceMessageCount = nnz(referenceAdjacency);
details.messageCountParityWithReference = true;
details.combinedSensorStrongConnected = ...
    isStronglyConnected(referenceAdjacency);
formationSupport = formationMixing > 1e-12;
formationSupport(1:size(formationSupport, 1)+1:end) = false;
details.formationStrongConnected = ...
    isStronglyConnected(formationSupport);
details.formationMixingMatrix = formationMixing;
details.formationMixingDiagnostics = ...
    computeStochasticMixingDiagnostics(formationMixing);
details.formationMixingSpectralGapProxy = 1 - ...
    details.formationMixingDiagnostics.centeredSpectralNorm;
details.posteriorUsed = posteriorUsed;
details.currentLinkReliabilityUsed = posteriorUsed;
details.compatibilityDetails = compatibilityDetails;
details.repairTriggered = false;
details.projectionFallbackUsed = false;
details.safeProjectorUsed = true;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureMeasurementUsed = false;
details.futureOutcomeUsed = false;
end

function groupIds = resolveGroupIds(context, nodeCount)
if ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('ConflictMixing:InvalidContract', ...
        'Formation group IDs are unavailable.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0)
    error('ConflictMixing:InvalidContract', ...
        'Formation group IDs are invalid.');
end
end

function validateFormationIds(values, groups, requiredCount)
values = reshape(values, 1, []);
if numel(values) ~= requiredCount || ...
        numel(unique(values)) ~= requiredCount || ...
        any(~ismember(values, groups))
    error('ConflictMixing:InvalidContract', ...
        'Action formation IDs are invalid.');
end
end

function adjacency = sourceMapToAdjacency(sources, nodeCount)
adjacency = false(nodeCount);
for receiverIdx = 1:nodeCount
    adjacency(receiverIdx, sources(receiverIdx)) = true;
end
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

function name = buildActionName(mode, formationIds)
if strcmp(mode, 'reference')
    name = 'reference';
elseif strcmp(mode, 'damp-reference-edge')
    name = sprintf('damp-reference-f%d', formationIds(1));
else
    name = sprintf('%s-f%d-f%d', mode, ...
        formationIds(1), formationIds(2));
end
end

function mode = normalizeMode(mode)
mode = lower(strrep(char(mode), '_', '-'));
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
