function [residual, details] = ...
    selectFormationB4V53ExactSelectiveResidual( ...
        context, candidateDominant, candidateResidual, protocol)
% SELECTFORMATIONB4V53EXACTSELECTIVERESIDUAL Exact LMB serve/hold control.

nodeCount = numel(context.localPosteriorBySensor);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
candidateDominant = logical(candidateDominant);
candidateResidual = logical(candidateResidual);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(candidateDominant), [nodeCount, nodeCount]) || ...
        ~isequal(size(candidateResidual), [nodeCount, nodeCount]) || ...
        nnz(candidateDominant) ~= nodeCount || ...
        nnz(candidateResidual) ~= nodeCount
    error('FormationB4V53Exact:InvalidInput', ...
        'The V46 route or formation registry is invalid.');
end

crossMask = groupIds(:) ~= groupIds(:)';
crossResidual = candidateResidual & crossMask;
if nnz(crossResidual) < 1
    error('FormationB4V53Exact:NoCrossResidual', ...
        'The current V46 residual layer has no cross input.');
end
[previousPulse, previousPulseKnown] = resolvePreviousPulse( ...
    context, protocol, nodeCount);
previouslyServed = true(1, formationCount);
if previousPulseKnown
    for formationIdx = 1:formationCount
        receivers = groupIds == formationIds(formationIdx);
        outside = ~receivers;
        previouslyServed(formationIdx) = any(any( ...
            previousPulse(receivers, outside)));
    end
end
if ~previousPulseKnown && protocol.referenceFallbackOnMissingHistory
    residual = candidateResidual;
    [sensorStrong, formationStrong] = temporalStrong( ...
        candidateDominant | candidateResidual, ...
        candidateDominant, residual, groupIds);
    details = fallbackDetails(formationIds, formationCount, ...
        previousPulseKnown, previouslyServed, sensorStrong, ...
        formationStrong);
    return;
end

referenceAdjacency = candidateDominant | candidateResidual;
referenceWeights = buildWeights( ...
    candidateDominant, candidateResidual, protocol);
[~, referenceNetwork] = scoreNetwork( ...
    context, referenceAdjacency, referenceWeights, protocol, []);
referenceDistributions = referenceNetwork.receiverDistributions;
referenceExpectedCardinality = expectedCardinality( ...
    referenceDistributions);

formationDebt = -inf(1, formationCount);
referenceFormationCardinality = nan(1, formationCount);
singleAvailable = false(1, formationCount);
singleSafe = false(1, formationCount);
singleRetentionRisk = nan(1, formationCount);
singleMinimumRetention = nan(1, formationCount);
singleThresholdCrossings = nan(1, formationCount);
for formationIdx = 1:formationCount
    formationId = formationIds(formationIdx);
    receivers = groupIds == formationId;
    if ~any(any(crossResidual(receivers, :)))
        continue;
    end
    singleResidual = removeFormationInputs( ...
        candidateResidual, crossResidual, groupIds, formationId);
    singleAdjacency = candidateDominant | singleResidual;
    singleWeights = buildWeights( ...
        candidateDominant, singleResidual, protocol);
    [~, singleNetwork] = scoreNetwork( ...
        context, singleAdjacency, singleWeights, protocol, ...
        struct('details', referenceNetwork, ...
            'adjacency', referenceAdjacency, ...
            'weights', referenceWeights));
    score = scoreRetention( ...
        referenceDistributions, singleNetwork.receiverDistributions, ...
        groupIds, protocol);
    referenceFormationCardinality(formationIdx) = mean( ...
        referenceExpectedCardinality(receivers));
    formationDebt(formationIdx) = ...
        score.details.formationMeanCardinalityChange(formationIdx) / ...
        max(referenceFormationCardinality(formationIdx), 1);
    singleAvailable(formationIdx) = true;
    singleSafe(formationIdx) = score.safe;
    singleRetentionRisk(formationIdx) = score.risk;
    singleMinimumRetention(formationIdx) = ...
        score.minimumSupportedLabelRetentionRatio;
    singleThresholdCrossings(formationIdx) = ...
        score.decisionThresholdCrossingCount;
end

thresholds = protocol.retentionDebtOnFraction * ...
    ones(1, formationCount);
thresholds(~previouslyServed) = protocol.retentionDebtOffFraction;
cardinalityEvidence = referenceFormationCardinality >= ...
    protocol.minimumReferenceFormationExpectedCardinality;
requested = singleAvailable & singleSafe & cardinalityEvidence & ...
    formationDebt >= thresholds - 1e-12;
eligible = requested & previouslyServed;
selectedDeferred = eligible;
projectionRecoveryOrder = zeros(1, 0);
jointEvaluationCount = 0;
while true
    residual = removeFormationInputs( ...
        candidateResidual, crossResidual, groupIds, ...
        formationIds(selectedDeferred));
    [sensorStrong, formationStrong] = temporalStrong( ...
        previousPulse, candidateDominant, residual, groupIds);
    jointSafe = true;
    jointScore = emptyJointScore();
    if any(selectedDeferred)
        jointEvaluationCount = jointEvaluationCount + 1;
        jointAdjacency = candidateDominant | residual;
        jointWeights = buildWeights( ...
            candidateDominant, residual, protocol);
        [~, jointNetwork] = scoreNetwork( ...
            context, jointAdjacency, jointWeights, protocol, ...
            struct('details', referenceNetwork, ...
                'adjacency', referenceAdjacency, ...
                'weights', referenceWeights));
        jointScore = scoreRetention( ...
            referenceDistributions, jointNetwork.receiverDistributions, ...
            groupIds, protocol);
        jointSafe = jointScore.safe;
    end
    if jointSafe && sensorStrong && formationStrong
        break;
    end
    if ~any(selectedDeferred)
        error('FormationB4V53Exact:ReferenceProjectionFailed', ...
            'The unchanged V46 route failed the temporal projection.');
    end
    selectedIndices = find(selectedDeferred);
    [~, localIdx] = min(formationDebt(selectedIndices));
    restoreIdx = selectedIndices(localIdx);
    selectedDeferred(restoreIdx) = false;
    projectionRecoveryOrder(end + 1) = ...
        formationIds(restoreIdx); %#ok<AGROW>
end

details = struct();
details.contractVersion = ...
    'formation-b4-v53-exact-selective-residual-v1';
details.formationIds = formationIds;
details.referenceFormationExpectedCardinality = ...
    referenceFormationCardinality;
details.formationRetentionDebtFraction = formationDebt;
details.retentionDebtThresholdByFormation = thresholds;
details.singleActionAvailableMask = singleAvailable;
details.singleActionSafetyMask = singleSafe;
details.singleActionRetentionRisk = singleRetentionRisk;
details.singleActionMinimumSupportedLabelRetentionRatio = ...
    singleMinimumRetention;
details.singleActionDecisionThresholdCrossingCount = ...
    singleThresholdCrossings;
details.requestedDeferredFormationMask = requested;
details.previousPulseKnown = previousPulseKnown;
details.previousPulseServedFormationMask = previouslyServed;
details.eligibleDeferredFormationMask = eligible;
details.selectedDeferredFormationMask = selectedDeferred;
details.selectedDeferredFormationIds = ...
    formationIds(selectedDeferred);
details.projectionRecoveryOrder = projectionRecoveryOrder;
details.deferredCrossEdgeCount = ...
    nnz(candidateResidual) - nnz(residual);
details.jointEvaluationCount = jointEvaluationCount;
details.jointRetentionRisk = jointScore.risk;
details.jointMinimumSupportedLabelRetentionRatio = ...
    jointScore.minimumSupportedLabelRetentionRatio;
details.jointDecisionThresholdCrossingCount = ...
    jointScore.decisionThresholdCrossingCount;
details.temporalSensorStrong = sensorStrong;
details.temporalFormationStrong = formationStrong;
details.currentPosteriorUsed = true;
details.currentLinkReliabilityUsed = true;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function [risk, details] = scoreNetwork( ...
        context, adjacency, weights, protocol, reference)
options = struct('maximumIncomingCount', ...
    protocol.maximumIncomingCountForOutcomeEnumeration);
if ~isempty(reference)
    options.referenceNetworkDetails = reference.details;
    options.referenceAdjacency = reference.adjacency;
    options.referenceFusionWeights = reference.weights;
end
[risk, details] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, adjacency, weights, options);
end

function score = scoreRetention(reference, candidate, groupIds, protocol)
options = struct( ...
    'referenceSupportThreshold', protocol.referenceSupportThreshold, ...
    'decisionExistenceThreshold', ...
        protocol.decisionExistenceThreshold, ...
    'receiverTailFraction', ...
        protocol.retentionReceiverTailFraction, ...
    'receiverTailWeight', protocol.retentionReceiverTailWeight);
[risk, details] = computeLmbReferenceExistenceRetentionRisk( ...
    reference, candidate, groupIds, options);
minimumFormationCardinalityChange = min( ...
    details.formationMeanCardinalityChange);
minimumSupportedLabelRetentionRatio = min( ...
    details.minimumSupportedLabelRetentionRatio);
decisionThresholdCrossingCount = sum( ...
    details.decisionThresholdCrossingCount);
safe = risk <= protocol.maximumRetentionRisk + 1e-12 && ...
    minimumFormationCardinalityChange >= ...
        protocol.minimumFormationMeanCardinalityChange - 1e-12 && ...
    minimumSupportedLabelRetentionRatio >= ...
        protocol.minimumSupportedLabelRetentionRatio - 1e-12 && ...
    decisionThresholdCrossingCount <= ...
        protocol.maximumDecisionThresholdCrossingCount;
score = struct( ...
    'risk', risk, ...
    'details', details, ...
    'minimumFormationCardinalityChange', ...
        minimumFormationCardinalityChange, ...
    'minimumSupportedLabelRetentionRatio', ...
        minimumSupportedLabelRetentionRatio, ...
    'decisionThresholdCrossingCount', ...
        decisionThresholdCrossingCount, ...
    'safe', safe);
end

function values = expectedCardinality(distributions)
values = zeros(1, numel(distributions));
for receiverIdx = 1:numel(distributions)
    distribution = distributions{receiverIdx};
    value = 0;
    for outcomeIdx = 1:numel(distribution.probability)
        value = value + distribution.probability(outcomeIdx) * ...
            sum(distribution.summary{outcomeIdx}.existence);
    end
    values(receiverIdx) = value;
end
end

function residual = removeFormationInputs( ...
        candidate, crossResidual, groupIds, deferredIds)
residual = candidate;
for idx = 1:numel(deferredIds)
    receivers = groupIds == deferredIds(idx);
    residual(receivers, :) = residual(receivers, :) & ...
        ~crossResidual(receivers, :);
end
end

function weights = buildWeights(dominant, residual, protocol)
nodeCount = size(dominant, 1);
weights = zeros(nodeCount);
weights(dominant) = protocol.dominantWeight;
weights(residual) = protocol.activeResidualWeight;
weights(1:nodeCount+1:end) = 1 - sum(weights, 2)';
end

function [pulse, known] = resolvePreviousPulse( ...
        context, protocol, nodeCount)
pulse = false(nodeCount);
known = false;
if ~isfield(context, 'previousAdjacencyHistory') || ...
        ~isfield(context, 'previousAdjacencyHistoryTimes')
    return;
end
times = reshape(context.previousAdjacencyHistoryTimes, 1, []);
index = find(times == context.currentTime - protocol.period, 1);
if isempty(index)
    return;
end
pulse = logical(context.previousAdjacencyHistory(:, :, index));
known = true;
end

function [sensorStrong, formationStrong] = temporalStrong( ...
        previousPulse, currentDominant, currentResidual, groupIds)
unionRoute = previousPulse | currentDominant | currentResidual;
sensorStrong = isStronglyConnectedLocal(unionRoute);
formationStrong = isStronglyConnectedLocal( ...
    collapseToFormations(unionRoute, groupIds));
end

function collapsed = collapseToFormations(adjacency, groupIds)
ids = unique(groupIds, 'stable');
collapsed = false(numel(ids));
for receiverIdx = 1:numel(ids)
    receivers = groupIds == ids(receiverIdx);
    for senderIdx = 1:numel(ids)
        senders = groupIds == ids(senderIdx);
        collapsed(receiverIdx, senderIdx) = any(any( ...
            adjacency(receivers, senders)));
    end
end
collapsed(1:size(collapsed, 1)+1:end) = false;
end

function connected = isStronglyConnectedLocal(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
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
passed = all(visited);
end

function details = fallbackDetails(formationIds, formationCount, ...
        previousPulseKnown, previouslyServed, sensorStrong, formationStrong)
details = struct();
details.contractVersion = ...
    'formation-b4-v53-exact-selective-residual-v1';
details.formationIds = formationIds;
details.referenceFormationExpectedCardinality = ...
    nan(1, formationCount);
details.formationRetentionDebtFraction = -inf(1, formationCount);
details.retentionDebtThresholdByFormation = nan(1, formationCount);
details.singleActionAvailableMask = false(1, formationCount);
details.singleActionSafetyMask = false(1, formationCount);
details.requestedDeferredFormationMask = false(1, formationCount);
details.previousPulseKnown = previousPulseKnown;
details.previousPulseServedFormationMask = previouslyServed;
details.eligibleDeferredFormationMask = false(1, formationCount);
details.selectedDeferredFormationMask = false(1, formationCount);
details.selectedDeferredFormationIds = zeros(1, 0);
details.projectionRecoveryOrder = zeros(1, 0);
details.deferredCrossEdgeCount = 0;
details.jointEvaluationCount = 0;
details.jointRetentionRisk = 0;
details.jointMinimumSupportedLabelRetentionRatio = 1;
details.jointDecisionThresholdCrossingCount = 0;
details.temporalSensorStrong = sensorStrong;
details.temporalFormationStrong = formationStrong;
details.currentPosteriorUsed = false;
details.currentLinkReliabilityUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function score = emptyJointScore()
score = struct( ...
    'risk', 0, ...
    'minimumSupportedLabelRetentionRatio', 1, ...
    'decisionThresholdCrossingCount', 0, ...
    'safe', true);
end
