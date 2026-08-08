function [residual, details] = ...
    selectFormationB4V51RetentionGatedResidual( ...
        context, candidateDominant, candidateResidual, protocol)
% SELECTFORMATIONB4V51RETENTIONGATEDRESIDUAL
% Defer cross-formation residual inputs whose current sender posterior has
% materially less label-existence mass than the receiver. Restore the
% least risky deferred formations until the previous/current pulse union
% remains strongly connected.

nodeCount = numel(context.localPosteriorBySensor);
groups = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationIds = unique(groups, 'stable');
formationCount = numel(formationIds);
candidateResidual = logical(candidateResidual);
candidateDominant = logical(candidateDominant);
if numel(groups) ~= nodeCount || ...
        ~isequal(size(candidateResidual), [nodeCount, nodeCount]) || ...
        ~isequal(size(candidateDominant), [nodeCount, nodeCount])
    error('FormationB4V51Retention:InvalidInput', ...
        'The residual route or formation registry is invalid.');
end

crossMask = groups(:) ~= groups(:)';
crossResidual = candidateResidual & crossMask;
[receivers, senders] = find(crossResidual);
if isempty(receivers)
    error('FormationB4V51Retention:NoCrossResidual', ...
        'The candidate cycle has no cross-formation residual input.');
end

[features, mask, names, metadata] = ...
    computeLabelSetDirectedActionFeatures( ...
        context, receivers, senders, struct());
receiverExistence = featureValue( ...
    features, names, 'receiver_existence');
senderExistence = featureValue( ...
    features, names, 'sender_existence');
edgeReliability = resolveReliability( ...
    context, receivers, senders);
edgeReceiverMass = reshape(sum( ...
    receiverExistence .* double(mask), 2), 1, []);
edgeNegativeGap = reshape(sum(max( ...
    receiverExistence - senderExistence, 0) .* ...
    double(mask), 2), 1, []);
edgePositiveGain = reshape(sum(max( ...
    senderExistence - receiverExistence, 0) .* ...
    double(mask), 2), 1, []);
edgeDebt = protocol.activeResidualWeight .* ...
    reshape(edgeReliability, 1, []) .* ...
    (edgeNegativeGap - edgePositiveGain) ./ ...
    max(edgeReceiverMass, 1);

formationDebt = zeros(1, formationCount);
formationCrossEdgeCount = zeros(1, formationCount);
for formationIdx = 1:formationCount
    edgeMask = groups(receivers) == formationIds(formationIdx);
    values = edgeDebt(edgeMask);
    if isempty(values)
        formationDebt(formationIdx) = -inf;
        continue;
    end
    formationCrossEdgeCount(formationIdx) = numel(values);
    formationDebt(formationIdx) = ...
        protocol.retentionDebtAggregationMeanWeight * mean(values) + ...
        protocol.retentionDebtAggregationTailWeight * max(values);
end

[previousPulse, previousPulseKnown] = resolvePreviousPulse( ...
    context, protocol, nodeCount);
previouslyServed = true(1, formationCount);
if previousPulseKnown
    for formationIdx = 1:formationCount
        receiverMask = groups == formationIds(formationIdx);
        senderMask = ~receiverMask;
        previouslyServed(formationIdx) = any(any( ...
            previousPulse(receiverMask, senderMask)));
    end
end
requested = formationDebt >= ...
    protocol.retentionDebtOnFraction - 1e-12;
if ~previousPulseKnown && ...
        protocol.referenceFallbackOnMissingHistory
    requested(:) = false;
end
eligible = requested & previouslyServed;
selectedDeferred = eligible;
projectionRecoveryOrder = zeros(1, 0);
residual = removeFormationInputs( ...
    candidateResidual, crossResidual, groups, ...
    formationIds(selectedDeferred));

if previousPulseKnown
    [sensorStrong, formationStrong] = temporalStrong( ...
        previousPulse, candidateDominant, residual, groups);
    while ~(sensorStrong && formationStrong) && ...
            any(selectedDeferred)
        selectedIndices = find(selectedDeferred);
        [~, localIdx] = min(formationDebt(selectedIndices));
        restoreIdx = selectedIndices(localIdx);
        selectedDeferred(restoreIdx) = false;
        projectionRecoveryOrder(end + 1) = ...
            formationIds(restoreIdx); %#ok<AGROW>
        residual = removeFormationInputs( ...
            candidateResidual, crossResidual, groups, ...
            formationIds(selectedDeferred));
        [sensorStrong, formationStrong] = temporalStrong( ...
            previousPulse, candidateDominant, residual, groups);
    end
else
    currentRoute = candidateDominant | candidateResidual;
    sensorStrong = isStronglyConnectedLocal(currentRoute);
    formationStrong = isStronglyConnectedLocal( ...
        collapseToFormations(currentRoute, groups));
end

if ~(sensorStrong && formationStrong) || ...
        any(residual(:) & ~candidateResidual(:))
    error('FormationB4V51Retention:InvalidProjection', ...
        'The retention projection violated the temporal route contract.');
end

details = struct();
details.contractVersion = ...
    'formation-b4-v51-retention-gated-residual-v1';
details.formationIds = formationIds;
details.formationCrossEdgeCount = formationCrossEdgeCount;
details.edgeReceivers = reshape(receivers, 1, []);
details.edgeSenders = reshape(senders, 1, []);
details.edgeReceiverExistenceMass = edgeReceiverMass;
details.edgeNegativeExistenceGap = edgeNegativeGap;
details.edgePositiveExistenceGain = edgePositiveGain;
details.edgeReliability = reshape(edgeReliability, 1, []);
details.edgeRetentionDebtFraction = edgeDebt;
details.formationRetentionDebtFraction = formationDebt;
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
details.temporalSensorStrong = sensorStrong;
details.temporalFormationStrong = formationStrong;
details.labelFeatureMetadata = metadata;
details.currentPosteriorUsed = true;
details.currentLinkReliabilityUsed = true;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function value = featureValue(features, names, name)
index = find(strcmp(names, name), 1);
if isempty(index)
    error('FormationB4V51Retention:MissingFeature', ...
        'Required label feature %s is unavailable.', name);
end
value = features(:, :, index);
end

function reliability = resolveReliability(context, receivers, senders)
reliability = ones(numel(receivers), 1);
if ~isfield(context.commConfig, 'pDropByEdge') || ...
        isempty(context.commConfig.pDropByEdge)
    return;
end
drop = context.commConfig.pDropByEdge;
for edgeIdx = 1:numel(receivers)
    if ndims(drop) >= 3
        timeIdx = min(context.currentTime, size(drop, 3));
        probability = drop(senders(edgeIdx), ...
            receivers(edgeIdx), timeIdx);
    else
        probability = drop(senders(edgeIdx), receivers(edgeIdx));
    end
    reliability(edgeIdx) = 1 - min(max(probability, 0), 1);
end
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

function residual = removeFormationInputs( ...
        candidate, crossResidual, groups, deferredIds)
residual = candidate;
for idx = 1:numel(deferredIds)
    receivers = groups == deferredIds(idx);
    residual(receivers, :) = residual(receivers, :) & ...
        ~crossResidual(receivers, :);
end
end

function [sensorStrong, formationStrong] = temporalStrong( ...
        previousPulse, currentDominant, currentResidual, groups)
unionRoute = previousPulse | currentDominant | currentResidual;
sensorStrong = isStronglyConnectedLocal(unionRoute);
formationStrong = isStronglyConnectedLocal( ...
    collapseToFormations(unionRoute, groups));
end

function collapsed = collapseToFormations(adjacency, groups)
ids = unique(groups, 'stable');
collapsed = false(numel(ids));
for receiverIdx = 1:numel(ids)
    receivers = groups == ids(receiverIdx);
    for senderIdx = 1:numel(ids)
        senders = groups == ids(senderIdx);
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
if isempty(adjacency)
    passed = false;
    return;
end
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
