function bank = buildTemporalAcquireBroadcastV86ActionBank(context, options)
% BUILDTEMPORALACQUIREBROADCASTV86ACTIONBANK Frozen two-phase route.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getTemporalAcquireBroadcastV86Protocol();
sourceBank = buildBraidedHandoverH3V84ActionBank(context, options);
caseInfo = sourceBank.sourceCase;
groupIds = sourceBank.groupIds;
referenceAdjacency = logical(sourceBank.actionAdjacency(:, :, 1));
referenceWeights = sourceBank.actionFusionWeights(:, :, 1);
acquireAdjacency = logical(sourceBank.actionAdjacency(:, :, 2));
acquireWeights = sourceBank.actionFusionWeights(:, :, 2);
physicalAdjacency = logical(context.physicalAdjacency);

gatewayIdx = caseInfo.receiverIdx;
receiverFormationId = groupIds(gatewayIdx);
formationMembers = reshape(find( ...
    groupIds == receiverFormationId), 1, []);
if numel(formationMembers) ~= protocol.expectedFormationSize
    error('TemporalAcquireBroadcastV86:FormationSizeDrift', ...
        'The frozen V86 gateway formation no longer has six sensors.');
end

broadcastAdjacency = referenceAdjacency;
broadcastWeights = referenceWeights;
broadcastReceivers = zeros(1, 0);
displacedDominantSenders = zeros(1, 0);
removedResidualSenders = zeros(1, 0);
for receiverIdx = formationMembers
    if receiverIdx == gatewayIdx
        continue;
    end
    dominantSender = unique(find( ...
        referenceAdjacency(receiverIdx, :) & ...
        groupIds == receiverFormationId & ...
        abs(referenceWeights(receiverIdx, :) - ...
            protocol.dominantWeight) <= 1e-12));
    residualSender = unique(find( ...
        referenceAdjacency(receiverIdx, :) & ...
        abs(referenceWeights(receiverIdx, :) - ...
            protocol.sourceWeight) <= 1e-12));
    if numel(dominantSender) ~= 1 || ...
            numel(residualSender) ~= 1 || ...
            dominantSender == gatewayIdx || ...
            ~physicalAdjacency(receiverIdx, gatewayIdx)
        error('TemporalAcquireBroadcastV86:BroadcastSlotDrift', ...
            ['A receiver no longer exposes one physical gateway, one ', ...
             '0.70 within-formation slot and one 0.05 residual slot.']);
    end

    if residualSender == gatewayIdx
        broadcastWeights(receiverIdx, dominantSender) = ...
            protocol.sourceWeight;
        broadcastWeights(receiverIdx, gatewayIdx) = ...
            protocol.dominantWeight;
    else
        broadcastAdjacency(receiverIdx, residualSender) = false;
        broadcastWeights(receiverIdx, residualSender) = 0;
        broadcastWeights(receiverIdx, dominantSender) = ...
            protocol.sourceWeight;
        broadcastAdjacency(receiverIdx, gatewayIdx) = true;
        broadcastWeights(receiverIdx, gatewayIdx) = ...
            protocol.dominantWeight;
    end
    broadcastReceivers(end + 1) = receiverIdx; %#ok<AGROW>
    displacedDominantSenders(end + 1) = dominantSender; %#ok<AGROW>
    removedResidualSenders(end + 1) = residualSender; %#ok<AGROW>
end

if numel(broadcastReceivers) ~= ...
        protocol.expectedBroadcastReceiverCount || ...
        numel(unique(broadcastReceivers)) ~= ...
            numel(broadcastReceivers)
    error('TemporalAcquireBroadcastV86:BroadcastBreadthDrift', ...
        'The V86 broadcast must change exactly five receiver rows.');
end
validateRouteParity( ...
    broadcastAdjacency, broadcastWeights, referenceAdjacency, ...
    referenceWeights, physicalAdjacency);

referenceSequence = repmat(referenceAdjacency, 1, 1, 3);
candidateSequence = cat(3, acquireAdjacency, ...
    broadcastAdjacency, referenceAdjacency);
[referenceSensorPass, referenceFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, referenceSequence, groupIds);
[candidateSensorPass, candidateFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, candidateSequence, groupIds);
if ~all(referenceSensorPass) || ~all(referenceFormationPass) || ...
        ~all(candidateSensorPass) || ~all(candidateFormationPass)
    error('TemporalAcquireBroadcastV86:RollingB3Failure', ...
        'The frozen acquire--broadcast sequence fails rolling-B3.');
end

acquireName = sourceBank.actionNames{2};
broadcastName = sprintf( ...
    'broadcast-gateway-%d-formation-%d', ...
    gatewayIdx, receiverFormationId);
bank = struct();
bank.contractVersion = ...
    'temporal-acquire-broadcast-primary-action-bank-v86-v1';
bank.protocolId = protocol.id;
bank.outcomePolicyName = protocol.outcomePolicyName;
bank.referenceMode = 'current-physical-tree';
bank.nodeCount = numel(groupIds);
bank.formationCount = numel(unique(groupIds, 'stable'));
bank.groupIds = groupIds;
bank.actionCount = 3;
bank.referenceActionIndex = 1;
bank.actionNames = {'reference', acquireName, broadcastName};
bank.actionFormationIds = { ...
    zeros(1, 0), receiverFormationId, receiverFormationId};
bank.actionSlotIndices = { ...
    zeros(1, 0), 1, 1:numel(broadcastReceivers)};
bank.actionAdjacency = cat(3, referenceAdjacency, ...
    acquireAdjacency, broadcastAdjacency);
bank.actionFusionWeights = cat(3, referenceWeights, ...
    acquireWeights, broadcastWeights);
bank.actionMessageCounts = [ ...
    nnz(referenceAdjacency), nnz(acquireAdjacency), ...
    nnz(broadcastAdjacency)];
bank.actionWithinReferencePayload = [true, true, true];
bank.actionPosteriorSafetyMask = [true, true, false];
bank.actionPosteriorProxyAllowed = [true, true, false];
bank.actionPosteriorObjective = [ ...
    0, caseInfo.sourceLocalNetFraction, 0];
bank.actionPredictedNetSavingBytes = [0, 0, 0];
bank.actionPayloadBytes = [NaN, NaN, NaN];
bank.actionFormationIndex = [0, receiverFormationId, ...
    receiverFormationId];
bank.actionModeIndex = [1, 2, 3];
bank.modeTrustWeights = [0, protocol.sourceWeight, ...
    protocol.dominantWeight];
bank.actionRollingB3SensorPass = [ ...
    referenceSensorPass; candidateSensorPass; candidateSensorPass];
bank.actionRollingB3FormationPass = [ ...
    referenceFormationPass; candidateFormationPass; ...
    candidateFormationPass];
bank.safeActionIndices = 1:2;
bank.referenceMessageCount = nnz(referenceAdjacency);
bank.selectedMessageCount = nnz(broadcastAdjacency);
bank.selectedUtilityMass = caseInfo.sourceLocalNetFraction;
bank.selectedNetworkAverageNetFraction = ...
    caseInfo.sourceLocalNetFraction;
bank.directSafeFormationIds = receiverFormationId;
bank.changedReceiverIndices = unique([ ...
    gatewayIdx, broadcastReceivers], 'stable');
bank.acquireReceiverIdx = gatewayIdx;
bank.broadcastReceiverIndices = broadcastReceivers;
bank.broadcastDisplacedDominantSenderIndices = ...
    displacedDominantSenders;
bank.broadcastRemovedResidualSenderIndices = ...
    removedResidualSenders;
bank.broadcastReceiverCoverageFraction = ...
    numel(broadcastReceivers) / numel(formationMembers);
bank.candidateTrustWeights = [ ...
    protocol.sourceWeight, protocol.dominantWeight];
bank.selfFundedTrust = true;
bank.trustFundingSource = 'reference-row-weight-multiset';
bank.rollingB3DurationSteps = 2;
bank.rollingB3ActionUse = logical([1, 1, 0]);
bank.sourceCase = caseInfo;
bank.currentSenderNoveltyFraction = ...
    sourceBank.currentSenderNoveltyFraction;
bank.temporalPosteriorSafetyClaim = false;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.openedDevelopmentEvidenceOnly = true;
bank.validationClaimAllowed = false;
end

function validateRouteParity( ...
        adjacency, weights, referenceAdjacency, referenceWeights, physical)
support = adjacency | logical(eye(size(adjacency, 1)));
valid = ~any(adjacency(:) & ~physical(:)) && ...
    nnz(adjacency) == nnz(referenceAdjacency) && ...
    isequal(sum(adjacency, 2), sum(referenceAdjacency, 2)) && ...
    rowWeightMultisetParity(weights, referenceWeights) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(isfinite(weights(:))) && all(weights(:) >= -1e-12) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
if ~valid
    error('TemporalAcquireBroadcastV86:RouteParityFailure', ...
        'The V86 broadcast violates its fixed-budget route contract.');
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

function [sensorPass, formationPass] = ...
        rollingB3Pass(previousHistory, sequence, groupIds)
history = logical(previousHistory(:, :, end-1:end));
sensorPass = false(1, size(sequence, 3));
formationPass = false(1, size(sequence, 3));
for stepIdx = 1:size(sequence, 3)
    pages = cat(3, history, sequence(:, :, stepIdx));
    window = any(pages(:, :, max(1, end-2):end), 3);
    sensorPass(stepIdx) = isStronglyConnected(window);
    formationPass(stepIdx) = isStronglyConnected( ...
        collapseToFormations(window, groupIds));
    history(:, :, end + 1) = sequence(:, :, stepIdx); %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
    end
end
end

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(reshape(groupIds, 1, []), 'stable');
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
