function [adjacencySequence, fusionWeightSequence, details] = ...
    buildIndexEquivariantResidualDutyCycleSchedule(context, options)
% BUILDINDEXEQUIVARIANTRESIDUALDUTYCYCLESCHEDULE
% Two-rate V43 route: retain the dominant layer every step and transmit a
% selected residual layer only on one registered pulse page per period.
%
% Matrices use receiver rows and sender columns.  The construction never
% changes the physical V43 edges; it only changes when a residual edge is
% attempted and returns omitted mass to the receiver's self weight.

if nargin < 2 || isempty(options)
    options = struct();
end
allowedFields = {'period', 'dutyLayer', 'phasePattern', ...
    'activeResidualWeight', 'dominantWeight', ...
    'referenceResidualWeight'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('IndexEquivariantResidualDutyCycle:InvalidOptions', ...
        'The residual duty-cycle options are malformed.');
end

period = getField(options, 'period', 2);
dutyLayer = lower(char(getField(options, 'dutyLayer', 'local')));
phasePattern = lower(char(getField( ...
    options, 'phasePattern', 'synchronized')));
dominantWeight = getField(options, 'dominantWeight', 0.70);
referenceResidualWeight = getField( ...
    options, 'referenceResidualWeight', 0.05);
activeResidualWeight = getField( ...
    options, 'activeResidualWeight', ...
    period * referenceResidualWeight);
if ~isscalar(period) || ~isfinite(period) || period < 2 || ...
        period ~= round(period) || period > 5 || ...
        ~ismember(dutyLayer, {'local', 'cross', 'all'}) || ...
        ~ismember(phasePattern, {'synchronized', ...
            'formation-staggered', 'receiver-staggered'}) || ...
        any(~isfinite([dominantWeight, referenceResidualWeight, ...
            activeResidualWeight])) || dominantWeight <= 0 || ...
        referenceResidualWeight <= 0 || activeResidualWeight <= 0 || ...
        dominantWeight + referenceResidualWeight >= 1 || ...
        1 - dominantWeight - activeResidualWeight < 0.05 - 1e-12
    error('IndexEquivariantResidualDutyCycle:InvalidOptions', ...
        'The period, duty layer or fusion weights are invalid.');
end

[referenceAdjacency, referenceDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(context, struct( ...
        'dominantWeight', dominantWeight, ...
        'residualWeight', referenceResidualWeight));
dominantAdjacency = logical(referenceDetails.dominantAdjacency);
residualAdjacency = logical(referenceDetails.residualAdjacency);
nodeCount = size(referenceAdjacency, 1);
formationUidsBySensor = reshape( ...
    referenceDetails.construction.formationPhysicalUidsBySensor, 1, []);
crossMask = formationUidsBySensor(:) ~= formationUidsBySensor(:)';
crossResidualAdjacency = residualAdjacency & crossMask;
localResidualAdjacency = residualAdjacency & ~crossMask;
switch dutyLayer
    case 'local'
        dutyAdjacency = localResidualAdjacency;
    case 'cross'
        dutyAdjacency = crossResidualAdjacency;
    case 'all'
        dutyAdjacency = residualAdjacency;
end
alwaysAdjacency = residualAdjacency & ~dutyAdjacency;
dutyPhaseByReceiver = assignDutyPhases( ...
    dutyAdjacency, formationUidsBySensor, ...
    referenceDetails.construction.sensorPhysicalUids, ...
    period, phasePattern);

adjacencySequence = false(nodeCount, nodeCount, period);
fusionWeightSequence = zeros(nodeCount, nodeCount, period);
messageCounts = zeros(1, period);
crossMessageCounts = zeros(1, period);
localResidualMessageCounts = zeros(1, period);
for phaseIdx = 1:period
    activeDutyReceivers = dutyPhaseByReceiver == phaseIdx;
    activeDutyAdjacency = dutyAdjacency & ...
        repmat(activeDutyReceivers(:), 1, nodeCount);
    activeResidual = alwaysAdjacency | activeDutyAdjacency;
    adjacency = dominantAdjacency | activeResidual;
    weights = zeros(nodeCount);
    weights(dominantAdjacency) = dominantWeight;
    weights(alwaysAdjacency) = referenceResidualWeight;
    weights(activeDutyAdjacency) = activeResidualWeight;
    weights(1:nodeCount+1:end) = 1 - sum(weights, 2)';
    adjacencySequence(:, :, phaseIdx) = adjacency;
    fusionWeightSequence(:, :, phaseIdx) = weights;
    messageCounts(phaseIdx) = nnz(adjacency);
    crossMessageCounts(phaseIdx) = nnz(adjacency & crossMask);
    localResidualMessageCounts(phaseIdx) = ...
        nnz(adjacency & localResidualAdjacency);
end

physical = logical(context.physicalAdjacency);
referenceMessageCount = nnz(referenceAdjacency);
referenceMessagesPerPeriod = period * referenceMessageCount;
actualMessagesPerPeriod = sum(messageCounts);
if any(reshape(adjacencySequence & ...
        ~repmat(physical, 1, 1, period), [], 1)) || ...
        any(abs(reshape(sum(fusionWeightSequence, 2), [], 1) - 1) ...
            > 1e-12) || ...
        any(fusionWeightSequence(:) < -1e-12) || ...
        any(fusionWeightSequence(:) > 1 + 1e-12) || ...
        ~isequal(any(adjacencySequence, 3), referenceAdjacency) || ...
        actualMessagesPerPeriod >= referenceMessagesPerPeriod
    error('IndexEquivariantResidualDutyCycle:InvalidSchedule', ...
        'The constructed duty-cycle schedule violates its route contract.');
end

formationSequence = collapseToPhysicalFormations( ...
    adjacencySequence, formationUidsBySensor);
sensorUnionStrongConnected = isStronglyConnected( ...
    any(adjacencySequence, 3));
formationUnionStrongConnected = isStronglyConnected( ...
    any(formationSequence, 3));
if ~sensorUnionStrongConnected || ~formationUnionStrongConnected
    error('IndexEquivariantResidualDutyCycle:InvalidSchedule', ...
        'The period union does not preserve the registered information flow.');
end

details = struct();
details.contractVersion = ...
    'index-equivariant-residual-duty-cycle-schedule-v1';
details.mode = 'index-equivariant-residual-duty-cycle';
details.dutyLayer = dutyLayer;
details.phasePattern = phasePattern;
details.period = period;
details.pulsePhase = NaN;
if strcmp(phasePattern, 'synchronized')
    details.pulsePhase = 1;
end
details.currentAbsolutePhase = ...
    mod(getField(context, 'currentTime', 1) - 1, period) + 1;
details.dutyPhaseByReceiver = dutyPhaseByReceiver;
details.sensorPhysicalUids = ...
    referenceDetails.construction.sensorPhysicalUids;
details.dominantWeight = dominantWeight;
details.referenceResidualWeight = referenceResidualWeight;
details.activeResidualWeight = activeResidualWeight;
details.averageDutyResidualWeight = ...
    activeResidualWeight / period;
details.residualMassMatchedToReference = ...
    abs(activeResidualWeight / period - referenceResidualWeight) <= 1e-12;
details.referenceAdjacency = referenceAdjacency;
details.referenceFusionWeights = referenceDetails.fusionWeightMatrix;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.crossResidualAdjacency = crossResidualAdjacency;
details.localResidualAdjacency = localResidualAdjacency;
details.dutyAdjacency = dutyAdjacency;
details.alwaysAdjacency = alwaysAdjacency;
details.referenceMessageCountPerStep = referenceMessageCount;
details.referenceMessagesPerPeriod = referenceMessagesPerPeriod;
details.messageCountsByPhase = messageCounts;
details.crossMessageCountsByPhase = crossMessageCounts;
details.localResidualMessageCountsByPhase = ...
    localResidualMessageCounts;
details.messagesPerPeriod = actualMessagesPerPeriod;
details.messageSavingCountPerPeriod = ...
    referenceMessagesPerPeriod - actualMessagesPerPeriod;
details.messageSavingFractionPerPeriod = ...
    details.messageSavingCountPerPeriod / referenceMessagesPerPeriod;
details.maximumMessagesPerReceiver = max(reshape( ...
    sum(adjacencySequence, 2), [], 1));
details.periodUnionEqualsReference = ...
    isequal(any(adjacencySequence, 3), referenceAdjacency);
details.periodUnionSensorStrongConnected = sensorUnionStrongConnected;
details.periodUnionFormationStrongConnected = ...
    formationUnionStrongConnected;
details.referenceRouteIndexEquivarianceInherited = true;
details.temporalRuleIndependentOfArrayIndex = true;
details.currentGeometryUsed = true;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.posteriorUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
details.validationClaimAllowed = false;
details.developmentEvidenceOnly = true;
end

function phases = assignDutyPhases(dutyAdjacency, ...
        formationUidsBySensor, sensorUids, period, pattern)
nodeCount = size(dutyAdjacency, 1);
phases = nan(1, nodeCount);
dutyReceivers = find(sum(dutyAdjacency, 2) == 1)';
if any(sum(dutyAdjacency, 2) > 1)
    error('IndexEquivariantResidualDutyCycle:InvalidReference', ...
        'The V43 reference has more than one residual edge per receiver.');
end
formationUids = sort(unique(formationUidsBySensor));
for receiver = dutyReceivers
    formationRank = find( ...
        formationUids == formationUidsBySensor(receiver));
    members = find(formationUidsBySensor == ...
        formationUidsBySensor(receiver));
    [~, memberOrder] = sort(sensorUids(members));
    orderedMembers = members(memberOrder);
    localRank = find(orderedMembers == receiver);
    switch pattern
        case 'synchronized'
            rawPhase = 1;
        case 'formation-staggered'
            rawPhase = formationRank;
        case 'receiver-staggered'
            rawPhase = formationRank + localRank - 1;
    end
    phases(receiver) = mod(rawPhase - 1, period) + 1;
end
end

function pages = collapseToPhysicalFormations(sensorPages, uidBySensor)
uids = sort(unique(uidBySensor));
pages = false(numel(uids), numel(uids), size(sensorPages, 3));
for pageIdx = 1:size(sensorPages, 3)
    for receiverIdx = 1:numel(uids)
        receivers = uidBySensor == uids(receiverIdx);
        for senderIdx = 1:numel(uids)
            senders = uidBySensor == uids(senderIdx);
            pages(receiverIdx, senderIdx, pageIdx) = ...
                any(any(sensorPages(receivers, senders, pageIdx)));
        end
    end
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

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
