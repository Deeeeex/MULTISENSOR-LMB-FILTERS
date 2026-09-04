function [adjacency, details] = ...
        selectTemporallyBalancedMinimumBackboneV272Policy(context)
% SELECTTEMPORALLYBALANCEDMINIMUMBACKBONEV272POLICY Rotate safe gateways.
%
% Matrices use receiver rows and sender columns. The V242 formation tree,
% local cycles, fusion weights and message count are retained. Only the
% cross-formation sensor embedding is changed. Candidate links must remain
% close to the current V242 link reliability; within that safe set the
% bounded route history gives priority to least-recently served receivers;
% the causal route product favors senders with broader external influence.

protocol = getTemporallyBalancedMinimumBackboneV272Protocol();
[referenceAdjacency, reference] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
identity = buildIdentity(context);
history = normalizeHistory(context, identity.nodeCount, ...
    protocol.historyDepth);
[receiverAge, receiverCount, senderAge, senderCount] = ...
    temporalServiceState(history, identity);
pastInfluence = reconstructPastInfluence(history, identity, protocol);
referenceAssignment = assignmentFromCrossAdjacency( ...
    reference.crossResidualAdjacency, identity);
[requestedAssignment, selection] = selectBalancedAssignment( ...
    referenceAssignment, context, identity, receiverAge, ...
    receiverCount, senderAge, senderCount, pastInfluence, protocol);

[adjacency, projected] = selectCausalGatewayEmbeddingV250Policy( ...
    context, requestedAssignment);
if ~projected.requestedGatewayAssignmentApplied
    error('TemporallyBalancedBackboneV272:UnexpectedProjectionFallback', ...
        'A V272 assignment passed locally but failed V250 projection.');
end

cross = logical(projected.crossResidualAdjacency);
reliability = 1 - context.commConfig.pDropByEdge';
[referenceReliability, selectedReliability] = alignedReliability( ...
    referenceAssignment, projected.appliedGatewayAssignment, ...
    reliability, identity);
drop = referenceReliability - selectedReliability;
ratio = selectedReliability ./ max(referenceReliability, realmin);
qualityPassed = all(drop <= ...
        protocol.maximumReliabilityDrop + 1e-12) && ...
    all(ratio >= protocol.minimumReliabilityRatio - 1e-12);
expectedMessages = identity.nodeCount + ...
    2 * (identity.formationCount - 1);
hardGate = qualityPassed && nnz(adjacency) == expectedMessages && ...
    ~any(adjacency(:) & ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(adjacency) && ...
    all(abs(sum(projected.fusionWeightMatrix, 2) - 1) <= 1e-12);
if ~hardGate
    error('TemporallyBalancedBackboneV272:HardGateFailed', ...
        'The selected temporal gateway route violates a hard invariant.');
end

details = projected;
details.contractVersion = ...
    'temporally-balanced-minimum-backbone-v272-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'temporally-balanced-minimum-formation-backbone';
details.backboneMode = details.mode;
details.referenceAdjacency = logical(referenceAdjacency);
details.referenceGatewayAssignment = referenceAssignment;
details.requestedGatewayAssignment = requestedAssignment;
details.temporalHistoryDepth = protocol.historyDepth;
details.temporalHistoryCount = size(history, 3);
details.temporalHistoryMature = ...
    size(history, 3) >= protocol.historyDepth;
details.crossReceiverServiceAge = receiverAge;
details.crossReceiverServiceCount = receiverCount;
details.crossSenderServiceAge = senderAge;
details.crossSenderServiceCount = senderCount;
details.minimumReliabilityRatio = protocol.minimumReliabilityRatio;
details.maximumReliabilityDrop = protocol.maximumReliabilityDrop;
details.balanceSenderRoles = protocol.balanceSenderRoles;
details.senderSelectionMode = protocol.senderSelectionMode;
details.referenceCrossReliability = referenceReliability;
details.selectedCrossReliability = selectedReliability;
details.realizedMinimumReliabilityRatio = min(ratio);
details.realizedMaximumReliabilityDrop = max(drop);
details.qualityFloorPassed = qualityPassed;
details.selectionByReceivingFormation = selection;
details.crossResidualAdjacency = cross;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.architectureMinimumPassed = true;
details.instantaneousSensorStrong = true;
details.instantaneousFormationStrong = true;
details.previousPolicyHistoryUsed = ~isempty(history);
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
details.referenceFallbackUsed = false;
schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'temporally-balanced-minimum-backbone-v272-schedule-v1';
schedule.phase = 'v272-temporal-gateway-balance';
schedule.temporalHistoryCount = size(history, 3);
schedule.temporalHistoryMature = details.temporalHistoryMature;
schedule.realizedMinimumReliabilityRatio = min(ratio);
schedule.realizedMaximumReliabilityDrop = max(drop);
schedule.qualityFloorPassed = qualityPassed;
schedule.currentMessageCount = nnz(adjacency);
details.scheduleCertificate = schedule;
end

function identity = buildIdentity(context)
required = {'localPosteriorBySensor', 'physicalAdjacency', ...
    'sensorPhysicalUids', 'formationPhysicalUidsBySensor', ...
    'commConfig'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~isfield(context.commConfig, 'pDropByEdge')
    error('TemporallyBalancedBackboneV272:InvalidContext', ...
        'The observable route context is incomplete.');
end
identity = struct();
identity.nodeCount = numel(context.localPosteriorBySensor);
identity.sensorUids = reshape(context.sensorPhysicalUids, 1, []);
identity.formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
identity.formationUids = sort(unique(identity.formationBySensor));
identity.formationCount = numel(identity.formationUids);
identity.members = cell(1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    members = find(identity.formationBySensor == ...
        identity.formationUids(formationIdx));
    [~, order] = sort(identity.sensorUids(members));
    identity.members{formationIdx} = members(order);
end
matrixSize = [identity.nodeCount, identity.nodeCount];
if numel(identity.sensorUids) ~= identity.nodeCount || ...
        numel(unique(identity.sensorUids)) ~= identity.nodeCount || ...
        numel(identity.formationBySensor) ~= identity.nodeCount || ...
        ~isequal(size(context.physicalAdjacency), matrixSize) || ...
        ~isequal(size(context.commConfig.pDropByEdge), matrixSize)
    error('TemporallyBalancedBackboneV272:InvalidContext', ...
        'The physical identities or route matrices are malformed.');
end
end

function history = normalizeHistory(context, nodeCount, depth)
history = false(nodeCount, nodeCount, 0);
if ~isfield(context, 'previousAdjacencyHistory') || ...
        isempty(context.previousAdjacencyHistory)
    return;
end
history = logical(context.previousAdjacencyHistory);
if size(history, 1) ~= nodeCount || size(history, 2) ~= nodeCount
    error('TemporallyBalancedBackboneV272:InvalidHistory', ...
        'The selected-route history has the wrong dimensions.');
end
if size(history, 3) > depth
    history = history(:, :, end-depth+1:end);
end
end

function [receiverAge, receiverCount, senderAge, senderCount] = ...
        temporalServiceState(history, identity)
historyCount = size(history, 3);
crossMask = identity.formationBySensor(:) ~= ...
    identity.formationBySensor(:)';
receiverService = false(identity.nodeCount, historyCount);
senderService = false(identity.nodeCount, historyCount);
for pageIdx = 1:historyCount
    cross = logical(history(:, :, pageIdx)) & crossMask;
    receiverService(:, pageIdx) = any(cross, 2);
    senderService(:, pageIdx) = any(cross, 1)';
end
receiverAge = serviceAge(receiverService);
senderAge = serviceAge(senderService);
receiverCount = sum(receiverService, 2)';
senderCount = sum(senderService, 2)';
end

function age = serviceAge(service)
nodeCount = size(service, 1);
historyCount = size(service, 2);
age = repmat(historyCount + 1, 1, nodeCount);
for nodeIdx = 1:nodeCount
    latest = find(service(nodeIdx, :), 1, 'last');
    if ~isempty(latest)
        age(nodeIdx) = historyCount - latest;
    end
end
end

function assignment = assignmentFromCrossAdjacency(cross, identity)
[receivers, senders] = find(logical(cross));
assignment = zeros(numel(receivers), 4);
for edgeIdx = 1:numel(receivers)
    assignment(edgeIdx, :) = [ ...
        identity.formationBySensor(senders(edgeIdx)), ...
        identity.formationBySensor(receivers(edgeIdx)), ...
        identity.sensorUids(senders(edgeIdx)), ...
        identity.sensorUids(receivers(edgeIdx))];
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function [assignment, selection] = selectBalancedAssignment( ...
        reference, context, identity, receiverAge, receiverCount, ...
        senderAge, senderCount, pastInfluence, protocol)
reliability = 1 - context.commConfig.pDropByEdge';
assignment = zeros(0, 4);
selection = cell(1, identity.formationCount);
for receiverFormationIdx = 1:identity.formationCount
    receiverFormationUid = ...
        identity.formationUids(receiverFormationIdx);
    rows = find(reference(:, 2) == receiverFormationUid);
    slots = reference(rows, :);
    slots = sortrows(slots, [1, 2]);
    receivers = identity.members{receiverFormationIdx};
    choices = orderedSelections(receivers, size(slots, 1));
    best = emptyCandidate();
    eligibleReceivers = zeros(1, 0);
    eligibleSenders = zeros(1, 0);
    for choiceIdx = 1:size(choices, 1)
        candidateReceivers = choices(choiceIdx, :);
        candidateSenders = zeros(1, size(slots, 1));
        feasible = true;
        for slotIdx = 1:size(slots, 1)
            sourceFormationUid = slots(slotIdx, 1);
            sourceFormationIdx = find( ...
                identity.formationUids == sourceFormationUid, 1);
            referenceSender = uidToIndex(slots(slotIdx, 3), identity);
            referenceReceiver = uidToIndex(slots(slotIdx, 4), identity);
            qReference = reliability(referenceReceiver, referenceSender);
            threshold = max( ...
                protocol.minimumReliabilityRatio * qReference, ...
                qReference - protocol.maximumReliabilityDrop);
            [sender, senderScore] = selectSender( ...
                candidateReceivers(slotIdx), ...
                identity.members{sourceFormationIdx}, threshold, ...
                context, identity, reliability, senderAge, ...
                senderCount, pastInfluence, receiverFormationUid, ...
                protocol.balanceSenderRoles);
            if ~isfinite(sender)
                feasible = false;
                break;
            end
            candidateSenders(slotIdx) = sender;
            if slotIdx == 1
                senderScores = senderScore;
            else
                senderScores(slotIdx) = senderScore; %#ok<AGROW>
            end
        end
        if ~feasible
            continue;
        end
        candidate = scoreAssignment(candidateReceivers, ...
            candidateSenders, slots, context, identity, reliability, ...
            receiverAge, receiverCount, senderAge, senderCount, ...
            senderScores, protocol.balanceSenderRoles);
        eligibleReceivers = union(eligibleReceivers, ...
            candidateReceivers, 'stable');
        eligibleSenders = union(eligibleSenders, ...
            candidateSenders, 'stable');
        if ~best.valid || assignmentBetter(candidate, best)
            best = candidate;
        end
    end
    if ~best.valid
        error('TemporallyBalancedBackboneV272:NoSafeAssignment', ...
            'No reliability-floor gateway assignment is feasible.');
    end
    local = zeros(size(slots, 1), 4);
    for slotIdx = 1:size(slots, 1)
        local(slotIdx, :) = [slots(slotIdx, 1:2), ...
            identity.sensorUids(best.senders(slotIdx)), ...
            identity.sensorUids(best.receivers(slotIdx))];
    end
    assignment = [assignment; local]; %#ok<AGROW>
    selected = rmfield(best, 'valid');
    selected.eligibleReceiverIndices = eligibleReceivers;
    selected.eligibleSenderIndices = eligibleSenders;
    selected.eligibleReceiverSensorPhysicalUids = ...
        identity.sensorUids(eligibleReceivers);
    selected.eligibleSenderSensorPhysicalUids = ...
        identity.sensorUids(eligibleSenders);
    selection{receiverFormationIdx} = selected;
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function [sender, score] = selectSender(receiver, senders, threshold, ...
        context, identity, reliability, senderAge, senderCount, ...
        pastInfluence, receiverFormationUid, balance)
sender = NaN;
score = emptySenderScore();
for candidate = reshape(senders, 1, [])
    if ~logical(context.physicalAdjacency(receiver, candidate)) || ...
            reliability(receiver, candidate) < threshold - 1e-12
        continue;
    end
    current = emptySenderScore();
    current.valid = true;
    current.age = senderAge(candidate);
    current.count = senderCount(candidate);
    current.externalInfluence = sum(pastInfluence(candidate, ...
        identity.formationBySensor ~= receiverFormationUid));
    current.formationEntropy = formationEntropy( ...
        pastInfluence(candidate, :), identity);
    current.reliability = reliability(receiver, candidate);
    delta = context.positions(:, receiver) - ...
        context.positions(:, candidate);
    current.distance = sqrt(sum(delta .^ 2));
    current.uid = identity.sensorUids(candidate);
    if ~isfinite(sender) || senderBetter(current, score, balance)
        sender = candidate;
        score = current;
    end
end
end

function candidate = scoreAssignment(receivers, senders, slots, ...
        context, identity, reliability, receiverAge, receiverCount, ...
        senderAge, senderCount, senderScores, balance)
linear = sub2ind(size(reliability), receivers, senders);
q = reliability(linear);
delta = context.positions(:, receivers) - context.positions(:, senders);
candidate = emptyCandidate();
candidate.valid = true;
candidate.receivers = receivers;
candidate.senders = senders;
candidate.minimumReceiverAge = min(receiverAge(receivers));
candidate.totalReceiverAge = sum(receiverAge(receivers));
candidate.totalReceiverCount = sum(receiverCount(receivers));
senderExternal = [senderScores.externalInfluence];
senderEntropy = [senderScores.formationEntropy];
candidate.minimumSenderExternalInfluence = min(senderExternal);
candidate.totalSenderExternalInfluence = sum(senderExternal);
candidate.minimumSenderFormationEntropy = min(senderEntropy);
if balance
    candidate.minimumSenderAge = min(senderAge(senders));
    candidate.totalSenderAge = sum(senderAge(senders));
    candidate.totalSenderCount = sum(senderCount(senders));
else
    candidate.minimumSenderAge = 0;
    candidate.totalSenderAge = 0;
    candidate.totalSenderCount = 0;
end
candidate.bottleneckReliability = min(q);
candidate.totalLogReliability = sum(log(max(q, realmin)));
candidate.totalDistance = sum(sqrt(sum(delta .^ 2, 1)));
candidate.senderScores = senderScores;
edgeKey = [reshape(slots(:, 1), [], 1), ...
    reshape(identity.sensorUids(senders), [], 1), ...
    reshape(identity.sensorUids(receivers), [], 1)];
edgeKey = sortrows(edgeKey, [1, 2, 3]);
candidate.tieKey = reshape(edgeKey', 1, []);
end

function better = assignmentBetter(candidate, incumbent)
tolerance = 1e-12;
fields = {'minimumReceiverAge', 'totalReceiverAge', ...
    'minimumSenderExternalInfluence', ...
    'totalSenderExternalInfluence', ...
    'minimumSenderFormationEntropy', ...
    'minimumSenderAge', 'totalSenderAge', ...
    'bottleneckReliability', 'totalLogReliability'};
directions = ones(1, numel(fields));
for idx = 1:numel(fields)
    left = candidate.(fields{idx});
    right = incumbent.(fields{idx});
    if left > right + tolerance * directions(idx)
        better = true;
        return;
    elseif left < right - tolerance * directions(idx)
        better = false;
        return;
    end
end
if candidate.totalReceiverCount < incumbent.totalReceiverCount
    better = true;
    return;
elseif candidate.totalReceiverCount > incumbent.totalReceiverCount
    better = false;
    return;
end
if candidate.totalSenderCount < incumbent.totalSenderCount
    better = true;
    return;
elseif candidate.totalSenderCount > incumbent.totalSenderCount
    better = false;
    return;
end
if candidate.totalDistance < incumbent.totalDistance - tolerance
    better = true;
    return;
elseif candidate.totalDistance > incumbent.totalDistance + tolerance
    better = false;
    return;
end
better = lexicographicallyLess(candidate.tieKey, incumbent.tieKey);
end

function better = senderBetter(candidate, incumbent, balance)
tolerance = 1e-12;
if candidate.externalInfluence > ...
        incumbent.externalInfluence + tolerance
    better = true;
    return;
elseif candidate.externalInfluence < ...
        incumbent.externalInfluence - tolerance
    better = false;
    return;
end
if candidate.formationEntropy > incumbent.formationEntropy + tolerance
    better = true;
    return;
elseif candidate.formationEntropy < ...
        incumbent.formationEntropy - tolerance
    better = false;
    return;
end
if balance && candidate.age ~= incumbent.age
    better = candidate.age > incumbent.age;
    return;
end
if balance && candidate.count ~= incumbent.count
    better = candidate.count < incumbent.count;
    return;
end
if candidate.reliability > incumbent.reliability + tolerance
    better = true;
    return;
elseif candidate.reliability < incumbent.reliability - tolerance
    better = false;
    return;
end
if candidate.distance < incumbent.distance - tolerance
    better = true;
    return;
elseif candidate.distance > incumbent.distance + tolerance
    better = false;
    return;
end
better = candidate.uid < incumbent.uid;
end

function [referenceQ, selectedQ] = alignedReliability( ...
        reference, selected, reliability, identity)
reference = sortrows(reference, [2, 1]);
selected = sortrows(selected, [2, 1]);
if ~isequal(reference(:, 1:2), selected(:, 1:2))
    error('TemporallyBalancedBackboneV272:FormationPairDrift', ...
        'The balanced route changed the directed formation edges.');
end
referenceQ = zeros(1, size(reference, 1));
selectedQ = zeros(1, size(selected, 1));
for rowIdx = 1:size(reference, 1)
    referenceSender = uidToIndex(reference(rowIdx, 3), identity);
    referenceReceiver = uidToIndex(reference(rowIdx, 4), identity);
    selectedSender = uidToIndex(selected(rowIdx, 3), identity);
    selectedReceiver = uidToIndex(selected(rowIdx, 4), identity);
    referenceQ(rowIdx) = ...
        reliability(referenceReceiver, referenceSender);
    selectedQ(rowIdx) = ...
        reliability(selectedReceiver, selectedSender);
end
end

function index = uidToIndex(uid, identity)
index = find(identity.sensorUids == uid);
if numel(index) ~= 1
    error('TemporallyBalancedBackboneV272:UnknownSensorUid', ...
        'A gateway assignment contains an unknown sensor UID.');
end
end

function selections = orderedSelections(values, count)
if count > numel(values)
    selections = zeros(0, count);
    return;
end
combinations = nchoosek(values, count);
selections = zeros(0, count);
for combinationIdx = 1:size(combinations, 1)
    selections = [selections; perms(combinations(combinationIdx, :))]; ...
        %#ok<AGROW>
end
end

function candidate = emptyCandidate()
candidate = struct('valid', false, 'receivers', zeros(1, 0), ...
    'senders', zeros(1, 0), 'minimumReceiverAge', -Inf, ...
    'totalReceiverAge', -Inf, 'totalReceiverCount', Inf, ...
    'minimumSenderExternalInfluence', -Inf, ...
    'totalSenderExternalInfluence', -Inf, ...
    'minimumSenderFormationEntropy', -Inf, ...
    'minimumSenderAge', -Inf, 'totalSenderAge', -Inf, ...
    'totalSenderCount', Inf, 'bottleneckReliability', -Inf, ...
    'totalLogReliability', -Inf, 'totalDistance', Inf, ...
    'senderScores', struct([]), 'tieKey', zeros(1, 0));
end

function score = emptySenderScore()
score = struct('valid', false, 'age', -Inf, 'count', Inf, ...
    'externalInfluence', -Inf, 'formationEntropy', -Inf, ...
    'reliability', -Inf, 'distance', Inf, 'uid', Inf);
end

function product = reconstructPastInfluence(history, identity, protocol)
product = eye(identity.nodeCount);
for pageIdx = 1:size(history, 3)
    weights = weightsFromMinimumBackbone( ...
        history(:, :, pageIdx), identity, protocol);
    product = weights * product;
end
end

function weights = weightsFromMinimumBackbone( ...
        adjacency, identity, protocol)
weights = zeros(identity.nodeCount);
for receiver = 1:identity.nodeCount
    localMask = identity.formationBySensor == ...
        identity.formationBySensor(receiver);
    local = find(adjacency(receiver, :) & localMask);
    cross = find(adjacency(receiver, :) & ~localMask);
    if numel(local) ~= 1 || numel(cross) > 1
        error('TemporallyBalancedBackboneV272:InvalidHistoryRoute', ...
            'A history page is not a minimum formation backbone.');
    end
    weights(receiver, local) = protocol.dominantWeight;
    if isempty(cross)
        weights(receiver, receiver) = 1 - protocol.dominantWeight;
    else
        weights(receiver, cross) = protocol.crossResidualWeight;
        weights(receiver, receiver) = 1 - protocol.dominantWeight - ...
            protocol.crossResidualWeight;
    end
end
end

function value = formationEntropy(profile, identity)
mass = zeros(1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    mass(formationIdx) = sum(profile(identity.members{formationIdx}));
end
positive = mass(mass > 0);
value = -sum(positive .* log(positive));
if identity.formationCount > 1
    value = value / log(identity.formationCount);
end
end

function less = lexicographicallyLess(left, right)
if isempty(right)
    less = true;
    return;
end
count = min(numel(left), numel(right));
first = find(left(1:count) ~= right(1:count), 1, 'first');
if isempty(first)
    less = numel(left) < numel(right);
else
    less = left(first) < right(first);
end
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; ...
        %#ok<AGROW>
end
passed = all(visited);
end
