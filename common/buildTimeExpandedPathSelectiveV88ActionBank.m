function bank = buildTimeExpandedPathSelectiveV88ActionBank( ...
        context, options)
% BUILDTIMEEXPANDEDPATHSELECTIVEV88ACTIONBANK Select V87-positive paths.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getTimeExpandedPathSelectiveV88Protocol();
base = buildTemporalAcquireBroadcastV86ActionBank(context, options);
caseInfo = resolveCase(protocol, base.sourceCase);
referenceAdjacency = logical(base.actionAdjacency(:, :, 1));
referenceWeights = base.actionFusionWeights(:, :, 1);
selectiveAdjacency = logical(base.actionAdjacency(:, :, 3));
selectiveWeights = base.actionFusionWeights(:, :, 3);
physicalAdjacency = logical(context.physicalAdjacency);
selectedReceivers = reshape( ...
    caseInfo.broadcastReceiverIndices, 1, []);
allReceivers = reshape(base.broadcastReceiverIndices, 1, []);
if numel(unique(selectedReceivers)) ~= numel(selectedReceivers) || ...
        ~all(ismember(selectedReceivers, allReceivers)) || ...
        numel(selectedReceivers) ~= caseInfo.sourceEnabledPathCount || ...
        numel(allReceivers) ~= caseInfo.sourceCandidatePathCount
    error('TimeExpandedPathSelectiveV88:SourceMaskDrift', ...
        'The frozen V87-positive receiver mask no longer matches V86.');
end

excludedReceivers = setdiff( ...
    allReceivers, selectedReceivers, 'stable');
for receiverIdx = excludedReceivers
    selectiveAdjacency(receiverIdx, :) = ...
        referenceAdjacency(receiverIdx, :);
    selectiveWeights(receiverIdx, :) = ...
        referenceWeights(receiverIdx, :);
end
validateRouteParity( ...
    selectiveAdjacency, selectiveWeights, referenceAdjacency, ...
    referenceWeights, physicalAdjacency);

groupIds = base.groupIds;
referenceSequence = repmat(referenceAdjacency, 1, 1, 3);
candidateSequence = cat(3, ...
    base.actionAdjacency(:, :, 2), selectiveAdjacency, ...
    referenceAdjacency);
[referenceSensorPass, referenceFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, referenceSequence, groupIds);
[candidateSensorPass, candidateFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, candidateSequence, groupIds);
if ~all(referenceSensorPass) || ~all(referenceFormationPass) || ...
        ~all(candidateSensorPass) || ~all(candidateFormationPass)
    error('TimeExpandedPathSelectiveV88:RollingB3Failure', ...
        'The frozen path-selective sequence fails rolling-B3.');
end

selectedPositions = arrayfun( ...
    @(receiver) find(allReceivers == receiver, 1), ...
    selectedReceivers);
bank = base;
bank.contractVersion = ...
    'time-expanded-path-selective-primary-action-bank-v88-v1';
bank.protocolId = protocol.id;
bank.outcomePolicyName = protocol.outcomePolicyName;
bank.actionNames{3} = sprintf( ...
    'path-selective-gateway-%d-formation-%d', ...
    base.acquireReceiverIdx, groupIds(base.acquireReceiverIdx));
bank.actionSlotIndices{3} = 1:numel(selectedReceivers);
bank.actionAdjacency(:, :, 3) = selectiveAdjacency;
bank.actionFusionWeights(:, :, 3) = selectiveWeights;
bank.actionMessageCounts(3) = nnz(selectiveAdjacency);
bank.actionPosteriorObjective(3) = ...
    caseInfo.sourceAggregateExistenceNetFraction;
bank.actionRollingB3SensorPass = [ ...
    referenceSensorPass; candidateSensorPass; candidateSensorPass];
bank.actionRollingB3FormationPass = [ ...
    referenceFormationPass; candidateFormationPass; ...
    candidateFormationPass];
bank.selectedMessageCount = nnz(selectiveAdjacency);
bank.selectedUtilityMass = ...
    caseInfo.sourceAggregateExistenceNetFraction;
bank.selectedNetworkAverageNetFraction = ...
    caseInfo.sourceAggregateExistenceNetFraction;
bank.changedReceiverIndices = unique([ ...
    base.acquireReceiverIdx, selectedReceivers], 'stable');
bank.broadcastReceiverIndices = selectedReceivers;
bank.broadcastExcludedReceiverIndices = excludedReceivers;
bank.broadcastDisplacedDominantSenderIndices = ...
    base.broadcastDisplacedDominantSenderIndices(selectedPositions);
bank.broadcastRemovedResidualSenderIndices = ...
    base.broadcastRemovedResidualSenderIndices(selectedPositions);
bank.broadcastReceiverCoverageFraction = ...
    numel(selectedReceivers) / max(numel(allReceivers), 1);
bank.sourceEnabledPathCount = caseInfo.sourceEnabledPathCount;
bank.sourceCandidatePathCount = caseInfo.sourceCandidatePathCount;
bank.sourceAggregateExistenceNetFraction = ...
    caseInfo.sourceAggregateExistenceNetFraction;
bank.sourceAggregateStateNetFraction = ...
    caseInfo.sourceAggregateStateNetFraction;
bank.pathSelectionCriterion = ...
    'positive-existence-survival-nonnegative-state-zero-crossing';
bank.sourceCase = caseInfo;
end

function caseInfo = resolveCase(protocol, sourceCase)
caseInfo = [];
for candidate = protocol.cases
    if strcmp(candidate.presetName, sourceCase.presetName) && ...
            candidate.seed == sourceCase.seed && ...
            candidate.currentTime == sourceCase.currentTime
        caseInfo = candidate;
        break;
    end
end
if isempty(caseInfo)
    error('TimeExpandedPathSelectiveV88:UnregisteredCase', ...
        'The base V86 action is not a registered V88 case.');
end
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
    error('TimeExpandedPathSelectiveV88:RouteParityFailure', ...
        'The V88 broadcast violates its fixed-budget route contract.');
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
