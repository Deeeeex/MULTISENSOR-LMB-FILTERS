function [adjacency, details] = ...
    selectTemporalCrossEdgeSuspensionPolicy( ...
        context, suspendedFormationIds, options)
% SELECTTEMPORALCROSSEDGESUSPENSIONPOLICY One-step v29 topology action.
%
% Selected reference-cycle cross-formation residual messages are omitted
% for the current step.  Their fusion weight is returned to receiver self
% weight.  The next two registered steps use the reference route, and the
% full three-step selected-topology sequence must pass rolling-B3 sensor
% and formation connectivity.

if nargin < 2 || isempty(suspendedFormationIds)
    suspendedFormationIds = zeros(1, 0);
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
if any(~isfinite([dominantWeight, residualWeight])) || ...
        dominantWeight <= 0 || residualWeight <= 0 || ...
        dominantWeight + residualWeight >= 1
    error('TemporalSuspension:InvalidContract', ...
        'Temporal-suspension fusion weights are invalid.');
end
nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context, nodeCount);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
suspendedFormationIds = reshape(suspendedFormationIds, 1, []);
if numel(unique(suspendedFormationIds)) ~= ...
        numel(suspendedFormationIds) || ...
        any(~ismember(suspendedFormationIds, groups))
    error('TemporalSuspension:InvalidContract', ...
        'Suspended formation IDs are invalid.');
end
if ~isfield(context, 'previousAdjacencyHistory') || ...
        size(context.previousAdjacencyHistory, 1) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 2) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 3) < 2
    error('TemporalSuspension:InvalidContract', ...
        'Rolling-B3 requires two prior selected topologies.');
end

[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', dominantWeight, ...
            'residualWeight', residualWeight));
referenceResidualSources = reshape( ...
    referenceDetails.residualSourcesByReceiver, 1, []);
referenceCrossMask = groupIds ~= ...
    groupIds(referenceResidualSources);
if nnz(referenceCrossMask) ~= formationCount
    error('TemporalSuspension:InvalidContract', ...
        'Reference route must have one cross edge per formation.');
end

suspendedReceivers = zeros(1, numel(suspendedFormationIds));
suspendedSenders = zeros(1, numel(suspendedFormationIds));
for suspendedIdx = 1:numel(suspendedFormationIds)
    formationId = suspendedFormationIds(suspendedIdx);
    receiver = find(referenceCrossMask & ...
        groupIds == formationId);
    if numel(receiver) ~= 1
        error('TemporalSuspension:InvalidContract', ...
            'Reference cross receiver is ambiguous.');
    end
    suspendedReceivers(suspendedIdx) = receiver;
    suspendedSenders(suspendedIdx) = ...
        referenceResidualSources(receiver);
end

adjacency = logical(referenceAdjacency);
fusionWeights = referenceDetails.fusionWeightMatrix;
suspendedWeights = zeros(1, numel(suspendedReceivers));
for suspendedIdx = 1:numel(suspendedReceivers)
    receiver = suspendedReceivers(suspendedIdx);
    sender = suspendedSenders(suspendedIdx);
    suspendedWeights(suspendedIdx) = ...
        fusionWeights(receiver, sender);
    if suspendedWeights(suspendedIdx) <= 0
        error('TemporalSuspension:InvalidContract', ...
            'Suspended reference edge has no positive weight.');
    end
    fusionWeights(receiver, receiver) = ...
        fusionWeights(receiver, receiver) + ...
            suspendedWeights(suspendedIdx);
    fusionWeights(receiver, sender) = 0;
    adjacency(receiver, sender) = false;
end

physical = logical(context.physicalAdjacency);
weightSupport = adjacency | logical(eye(nodeCount));
if any(adjacency(:) & ~physical(:)) || ...
        any(fusionWeights(:) < -1e-12) || ...
        any(fusionWeights(:) > 0 & ~weightSupport(:)) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        nnz(adjacency) ~= ...
            nnz(referenceAdjacency) - numel(suspendedReceivers)
    error('TemporalSuspension:InvalidCandidate', ...
        'Suspended route violates physical, weight, or message rules.');
end

candidateInstantSensorStrong = isStronglyConnected(adjacency);
candidateFormationAdjacency = collapseToFormations( ...
    adjacency, groupIds);
candidateInstantFormationStrong = ...
    isStronglyConnected(candidateFormationAdjacency);
sequence = cat(3, adjacency, ...
    referenceAdjacency, referenceAdjacency);
[sensorB3Pass, formationB3Pass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, sequence, groupIds);
if ~all(sensorB3Pass) || ~all(formationB3Pass)
    error('TemporalSuspension:Infeasible', ...
        'One-step suspension fails the frozen rolling-B3 sequence.');
end

details = referenceDetails;
details.contractVersion = ...
    'temporal-cross-formation-edge-suspension-v1';
details.mode = 'temporal-cross-edge-suspension';
details.actionName = buildActionName(suspendedFormationIds);
details.suspendedFormationIds = suspendedFormationIds;
details.suspendedReceivers = suspendedReceivers;
details.suspendedSenders = suspendedSenders;
details.suspendedWeights = suspendedWeights;
details.suspendedCrossEdgeCount = numel(suspendedReceivers);
details.referenceAdjacency = referenceAdjacency;
details.referenceFusionWeights = ...
    referenceDetails.fusionWeightMatrix;
details.fusionWeightMatrix = fusionWeights;
details.referenceMessageCount = nnz(referenceAdjacency);
details.messageCount = nnz(adjacency);
details.messageSavingCount = ...
    nnz(referenceAdjacency) - nnz(adjacency);
details.messageSavingFraction = details.messageSavingCount / ...
    max(details.referenceMessageCount, 1);
details.referenceCrossFormationMessageCount = ...
    nnz(referenceCrossMask);
details.crossFormationMessageCount = ...
    nnz(referenceCrossMask) - numel(suspendedReceivers);
details.instantaneousSensorStrongConnected = ...
    candidateInstantSensorStrong;
details.instantaneousFormationStrongConnected = ...
    candidateInstantFormationStrong;
details.rollingB3SensorPass = sensorB3Pass;
details.rollingB3FormationPass = formationB3Pass;
details.rollingB3SensorStrongConnected = all(sensorB3Pass);
details.rollingB3FormationStrongConnected = ...
    all(formationB3Pass);
details.interventionDurationSteps = 1;
details.registeredRecoverySteps = 2;
details.currentPhysicalActionSetUsed = true;
details.currentPosteriorUsed = false;
details.currentLinkReliabilityUsed = false;
details.repairTriggered = false;
details.projectionFallbackUsed = false;
details.payloadEmergencyUsed = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureMeasurementUsed = false;
details.futureOutcomeUsed = false;
details.selectionSeconds = toc(timerId);
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

function groupIds = resolveGroupIds(context, nodeCount)
if ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('TemporalSuspension:InvalidContract', ...
        'Formation group IDs are unavailable.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('TemporalSuspension:InvalidContract', ...
        'Formation group IDs are invalid.');
end
end

function name = buildActionName(formationIds)
if isempty(formationIds)
    name = 'reference';
    return;
end
text = sprintf('f%d-', formationIds);
name = ['suspend-', text(1:end-1)];
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
