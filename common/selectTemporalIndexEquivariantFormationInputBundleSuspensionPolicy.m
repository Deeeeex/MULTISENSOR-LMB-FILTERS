function [adjacency, details] = ...
    selectTemporalIndexEquivariantFormationInputBundleSuspensionPolicy( ...
        context, suspendedFormationPhysicalUids, options)
% SELECTTEMPORALINDEXEQUIVARIANTFORMATIONINPUTBUNDLESUSPENSIONPOLICY
% Suppress one page of low-weight incoming cross-formation inputs while
% preserving the v43 physical-UID reference as the recovery route.

if nargin < 2 || isempty(suspendedFormationPhysicalUids)
    suspendedFormationPhysicalUids = zeros(1, 0);
end
if nargin < 3 || isempty(options)
    options = struct();
end
allowedFields = {'dominantWeight', 'residualWeight'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('IndexEquivariantBundleSuspension:InvalidOptions', ...
        'Only frozen fusion weights may be supplied.');
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
[referenceAdjacency, referenceDetails] = ...
    selectIndexEquivariantFormationBackbonePolicy(context, struct( ...
        'dominantWeight', dominantWeight, ...
        'residualWeight', residualWeight));

formationUids = reshape( ...
    referenceDetails.formationPhysicalUids, 1, []);
suspendedFormationPhysicalUids = reshape( ...
    suspendedFormationPhysicalUids, 1, []);
nodeCount = size(referenceAdjacency, 1);
if numel(unique(suspendedFormationPhysicalUids)) ~= ...
        numel(suspendedFormationPhysicalUids) || ...
        any(~ismember(suspendedFormationPhysicalUids, formationUids)) || ...
        ~isfield(context, 'previousAdjacencyHistory') || ...
        ndims(context.previousAdjacencyHistory) ~= 3 || ...
        size(context.previousAdjacencyHistory, 1) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 2) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 3) < 2
    error('IndexEquivariantBundleSuspension:InvalidContract', ...
        'The requested physical-formation suspension is invalid.');
end

adjacency = logical(referenceAdjacency);
fusionWeights = referenceDetails.fusionWeightMatrix;
suspendedReceivers = zeros(1, 0);
suspendedSenders = zeros(1, 0);
suspendedWeights = zeros(1, 0);
suspendedMessageCountByFormation = zeros(1, numel(formationUids));
for formationIdx = 1:numel(formationUids)
    if ~ismember(formationUids(formationIdx), ...
            suspendedFormationPhysicalUids)
        continue;
    end
    receivers = referenceDetails. ...
        incomingCrossReceiversByFormation{formationIdx};
    senders = referenceDetails. ...
        incomingCrossSendersByFormation{formationIdx};
    if isempty(receivers) || numel(receivers) ~= numel(senders)
        error('IndexEquivariantBundleSuspension:InvalidReference', ...
            'A registered physical input bundle is empty or malformed.');
    end
    suspendedMessageCountByFormation(formationIdx) = numel(receivers);
    for localIdx = 1:numel(receivers)
        receiver = receivers(localIdx);
        sender = senders(localIdx);
        weight = fusionWeights(receiver, sender);
        if weight <= 0 || ~adjacency(receiver, sender)
            error('IndexEquivariantBundleSuspension:InvalidReference', ...
                'A registered cross input lacks positive fusion weight.');
        end
        fusionWeights(receiver, receiver) = ...
            fusionWeights(receiver, receiver) + weight;
        fusionWeights(receiver, sender) = 0;
        adjacency(receiver, sender) = false;
        suspendedReceivers(end + 1) = receiver; %#ok<AGROW>
        suspendedSenders(end + 1) = sender; %#ok<AGROW>
        suspendedWeights(end + 1) = weight; %#ok<AGROW>
    end
end

physical = logical(context.physicalAdjacency);
weightSupport = adjacency | logical(eye(nodeCount));
if any(adjacency(:) & ~physical(:)) || ...
        any(~isfinite(fusionWeights(:))) || ...
        any(fusionWeights(:) < -1e-12) || ...
        any(fusionWeights(:) > 1e-12 & ~weightSupport(:)) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        nnz(adjacency) ~= ...
            nnz(referenceAdjacency) - numel(suspendedReceivers)
    error('IndexEquivariantBundleSuspension:InvalidCandidate', ...
        'The v43 input-bundle suspension is invalid.');
end

sequence = cat(3, adjacency, referenceAdjacency, referenceAdjacency);
sensorPass = rollingB3Pass( ...
    context.previousAdjacencyHistory, sequence);
formationPass = rollingB3Pass( ...
    collapseToPhysicalFormations( ...
        context.previousAdjacencyHistory, ...
        context.formationPhysicalUidsBySensor), ...
    collapseToPhysicalFormations( ...
        sequence, context.formationPhysicalUidsBySensor));
if ~all(sensorPass) || ~all(formationPass)
    error('IndexEquivariantBundleSuspension:Infeasible', ...
        'The v43 suspension fails the one-step rolling-B3 reserve.');
end

details = referenceDetails;
details.contractVersion = ...
    'temporal-index-equivariant-formation-input-bundle-suspension-v1';
details.mode = ...
    'temporal-index-equivariant-formation-input-bundle-suspension';
details.actionName = buildActionName(suspendedFormationPhysicalUids);
details.suspendedFormationPhysicalUids = ...
    suspendedFormationPhysicalUids;
details.suspendedReceivers = suspendedReceivers;
details.suspendedSenders = suspendedSenders;
details.suspendedWeights = suspendedWeights;
details.suspendedMessageCountByFormation = ...
    suspendedMessageCountByFormation;
details.suspendedCrossEdgeCount = numel(suspendedReceivers);
details.referenceAdjacency = referenceAdjacency;
details.referenceFusionWeights = ...
    referenceDetails.fusionWeightMatrix;
details.fusionWeightMatrix = fusionWeights;
details.referenceMessageCount = nnz(referenceAdjacency);
details.messageCount = nnz(adjacency);
details.messageSavingCount = numel(suspendedReceivers);
details.messageSavingFraction = details.messageSavingCount / ...
    max(details.referenceMessageCount, 1);
details.referenceCrossFormationMessageCount = ...
    referenceDetails.crossFormationMessageCount;
details.crossFormationMessageCount = ...
    referenceDetails.crossFormationMessageCount - ...
        numel(suspendedReceivers);
details.instantaneousSensorStrongConnected = ...
    isStronglyConnected(adjacency);
details.instantaneousFormationStrongConnected = ...
    isStronglyConnected(collapseToPhysicalFormations( ...
        adjacency, context.formationPhysicalUidsBySensor));
details.rollingB3SensorPass = sensorPass;
details.rollingB3FormationPass = formationPass;
details.oneStepTopologyReserveChecked = true;
details.oneStepTopologyReservePassed = ...
    all(sensorPass) && all(formationPass);
details.currentGeometryUsed = true;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.posteriorUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function pass = rollingB3Pass(previousHistory, sequence)
history = logical(previousHistory(:, :, end-1:end));
pass = false(1, size(sequence, 3));
for stepIdx = 1:size(sequence, 3)
    pages = cat(3, history, sequence(:, :, stepIdx));
    window = any(pages(:, :, max(1, end-2):end), 3);
    pass(stepIdx) = isStronglyConnected(window);
    history(:, :, end + 1) = sequence(:, :, stepIdx); %#ok<AGROW>
    history = history(:, :, max(1, end-1):end);
end
end

function formationPages = collapseToPhysicalFormations( ...
        sensorPages, formationUidsBySensor)
uids = sort(unique(reshape(formationUidsBySensor, 1, [])));
formationPages = false(numel(uids), numel(uids), ...
    size(sensorPages, 3));
for pageIdx = 1:size(sensorPages, 3)
    for receiverIdx = 1:numel(uids)
        receivers = formationUidsBySensor == uids(receiverIdx);
        for senderIdx = 1:numel(uids)
            senders = formationUidsBySensor == uids(senderIdx);
            formationPages(receiverIdx, senderIdx, pageIdx) = ...
                any(any(sensorPages(receivers, senders, pageIdx)));
        end
    end
end
end

function name = buildActionName(formationUids)
if isempty(formationUids)
    name = 'reference';
    return;
end
value = sprintf('uid%.0f-', sort(formationUids));
name = ['suspend-input-bundle-', value(1:end-1)];
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

function value = getField(structure, fieldName, defaultValue)
if isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
