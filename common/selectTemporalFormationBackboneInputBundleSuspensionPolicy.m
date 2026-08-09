function [adjacency, details] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, suspendedFormationIds, options)
% SELECTTEMPORALFORMATIONINPUTBUNDLESUSPENSIONPOLICY One-step v38 action.
%
% A formation action suppresses every low-weight cross-formation residual
% input entering that formation in the registered-backbone tour.  The
% removed weight returns to receiver self weight.  The high-weight local
% route is untouched.

if nargin < 2 || isempty(suspendedFormationIds)
    suspendedFormationIds = zeros(1, 0);
end
if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), ...
            {'policyConfig', 'referenceMode'}))
    error('FormationBundleSuspension:InvalidOptions', ...
        'Only the frozen policy config and reference mode may be supplied.');
end
policy = getField(options, 'policyConfig', ...
    getFormationBackboneBundleStaggeredRecoveryPolicyConfig());
referenceMode = getField(options, 'referenceMode', 'registered-static');
if ~ischar(referenceMode) || ~ismember(referenceMode, ...
        {'registered-static', 'current-physical-tree'})
    error('FormationBundleSuspension:InvalidOptions', ...
        'The input-bundle reference mode is not registered.');
end
validatePolicyConfig(policy);
nodeCount = numel(context.localPosteriorBySensor);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
suspendedFormationIds = reshape(suspendedFormationIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        formationCount < 2 || ...
        formationCount > policy.maximumFormationCount || ...
        numel(unique(suspendedFormationIds)) ~= ...
            numel(suspendedFormationIds) || ...
        any(~ismember(suspendedFormationIds, groups)) || ...
        ~isfield(context, 'previousAdjacencyHistory') || ...
        size(context.previousAdjacencyHistory, 1) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 2) ~= nodeCount || ...
        size(context.previousAdjacencyHistory, 3) < 2
    error('FormationBundleSuspension:InvalidContract', ...
        'Formation input-bundle suspension context is invalid.');
end

referenceOptions = struct( ...
    'dominantWeight', policy.dominantWeight, ...
    'residualWeight', policy.residualWeight);
if strcmp(referenceMode, 'current-physical-tree')
    [referenceAdjacency, referenceDetails] = ...
        selectPhysicalFormationTreeResidualTourPolicy( ...
            context, referenceOptions);
else
    [referenceAdjacency, referenceDetails] = ...
        selectFormationBackboneResidualTourPolicy( ...
            context, referenceOptions);
end
adjacency = logical(referenceAdjacency);
fusionWeights = referenceDetails.fusionWeightMatrix;
suspendedReceivers = zeros(1, 0);
suspendedSenders = zeros(1, 0);
suspendedWeights = zeros(1, 0);
suspendedMessageCountByFormation = zeros(1, formationCount);
for formationIdx = 1:formationCount
    if ~ismember(groups(formationIdx), suspendedFormationIds)
        continue;
    end
    receivers = referenceDetails. ...
        incomingCrossReceiversByFormation{formationIdx};
    senders = referenceDetails. ...
        incomingCrossSendersByFormation{formationIdx};
    if isempty(receivers) || numel(receivers) ~= numel(senders)
        error('FormationBundleSuspension:InvalidReference', ...
            'A registered formation input bundle is empty or malformed.');
    end
    suspendedMessageCountByFormation(formationIdx) = numel(receivers);
    for localIdx = 1:numel(receivers)
        receiver = receivers(localIdx);
        sender = senders(localIdx);
        weight = fusionWeights(receiver, sender);
        if weight <= 0 || ~adjacency(receiver, sender)
            error('FormationBundleSuspension:InvalidReference', ...
                'A registered cross input has no positive fusion weight.');
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
    error('FormationBundleSuspension:InvalidCandidate', ...
        'The selected input-bundle suspension is invalid.');
end

sequence = cat(3, adjacency, ...
    referenceAdjacency, referenceAdjacency);
[sensorPass, formationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, sequence, groupIds);
if ~all(sensorPass) || ~all(formationPass)
    error('FormationBundleSuspension:Infeasible', ...
        'The input-bundle suspension fails the one-step rolling-B3 reserve.');
end

details = referenceDetails;
details.contractVersion = ...
    'temporal-formation-input-bundle-suspension-v1';
details.mode = 'temporal-formation-input-bundle-suspension';
details.referenceMode = referenceMode;
details.actionName = buildActionName(suspendedFormationIds);
details.suspendedFormationIds = suspendedFormationIds;
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
details.messageSavingCount = ...
    nnz(referenceAdjacency) - nnz(adjacency);
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
    isStronglyConnected(collapseToFormations(adjacency, groupIds));
details.rollingB3SensorPass = sensorPass;
details.rollingB3FormationPass = formationPass;
details.rollingB3SensorStrongConnected = all(sensorPass);
details.rollingB3FormationStrongConnected = all(formationPass);
details.sensorWindowMature = true;
details.sensorWindowStrongConnected = all(sensorPass);
details.formationWindowMature = true;
details.formationWindowStrongConnected = all(formationPass);
details.oneStepTopologyReserveChecked = true;
details.oneStepTopologyReservePassed = ...
    all(sensorPass) && all(formationPass);
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
end

function validatePolicyConfig(policy)
expected = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
if ~isstruct(policy) || ~isscalar(policy) || ...
        ~isfield(policy, 'canonicalSha256') || ...
        ~strcmp(policy.canonicalSha256, expected.canonicalSha256) || ...
        ~isequaln(policy, expected)
    error('FormationBundleSuspension:InvalidPolicyConfig', ...
        'The frozen shared v38 policy config changed.');
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
if isempty(adjacency)
    connected = false;
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
connected = all(visited);
end

function name = buildActionName(formationIds)
if isempty(formationIds)
    name = 'reference';
    return;
end
value = sprintf('f%d-', formationIds);
name = ['suspend-input-bundle-', value(1:end-1)];
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
