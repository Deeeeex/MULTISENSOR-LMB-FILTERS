function bank = buildBraidedHandoverH3V84ActionBank(context, options)
% BUILDBRAIDEDHANDOVERH3V84ACTIONBANK Frozen single-edge handover.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getBraidedHandoverH3V84Protocol();
caseInfo = resolveCase(context, options, protocol);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
[~, control] = computeFormationRoutingLeverageSignature( ...
    context, groupIds, struct('referenceMode', 'current-physical-tree'));

referenceAdjacency = logical(control.referenceAdjacency);
referenceWeights = control.referenceFusionWeights;
physicalAdjacency = logical(context.physicalAdjacency);
receiverIdx = caseInfo.receiverIdx;
incumbentIdx = caseInfo.incumbentSenderIdx;
candidateIdx = caseInfo.candidateSenderIdx;
sourceWeight = protocol.sourceWeight;
validateFrozenEdge( ...
    groupIds, referenceAdjacency, referenceWeights, ...
    physicalAdjacency, receiverIdx, incumbentIdx, ...
    candidateIdx, sourceWeight);
novelty = candidateNovelSupportFraction( ...
    context.localPosteriorBySensor{receiverIdx}, ...
    context.localPosteriorBySensor{incumbentIdx}, ...
    context.localPosteriorBySensor{candidateIdx}, ...
    linkReliability(context.commConfig, candidateIdx, ...
        receiverIdx, context.currentTime), ...
    receiverFormationReferenceMass(control, groupIds, receiverIdx));
sourceProtocol = getBraidedHandoverOpportunityV84Protocol();
if novelty < sourceProtocol.minimumSenderNoveltyFraction - 1e-12
    error('BraidedHandoverH3V84:SourceNoveltyDrift', ...
        'The frozen V84 sender no longer clears its novelty gate.');
end

candidateAdjacency = referenceAdjacency;
candidateWeights = referenceWeights;
candidateAdjacency(receiverIdx, incumbentIdx) = false;
candidateAdjacency(receiverIdx, candidateIdx) = true;
candidateWeights(receiverIdx, incumbentIdx) = 0;
candidateWeights(receiverIdx, candidateIdx) = sourceWeight;
validateRouteParity( ...
    candidateAdjacency, candidateWeights, referenceAdjacency, ...
    referenceWeights, physicalAdjacency);

referenceSequence = repmat(referenceAdjacency, 1, 1, 3);
candidateSequence = cat(3, candidateAdjacency, ...
    referenceAdjacency, referenceAdjacency);
[referenceSensorPass, referenceFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, referenceSequence, groupIds);
[candidateSensorPass, candidateFormationPass] = rollingB3Pass( ...
    context.previousAdjacencyHistory, candidateSequence, groupIds);
if ~all(referenceSensorPass) || ~all(referenceFormationPass) || ...
        ~all(candidateSensorPass) || ~all(candidateFormationPass)
    error('BraidedHandoverH3V84:RollingB3Failure', ...
        'The frozen V84 primary sequence fails rolling-B3.');
end

receiverFormationId = groupIds(receiverIdx);
actionName = sprintf( ...
    'braided-handover-%d-to-%d-replace-%d', ...
    candidateIdx, receiverIdx, incumbentIdx);
bank = struct();
bank.contractVersion = ...
    'braided-handover-primary-action-bank-v84-v1';
bank.protocolId = protocol.id;
bank.outcomePolicyName = protocol.outcomePolicyName;
bank.referenceMode = 'current-physical-tree';
bank.nodeCount = numel(groupIds);
bank.formationCount = numel(unique(groupIds, 'stable'));
bank.groupIds = groupIds;
bank.actionCount = 2;
bank.referenceActionIndex = 1;
bank.actionNames = {'reference', actionName};
bank.actionFormationIds = {zeros(1, 0), receiverFormationId};
bank.actionSlotIndices = {zeros(1, 0), 1};
bank.actionAdjacency = cat(3, ...
    referenceAdjacency, candidateAdjacency);
bank.actionFusionWeights = cat(3, ...
    referenceWeights, candidateWeights);
bank.actionMessageCounts = [ ...
    nnz(referenceAdjacency), nnz(candidateAdjacency)];
bank.actionWithinReferencePayload = [true, true];
bank.actionPosteriorSafetyMask = [true, true];
bank.actionPosteriorProxyAllowed = [true, true];
bank.actionPosteriorObjective = [ ...
    0, caseInfo.sourceLocalNetFraction];
bank.actionPredictedNetSavingBytes = [0, 0];
bank.actionPayloadBytes = [NaN, NaN];
bank.actionFormationIndex = [0, receiverFormationId];
bank.actionModeIndex = [1, 2];
bank.modeTrustWeights = [0, sourceWeight];
bank.actionRollingB3SensorPass = [ ...
    referenceSensorPass; candidateSensorPass];
bank.actionRollingB3FormationPass = [ ...
    referenceFormationPass; candidateFormationPass];
bank.safeActionIndices = 1:2;
bank.referenceMessageCount = nnz(referenceAdjacency);
bank.selectedMessageCount = nnz(candidateAdjacency);
bank.selectedUtilityMass = caseInfo.sourceLocalNetFraction;
bank.selectedNetworkAverageNetFraction = ...
    caseInfo.sourceLocalNetFraction;
bank.directSafeFormationIds = receiverFormationId;
bank.changedReceiverIndices = receiverIdx;
bank.candidateTrustWeights = sourceWeight;
bank.selfFundedTrust = false;
bank.trustFundingSource = 'reference-row-weight-multiset';
bank.rollingB3DurationSteps = 1;
bank.rollingB3ActionUse = logical([1, 0, 0]);
bank.sourceCase = caseInfo;
bank.currentSenderNoveltyFraction = novelty;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.openedDevelopmentEvidenceOnly = true;
bank.validationClaimAllowed = false;
end

function caseInfo = resolveCase(context, options, protocol)
required = {'presetName', 'seed', 'currentTime'};
if ~all(isfield(options, required)) || ...
        ~ischar(options.presetName) || ...
        ~isscalar(options.seed) || ~isfinite(options.seed) || ...
        ~isscalar(options.currentTime) || ...
        ~isfinite(options.currentTime) || ...
        options.currentTime ~= context.currentTime
    error('BraidedHandoverH3V84:UnregisteredCase', ...
        'The V84 H=3 bank needs one explicit frozen case.');
end
caseInfo = [];
for candidate = protocol.cases
    if strcmp(options.presetName, candidate.presetName) && ...
            options.seed == candidate.seed && ...
            options.currentTime == candidate.currentTime
        caseInfo = candidate;
        break;
    end
end
if isempty(caseInfo) || ...
        numel(context.localPosteriorBySensor) ~= ...
            caseInfo.expectedNodeCount
    error('BraidedHandoverH3V84:UnregisteredCase', ...
        'The V84 H=3 bank is restricted to its primary anchors.');
end
end

function validateFrozenEdge( ...
        groupIds, adjacency, weights, physical, receiver, incumbent, ...
        candidate, sourceWeight)
nodeCount = numel(groupIds);
indices = [receiver, incumbent, candidate];
valid = all(isfinite(indices)) && all(indices == round(indices)) && ...
    all(indices >= 1) && all(indices <= nodeCount) && ...
    groupIds(receiver) ~= groupIds(incumbent) && ...
    groupIds(receiver) ~= groupIds(candidate) && ...
    groupIds(incumbent) ~= groupIds(candidate) && ...
    adjacency(receiver, incumbent) && ...
    ~adjacency(receiver, candidate) && ...
    physical(receiver, candidate) && ...
    abs(weights(receiver, incumbent) - sourceWeight) <= 1e-12 && ...
    weights(receiver, candidate) <= 1e-12;
if ~valid
    error('BraidedHandoverH3V84:SourceEdgeDrift', ...
        'The frozen V84 sender replacement no longer matches reference.');
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
    error('BraidedHandoverH3V84:RouteParityFailure', ...
        'The frozen V84 action violates its fixed-budget route contract.');
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

function fraction = candidateNovelSupportFraction( ...
        receiver, incumbent, candidate, reliability, denominator)
labels = collectLabels({receiver, incumbent, candidate});
baseline = max( ...
    supportByLabel(receiver, labels), ...
    supportByLabel(incumbent, labels));
candidateSupport = supportByLabel(candidate, labels);
candidateExistence = existenceByLabel(candidate, labels);
fraction = reliability * sum(max(candidateSupport - baseline, 0) .* ...
    candidateExistence) / denominator;
end

function reliability = linkReliability(config, senderIdx, receiverIdx, time)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(time, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(senderIdx, receiverIdx, timeIdx);
    else
        drop = config.pDropByEdge(senderIdx, receiverIdx);
    end
else
    drop = getField(config, 'defaultDropProbability', 0);
end
reliability = 1 - min(max(drop, 0), 1);
end

function mass = receiverFormationReferenceMass(control, groupIds, receiver)
details = control.referenceScore.retentionDetails;
values = details.expectedReferenceExistenceByReceiver;
receiverFormation = groupIds(receiver);
mass = 0;
for idx = find(groupIds == receiverFormation)
    mass = mass + sum(max(reshape(values{idx}, 1, []), 0));
end
if ~isfinite(mass) || mass <= eps
    error('BraidedHandoverH3V84:InvalidReferenceMass', ...
        'The receiver formation reference mass is unavailable.');
end
end

function labels = collectLabels(posteriors)
labels = zeros(2, 0);
for posteriorIdx = 1:numel(posteriors)
    objects = posteriors{posteriorIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents <= 0
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ...
                ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
end

function values = existenceByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    objectIdx = findObject(objects, labels(:, labelIdx));
    if ~isempty(objectIdx)
        values(labelIdx) = clamp01(objects(objectIdx).r);
    end
end
end

function values = supportByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    objectIdx = findObject(objects, labels(:, labelIdx));
    if ~isempty(objectIdx)
        values(labelIdx) = clamp01(getField( ...
            objects(objectIdx), 'detectionAssociationMass', 0));
    end
end
end

function idx = findObject(objects, label)
idx = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
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

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
