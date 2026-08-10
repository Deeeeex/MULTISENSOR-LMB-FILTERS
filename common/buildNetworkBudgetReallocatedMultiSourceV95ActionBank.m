function bank = buildNetworkBudgetReallocatedMultiSourceV95ActionBank( ...
        context, options)
% BUILDNETWORKBUDGETREALLOCATEDMULTISOURCEV95ACTIONBANK Three causal pages.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getNetworkBudgetReallocatedMultiSourceV95Protocol();
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
nodeCount = numel(groupIds);
requiredTokenCount = getField(options, ...
    'requiredTokenCount', ceil(formationCount / 2));
requiredTargetFormationCount = getField(options, ...
    'requiredTargetFormationCount', requiredTokenCount);
requiredSourceFormationCount = getField(options, ...
    'requiredSourceFormationCount', requiredTokenCount);
currentFormationGraphDiameter = formationGraphDiameter( ...
    logical(context.physicalAdjacency), groupIds);
horizonSteps = getField(options, 'horizonSteps', ...
    currentFormationGraphDiameter + ...
        protocol.recoveryStepsBeyondFormationDiameter);
horizonMatchesFormationGraph = horizonSteps == ...
    currentFormationGraphDiameter + ...
        protocol.recoveryStepsBeyondFormationDiameter;

nominationOptions = getRepeatedMultiGatewayHandoverV89Protocol();
nominationOptions.maximumGatewayCount = inf;
nominationOptions.requireFullFormationBroadcastPhysicality = false;
nominationOptions.minimumSenderNoveltyFraction = ...
    protocol.minimumSenderNoveltyFraction;
nominationOptions.minimumCandidateAssociationSupport = ...
    protocol.minimumCandidateAssociationSupport;
metrics = computeRepeatedMultiGatewayHandoverNominationsV89( ...
    context, groupIds, nominationOptions);
referenceAdjacency = logical(metrics.referenceAdjacency);
referenceWeights = metrics.referenceFusionWeights;
candidateTokens = buildCandidateTokens( ...
    metrics.edgeRecords, context, groupIds, ...
    referenceAdjacency, referenceWeights, protocol);
projection = projectSenderBudgetReallocationV95( ...
    candidateTokens, referenceAdjacency, referenceWeights, ...
    context.physicalAdjacency, groupIds, struct( ...
        'requiredTokenCount', requiredTokenCount, ...
        'sourceWeight', protocol.sourceWeight, ...
        'minimumNetUtilityFraction', ...
            protocol.minimumNetUtilityFraction, ...
        'maximumSolverSeconds', protocol.maximumSolverSeconds));

referenceSequence = repmat(referenceAdjacency, 1, 1, horizonSteps);
donorOnlySequence = referenceSequence;
candidateSequence = referenceSequence;
fixedCandidateSequence = repmat( ...
    projection.candidateAdjacency, 1, 1, horizonSteps);
donorOnlySequence(:, :, 1) = projection.donorOnlyAdjacency;
candidateSequence(:, :, 1) = projection.candidateAdjacency;
[referenceSensorB3, referenceFormationB3] = rollingB3Pass( ...
    context.previousAdjacencyHistory, referenceSequence, groupIds);
[donorSensorB3, donorFormationB3] = rollingB3Pass( ...
    context.previousAdjacencyHistory, donorOnlySequence, groupIds);
[candidateSensorB3, candidateFormationB3] = rollingB3Pass( ...
    context.previousAdjacencyHistory, candidateSequence, groupIds);
[fixedSensorB3, fixedFormationB3] = rollingB3Pass( ...
    context.previousAdjacencyHistory, fixedCandidateSequence, groupIds);
candidateInstantSensor = isStronglyConnected( ...
    projection.candidateAdjacency);
candidateInstantFormation = isStronglyConnected( ...
    collapseToFormations(projection.candidateAdjacency, groupIds));
targetCoverage = numel( ...
    projection.selectedTargetFormationIds);
sourceCoverage = numel( ...
    projection.selectedSourceFormationIds);
structuralGate = projection.feasible && ...
    horizonMatchesFormationGraph && ...
    projection.selectedTokenCount == requiredTokenCount && ...
    targetCoverage >= requiredTargetFormationCount && ...
    sourceCoverage >= requiredSourceFormationCount && ...
    candidateInstantSensor && candidateInstantFormation && ...
    all(referenceSensorB3) && all(referenceFormationB3) && ...
    all(donorSensorB3) && all(donorFormationB3) && ...
    all(candidateSensorB3) && all(candidateFormationB3) && ...
    all(fixedSensorB3) && all(fixedFormationB3);

selectedTargetFormations = ...
    projection.selectedTargetFormationIds;
selectedSourceFormations = ...
    projection.selectedSourceFormationIds;
bank = struct();
bank.contractVersion = ...
    'network-budget-reallocated-multi-source-v95-action-bank-v1';
bank.protocolId = protocol.id;
bank.outcomePolicyName = protocol.outcomePolicyName;
bank.referenceMode = protocol.referenceMode;
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.groupIds = groupIds;
bank.actionCount = 3;
bank.referenceActionIndex = 1;
bank.actionNames = { ...
    'matched-static-physical-tree', ...
    'donor-only-ablation', ...
    'sender-budget-reallocated-multi-source'};
bank.actionFormationIds = { ...
    zeros(1, 0), ...
    selectedTargetFormations, ...
    selectedTargetFormations};
bank.actionSlotIndices = { ...
    zeros(1, 0), ...
    1:projection.selectedTokenCount, ...
    1:projection.selectedTokenCount};
bank.actionAdjacency = cat(3, ...
    referenceAdjacency, projection.donorOnlyAdjacency, ...
    projection.candidateAdjacency);
bank.actionFusionWeights = cat(3, ...
    referenceWeights, projection.donorOnlyWeights, ...
    projection.candidateWeights);
bank.actionMessageCounts = [ ...
    projection.referenceMessageCount, ...
    projection.donorOnlyMessageCount, ...
    projection.candidateMessageCount];
bank.actionWithinReferencePayload = [true, true, true];
bank.actionPosteriorSafetyMask = [true, true, structuralGate];
bank.actionPosteriorProxyAllowed = [true, true, structuralGate];
bank.actionPosteriorObjective = [ ...
    0, -projection.aggregateDonorUniqueFraction, ...
    projection.aggregateNetUtilityFraction];
bank.actionPredictedNetSavingBytes = [0, NaN, 0];
bank.actionPayloadBytes = [NaN, NaN, NaN];
bank.actionFormationIndex = [0, 0, 0];
bank.actionModeIndex = [1, 2, 3];
bank.modeTrustWeights = [0, 0, protocol.sourceWeight];
bank.actionRollingB3SensorPass = [ ...
    referenceSensorB3; donorSensorB3; candidateSensorB3];
bank.actionRollingB3FormationPass = [ ...
    referenceFormationB3; donorFormationB3; ...
    candidateFormationB3];
bank.safeActionIndices = find(bank.actionPosteriorSafetyMask);
bank.referenceMessageCount = projection.referenceMessageCount;
bank.selectedMessageCount = projection.candidateMessageCount;
bank.selectedUtilityMass = projection.aggregateNetUtilityFraction;
bank.selectedNetworkAverageNetFraction = ...
    projection.aggregateNetUtilityFraction;
bank.directSafeFormationIds = selectedTargetFormations;
bank.changedReceiverIndices = unique([ ...
    [projection.selectedCandidates.targetReceiverIdx], ...
    [projection.selectedCandidates.donorReceiverIdx]], 'stable');
bank.candidateTrustWeights = protocol.sourceWeight;
bank.selfFundedTrust = true;
bank.trustFundingSource = ...
    'same-sender-residual-token-reallocated-across-receivers';
bank.requiredTokenCount = requiredTokenCount;
bank.requiredTargetFormationCount = ...
    requiredTargetFormationCount;
bank.requiredSourceFormationCount = ...
    requiredSourceFormationCount;
bank.selectedTokenCount = projection.selectedTokenCount;
bank.selectedTargetFormationIds = selectedTargetFormations;
bank.selectedSourceFormationIds = selectedSourceFormations;
bank.targetFormationCoverageCount = targetCoverage;
bank.sourceFormationCoverageCount = sourceCoverage;
bank.horizonSteps = horizonSteps;
bank.interventionDurationSteps = 1;
bank.formationGraphDiameter = currentFormationGraphDiameter;
bank.horizonMatchesFormationGraph = horizonMatchesFormationGraph;
bank.senderMessageParityPassed = ...
    projection.senderMessageParityPassed;
bank.networkMessageParityPassed = ...
    projection.networkMessageParityPassed;
bank.candidateInstantSensorStrong = candidateInstantSensor;
bank.candidateInstantFormationStrong = candidateInstantFormation;
bank.fixedCandidateRollingB3SensorPass = fixedSensorB3;
bank.fixedCandidateRollingB3FormationPass = fixedFormationB3;
bank.structuralGatePassed = structuralGate;
bank.candidateTokens = candidateTokens;
bank.projection = projection;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.openedDevelopmentEvidenceOnly = true;
bank.validationClaimAllowed = false;
end

function tokens = buildCandidateTokens( ...
        edgeRecords, context, groupIds, referenceAdjacency, ...
        referenceWeights, protocol)
tokens = repmat(emptyToken(), 1, 0);
posteriors = reshape(context.localPosteriorBySensor, 1, []);
formationMass = localExistenceMassByFormation(posteriors, groupIds);
for edge = edgeRecords
    if ~edge.actionEnabled
        continue;
    end
    target = edge.receiverIdx;
    sender = edge.candidateSenderIdx;
    donor = find(referenceAdjacency(:, sender) & ...
        abs(referenceWeights(:, sender) - ...
            protocol.sourceWeight) <= 1e-12);
    if numel(donor) ~= 1 || donor == target
        continue;
    end
    donor = donor(1);
    dominant = find(referenceAdjacency(donor, :) & ...
        abs(referenceWeights(donor, :) - ...
            protocol.dominantWeight) <= 1e-12);
    if numel(dominant) ~= 1
        continue;
    end
    donorReliability = linkReliability( ...
        context.commConfig, sender, donor);
    donorUniqueMass = uniqueSupportMass( ...
        posteriors{donor}, posteriors{dominant(1)}, ...
        posteriors{sender}, donorReliability);
    donorUniqueFraction = donorUniqueMass / ...
        formationMass(groupIds(donor));
    netUtility = edge.senderNoveltyFraction - ...
        donorUniqueFraction;
    tokens(end + 1) = struct( ... %#ok<AGROW>
        'targetReceiverIdx', target, ...
        'donorReceiverIdx', donor, ...
        'senderIdx', sender, ...
        'targetFormationId', groupIds(target), ...
        'donorFormationId', groupIds(donor), ...
        'sourceFormationId', groupIds(sender), ...
        'targetLinkReliability', edge.linkReliability, ...
        'donorLinkReliability', donorReliability, ...
        'targetNoveltyFraction', ...
            edge.senderNoveltyFraction, ...
        'donorUniqueFraction', donorUniqueFraction, ...
        'netUtilityFraction', netUtility, ...
        'maximumNovelAssociationSupport', ...
            edge.maximumNovelAssociationSupport);
end
if isempty(tokens)
    return;
end
ranking = [-[tokens.netUtilityFraction]', ...
    -[tokens.targetNoveltyFraction]', ...
    [tokens.targetReceiverIdx]', [tokens.senderIdx]'];
[~, order] = sortrows(ranking, [1, 2, 3, 4]);
tokens = tokens(order);
end

function mass = uniqueSupportMass(receiver, incumbent, source, reliability)
labels = collectLabels({receiver, incumbent, source});
baseline = max( ...
    supportByLabel(receiver, labels), ...
    supportByLabel(incumbent, labels));
sourceSupport = supportByLabel(source, labels);
sourceExistence = existenceByLabel(source, labels);
mass = reliability * sum(max(sourceSupport - baseline, 0) .* ...
    sourceExistence);
end

function mass = localExistenceMassByFormation(posteriors, groupIds)
mass = zeros(1, max(groupIds));
for sensorIdx = 1:numel(posteriors)
    objects = posteriors{sensorIdx};
    if isempty(objects)
        continue;
    end
    mass(groupIds(sensorIdx)) = mass(groupIds(sensorIdx)) + ...
        sum(max(reshape([objects.r], 1, []), 0));
end
if any(mass(unique(groupIds)) <= eps)
    error('NetworkBudgetV95:InvalidFormationMass', ...
        'A V95 formation has no current existence mass.');
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
    object = objectByLabel(objects, labels(:, labelIdx));
    if ~isempty(object)
        values(labelIdx) = clamp01(object.r);
    end
end
end

function values = supportByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    object = objectByLabel(objects, labels(:, labelIdx));
    if ~isempty(object)
        values(labelIdx) = clamp01(getField( ...
            object, 'detectionAssociationMass', 0));
    end
end
end

function object = objectByLabel(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function reliability = linkReliability(config, sender, receiver)
drop = getField(config, 'defaultDropProbability', 0);
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) > 2
        error('NetworkBudgetV95:FutureLinkProbabilityExposed', ...
            'V95 expects exactly one current link-probability page.');
    end
    drop = config.pDropByEdge(sender, receiver);
end
reliability = 1 - clamp01(drop);
end

function diameter = formationGraphDiameter(physical, groupIds)
formation = collapseToFormations(physical, groupIds);
formation = formation | formation';
count = size(formation, 1);
distance = inf(count);
distance(1:count+1:end) = 0;
distance(formation) = 1;
for via = 1:count
    distance = min(distance, bsxfun(@plus, ...
        distance(:, via), distance(via, :)));
end
finiteDistance = distance(isfinite(distance));
if numel(finiteDistance) ~= count * count
    error('NetworkBudgetV95:DisconnectedFormationGraph', ...
        'The current physical formation graph is disconnected.');
end
diameter = max(finiteDistance);
end

function [sensorPass, formationPass] = ...
        rollingB3Pass(previousHistory, sequence, groupIds)
history = logical(previousHistory);
if size(history, 3) > 2
    history = history(:, :, end-1:end);
end
sensorPass = false(1, size(sequence, 3));
formationPass = false(1, size(sequence, 3));
for stepIdx = 1:size(sequence, 3)
    pages = cat(3, history, logical(sequence(:, :, stepIdx)));
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

function token = emptyToken()
token = struct( ...
    'targetReceiverIdx', NaN, ...
    'donorReceiverIdx', NaN, ...
    'senderIdx', NaN, ...
    'targetFormationId', NaN, ...
    'donorFormationId', NaN, ...
    'sourceFormationId', NaN, ...
    'targetLinkReliability', NaN, ...
    'donorLinkReliability', NaN, ...
    'targetNoveltyFraction', NaN, ...
    'donorUniqueFraction', NaN, ...
    'netUtilityFraction', NaN, ...
    'maximumNovelAssociationSupport', NaN);
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
