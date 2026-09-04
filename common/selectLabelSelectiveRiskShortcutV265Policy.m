function [adjacency, details] = ...
        selectLabelSelectiveRiskShortcutV265Policy(context)
% SELECTLABELSELECTIVERISKSHORTCUTV265POLICY Keep V242; route one label.
%
% V261 supplies the causal formation/label/donor decision.  V265 deliberately
% discards V261's full-posterior tree change and keeps V242 byte-for-byte.
% It schedules only the first missing physical hop of the shorter path.  The
% filter later transports one complete labeled Bernoulli density on that hop
% and inserts it into one ordinary label-wise KLA call.

protocol = getLabelSelectiveRiskShortcutV265Protocol();
[adjacency, base] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
[~, risk] = selectRiskTriggeredFormationShortcutV261Policy(context);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationIds = unique(groupIds, 'stable');
nodeCount = numel(groupIds);

selectedLabel = reshape(getField( ...
    risk, 'selectedLocalizationLabel', zeros(2, 0)), 2, []);
donorFormationId = getField(risk, ...
    'selectedDonorFormationId', 0);
targetFormationId = getField(risk, ...
    'selectedLocalizationFormationId', 0);
pathFormationIds = reshape(getField(risk, ...
    'shortcutPathFormationIds', zeros(1, 0)), 1, []);
routeRequested = logical(getField(risk, ...
    'shortcutRequestAttempted', false)) && ...
    size(selectedLabel, 2) == 1 && numel(pathFormationIds) >= 2 && ...
    pathFormationIds(1) == donorFormationId && ...
    pathFormationIds(end) == targetFormationId;

sourceSensorId = 0;
receiverSensorId = 0;
relayFormationId = 0;
downstreamFormationId = 0;
gatewayAligned = false;
candidateCount = 0;
sourceLabelExistence = NaN;
sourceLabelLocalizationRisk = NaN;
selectedLinkReliability = NaN;
if routeRequested
    relayFormationId = pathFormationIds(2);
    if numel(pathFormationIds) >= 3
        downstreamFormationId = pathFormationIds(3);
    else
        downstreamFormationId = relayFormationId;
    end
    [sourceSensorId, receiverSensorId, gatewayAligned, ...
     candidateCount, sourceLabelExistence, ...
     sourceLabelLocalizationRisk, selectedLinkReliability] = ...
        selectGateway(context, adjacency, groupIds, selectedLabel, ...
            donorFormationId, relayFormationId, ...
            downstreamFormationId);
end
routeActive = routeRequested && sourceSensorId > 0 && ...
    receiverSensorId > 0;

formationCount = numel(formationIds);
expectedMessages = nodeCount + 2 * (formationCount - 1);
positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
weights = base.fusionWeightMatrix;
hardGate = nnz(adjacency) == expectedMessages && ...
    ~any(logical(adjacency(:)) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(logical(adjacency)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if routeActive
    hardGate = hardGate && ...
        logical(context.physicalAdjacency( ...
            receiverSensorId, sourceSensorId)) && ...
        ~logical(adjacency(receiverSensorId, sourceSensorId)) && ...
        groupIds(sourceSensorId) == donorFormationId && ...
        groupIds(receiverSensorId) == relayFormationId;
end
if ~hardGate
    error('LabelSelectiveRiskShortcutV265:ProjectionFailed', ...
        'The V265 backbone or label shortcut violates its graph contract.');
end

details = base;
details.contractVersion = ...
    'label-selective-risk-shortcut-v265-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v242-backbone-with-one-label-selective-physical-hop';
details.backboneMode = details.mode;
details.fusionWeightMatrix = weights;
details.minimumBackboneAdjacency = logical(adjacency);
details.localizationTailRiskByFormation = getField(risk, ...
    'localizationTailRiskByFormation', zeros(1, formationCount));
details.localizationRelativeRiskByFormation = getField(risk, ...
    'localizationRelativeRiskByFormation', zeros(1, formationCount));
details.selectedLocalizationFormationId = targetFormationId;
details.selectedDonorFormationId = donorFormationId;
details.selectedLocalizationLabel = selectedLabel;
details.targetLabelLocalizationRisk = getField(risk, ...
    'targetLabelLocalizationRisk', NaN);
details.donorLabelLocalizationRisk = getField(risk, ...
    'donorLabelLocalizationRisk', NaN);
details.referenceFormationHopDistance = getField(risk, ...
    'referenceFormationHopDistance', Inf);
details.physicalFormationHopDistance = getField(risk, ...
    'physicalFormationHopDistance', Inf);
details.formationHopReduction = getField(risk, ...
    'formationHopReduction', 0);
details.shortcutPathFormationIds = pathFormationIds;
details.labelShortcutRequested = routeRequested;
details.labelShortcutActive = routeActive;
details.labelShortcutSourceSensorId = sourceSensorId;
details.labelShortcutReceiverSensorId = receiverSensorId;
details.labelShortcutRelayFormationId = relayFormationId;
details.labelShortcutDownstreamFormationId = downstreamFormationId;
details.labelShortcutGatewayAligned = gatewayAligned;
details.labelShortcutCandidateCount = candidateCount;
details.labelShortcutSourceExistence = sourceLabelExistence;
details.labelShortcutSourceLocalizationRisk = ...
    sourceLabelLocalizationRisk;
details.labelShortcutLinkReliability = selectedLinkReliability;
details.labelShortcutSourceWeight = protocol.sourceWeight;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = false;
details.currentNetworkPosteriorSynopsisUsed = true;
details.distributedControlSynopsisCostIncluded = false;
details.centralizedDevelopmentController = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.trackingOutcomeScored = false;

schedule = base.scheduleCertificate;
schedule.contractVersion = ...
    'label-selective-risk-shortcut-v265-schedule-v1';
schedule.phase = conditionalValue(routeActive, ...
    'v265-label-shortcut-active', 'v265-v242-backbone');
schedule.localizationTailRiskByFormation = ...
    details.localizationTailRiskByFormation;
schedule.localizationRelativeRiskByFormation = ...
    details.localizationRelativeRiskByFormation;
schedule.selectedLocalizationFormationId = targetFormationId;
schedule.selectedDonorFormationId = donorFormationId;
schedule.selectedLocalizationLabel = selectedLabel;
schedule.targetLabelLocalizationRisk = ...
    details.targetLabelLocalizationRisk;
schedule.donorLabelLocalizationRisk = ...
    details.donorLabelLocalizationRisk;
schedule.referenceFormationHopDistance = ...
    details.referenceFormationHopDistance;
schedule.physicalFormationHopDistance = ...
    details.physicalFormationHopDistance;
schedule.formationHopReduction = details.formationHopReduction;
schedule.shortcutPathFormationIds = pathFormationIds;
schedule.labelShortcutRequested = routeRequested;
schedule.labelShortcutActive = routeActive;
schedule.labelShortcutSourceSensorId = sourceSensorId;
schedule.labelShortcutReceiverSensorId = receiverSensorId;
schedule.labelShortcutRelayFormationId = relayFormationId;
schedule.labelShortcutDownstreamFormationId = downstreamFormationId;
schedule.labelShortcutGatewayAligned = gatewayAligned;
schedule.labelShortcutCandidateCount = candidateCount;
schedule.labelShortcutSourceExistence = sourceLabelExistence;
schedule.labelShortcutSourceLocalizationRisk = ...
    sourceLabelLocalizationRisk;
schedule.labelShortcutLinkReliability = selectedLinkReliability;
schedule.labelShortcutSourceWeight = protocol.sourceWeight;
schedule.currentMessageCount = nnz(adjacency);
schedule.centralizedDevelopmentController = true;
schedule.distributedControlSynopsisCostIncluded = false;
details.scheduleCertificate = schedule;
end

function [sourceId, receiverId, aligned, candidateCount, ...
          selectedExistence, selectedRisk, selectedReliability] = ...
        selectGateway(context, adjacency, groupIds, label, ...
            donorFormationId, relayFormationId, downstreamFormationId)
sourceId = 0;
receiverId = 0;
aligned = false;
candidateCount = 0;
selectedExistence = NaN;
selectedRisk = NaN;
selectedReliability = NaN;
sourceMembers = find(groupIds == donorFormationId);
receiverMembers = find(groupIds == relayFormationId);
downstreamMembers = find(groupIds == downstreamFormationId);
if isempty(sourceMembers) || isempty(receiverMembers)
    return;
end
summary = summarizeFormationLmbRiskModesV259( ...
    context.localPosteriorBySensor, groupIds, context.model);
labelIdx = find(all(summary.labels == label, 1), 1);
if isempty(labelIdx)
    return;
end
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
rows = zeros(0, 10);
for candidateReceiver = reshape(receiverMembers, 1, [])
    if downstreamFormationId == relayFormationId
        outbound = true;
    else
        outbound = any(logical(adjacency( ...
            downstreamMembers, candidateReceiver)));
    end
    for candidateSource = reshape(sourceMembers, 1, [])
        if ~logical(context.physicalAdjacency( ...
                candidateReceiver, candidateSource)) || ...
                logical(adjacency(candidateReceiver, candidateSource))
            continue;
        end
        existence = summary.existenceBySensorLabel( ...
            candidateSource, labelIdx);
        risk = summary.normalizedPositionMseBySensorLabel( ...
            candidateSource, labelIdx);
        if existence < 0.50 || ~isfinite(risk)
            continue;
        end
        reliability = edgeReliability( ...
            context.commConfig, candidateSource, ...
            candidateReceiver, context.currentTime);
        candidateCount = candidateCount + 1;
        rows(end + 1, :) = [ ...
            -double(outbound), risk, -existence, -reliability, ...
            sensorUids(candidateReceiver), sensorUids(candidateSource), ...
            candidateReceiver, candidateSource, existence, reliability]; ...
            %#ok<AGROW>
    end
end
if isempty(rows)
    return;
end
rows = sortrows(rows, [1, 2, 3, 4, 5, 6]);
receiverId = rows(1, 7);
sourceId = rows(1, 8);
selectedExistence = rows(1, 9);
selectedReliability = rows(1, 10);
selectedRisk = rows(1, 2);
aligned = rows(1, 1) < 0;
end

function value = edgeReliability(config, sender, receiver, currentTime)
if isfield(config, 'forceDelivery') && config.forceDelivery
    value = 1;
elseif isfield(config, 'pDropByEdge') && ...
        ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        page = min(currentTime, size(config.pDropByEdge, 3));
        value = 1 - config.pDropByEdge(sender, receiver, page);
    else
        value = 1 - config.pDropByEdge(sender, receiver);
    end
else
    value = 1;
end
value = min(max(value, 0), 1);
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

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
