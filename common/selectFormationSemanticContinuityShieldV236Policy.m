function [adjacency, details] = ...
        selectFormationSemanticContinuityShieldV236Policy( ...
            context, staticRegistration)
% SELECTFORMATIONSEMANTICCONTINUITYSHIELDV236POLICY Formation-row teacher.

protocol = getFormationSemanticContinuityShieldV236Protocol();
validateRegistration(staticRegistration, context, protocol);
[dynamicAdjacency, details] = ...
    selectCorrectedDynamicFormationBackboneV227Policy(context);
adjacency = logical(dynamicAdjacency);
weights = details.fusionWeightMatrix;
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
members = find(groupIds == protocol.targetFormationId);
if isempty(members)
    error('FormationSemanticContinuityV236:MissingFormation', ...
        'The registered target formation is absent from this scene.');
end

shieldApplied = context.currentTime >= protocol.shieldStartTime;
if shieldApplied
    adjacency(members, :) = ...
        logical(staticRegistration.adjacency(members, :));
    weights(members, :) = ...
        staticRegistration.fusionWeightMatrix(members, :);
    details = replaceRouteRows(details, ...
        staticRegistration.selectionDetails, members);
end

nodeCount = numel(groupIds);
physical = logical(context.physicalAdjacency);
weightSupport = adjacency | logical(eye(nodeCount));
sensorStrong = isStronglyConnected(adjacency);
formationStrong = isStronglyConnected( ...
    collapseToFormations(adjacency, groupIds));
if any(adjacency(:) & ~physical(:)) || ...
        nnz(adjacency) ~= ...
            protocol.directedInputsPerReceiver * nodeCount || ...
        any(sum(adjacency, 2) ~= protocol.directedInputsPerReceiver) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12) || ...
        any(weights(:) < 0) || ...
        any(weights(:) > 0 & ~weightSupport(:))
    error('FormationSemanticContinuityV236:InvalidHybridRoute', ...
        'The formation-row shield violated a hard route invariant.');
end

details.contractVersion = ...
    'formation-semantic-continuity-shield-v236-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'outcome-derived-formation-static-row-shield-teacher';
details.backboneMode = details.mode;
details.fusionWeightMatrix = weights;
details.currentMessageCount = nnz(adjacency);
details.referenceMessageCount = nnz(adjacency);
details.referenceFallbackUsed = shieldApplied;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.instantaneousSensorStrongConnected = sensorStrong;
details.instantaneousFormationStrongConnected = formationStrong;
details.sensorWindowStrongConnected = sensorStrong;
details.formationWindowStrongConnected = formationStrong;
details.oneStepTopologyReserveChecked = true;
details.oneStepTopologyReservePassed = sensorStrong && formationStrong;
details.recursiveSafetyClaimed = false;
details.teacherScheduleOutcomeDerived = true;
details.targetFormationId = protocol.targetFormationId;
details.shieldStartTime = protocol.shieldStartTime;
details.shieldApplied = shieldApplied;
details.scheduleCertificate = buildSchedule( ...
    context.currentTime, protocol, shieldApplied, ...
    sensorStrong, formationStrong);
end

function details = replaceRouteRows(details, staticDetails, members)
rowFields = {'dominantAdjacency', 'residualAdjacency'};
for fieldIndex = 1:numel(rowFields)
    name = rowFields{fieldIndex};
    if isfield(details, name) && isfield(staticDetails, name)
        details.(name)(members, :) = staticDetails.(name)(members, :);
    end
end
sourceFields = {'dominantSourcesByReceiver', ...
    'residualSourcesByReceiver', 'dominantWeightsByReceiver', ...
    'residualWeightsByReceiver', 'selfWeightsByReceiver'};
for fieldIndex = 1:numel(sourceFields)
    name = sourceFields{fieldIndex};
    if isfield(details, name) && isfield(staticDetails, name)
        details.(name)(members) = staticDetails.(name)(members);
    end
end
end

function validateRegistration(registration, context, protocol)
required = {'contractVersion', 'adjacency', 'fusionWeightMatrix', ...
    'selectionDetails', 'messageCount'};
nodeCount = numel(context.localPosteriorBySensor);
if ~isstruct(registration) || ~isscalar(registration) || ...
        ~all(isfield(registration, required)) || ...
        ~strcmp(registration.contractVersion, ...
            'corrected-static-routing-v227-registration-v1') || ...
        ~isequal(size(registration.adjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(registration.fusionWeightMatrix), ...
            [nodeCount, nodeCount]) || ...
        registration.messageCount ~= ...
            protocol.directedInputsPerReceiver * nodeCount
    error('FormationSemanticContinuityV236:InvalidRegistration', ...
        'V236 requires the matched V227 static-route registration.');
end
end

function schedule = buildSchedule( ...
        currentTime, protocol, applied, sensorStrong, formationStrong)
if applied
    phase = 'dynamic-with-f4-static-row-shield';
    phaseIndex = 2;
    formationIds = protocol.targetFormationId;
else
    phase = 'dynamic-incumbent';
    phaseIndex = 1;
    formationIds = zeros(1, 0);
end
schedule = struct( ...
    'contractVersion', ...
        'formation-semantic-continuity-v236-schedule-v1', ...
    'currentTime', currentTime, 'phase', phase, ...
    'phaseIndex', phaseIndex, 'gatewayIndices', zeros(1, 0), ...
    'formationIds', formationIds, 'sourceIndices', zeros(1, 0), ...
    'proposalGatewayCount', 0, 'selectedGatewayCount', 0, ...
    'pendingAcquisitionTime', NaN, 'messageParityPassed', true, ...
    'rollingSensorStrong', sensorStrong, ...
    'rollingFormationStrong', formationStrong, ...
    'referenceFallbackUsed', applied, 'cycleSelected', true, ...
    'teacherScheduleOutcomeDerived', true, ...
    'truthReadAtRuntime', false, 'futureOutcomeReadAtRuntime', false);
end

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(groupIds, 'stable');
formation = false(numel(groups));
for receiverGroupIndex = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIndex);
    for senderGroupIndex = 1:numel(groups)
        senders = groupIds == groups(senderGroupIndex);
        formation(receiverGroupIndex, senderGroupIndex) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formation(1:numel(groups)+1:end) = false;
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
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end
