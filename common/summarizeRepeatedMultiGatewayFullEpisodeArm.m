function summary = summarizeRepeatedMultiGatewayFullEpisodeArm( ...
        armId, estimates, diagnostics, truth, config, ...
        representatives, protocol, elapsedSeconds)
% SUMMARIZEREPEATEDMULTIGATEWAYFULLEPISODEARM Shared V89/V90 metrics.

sensorCount = numel(estimates);
timeCount = numel(truth.x);
eospa = zeros(sensorCount, timeCount);
cardinality = zeros(sensorCount, timeCount);
for sensorIdx = 1:sensorCount
    for currentTime = 1:timeCount
        components = computePositionEuclideanOspa( ...
            truth.x{currentTime}, estimates{sensorIdx}.mu{currentTime}, ...
            config.ospaPositionCutoff, 2, [1, 2]);
        eospa(sensorIdx, currentTime) = components(1);
        cardinality(sensorIdx, currentTime) = abs( ...
            numel(estimates{sensorIdx}.mu{currentTime}) - ...
            numel(truth.x{currentTime}));
    end
end

interFormation = zeros(1, timeCount);
for currentTime = 1:timeCount
    values = zeros(1, nchoosek(numel(representatives), 2));
    cursor = 0;
    for left = 1:numel(representatives)-1
        for right = left+1:numel(representatives)
            cursor = cursor + 1;
            components = computePositionEuclideanOspa( ...
                estimates{representatives(left)}.mu{currentTime}, ...
                estimates{representatives(right)}.mu{currentTime}, ...
                config.ospaPositionCutoff, 2, [1, 2]);
            values(cursor) = components(1);
        end
    end
    interFormation(currentTime) = mean(values);
end

focusTimes = protocol.activationStartTime:min( ...
    protocol.activationEndTime, timeCount);
groupIds = reshape(config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
perSensor = mean(eospa, 2);
perFormation = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    perFormation(formationIdx) = mean( ...
        perSensor(groupIds == groups(formationIdx)));
end

phaseCounts = struct('acquire', 0, 'broadcast', 0, 'reference', 0);
phaseByTime = repmat({''}, 1, timeCount);
selectedGatewayCountByTime = zeros(1, timeCount);
referenceFallbackByTime = false(1, timeCount);
for currentTime = 1:timeCount
    schedule = diagnostics.topologyPolicyScheduleCertificate{currentTime};
    if ~isstruct(schedule) || ~isfield(schedule, 'phase')
        continue;
    end
    if isfield(phaseCounts, schedule.phase)
        phaseCounts.(schedule.phase) = ...
            phaseCounts.(schedule.phase) + 1;
    end
    phaseByTime{currentTime} = schedule.phase;
    selectedGatewayCountByTime(currentTime) = ...
        getField(schedule, 'selectedGatewayCount', 0);
    referenceFallbackByTime(currentTime) = logical( ...
        getField(schedule, 'referenceFallbackUsed', false));
end

[sensorB3, formationB3] = selectedRollingB3( ...
    diagnostics.topologyActiveEdge, groupIds);
summary = struct();
summary.armId = armId;
summary.elapsedSeconds = elapsedSeconds;
summary.fullHorizonPositionEospa = mean(eospa(:));
summary.focusWindowPositionEospa = mean( ...
    reshape(eospa(:, focusTimes), 1, []));
summary.worstSensorPositionEospa = max(perSensor);
summary.perSensorPositionEospa = reshape(perSensor, 1, []);
summary.perFormationPositionEospa = perFormation;
summary.positionEospaBySensorTime = eospa;
summary.networkMeanPositionEospaByTime = mean(eospa, 1);
summary.meanAbsoluteCardinalityError = mean(cardinality(:));
summary.focusAbsoluteCardinalityError = mean( ...
    reshape(cardinality(:, focusTimes), 1, []));
summary.absoluteCardinalityErrorBySensorTime = cardinality;
summary.networkMeanAbsoluteCardinalityErrorByTime = ...
    mean(cardinality, 1);
summary.meanInterFormationPositionOspa = mean(interFormation);
summary.focusInterFormationPositionOspa = mean( ...
    interFormation(focusTimes));
summary.terminalInterFormationPositionOspa = interFormation(end);
summary.interFormationPositionOspaByTime = interFormation;
summary.attemptedMessageCount = nnz(diagnostics.attempted);
summary.deliveredMessageCount = nnz(diagnostics.delivered);
summary.attemptedPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(:));
summary.deliveredPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(logical(diagnostics.delivered)));
summary.phaseCounts = phaseCounts;
summary.phaseByTime = phaseByTime;
summary.selectedGatewayCount = sum(selectedGatewayCountByTime);
summary.selectedGatewayCountByTime = selectedGatewayCountByTime;
summary.referenceFallbackCount = nnz(referenceFallbackByTime);
summary.referenceFallbackByTime = referenceFallbackByTime;
summary.exactMessageParityPassed = all( ...
    diagnostics.topologyDirectedMessageCount == 2 * sensorCount);
summary.physicalFeasibilityPassed = all(diagnostics.topologyFeasible);
summary.selectedRollingB3SensorPassed = sensorB3;
summary.selectedRollingB3FormationPassed = formationB3;
summary.missingLabelFusedLabelCount = sumDiagnostic( ...
    diagnostics, 'missingLabelFusedLabelCount');
summary.missingLabelAffectedLabelCount = sumDiagnostic( ...
    diagnostics, 'missingLabelAffectedLabelCount');
summary.missingLabelSourceCount = sumDiagnostic( ...
    diagnostics, 'missingLabelSourceCount');
summary.missingLabelObservableCensoredSourceCount = sumDiagnostic( ...
    diagnostics, 'missingLabelObservableCensoredSourceCount');
summary.missingLabelLegacyExcludedSourceCount = sumDiagnostic( ...
    diagnostics, 'missingLabelLegacyExcludedSourceCount');
summary.missingLabelUninformativeExcludedSourceCount = sumDiagnostic( ...
    diagnostics, 'missingLabelUninformativeExcludedSourceCount');
summary.missingLabelStaleIgnoredSourceCount = sumDiagnostic( ...
    diagnostics, 'missingLabelStaleIgnoredSourceCount');
summary.missingLabelStrictVetoedLabelCount = sumDiagnostic( ...
    diagnostics, 'missingLabelStrictVetoedLabelCount');
end

function value = sumDiagnostic(diagnostics, name)
value = 0;
if isstruct(diagnostics) && isfield(diagnostics, name)
    entries = diagnostics.(name);
    value = sum(entries(:));
end
end

function [sensorPassed, formationPassed] = ...
        selectedRollingB3(runtimeEdges, groupIds)
sensorPassed = true;
formationPassed = true;
policyPages = permute(logical(runtimeEdges), [2, 1, 3]);
for currentTime = 3:size(policyPages, 3)
    window = any(policyPages(:, :, currentTime-2:currentTime), 3);
    sensorPassed = sensorPassed && isStronglyConnectedLocal(window);
    formationPassed = formationPassed && isStronglyConnectedLocal( ...
        collapseToFormations(window, groupIds));
end
end

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(groupIds, 'stable');
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

function connected = isStronglyConnectedLocal(adjacency)
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

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
