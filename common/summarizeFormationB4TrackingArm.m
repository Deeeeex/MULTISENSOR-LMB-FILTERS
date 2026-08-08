function summary = summarizeFormationB4TrackingArm( ...
        armId, estimates, diagnostics, truth, config, ...
        representativeSensors, protocol, elapsedSeconds)
% SUMMARIZEFORMATIONB4TRACKINGARM Tracking and communication outcomes.

sensorCount = numel(estimates);
timeCount = numel(truth.x);
cutoff = config.ospaPositionCutoff;
eospa = zeros(sensorCount, timeCount);
cardinalityError = zeros(sensorCount, timeCount);
for sensorIdx = 1:sensorCount
    for currentTime = 1:timeCount
        components = computePositionEuclideanOspa( ...
            truth.x{currentTime}, estimates{sensorIdx}.mu{currentTime}, ...
            cutoff, 2, [1, 2]);
        eospa(sensorIdx, currentTime) = components(1);
        cardinalityError(sensorIdx, currentTime) = abs( ...
            numel(estimates{sensorIdx}.mu{currentTime}) - ...
            numel(truth.x{currentTime}));
    end
end

interFormation = zeros(1, timeCount);
for currentTime = 1:timeCount
    pairValues = zeros(1, nchoosek(numel(representativeSensors), 2));
    cursor = 0;
    for leftIdx = 1:numel(representativeSensors)-1
        for rightIdx = leftIdx+1:numel(representativeSensors)
            cursor = cursor + 1;
            components = computePositionEuclideanOspa( ...
                estimates{representativeSensors(leftIdx)}.mu{currentTime}, ...
                estimates{representativeSensors(rightIdx)}.mu{currentTime}, ...
                cutoff, 2, [1, 2]);
            pairValues(cursor) = components(1);
        end
    end
    interFormation(currentTime) = mean(pairValues);
end

focusTimes = config.focusWindow(1):config.focusWindow(2);
attempted = logical(diagnostics.attempted);
delivered = logical(diagnostics.delivered);
alignedEnds = protocol.period:protocol.period:timeCount;
deliveredStrong = false(size(alignedEnds));
for windowIdx = 1:numel(alignedEnds)
    windowEnd = alignedEnds(windowIdx);
    window = windowEnd-protocol.period+1:windowEnd;
    deliveredStrong(windowIdx) = isStronglyConnectedLocal( ...
        any(delivered(:, :, window), 3));
end

phaseOneTimes = 1:protocol.period:timeCount;
cycleSelected = false(size(phaseOneTimes));
if strcmp(armId, protocol.candidateArmId)
    for phaseIdx = 1:numel(phaseOneTimes)
        schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
            phaseOneTimes(phaseIdx)};
        cycleSelected(phaseIdx) = isstruct(schedule) && ...
            isfield(schedule, 'cycleSelected') && ...
            schedule.cycleSelected;
    end
end

groups = reshape(config.sensorGroupIds, 1, []);
crossMask = groups(:) ~= groups(:)';
burstCrossChanges = zeros(1, max(0, numel(phaseOneTimes) - 1));
for phaseIdx = 2:numel(phaseOneTimes)
    previous = logical(diagnostics.topologyActiveEdge( ...
        :, :, phaseOneTimes(phaseIdx - 1)));
    current = logical(diagnostics.topologyActiveEdge( ...
        :, :, phaseOneTimes(phaseIdx)));
    burstCrossChanges(phaseIdx - 1) = nnz( ...
        xor(previous, current) & crossMask);
end

selectedObjective = diagnostics.topologyPolicyObjective(phaseOneTimes);
selectedObjective = selectedObjective(cycleSelected & ...
    isfinite(selectedObjective));
summary = struct();
summary.armId = armId;
summary.elapsedSeconds = elapsedSeconds;
summary.fullHorizonPositionEospa = mean(eospa(:));
summary.focusWindowPositionEospa = mean( ...
    reshape(eospa(:, focusTimes), 1, []));
summary.worstSensorPositionEospa = max(mean(eospa, 2));
summary.meanAbsoluteCardinalityError = mean(cardinalityError(:));
summary.meanInterFormationPositionOspa = mean(interFormation);
summary.focusInterFormationPositionOspa = mean( ...
    interFormation(focusTimes));
summary.terminalInterFormationPositionOspa = interFormation(end);
summary.attemptedMessageCount = nnz(attempted);
summary.deliveredMessageCount = nnz(delivered);
summary.attemptedPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(:));
summary.deliveredPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(delivered));
summary.deliveredAlignedB4StrongFraction = mean(deliveredStrong);
summary.cycleSelectedCount = nnz(cycleSelected);
summary.cycleOpportunityCount = numel(phaseOneTimes);
summary.cycleSelectionFraction = mean(cycleSelected);
summary.meanSelectedPosteriorObjective = 0;
if ~isempty(selectedObjective)
    summary.meanSelectedPosteriorObjective = mean(selectedObjective);
end
summary.meanBurstCrossEdgeChanges = 0;
summary.burstRouteChangeFraction = 0;
if ~isempty(burstCrossChanges)
    summary.meanBurstCrossEdgeChanges = mean(burstCrossChanges);
    summary.burstRouteChangeFraction = mean(burstCrossChanges > 0);
end
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
