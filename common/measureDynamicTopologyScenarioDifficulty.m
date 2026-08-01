function metrics = measureDynamicTopologyScenarioDifficulty( ...
    config, sensorTrajectories, targetTrajectories, graphData)
% MEASUREDYNAMICTOPOLOGYSCENARIODIFFICULTY Quantify useful scene pressure.
%
% The metrics deliberately separate useful partial observability from
% impossible tracking. A difficult dynamic-topology scene should contain
% repeated changes in which formation owns useful target information, short
% overlap intervals that permit handover, and few complete sensing blackouts.

timeCount = config.simulationLength;
formationCount = config.formationCount;
targetCount = numel(targetTrajectories);
focusWindow = resolveFocusWindow(config, timeCount);
closeDistance = getField(config, ...
    'difficultyCloseTargetDistance', 80);
headingSchedule = buildSensorFovHeadingSchedule( ...
    config, sensorTrajectories);
qualityModel = buildQualityAuditModel( ...
    config, sensorTrajectories, headingSchedule);

visibilityCount = nan(targetCount, timeCount);
visibleSensorCount = nan(targetCount, timeCount);
expectedDetectionCount = nan(targetCount, timeCount);
atLeastOneDetectionProbability = nan(targetCount, timeCount);
visibleTargetCountBySensorTime = zeros( ...
    config.numberOfSensors, timeCount);
ownerByTarget = zeros(targetCount, timeCount);
activeMask = false(targetCount, timeCount);
formationOwnershipSamples = zeros(1, formationCount);
activeSampleCount = 0;
blackoutCount = 0;
singleFormationCount = 0;
multiFormationCount = 0;

for targetIdx = 1:targetCount
    trajectory = targetTrajectories{targetIdx};
    for timeIdx = 1:timeCount
        state = trajectory(:, timeIdx);
        if any(~isfinite(state))
            continue;
        end
        activeMask(targetIdx, timeIdx) = true;
        activeSampleCount = activeSampleCount + 1;
        [visible, nearestDistance, visibleSensors, ...
            detectionProbabilities] = visibleFormations( ...
                config, sensorTrajectories, qualityModel, ...
                state, timeIdx);
        visibleCount = sum(visible);
        visibilityCount(targetIdx, timeIdx) = visibleCount;
        visibleSensorCount(targetIdx, timeIdx) = ...
            sum(visibleSensors);
        expectedDetectionCount(targetIdx, timeIdx) = ...
            sum(detectionProbabilities);
        atLeastOneDetectionProbability(targetIdx, timeIdx) = ...
            1 - prod(1 - detectionProbabilities);
        visibleTargetCountBySensorTime(:, timeIdx) = ...
            visibleTargetCountBySensorTime(:, timeIdx) + ...
                double(visibleSensors(:));
        if visibleCount == 0
            blackoutCount = blackoutCount + 1;
        elseif visibleCount == 1
            singleFormationCount = singleFormationCount + 1;
        else
            multiFormationCount = multiFormationCount + 1;
        end
        if visibleCount > 0
            nearestDistance(~visible) = inf;
            [~, owner] = min(nearestDistance);
            ownerByTarget(targetIdx, timeIdx) = owner;
            formationOwnershipSamples(owner) = ...
                formationOwnershipSamples(owner) + 1;
        end
    end
end

handoverCounts = zeros(1, targetCount);
focusHandoverCounts = zeros(1, targetCount);
for targetIdx = 1:targetCount
    owners = ownerByTarget(targetIdx, :);
    handoverCounts(targetIdx) = countOwnerChanges(owners, 1:timeCount);
    focusHandoverCounts(targetIdx) = countOwnerChanges( ...
        owners, focusWindow);
end

closeEncounterTimes = false(1, timeCount);
closestTargetDistance = nan(1, timeCount);
crossGroupClosestDistance = nan(1, timeCount);
for timeIdx = 1:timeCount
    activeTargets = find(activeMask(:, timeIdx));
    if numel(activeTargets) < 2
        continue;
    end
    minimumDistance = inf;
    for leftCursor = 1:numel(activeTargets)-1
        leftPosition = targetTrajectories{ ...
            activeTargets(leftCursor)}(1:2, timeIdx);
        for rightCursor = leftCursor+1:numel(activeTargets)
            rightPosition = targetTrajectories{ ...
                activeTargets(rightCursor)}(1:2, timeIdx);
            minimumDistance = min(minimumDistance, ...
                norm(leftPosition - rightPosition));
        end
    end
    closestTargetDistance(timeIdx) = minimumDistance;
    minimumCrossGroupDistance = inf;
    for leftCursor = 1:numel(activeTargets)-1
        leftIdx = activeTargets(leftCursor);
        leftPosition = targetTrajectories{leftIdx}(1:2, timeIdx);
        for rightCursor = leftCursor+1:numel(activeTargets)
            rightIdx = activeTargets(rightCursor);
            if config.targetGroupIds(leftIdx) == ...
                    config.targetGroupIds(rightIdx)
                continue;
            end
            rightPosition = targetTrajectories{rightIdx}(1:2, timeIdx);
            minimumCrossGroupDistance = min( ...
                minimumCrossGroupDistance, ...
                norm(leftPosition - rightPosition));
        end
    end
    if isfinite(minimumCrossGroupDistance)
        crossGroupClosestDistance(timeIdx) = ...
            minimumCrossGroupDistance;
        closeEncounterTimes(timeIdx) = ...
            minimumCrossGroupDistance <= closeDistance;
    end
end

if nargin < 4 || isempty(graphData)
    physicalEdgeChurnRate = NaN;
    meanPhysicalInterFormationEdges = NaN;
else
    [physicalEdgeChurnRate, meanPhysicalInterFormationEdges] = ...
        physicalGraphMetrics( ...
            graphData.physicalAdjacency, config.sensorGroupIds);
end

metrics = struct();
metrics.activeTargetSamples = activeSampleCount;
metrics.blackoutFraction = blackoutCount / max(activeSampleCount, 1);
blackoutMask = activeMask & visibilityCount == 0;
focusActiveMask = activeMask(:, focusWindow);
focusBlackoutMask = blackoutMask(:, focusWindow);
metrics.focusBlackoutFraction = sum(focusBlackoutMask(:)) / ...
    max(sum(focusActiveMask(:)), 1);
activeSamplesByTarget = sum(activeMask, 2);
blackoutSamplesByTarget = sum(blackoutMask, 2);
activeTargets = activeSamplesByTarget > 0;
perTargetBlackoutFraction = zeros(targetCount, 1);
perTargetBlackoutFraction(activeTargets) = ...
    blackoutSamplesByTarget(activeTargets) ./ ...
        activeSamplesByTarget(activeTargets);
metrics.maximumPerTargetBlackoutFraction = ...
    max(perTargetBlackoutFraction);
maximumConsecutiveBlackoutSteps = 0;
for targetIdx = 1:targetCount
    maximumConsecutiveBlackoutSteps = max( ...
        maximumConsecutiveBlackoutSteps, ...
        longestTrueRun(blackoutMask(targetIdx, :)));
end
metrics.maximumConsecutiveBlackoutSteps = ...
    maximumConsecutiveBlackoutSteps;
metrics.singleFormationFraction = ...
    singleFormationCount / max(activeSampleCount, 1);
metrics.multiFormationFraction = ...
    multiFormationCount / max(activeSampleCount, 1);
metrics.meanVisibleFormationCount = finiteMean(visibilityCount(:));
metrics.meanVisibleSensorCount = finiteMean(visibleSensorCount(:));
metrics.focusMeanVisibleSensorCount = finiteMean( ...
    visibleSensorCount(:, focusWindow));
metrics.meanExpectedTargetDetectionCount = finiteMean( ...
    expectedDetectionCount(:));
metrics.focusMeanExpectedTargetDetectionCount = finiteMean( ...
    expectedDetectionCount(:, focusWindow));
metrics.meanAtLeastOneDetectionProbability = finiteMean( ...
    atLeastOneDetectionProbability(:));
metrics.focusMeanAtLeastOneDetectionProbability = finiteMean( ...
    atLeastOneDetectionProbability(:, focusWindow));
metrics.meanVisibleSensorsPerVisibleFormation = ...
    metrics.meanVisibleSensorCount / ...
        max(metrics.meanVisibleFormationCount, eps);
metrics.meanVisibleTargetsPerSensorTime = mean( ...
    visibleTargetCountBySensorTime(:));
focusVisibleTargetCountBySensorTime = ...
    visibleTargetCountBySensorTime(:, focusWindow);
metrics.focusMeanVisibleTargetsPerSensorTime = mean( ...
    focusVisibleTargetCountBySensorTime(:));
metrics.maximumVisibleTargetsPerSensorTime = max( ...
    visibleTargetCountBySensorTime(:));
metrics.targetHandoverCounts = handoverCounts;
metrics.totalHandovers = sum(handoverCounts);
metrics.focusTargetHandoverCounts = focusHandoverCounts;
metrics.focusHandovers = sum(focusHandoverCounts);
metrics.ownerByTarget = ownerByTarget;
metrics.formationOwnershipFraction = formationOwnershipSamples / ...
    max(sum(formationOwnershipSamples), 1);
metrics.formationOwnershipEntropy = normalizedEntropy( ...
    metrics.formationOwnershipFraction);
metrics.closeTargetDistance = closeDistance;
metrics.closeEncounterTimeFraction = mean(closeEncounterTimes);
metrics.focusCloseEncounterTimeFraction = mean( ...
    closeEncounterTimes(focusWindow));
metrics.minimumTargetSeparation = finiteMinimum( ...
    closestTargetDistance);
metrics.minimumCrossGroupTargetSeparation = finiteMinimum( ...
    crossGroupClosestDistance);
metrics.physicalEdgeChurnRate = physicalEdgeChurnRate;
metrics.meanPhysicalInterFormationEdges = ...
    meanPhysicalInterFormationEdges;
metrics.blockageFocusOverlapFraction = ...
    blockageFocusOverlap(config, focusWindow);
metrics.focusWindow = [focusWindow(1), focusWindow(end)];
end

function longest = longestTrueRun(mask)
longest = 0;
current = 0;
for sample = reshape(logical(mask), 1, [])
    if sample
        current = current + 1;
        longest = max(longest, current);
    else
        current = 0;
    end
end
end

function [visible, nearestDistance, visibleSensors, ...
        detectionProbabilities] = visibleFormations( ...
    config, sensorTrajectories, qualityModel, targetState, timeIdx)
visible = false(1, config.formationCount);
nearestDistance = inf(1, config.formationCount);
visibleSensors = false(1, config.numberOfSensors);
detectionProbabilities = zeros(1, config.numberOfSensors);
for formationIdx = 1:config.formationCount
    sensors = find(config.sensorGroupIds == formationIdx);
    for sensorIdx = reshape(sensors, 1, [])
        sensorState = sensorTrajectories{sensorIdx}(:, timeIdx);
        relative = targetState(1:2) - sensorState(1:2);
        distance = norm(relative);
        nearestDistance(formationIdx) = min( ...
            nearestDistance(formationIdx), distance);
        [detectionProbability, ~, qualityInfo] = ...
            evaluateSensorQuality( ...
                qualityModel, sensorIdx, targetState, timeIdx);
        visibleSensors(sensorIdx) = qualityInfo.inFov;
        detectionProbabilities(sensorIdx) = ...
            detectionProbability;
        visible(formationIdx) = ...
            visible(formationIdx) || qualityInfo.inFov;
    end
end
end

function model = buildQualityAuditModel( ...
        config, sensorTrajectories, headingSchedule)
sensorCount = config.numberOfSensors;
model = struct();
model.detectionProbability = ...
    config.detectionProbability * ones(sensorCount, 1);
model.Q = repmat( ...
    {(config.measurementNoiseStd ^ 2) * eye(2)}, ...
    1, sensorCount);
model.sensorTrajectories = sensorTrajectories;
model.sensorMotionEnabled = true;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = ...
    config.fovHalfAngleDeg * ones(1, sensorCount);
model.sensorFovRange = ...
    config.fovRange * ones(1, sensorCount);
model.sensorFovHeadingRad = headingSchedule;
model.sensorQuality = config.sensorQuality;
end

function count = countOwnerChanges(owners, indices)
count = 0;
previous = 0;
for timeIdx = reshape(indices, 1, [])
    current = owners(timeIdx);
    if current <= 0
        continue;
    end
    if previous > 0 && current ~= previous
        count = count + 1;
    end
    previous = current;
end
end

function [churnRate, meanInterFormationEdges] = physicalGraphMetrics( ...
    physicalAdjacency, groupIds)
timeCount = size(physicalAdjacency, 3);
edgeCounts = zeros(1, timeCount);
interFormationCounts = zeros(1, timeCount);
changes = zeros(1, max(timeCount - 1, 1));
for timeIdx = 1:timeCount
    current = logical(physicalAdjacency(:, :, timeIdx));
    edgeCounts(timeIdx) = nnz(triu(current, 1));
    interMask = false(size(current));
    for leftIdx = 1:numel(groupIds)-1
        for rightIdx = leftIdx+1:numel(groupIds)
            if groupIds(leftIdx) ~= groupIds(rightIdx)
                interMask(leftIdx, rightIdx) = true;
                interMask(rightIdx, leftIdx) = true;
            end
        end
    end
    interFormationCounts(timeIdx) = nnz(triu( ...
        current & interMask, 1));
    if timeIdx > 1
        previous = logical(physicalAdjacency(:, :, timeIdx - 1));
        changes(timeIdx - 1) = nnz(triu(xor(current, previous), 1)) / ...
            max(edgeCounts(timeIdx - 1), 1);
    end
end
if timeCount <= 1
    churnRate = 0;
else
    churnRate = mean(changes(1:timeCount-1));
end
meanInterFormationEdges = mean(interFormationCounts);
end

function overlap = blockageFocusOverlap(config, focusWindow)
windows = getField(config, 'blockageWindows', zeros(0, 4));
if isempty(windows)
    overlap = 0;
    return;
end
blocked = false(1, config.simulationLength);
for windowIdx = 1:size(windows, 1)
    startTime = max(1, round(windows(windowIdx, 3)));
    stopTime = min(config.simulationLength, ...
        round(windows(windowIdx, 4)));
    if stopTime >= startTime
        blocked(startTime:stopTime) = true;
    end
end
overlap = mean(blocked(focusWindow));
end

function entropyValue = normalizedEntropy(probabilities)
positive = probabilities(probabilities > 0);
if numel(positive) <= 1
    entropyValue = 0;
    return;
end
entropyValue = -sum(positive .* log(positive)) / ...
    log(numel(probabilities));
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = finiteMinimum(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function indices = resolveFocusWindow(config, timeCount)
window = getField(config, 'focusWindow', [1, timeCount]);
startTime = max(1, min(timeCount, round(window(1))));
stopTime = max(startTime, min(timeCount, round(window(end))));
indices = startTime:stopTime;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
