function validation = validateDynamicTopologyScenario( ...
    config, sensorTrajectories, targetTrajectories, graphData)
% VALIDATEDYNAMICTOPOLOGYSCENARIO Fail closed on invalid experiment scenes.

sensorCount = config.numberOfSensors;
timeCount = config.simulationLength;
staticAdjacency = graphData.staticAdjacency;
hardFailures = {};

if numel(sensorTrajectories) ~= sensorCount
    hardFailures{end+1} = 'sensor-count-mismatch'; %#ok<AGROW>
end
if numel(targetTrajectories) ~= config.numberOfTargets
    hardFailures{end+1} = 'target-count-mismatch'; %#ok<AGROW>
end

[maxSensorSpeed, maxSensorAcceleration, minSeparation, sensorsInBounds] = ...
    sensorTrajectoryMetrics(config, sensorTrajectories);
[maxTargetSpeed, maxTargetAcceleration, minTargetSeparation, ...
    targetsInBounds, targetStatesWellFormed] = ...
    targetTrajectoryMetrics(config, targetTrajectories);

if minSeparation + 1e-9 < config.minimumSensorSeparation
    hardFailures{end+1} = 'sensor-separation'; %#ok<AGROW>
end
if maxSensorSpeed > config.sensorSpeedLimit + 1e-9
    hardFailures{end+1} = 'sensor-speed'; %#ok<AGROW>
end
if maxSensorAcceleration > config.sensorAccelerationLimit + 1e-9
    hardFailures{end+1} = 'sensor-acceleration'; %#ok<AGROW>
end
if maxTargetSpeed > config.targetSpeedLimit + 1e-9
    hardFailures{end+1} = 'target-speed'; %#ok<AGROW>
end
if isfield(config, 'targetAccelerationLimit') && ...
        maxTargetAcceleration > config.targetAccelerationLimit + 1e-9
    hardFailures{end+1} = 'target-acceleration'; %#ok<AGROW>
end
if isfield(config, 'minimumTargetSeparation') && ...
        minTargetSeparation + 1e-9 < config.minimumTargetSeparation
    hardFailures{end+1} = 'target-separation'; %#ok<AGROW>
end
if ~sensorsInBounds
    hardFailures{end+1} = 'sensor-bounds'; %#ok<AGROW>
end
if ~targetsInBounds
    hardFailures{end+1} = 'target-bounds'; %#ok<AGROW>
end
if ~targetStatesWellFormed
    hardFailures{end+1} = 'target-state-column-integrity'; %#ok<AGROW>
end

staticEdgeCount = nnz(triu(staticAdjacency, 1));
if staticEdgeCount > config.edgeBudget
    hardFailures{end+1} = 'static-edge-budget'; %#ok<AGROW>
end
if max(sum(staticAdjacency, 2)) > config.maxNodeDegree
    hardFailures{end+1} = 'static-degree-cap'; %#ok<AGROW>
end
if ~isConnected(staticAdjacency)
    hardFailures{end+1} = 'static-global-connectivity'; %#ok<AGROW>
end
for groupIdx = 1:config.formationCount
    group = find(config.sensorGroupIds == groupIdx);
    if ~isConnected(staticAdjacency(group, group))
        hardFailures{end+1} = sprintf( ...
            'static-group-%d-connectivity', groupIdx); %#ok<AGROW>
    end
end
staticPhysicalViolationCount = 0;
for timeIdx = 1:timeCount
    staticPhysicalViolationCount = staticPhysicalViolationCount + nnz( ...
        staticAdjacency & ~graphData.physicalAdjacency(:, :, timeIdx));
end
if staticPhysicalViolationCount > 0 && ...
        getField(config, 'requireStaticPhysicalAllTimes', true)
    hardFailures{end+1} = 'static-physical-violation'; %#ok<AGROW>
end

candidateCount = size(graphData.candidateAdjacency, 3);
candidateViolationCount = 0;
for candidateIdx = 1:candidateCount
    candidate = graphData.candidateAdjacency(:, :, candidateIdx);
    if nnz(triu(candidate, 1)) > config.edgeBudget || ...
            ~isConnected(candidate)
        candidateViolationCount = candidateViolationCount + 1;
        continue;
    end
    for timeIdx = 1:timeCount
        if any(any(candidate & ...
                ~graphData.physicalAdjacency(:, :, timeIdx)))
            candidateViolationCount = candidateViolationCount + 1;
            break;
        end
    end
end
if candidateViolationCount > 0
    hardFailures{end+1} = 'candidate-topology-violation'; %#ok<AGROW>
end
if strcmpi(config.topologyFamily, 'd12-enumerated') && ...
        candidateCount ~= 48
    hardFailures{end+1} = 'd12-candidate-count'; %#ok<AGROW>
end

handoverCounts = countTargetGroupHandovers( ...
    config, sensorTrajectories, targetTrajectories);
if any(strcmpi(config.variant, {'handover', 'composite'})) && ...
        any(handoverCounts < 1)
    hardFailures{end+1} = 'missing-target-handover'; %#ok<AGROW>
end
difficulty = measureDynamicTopologyScenarioDifficulty( ...
    config, sensorTrajectories, targetTrajectories, graphData);
if isfield(config, 'enforceDifficultyRequirements') && ...
        config.enforceDifficultyRequirements
    difficultyFailures = validateDifficultyRequirements( ...
        difficulty, config.difficultyRequirements);
    hardFailures = [hardFailures, difficultyFailures]; %#ok<AGROW>
end

validation = struct();
validation.isValid = isempty(hardFailures);
validation.hardFailures = hardFailures;
validation.maxSensorSpeed = maxSensorSpeed;
validation.maxSensorAcceleration = maxSensorAcceleration;
validation.minimumSensorSeparation = minSeparation;
validation.maxTargetSpeed = maxTargetSpeed;
validation.maxTargetAcceleration = maxTargetAcceleration;
validation.minimumTargetSeparation = minTargetSeparation;
validation.sensorsInBounds = sensorsInBounds;
validation.targetsInBounds = targetsInBounds;
validation.targetStatesWellFormed = targetStatesWellFormed;
validation.staticEdgeCount = staticEdgeCount;
validation.staticPhysicalViolationCount = staticPhysicalViolationCount;
validation.candidateCount = candidateCount;
validation.candidateViolationCount = candidateViolationCount;
validation.targetGroupHandoverCounts = handoverCounts;
validation.difficulty = difficulty;

if ~validation.isValid
    error('Dynamic-topology scenario validation failed: %s', ...
        strjoin(hardFailures, ', '));
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end

function failures = validateDifficultyRequirements(metrics, requirements)
failures = {};
checks = { ...
    'maxBlackoutFraction', metrics.blackoutFraction, @le, ...
        'difficulty-blackout'; ...
    'maxFocusBlackoutFraction', metrics.focusBlackoutFraction, @le, ...
        'difficulty-focus-blackout'; ...
    'maxPerTargetBlackoutFraction', ...
        metrics.maximumPerTargetBlackoutFraction, @le, ...
        'difficulty-per-target-blackout'; ...
    'maxConsecutiveBlackoutSteps', ...
        metrics.maximumConsecutiveBlackoutSteps, @le, ...
        'difficulty-consecutive-blackout'; ...
    'minSingleFormationFraction', metrics.singleFormationFraction, @ge, ...
        'difficulty-single-formation'; ...
    'minMultiFormationFraction', metrics.multiFormationFraction, @ge, ...
        'difficulty-multi-formation'; ...
    'maxFocusVisibleTargetsPerSensorTime', ...
        metrics.focusMeanVisibleTargetsPerSensorTime, @le, ...
        'difficulty-sensor-target-load'; ...
    'maxFocusVisibleTargetFractionPerSensorTime', ...
        metrics.focusMeanVisibleTargetsPerSensorTime / ...
            max(config.numberOfTargets, 1), @le, ...
        'difficulty-sensor-target-load-fraction'; ...
    'minFocusHandovers', metrics.focusHandovers, @ge, ...
        'difficulty-focus-handovers'; ...
    'minCrossGroupCloseEncounterFraction', ...
        metrics.focusCloseEncounterTimeFraction, @ge, ...
        'difficulty-cross-group-encounters'; ...
    'minFormationOwnershipEntropy', ...
        metrics.formationOwnershipEntropy, @ge, ...
        'difficulty-ownership-balance'; ...
    'minBlockageFocusOverlapFraction', ...
        metrics.blockageFocusOverlapFraction, @ge, ...
        'difficulty-blockage-overlap'};
for checkIdx = 1:size(checks, 1)
    fieldName = checks{checkIdx, 1};
    if ~isfield(requirements, fieldName)
        continue;
    end
    observed = checks{checkIdx, 2};
    comparator = checks{checkIdx, 3};
    threshold = requirements.(fieldName);
    if ~comparator(observed, threshold)
        failures{end+1} = checks{checkIdx, 4}; %#ok<AGROW>
    end
end
end
end

function [maxSpeed, maxAcceleration, minSeparation, inBounds] = ...
    sensorTrajectoryMetrics(config, trajectories)
sensorCount = numel(trajectories);
timeCount = config.simulationLength;
maxSpeed = 0;
maxAcceleration = 0;
minSeparation = inf;
inBounds = true;
for sensorIdx = 1:sensorCount
    trajectory = trajectories{sensorIdx};
    speed = sqrt(sum(trajectory(3:4, :).^2, 1));
    acceleration = diff(trajectory(3:4, :), 1, 2) / ...
        config.samplingPeriod;
    maxSpeed = max(maxSpeed, max(speed));
    if ~isempty(acceleration)
        maxAcceleration = max(maxAcceleration, ...
            max(sqrt(sum(acceleration.^2, 1))));
    end
    inBounds = inBounds && pointsInBounds( ...
        trajectory(1:2, :), config.regionLimits);
end
for timeIdx = 1:timeCount
    for leftIdx = 1:sensorCount-1
        for rightIdx = leftIdx+1:sensorCount
            minSeparation = min(minSeparation, norm( ...
                trajectories{leftIdx}(1:2, timeIdx) - ...
                trajectories{rightIdx}(1:2, timeIdx)));
        end
    end
end
end

function [maxSpeed, maxAcceleration, minSeparation, inBounds, ...
    statesWellFormed] = ...
    targetTrajectoryMetrics(config, trajectories)
maxSpeed = 0;
maxAcceleration = 0;
minSeparation = inf;
inBounds = true;
statesWellFormed = true;
for targetIdx = 1:numel(trajectories)
    trajectory = trajectories{targetIdx};
    finiteByState = isfinite(trajectory);
    active = all(finiteByState, 1);
    inactive = all(isnan(trajectory), 1);
    statesWellFormed = statesWellFormed && all(active | inactive);
    if any(active)
        maxSpeed = max(maxSpeed, max(sqrt(sum( ...
            trajectory(3:4, active).^2, 1))));
        activeTimes = find(active);
        contiguous = find(diff(activeTimes) == 1);
        if ~isempty(contiguous)
            velocity = trajectory(3:4, activeTimes);
            acceleration = (velocity(:, contiguous + 1) - ...
                velocity(:, contiguous)) / config.samplingPeriod;
            maxAcceleration = max(maxAcceleration, ...
                max(sqrt(sum(acceleration.^2, 1))));
        end
        inBounds = inBounds && pointsInBounds( ...
            trajectory(1:2, active), config.regionLimits);
    end
end
for timeIdx = 1:config.simulationLength
    activeTargets = [];
    for targetIdx = 1:numel(trajectories)
        if all(isfinite(trajectories{targetIdx}(:, timeIdx)))
            activeTargets(end+1) = targetIdx; %#ok<AGROW>
        end
    end
    for leftCursor = 1:numel(activeTargets)-1
        for rightCursor = leftCursor+1:numel(activeTargets)
            left = trajectories{activeTargets(leftCursor)}(1:2, timeIdx);
            right = trajectories{activeTargets(rightCursor)}(1:2, timeIdx);
            minSeparation = min(minSeparation, norm(left - right));
        end
    end
end
end

function inBounds = pointsInBounds(points, limits)
inBounds = all(points(1, :) >= limits(1, 1) - 1e-9) && ...
    all(points(1, :) <= limits(1, 2) + 1e-9) && ...
    all(points(2, :) >= limits(2, 1) - 1e-9) && ...
    all(points(2, :) <= limits(2, 2) + 1e-9);
end

function handoverCounts = countTargetGroupHandovers( ...
    config, sensorTrajectories, targetTrajectories)
handoverCounts = zeros(1, config.targetGroupCount);
for targetGroupIdx = 1:config.targetGroupCount
    targetIdx = find(config.targetGroupIds == targetGroupIdx, 1, 'first');
    trajectory = targetTrajectories{targetIdx};
    activeTimes = find(all(isfinite(trajectory), 1));
    nearestGroups = zeros(1, numel(activeTimes));
    for localTimeIdx = 1:numel(activeTimes)
        timeIdx = activeTimes(localTimeIdx);
        groupDistances = inf(1, config.formationCount);
        for groupIdx = 1:config.formationCount
            sensors = find(config.sensorGroupIds == groupIdx);
            distances = zeros(1, numel(sensors));
            for sensorLocalIdx = 1:numel(sensors)
                distances(sensorLocalIdx) = norm( ...
                    trajectory(1:2, timeIdx) - ...
                    sensorTrajectories{sensors(sensorLocalIdx)}( ...
                        1:2, timeIdx));
            end
            groupDistances(groupIdx) = min(distances);
        end
        [~, nearestGroups(localTimeIdx)] = min(groupDistances);
    end
    handoverCounts(targetGroupIdx) = ...
        max(0, numel(unique(nearestGroups)) - 1);
end
end

function connected = isConnected(adjacency)
nodeCount = size(adjacency, 1);
if nodeCount <= 1
    connected = true;
    return;
end
visited = false(1, nodeCount);
queue = 1;
visited(1) = true;
while ~isempty(queue)
    node = queue(1);
    queue(1) = [];
    neighbors = find(adjacency(node, :) & ~visited);
    visited(neighbors) = true;
    queue = [queue, neighbors]; %#ok<AGROW>
end
connected = all(visited);
end
