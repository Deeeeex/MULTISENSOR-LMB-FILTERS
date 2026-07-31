function summary = scanFormationFovSensingGeometry( ...
        presetName, seedList, fovRanges, qualityReferenceRanges, overrides)
% SCANFORMATIONFOVSENSINGGEOMETRY Fast geometry-only FoV calibration scan.
%
% This scanner reproduces the sensing-support quantities used by
% measureDynamicTopologyScenarioDifficulty without running the tracker or
% rebuilding trajectories for every range candidate. It is a development
% calibration utility, not tracking or policy-generalization evidence.

if nargin < 2 || isempty(seedList)
    seedList = [41, 43, 47, 53, 59];
end
if nargin < 3 || isempty(fovRanges)
    fovRanges = 300:10:500;
end
if nargin < 4 || isempty(qualityReferenceRanges)
    qualityReferenceRanges = 250:10:450;
end
if nargin < 5 || isempty(overrides)
    overrides = struct();
end

seedList = reshape(seedList, 1, []);
fovRanges = reshape(fovRanges, 1, []);
qualityReferenceRanges = reshape(qualityReferenceRanges, 1, []);
records = repmat(emptyRecord(), ...
    numel(fovRanges), numel(qualityReferenceRanges), numel(seedList));

for seedIdx = 1:numel(seedList)
    geometry = precomputeGeometry( ...
        presetName, seedList(seedIdx), overrides);
    for rangeIdx = 1:numel(fovRanges)
        for qualityIdx = 1:numel(qualityReferenceRanges)
            records(rangeIdx, qualityIdx, seedIdx) = evaluateGeometry( ...
                geometry, fovRanges(rangeIdx), ...
                qualityReferenceRanges(qualityIdx));
        end
    end
end

summary = struct();
summary.contractVersion = 'formation-shared-fov-geometry-scan-v1';
summary.presetName = presetName;
summary.seeds = seedList;
summary.fovRanges = fovRanges;
summary.qualityReferenceRanges = qualityReferenceRanges;
summary.overrides = overrides;
summary.records = records;
summary.aggregate = aggregateRecords(records);
summary.evidenceBoundary = [ ...
    'Geometry-only development calibration on opened seeds; these ', ...
    'statistics do not establish tracking gain or policy generalization.'];
end

function geometry = precomputeGeometry(presetName, seed, overrides)
rng(seed);
config = buildDynamicTopologyScenarioConfig(presetName, overrides);
[sensorTrajectories, ~] = generateMultiFormationTrajectories(config);
[targetTrajectories, ~] = generateCorridorTargetTrajectories(config);
headingSchedule = buildSensorFovHeadingSchedule( ...
    config, sensorTrajectories);

targetCount = config.numberOfTargets;
sensorCount = config.numberOfSensors;
timeCount = config.simulationLength;
distances = inf(targetCount, sensorCount, timeCount);
offAxisDeg = inf(targetCount, sensorCount, timeCount);
activeMask = false(targetCount, timeCount);
for timeIdx = 1:timeCount
    sensorPositions = zeros(2, sensorCount);
    for sensorIdx = 1:sensorCount
        sensorPositions(:, sensorIdx) = ...
            sensorTrajectories{sensorIdx}(1:2, timeIdx);
    end
    for targetIdx = 1:targetCount
        targetState = targetTrajectories{targetIdx}(:, timeIdx);
        if any(~isfinite(targetState))
            continue;
        end
        activeMask(targetIdx, timeIdx) = true;
        relative = bsxfun(@minus, targetState(1:2), sensorPositions);
        distances(targetIdx, :, timeIdx) = ...
            sqrt(sum(relative .^ 2, 1));
        targetBearings = atan2(relative(2, :), relative(1, :));
        angleError = atan2( ...
            sin(targetBearings - headingSchedule(:, timeIdx)'), ...
            cos(targetBearings - headingSchedule(:, timeIdx)'));
        offAxisDeg(targetIdx, :, timeIdx) = abs(rad2deg(angleError));
    end
end

geometry = struct();
geometry.config = config;
geometry.distances = distances;
geometry.offAxisDeg = offAxisDeg;
geometry.activeMask = activeMask;
end

function record = evaluateGeometry(geometry, fovRange, referenceRange)
config = geometry.config;
targetCount = config.numberOfTargets;
sensorCount = config.numberOfSensors;
timeCount = config.simulationLength;
active3 = repmat(reshape(geometry.activeMask, ...
    targetCount, 1, timeCount), 1, sensorCount, 1);
inFov = active3 & geometry.distances <= fovRange & ...
    geometry.offAxisDeg <= config.fovHalfAngleDeg + 1e-12;

visibleSensorCount = reshape(sum(inFov, 2), targetCount, timeCount);
focusWindow = config.focusWindow(1):config.focusWindow(end);
focusActive = geometry.activeMask(:, focusWindow);
focusSupport = visibleSensorCount(:, focusWindow);

quality = config.sensorQuality;
angleRatio = min(max(geometry.offAxisDeg / ...
    config.fovHalfAngleDeg, 0), 1);
rangeRatio = max(geometry.distances / max(referenceRange, eps), 0);
rangeScore = exp(-quality.detectionRangeDecay * ...
    rangeRatio .^ quality.detectionRangePower);
angleScore = 1 - quality.edgeDetectionPenalty * ...
    angleRatio .^ quality.anglePower;
detectionProbability = config.detectionProbability * ...
    rangeScore .* angleScore;
detectionProbability = min(max(detectionProbability, ...
    quality.minDetectionProbability), 1);
detectionProbability(~inFov) = 0;
expectedDetectionCount = reshape(sum( ...
    detectionProbability, 2), targetCount, timeCount);

visibleFormationCount = zeros(targetCount, timeCount);
nearestVisibleFormationDistance = inf( ...
    targetCount, config.formationCount, timeCount);
for groupIdx = 1:config.formationCount
    groupSensors = find(config.sensorGroupIds == groupIdx);
    groupVisible = any(inFov(:, groupSensors, :), 2);
    visibleFormationCount = visibleFormationCount + ...
        reshape(groupVisible, targetCount, timeCount);
    groupDistances = min( ...
        geometry.distances(:, groupSensors, :), [], 2);
    groupDistances(~groupVisible) = inf;
    nearestVisibleFormationDistance(:, groupIdx, :) = groupDistances;
end
[nearestDistance, owner] = min( ...
    nearestVisibleFormationDistance, [], 2);
nearestDistance = reshape(nearestDistance, targetCount, timeCount);
owner = reshape(owner, targetCount, timeCount);
owner(~isfinite(nearestDistance) | ~geometry.activeMask) = 0;

activeFormationCounts = visibleFormationCount(geometry.activeMask);
focusOwner = owner(:, focusWindow);
record = emptyRecord();
record.fovRange = fovRange;
record.qualityReferenceRange = referenceRange;
record.focusMeanVisibleSensorCount = ...
    mean(focusSupport(focusActive));
focusExpected = expectedDetectionCount(:, focusWindow);
record.focusMeanExpectedTargetDetectionCount = ...
    mean(focusExpected(focusActive));
focusVisible = inFov(:, :, focusWindow);
record.focusMeanVisibleTargetsPerSensorTime = ...
    sum(focusVisible(:)) / (sensorCount * numel(focusWindow));
record.blackoutFraction = mean(activeFormationCounts == 0);
record.singleFormationFraction = mean(activeFormationCounts == 1);
record.multiFormationFraction = mean(activeFormationCounts >= 2);
record.focusHandovers = countOwnerChanges(focusOwner);
ownerSamples = owner(owner > 0);
counts = zeros(1, config.formationCount);
for groupIdx = 1:config.formationCount
    counts(groupIdx) = sum(ownerSamples == groupIdx);
end
fractions = counts / max(sum(counts), 1);
positive = fractions(fractions > 0);
record.formationOwnershipEntropy = ...
    -sum(positive .* log(positive)) / log(config.formationCount);
end

function count = countOwnerChanges(owners)
count = 0;
for targetIdx = 1:size(owners, 1)
    previous = 0;
    for timeIdx = 1:size(owners, 2)
        current = owners(targetIdx, timeIdx);
        if current <= 0
            continue;
        end
        if previous > 0 && current ~= previous
            count = count + 1;
        end
        previous = current;
    end
end
end

function aggregate = aggregateRecords(records)
aggregate = struct();
fields = fieldnames(emptyRecord());
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    values = reshape([records.(fieldName)], size(records));
    aggregate.([fieldName, 'Mean']) = mean(values, 3);
    aggregate.([fieldName, 'Minimum']) = min(values, [], 3);
    aggregate.([fieldName, 'Maximum']) = max(values, [], 3);
end
end

function record = emptyRecord()
record = struct( ...
    'fovRange', NaN, ...
    'qualityReferenceRange', NaN, ...
    'focusMeanVisibleSensorCount', NaN, ...
    'focusMeanExpectedTargetDetectionCount', NaN, ...
    'focusMeanVisibleTargetsPerSensorTime', NaN, ...
    'blackoutFraction', NaN, ...
    'singleFormationFraction', NaN, ...
    'multiFormationFraction', NaN, ...
    'focusHandovers', NaN, ...
    'formationOwnershipEntropy', NaN);
end
