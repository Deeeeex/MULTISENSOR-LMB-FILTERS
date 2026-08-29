function outputPath = exportTargetGroupOverlapSplitFigureData( ...
        outputPath, seed, presetName)
% EXPORTTARGETGROUPOVERLAPSPLITFIGUREDATA Exact three-phase geometry.

if nargin < 1 || isempty(outputPath)
    outputPath = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'figures', 'source', ...
        'target_group_overlap_split_m24_seed41.json');
end
if nargin < 2 || isempty(seed)
    seed = 41;
end
if nargin < 3 || isempty(presetName)
    presetName = 'm24-formation-fov-target-overlap';
end

rng(seed);
config = buildDynamicTopologyScenarioConfig(presetName);
if ~strcmp(config.sceneStyle, 'target-group-overlap-split') || ...
        ~strcmp(config.sceneCalibrationStatus, 'development-only')
    error('The figure exporter requires the development target-overlap scene.');
end
[sensorTrajectories, sensorMetadata] = ...
    generateMultiFormationTrajectories(config);
[targetTrajectories, targetMetadata] = ...
    generateCorridorTargetTrajectories(config);
graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
validation = validateDynamicTopologyScenario( ...
    config, sensorTrajectories, targetTrajectories, graphData);
headings = buildSensorFovHeadingSchedule(config, sensorTrajectories);

sensorX = trajectoryComponent(sensorTrajectories, 1, ...
    config.simulationLength);
sensorY = trajectoryComponent(sensorTrajectories, 2, ...
    config.simulationLength);
targetX = trajectoryComponent(targetTrajectories, 1, ...
    config.simulationLength);
targetY = trajectoryComponent(targetTrajectories, 2, ...
    config.simulationLength);
formationCenterX = groupedMean( ...
    sensorX, config.sensorGroupIds, config.formationCount);
formationCenterY = groupedMean( ...
    sensorY, config.sensorGroupIds, config.formationCount);

snapshotTimes = [1, 80, 160];
physicalFormationAdjacency = false( ...
    config.formationCount, config.formationCount, numel(snapshotTimes));
for snapshotIdx = 1:numel(snapshotTimes)
    physicalFormationAdjacency(:, :, snapshotIdx) = ...
        formationPhysicalAdjacency( ...
            graphData.physicalAdjacency(:, :, snapshotTimes(snapshotIdx)), ...
            config.sensorGroupIds, config.formationCount);
end

payload = struct();
payload.contractVersion = ...
    'target-group-overlap-split-figure-source-v1';
payload.rendererContract = 'python-matplotlib-svg-v1';
payload.seed = seed;
payload.presetName = presetName;
payload.sceneStyle = config.sceneStyle;
payload.sceneCalibrationStatus = config.sceneCalibrationStatus;
payload.geometryTruthUsed = true;
payload.posteriorUsed = false;
payload.trackingResultUsed = false;
payload.snapshotTimes = snapshotTimes;
payload.phaseNames = {'separated', 'overlap', 're-separated'};
payload.regionLimits = config.regionLimits;
payload.nodeCount = config.numberOfSensors;
payload.formationCount = config.formationCount;
payload.sensorsPerFormation = config.sensorsPerFormation;
payload.targetCount = config.numberOfTargets;
payload.targetGroupCount = config.targetGroupCount;
payload.sensorGroupIds = sensorMetadata.sensorGroupIds;
payload.targetGroupIds = targetMetadata.targetGroupIds;
payload.sensorX = sensorX;
payload.sensorY = sensorY;
payload.targetX = targetX;
payload.targetY = targetY;
payload.formationCenterX = formationCenterX;
payload.formationCenterY = formationCenterY;
payload.sensorHeadingRadByTime = headings;
payload.physicalFormationAdjacency = physicalFormationAdjacency;
payload.fovHalfAngleDeg = config.fovHalfAngleDeg;
payload.fovTotalAngleDeg = config.fovTotalAngleDeg;
payload.fovRange = config.fovRange;
payload.minimumTargetSeparation = ...
    validation.minimumTargetSeparation;
payload.minimumSensorTargetSeparation = ...
    validation.minimumSensorTargetSeparation;

parentDirectory = fileparts(outputPath);
if exist(parentDirectory, 'dir') ~= 7
    mkdir(parentDirectory);
end
fileId = fopen(outputPath, 'w');
if fileId < 0
    error('Unable to open figure source output: %s.', outputPath);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '%s\n', jsonencode(payload));
fprintf('Target-group overlap/split figure data: %s\n', outputPath);
end

function values = trajectoryComponent(trajectories, rowIdx, timeCount)
values = nan(numel(trajectories), timeCount);
for trajectoryIdx = 1:numel(trajectories)
    values(trajectoryIdx, :) = trajectories{trajectoryIdx}(rowIdx, :);
end
end

function values = groupedMean(source, groupIds, groupCount)
values = zeros(groupCount, size(source, 2));
for groupIdx = 1:groupCount
    values(groupIdx, :) = mean(source(groupIds == groupIdx, :), 1);
end
end

function adjacency = formationPhysicalAdjacency( ...
        sensorAdjacency, groupIds, formationCount)
adjacency = false(formationCount);
for leftIdx = 1:formationCount-1
    leftMembers = find(groupIds == leftIdx);
    for rightIdx = leftIdx+1:formationCount
        rightMembers = find(groupIds == rightIdx);
        linked = any(any(sensorAdjacency(leftMembers, rightMembers)));
        adjacency(leftIdx, rightIdx) = linked;
        adjacency(rightIdx, leftIdx) = linked;
    end
end
end
