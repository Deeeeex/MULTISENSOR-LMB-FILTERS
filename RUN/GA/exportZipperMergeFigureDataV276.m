function outputPath = exportZipperMergeFigureDataV276(outputPath, seed)
% EXPORTZIPPERMERGEFIGUREDATAV276 Exact geometry for the V276 scene figure.
%
% This function exports generated geometry and physical-graph metadata only.
% Python/Matplotlib is the exclusive renderer for the resulting figure.

if nargin < 1 || isempty(outputPath)
    outputPath = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'figures', 'source', ...
        'zipper_merge_v276_seed1301.json');
end
if nargin < 2 || isempty(seed)
    seed = 1301;
end

presetNames = { ...
    'm24-formation-fov-zipper-merge', ...
    'x36-formation-fov-zipper-merge'};
snapshotTimes = [1, 80, 160];
scenes = repmat(struct(), 1, numel(presetNames));

for presetIdx = 1:numel(presetNames)
    rng(seed, 'twister');
    config = buildDynamicTopologyScenarioConfig(presetNames{presetIdx});
    [sensorTrajectories, sensorMetadata] = ...
        generateMultiFormationTrajectories(config);
    [targetTrajectories, targetMetadata] = ...
        generateCorridorTargetTrajectories(config);
    graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
    validation = validateDynamicTopologyScenario( ...
        config, sensorTrajectories, targetTrajectories, graphData, ...
        struct('throwOnInvalid', false));
    if ~validation.isValid
        error('V276 figure source is not geometrically valid: %s.', ...
            presetNames{presetIdx});
    end

    timeCount = config.simulationLength;
    sensorX = trajectoryComponent( ...
        sensorTrajectories, 1, timeCount);
    sensorY = trajectoryComponent( ...
        sensorTrajectories, 2, timeCount);
    targetX = trajectoryComponent( ...
        targetTrajectories, 1, timeCount);
    targetY = trajectoryComponent( ...
        targetTrajectories, 2, timeCount);
    formationCenterX = groupedMean( ...
        sensorX, config.sensorGroupIds, config.formationCount);
    formationCenterY = groupedMean( ...
        sensorY, config.sensorGroupIds, config.formationCount);
    headings = buildSensorFovHeadingSchedule( ...
        config, sensorTrajectories);

    initialTree = formationAdjacency( ...
        graphData.staticAdjacency, config.sensorGroupIds, ...
        config.formationCount);
    physicalByTime = false( ...
        config.formationCount, config.formationCount, timeCount);
    treeFeasible = false(1, timeCount);
    alternativeEdgeCount = zeros(1, timeCount);
    formationConnected = false(1, timeCount);
    for currentTime = 1:timeCount
        physicalByTime(:, :, currentTime) = formationAdjacency( ...
            graphData.physicalAdjacency(:, :, currentTime), ...
            config.sensorGroupIds, config.formationCount);
        currentPhysical = physicalByTime(:, :, currentTime);
        treeFeasible(currentTime) = ...
            ~any(initialTree(:) & ~currentPhysical(:));
        alternativeEdgeCount(currentTime) = nnz(triu( ...
            currentPhysical & ~initialTree, 1));
        formationConnected(currentTime) = isConnected(currentPhysical);
    end
    [failureStarts, failureStops] = failureEpisodes(~treeFeasible);
    if numel(failureStarts) ~= 1 || ~all(formationConnected)
        error(['V276 figure requires one fixed-tree failure episode ', ...
            'and an always-connected physical formation graph.']);
    end

    snapshotPhysical = false( ...
        config.formationCount, config.formationCount, ...
        numel(snapshotTimes));
    for snapshotIdx = 1:numel(snapshotTimes)
        snapshotPhysical(:, :, snapshotIdx) = ...
            physicalByTime(:, :, snapshotTimes(snapshotIdx));
    end

    scene = struct();
    scene.presetName = presetNames{presetIdx};
    scene.sceneStyle = config.sceneStyle;
    scene.sceneCalibrationStatus = config.sceneCalibrationStatus;
    scene.nodeCount = config.numberOfSensors;
    scene.formationCount = config.formationCount;
    scene.sensorsPerFormation = config.sensorsPerFormation;
    scene.targetCount = config.numberOfTargets;
    scene.targetGroupCount = config.targetGroupCount;
    scene.simulationLength = timeCount;
    scene.snapshotTimes = snapshotTimes;
    scene.regionLimits = config.regionLimits;
    scene.fovHalfAngleDeg = config.fovHalfAngleDeg;
    scene.fovTotalAngleDeg = config.fovTotalAngleDeg;
    scene.fovRange = config.fovRange;
    scene.commRange = config.commRange;
    scene.sensorFovHeadingMode = config.sensorFovHeadingMode;
    scene.sensorGroupIds = sensorMetadata.sensorGroupIds;
    scene.targetGroupIds = targetMetadata.targetGroupIds;
    scene.sensorX = sensorX;
    scene.sensorY = sensorY;
    scene.targetX = targetX;
    scene.targetY = targetY;
    scene.formationCenterX = formationCenterX;
    scene.formationCenterY = formationCenterY;
    scene.sensorHeadingRadByTime = headings;
    scene.initialTreeFormationAdjacency = initialTree;
    scene.physicalFormationAdjacencyAtSnapshots = snapshotPhysical;
    scene.initialTreeFeasibleByTime = treeFeasible;
    scene.alternativeFormationEdgeCountByTime = alternativeEdgeCount;
    scene.formationConnectedByTime = formationConnected;
    scene.failureStartTime = failureStarts(1);
    scene.failureStopTime = failureStops(1);
    scene.plannedHandoverTimes = unique(round(1 + ...
        (timeCount - 1) * config.targetRouteHandoverFractions));
    scene.minimumSensorTargetSeparation = ...
        validation.minimumSensorTargetSeparation;
    scene.blackoutFraction = validation.difficulty.blackoutFraction;
    scene.focusBlackoutFraction = ...
        validation.difficulty.focusBlackoutFraction;
    scene.focusHandoverCount = validation.difficulty.focusHandovers;
    scenes(presetIdx) = scene;
end

payload = struct();
payload.contractVersion = 'zipper-merge-v276-figure-source-v1';
payload.seed = seed;
payload.rendererContract = 'python-matplotlib-only';
payload.geometryTruthUsed = true;
payload.physicalGraphTruthUsed = true;
payload.posteriorUsed = false;
payload.trackingResultUsed = false;
payload.snapshotTimes = snapshotTimes;
payload.snapshotPhaseNames = {'separated', 'zipper bottleneck', 'split'};
payload.scenes = scenes;

parentDirectory = fileparts(outputPath);
if exist(parentDirectory, 'dir') ~= 7
    mkdir(parentDirectory);
end
fileId = fopen(outputPath, 'w');
if fileId < 0
    error('Unable to open V276 figure source: %s.', outputPath);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '%s\n', jsonencode(payload));
fprintf('V276 zipper-merge figure data: %s\n', outputPath);
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

function adjacency = formationAdjacency( ...
        sensorAdjacency, groupIds, formationCount)
adjacency = false(formationCount);
for leftIdx = 1:formationCount-1
    leftMembers = find(groupIds == leftIdx);
    for rightIdx = leftIdx+1:formationCount
        rightMembers = find(groupIds == rightIdx);
        block = logical(sensorAdjacency(leftMembers, rightMembers));
        adjacency(leftIdx, rightIdx) = any(block(:));
        adjacency(rightIdx, leftIdx) = adjacency(leftIdx, rightIdx);
    end
end
end

function connected = isConnected(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    current = frontier(end);
    frontier(end) = [];
    if visited(current)
        continue;
    end
    visited(current) = true;
    frontier = [frontier, find(adjacency(current, :) & ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end

function [starts, stops] = failureEpisodes(failed)
starts = find(failed & [true, ~failed(1:end-1)]);
stops = find(failed & [~failed(2:end), true]);
end
