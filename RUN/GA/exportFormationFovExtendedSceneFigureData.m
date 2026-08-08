function outputPath = exportFormationFovExtendedSceneFigureData( ...
        outputPath, seed, snapshotTime)
% EXPORTFORMATIONFOVEXTENDEDSCENEFIGUREDATA Exact development geometry.
%
% This function only exports generated geometry and sensing metadata.
% Python/Matplotlib is the exclusive renderer for the resulting figure.

if nargin < 1 || isempty(outputPath)
    outputPath = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'figures', 'source', ...
        'formation_fov_extended_scenes_v1_seed41.json');
end
if nargin < 2 || isempty(seed)
    seed = 41;
end
if nargin < 3 || isempty(snapshotTime)
    snapshotTime = 80;
end

presets = { ...
    'm24-formation-fov-merge-split', ...
    'm24-formation-fov-curved-corridor', ...
    'x36-formation-fov-merge-split', ...
    'x36-formation-fov-curved-corridor'};
panelLabels = {'a', 'b', 'c', 'd'};
scenes = repmat(struct(), 1, numel(presets));

for presetIdx = 1:numel(presets)
    rng(seed);
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    if snapshotTime < 1 || snapshotTime > config.simulationLength
        error('Snapshot time lies outside preset %s.', presets{presetIdx});
    end
    [sensorTrajectories, sensorMetadata] = ...
        generateMultiFormationTrajectories(config);
    [targetTrajectories, targetMetadata] = ...
        generateCorridorTargetTrajectories(config);
    graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
    validation = validateDynamicTopologyScenario( ...
        config, sensorTrajectories, targetTrajectories, graphData, ...
        struct('throwOnInvalid', false));
    if ~validation.isValid
        error('Extended scene is not geometrically valid: %s.', ...
            presets{presetIdx});
    end
    headings = buildSensorFovHeadingSchedule( ...
        config, sensorTrajectories);

    sensorX = zeros(config.numberOfSensors, config.simulationLength);
    sensorY = sensorX;
    for sensorIdx = 1:config.numberOfSensors
        sensorX(sensorIdx, :) = sensorTrajectories{sensorIdx}(1, :);
        sensorY(sensorIdx, :) = sensorTrajectories{sensorIdx}(2, :);
    end
    targetX = nan(config.numberOfTargets, config.simulationLength);
    targetY = targetX;
    for targetIdx = 1:config.numberOfTargets
        targetX(targetIdx, :) = targetTrajectories{targetIdx}(1, :);
        targetY(targetIdx, :) = targetTrajectories{targetIdx}(2, :);
    end

    centerX = zeros(config.formationCount, config.simulationLength);
    centerY = centerX;
    for formationIdx = 1:config.formationCount
        members = find(config.sensorGroupIds == formationIdx);
        centerX(formationIdx, :) = mean(sensorX(members, :), 1);
        centerY(formationIdx, :) = mean(sensorY(members, :), 1);
    end
    physicalFormationAdjacency = formationPhysicalAdjacency( ...
        graphData.physicalAdjacency(:, :, snapshotTime), ...
        config.sensorGroupIds, config.formationCount);

    metrics = validation.difficulty;
    scene = struct();
    scene.panelLabel = panelLabels{presetIdx};
    scene.presetName = presets{presetIdx};
    scene.sceneStyle = config.sceneStyle;
    scene.informationFlowStyle = config.informationFlowStyle;
    scene.sceneCalibrationStatus = config.sceneCalibrationStatus;
    scene.nodeCount = config.numberOfSensors;
    scene.formationCount = config.formationCount;
    scene.sensorsPerFormation = config.sensorsPerFormation;
    scene.targetCount = config.numberOfTargets;
    scene.simulationLength = config.simulationLength;
    scene.snapshotTime = snapshotTime;
    scene.regionLimits = config.regionLimits;
    scene.fovHalfAngleDeg = config.fovHalfAngleDeg;
    scene.fovTotalAngleDeg = config.fovTotalAngleDeg;
    scene.fovRange = config.fovRange;
    scene.sensorFovHeadingMode = config.sensorFovHeadingMode;
    scene.sensorGroupIds = sensorMetadata.sensorGroupIds;
    scene.targetGroupIds = targetMetadata.targetGroupIds;
    scene.sensorX = sensorX;
    scene.sensorY = sensorY;
    scene.sensorHeadingRad = headings(:, snapshotTime);
    scene.formationCenterX = centerX;
    scene.formationCenterY = centerY;
    scene.targetX = targetX;
    scene.targetY = targetY;
    scene.physicalFormationAdjacency = physicalFormationAdjacency;
    scene.geometryMetrics = struct( ...
        'blackoutFraction', metrics.blackoutFraction, ...
        'focusBlackoutFraction', metrics.focusBlackoutFraction, ...
        'singleFormationFraction', metrics.singleFormationFraction, ...
        'multiFormationFraction', metrics.multiFormationFraction, ...
        'focusHandovers', metrics.focusHandovers, ...
        'physicalEdgeChurnRate', metrics.physicalEdgeChurnRate, ...
        'meanVisibleFormationCount', metrics.meanVisibleFormationCount, ...
        'focusMeanVisibleTargetsPerSensorTime', ...
            metrics.focusMeanVisibleTargetsPerSensorTime);
    scenes(presetIdx) = scene;
end

payload = struct();
payload.contractVersion = 'formation-fov-extended-scene-figure-source-v1';
payload.seed = seed;
payload.snapshotTime = snapshotTime;
payload.rendererContract = 'python-matplotlib-only';
payload.geometryTruthUsed = true;
payload.posteriorUsed = false;
payload.trackingResultUsed = false;
payload.scenes = scenes;

parentDirectory = fileparts(outputPath);
if exist(parentDirectory, 'dir') ~= 7
    mkdir(parentDirectory);
end
encoded = jsonencode(payload);
fileId = fopen(outputPath, 'w');
if fileId < 0
    error('Unable to open figure source output: %s.', outputPath);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '%s\n', encoded);
fprintf('Extended formation-FoV figure data: %s\n', outputPath);
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
