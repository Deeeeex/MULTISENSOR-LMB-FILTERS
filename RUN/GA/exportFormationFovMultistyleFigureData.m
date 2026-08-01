function outputPath = exportFormationFovMultistyleFigureData( ...
        outputPath, seed, snapshotTime)
% EXPORTFORMATIONFOVMULTISTYLEFIGUREDATA Exact geometry for Python figure.
%
% This is a data-only export.  It opens no graphics device and performs no
% plotting.  Python/Matplotlib is the exclusive renderer for the figure.

if nargin < 1 || isempty(outputPath)
    outputPath = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'figures', 'source', ...
        'formation_fov_multistyle_suite_seed41.json');
end
if nargin < 2 || isempty(seed)
    seed = 41;
end
if nargin < 3 || isempty(snapshotTime)
    snapshotTime = 80;
end

presets = { ...
    'x36-formation-fov-convoy', ...
    'x36-formation-fov-crossing', ...
    'x36-formation-fov-relay', ...
    'm24-formation-fov-convoy', ...
    'm24-formation-fov-crossing', ...
    'm24-formation-fov-relay'};
panelLabels = {'a', 'b', 'c', 'd', 'e', 'f'};
scenes = repmat(struct(), 1, numel(presets));

for presetIdx = 1:numel(presets)
    rng(seed);
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    if snapshotTime < 1 || snapshotTime > config.simulationLength
        error('Figure snapshotTime lies outside preset %s.', presets{presetIdx});
    end
    [sensorTrajectories, ~] = ...
        generateMultiFormationTrajectories(config);
    [targetTrajectories, ~] = ...
        generateCorridorTargetTrajectories(config);
    graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
    headings = buildSensorFovHeadingSchedule(config, sensorTrajectories);

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
    formationAdjacency = false(config.formationCount);
    for formationIdx = 1:config.formationCount
        members = find(config.sensorGroupIds == formationIdx);
        centerX(formationIdx, :) = mean(sensorX(members, :), 1);
        centerY(formationIdx, :) = mean(sensorY(members, :), 1);
        for otherIdx = formationIdx+1:config.formationCount
            otherMembers = find(config.sensorGroupIds == otherIdx);
            linked = any(any(graphData.staticAdjacency( ...
                members, otherMembers)));
            formationAdjacency(formationIdx, otherIdx) = linked;
            formationAdjacency(otherIdx, formationIdx) = linked;
        end
    end

    scene = struct();
    scene.panelLabel = panelLabels{presetIdx};
    scene.presetName = presets{presetIdx};
    scene.sceneStyle = config.sceneStyle;
    scene.informationFlowStyle = config.informationFlowStyle;
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
    scene.sensorHardwareProfile = config.sensorHardwareProfile;
    scene.sensorFovHeadingMode = config.sensorFovHeadingMode;
    scene.formationBackboneMode = config.formationBackboneMode;
    scene.formalValidationAuthorized = config.formalValidationAuthorized;
    scene.sensorGroupIds = config.sensorGroupIds;
    scene.sensorX = sensorX;
    scene.sensorY = sensorY;
    scene.sensorHeadingRad = headings(:, snapshotTime);
    scene.formationCenterX = centerX;
    scene.formationCenterY = centerY;
    scene.targetX = targetX;
    scene.targetY = targetY;
    scene.formationAdjacency = formationAdjacency;
    scene.staticAllTimePhysical = graphData.staticAllTimePhysical;
    scenes(presetIdx) = scene;
end

payload = struct();
payload.contractVersion = 'formation-fov-multistyle-figure-source-v1';
payload.seed = seed;
payload.snapshotTime = snapshotTime;
payload.rendererContract = 'python-matplotlib-only';
payload.truthOutcomeUsed = false;
payload.trackingResultUsed = false;
payload.scenes = scenes;

parentDirectory = fileparts(outputPath);
if exist(parentDirectory, 'dir') ~= 7
    mkdir(parentDirectory);
end
encoded = jsonencode(payload);
fid = fopen(outputPath, 'w');
if fid < 0
    error('Unable to open figure source output: %s', outputPath);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', encoded);
fprintf('Formation-FoV multistyle figure data: %s\n', outputPath);
end
