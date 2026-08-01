function config = buildDynamicTopologyScenarioConfig(presetName, overrides)
% BUILDDYNAMICTOPOLOGYSCENARIOCONFIG Configurable multi-formation presets.
%
%   config = buildDynamicTopologyScenarioConfig('d12-handover')
%   config = buildDynamicTopologyScenarioConfig('m24-composite', overrides)
%
% The returned struct is intentionally data-only. Experiment scripts can
% switch scene families with one preset string and override individual
% fields without editing the generators.

if nargin < 1 || isempty(presetName)
    presetName = 'd12-handover';
end
if nargin < 2 || isempty(overrides)
    overrides = struct();
end

canonicalName = lower(strrep(strrep(char(presetName), '_', '-'), ' ', '-'));
config = baseConfig(canonicalName);
switch canonicalName
    case {'r8', 'r8-legacy'}
        config = configureR8(config);
    case {'d12', 'd12-handover'}
        config = configureD12(config, 'handover');
    case {'d12-link', 'd12-outage'}
        config = configureD12(config, 'link');
    case {'d12-hard', 'd12-teacher'}
        config = configureD12(config, 'teacher');
    case {'m24', 'm24-handover'}
        config = configureM24(config, 'handover');
    case 'm24-link'
        config = configureM24(config, 'link');
    case 'm24-composite'
        config = configureM24(config, 'composite');
    case {'m24-hard', 'm24-teacher'}
        config = configureM24(config, 'teacher');
    case {'m24-formation-fov', 'm24-realistic-fov'}
        config = configureM24(config, 'formation-fov');
    case {'x36', 'x36-topology'}
        config = configureX36(config, 'topology');
    case 'x36-joint'
        config = configureX36(config, 'joint');
    case {'x36-matched', 'x36-scale-matched'}
        config = configureX36(config, 'matched');
    case {'x36-clean-scale', 'x36-scale'}
        config = configureX36(config, 'clean-scale');
    case {'x36-formation-fov', 'x36-realistic-fov'}
        config = configureX36(config, 'formation-fov');
    case {'x36-hard', 'x36-teacher'}
        config = configureX36(config, 'teacher');
    case {'x48-formation-fov', 'x48-realistic-fov'}
        config = configureX48(config, 'formation-fov');
    otherwise
        error('Unknown dynamic-topology scenario preset: %s', presetName);
end

config = mergeStructRecursive(config, overrides);
if isfield(config, 'fovTotalAngleDeg')
    config.fovTotalAngleDeg = 2 * config.fovHalfAngleDeg;
end
config.numberOfSensors = ...
    config.formationCount * config.sensorsPerFormation;
config.numberOfTargets = config.targetGroupCount * ...
    config.targetsPerTargetGroup;
config.sensorGroupIds = repelem( ...
    1:config.formationCount, config.sensorsPerFormation);
config.targetGroupIds = repelem( ...
    1:config.targetGroupCount, config.targetsPerTargetGroup);
config.presetName = canonicalName;
validateConfig(config);
end

function config = baseConfig(name)
config = struct();
config.presetName = name;
config.variant = 'handover';
config.simulationLength = 120;
config.samplingPeriod = 1;
config.regionLimits = [-650, 650; -650, 650];
config.formationCount = 3;
config.sensorsPerFormation = 4;
config.formationRadius = 25;
config.formationRadiusJitterFraction = 0.05;
config.formationRotationJitterDeg = 5;
config.formationHeadingMode = 'motion';
config.sensorWaypointTimes = [1, 120];
config.sensorCenterWaypoints = {};
config.sensorSpeedLimit = 15;
config.sensorAccelerationLimit = 2.0;
config.minimumSensorSeparation = 10;
config.targetGroupCount = 3;
config.targetsPerTargetGroup = 3;
config.targetRoutes = {};
config.targetBirthTimesByGroup = [1, 11, 21];
config.targetDeathTimesByGroup = 120 * ones(1, 3);
config.targetCrossTrackSpacing = 48;
config.targetSpeedLimit = 14;
config.normalizeTargetRouteDuration = false;
config.clutterRate = 2;
config.detectionProbability = 0.92;
config.measurementNoiseStd = 5;
config.birthProbability = 0.08;
config.birthCovarianceDiagonal = [45, 45, 8, 8].^2;
config.survivalProbability = 0.99;
config.ospaPositionCutoff = 100;
config.fovRange = 320;
config.fovHalfAngleDeg = 180;
config.sensorFovHeadingMode = 'velocity';
config.sensorFovPointingCenter = [0; 0];
config.sensorQuality = struct( ...
    'enabled', false, ...
    'referenceRange', 320, ...
    'detectionRangeDecay', 0.20, ...
    'detectionRangePower', 1.5, ...
    'edgeDetectionPenalty', 0.20, ...
    'anglePower', 2.0, ...
    'minDetectionProbability', 0.20, ...
    'rangeNoiseScale', 0.60, ...
    'edgeNoiseScale', 0.80, ...
    'minCovarianceScale', 1.0, ...
    'maxCovarianceScale', 5.0);
config.commRange = 900;
config.linkMode = 'ideal';
config.forceDelivery = true;
config.intraFormationDropProbability = 0.02;
config.interFormationDropMinimum = 0.05;
config.interFormationDropScale = 0.20;
config.blockageWindows = zeros(0, 4);
config.edgeBudget = 14;
config.maxEdgeReplacementsPerStep = 1;
config.maxNodeDegree = 4;
config.maxInterFormationDegree = 2;
config.topologyFamily = 'd12-enumerated';
config.staticTopologyMode = 'robust-geometry';
config.attemptedByteToleranceFraction = 0.02;
config.focusWindowName = 'full';
config.focusWindow = [1, 120];
config.requireStaticPhysicalAllTimes = true;
config.requireGlobalConnectivity = true;
config.useRingIntraFormationBackbone = true;
config.difficultyCloseTargetDistance = 80;
config.enforceDifficultyRequirements = false;
config.difficultyRequirements = struct();
end

function config = configureR8(config)
config.variant = 'legacy-regression';
config.simulationLength = 100;
config.regionLimits = [-600, 600; -600, 600];
config.formationCount = 2;
config.sensorsPerFormation = 4;
config.formationRadius = 28;
config.sensorWaypointTimes = [1, 100];
config.sensorCenterWaypoints = { ...
    [-240, -120; -80, -40], ...
    [ 240,  120;  80,  40]};
config.targetGroupCount = 2;
config.targetsPerTargetGroup = 5;
config.targetBirthTimesByGroup = [1, 20];
config.targetDeathTimesByGroup = [100, 100];
config.targetRoutes = { ...
    [-520, 0, 520; -120, 0, 120], ...
    [ 520, 0,-520;  140, 0,-140]};
config.fovRange = 60000;
config.fovHalfAngleDeg = 180;
config.commRange = inf;
config.edgeBudget = 16;
config.maxEdgeReplacementsPerStep = 0;
config.topologyFamily = 'r8-fixed';
config.focusWindowName = 'full';
config.focusWindow = [1, 100];
config.forceDelivery = true;
config.linkMode = 'ideal';
end

function config = configureD12(config, variant)
config.variant = variant;
config.simulationLength = 120;
config.regionLimits = [-650, 650; -650, 650];
config.formationCount = 3;
config.sensorsPerFormation = 4;
config.formationRadius = 25;
config.sensorWaypointTimes = [1, 120];
angles = deg2rad([210, 90, -30]);
centers = 360 * [cos(angles); sin(angles)];
config.sensorCenterWaypoints = cell(1, 3);
for groupIdx = 1:3
    config.sensorCenterWaypoints{groupIdx} = ...
        repmat(centers(:, groupIdx), 1, 2);
end
config.targetGroupCount = 3;
config.targetsPerTargetGroup = 3;
config.targetBirthTimesByGroup = [1, 11, 21];
config.targetDeathTimesByGroup = [120, 120, 120];
config.targetRoutes = cell(1, 3);
for groupIdx = 1:3
    nextGroup = mod(groupIdx, 3) + 1;
    config.targetRoutes{groupIdx} = [ ...
        0.92 * centers(:, groupIdx), [0; 0], ...
        0.92 * centers(:, nextGroup)];
end
config.targetCrossTrackSpacing = 42;
config.fovRange = 320;
% D12 isolates range-limited handover. Static platforms are treated as
% omnidirectional by evaluateSensorQuality.
config.fovHalfAngleDeg = 180;
config.commRange = 900;
config.edgeBudget = 14;
config.maxEdgeReplacementsPerStep = 1;
config.topologyFamily = 'd12-enumerated';
if strcmp(variant, 'link')
    config.fovRange = 900;
    config.forceDelivery = false;
    config.linkMode = 'correlated-blockage';
    % [left formation, right formation, start, stop]
    config.blockageWindows = [1, 2, 40, 60; 2, 3, 75, 95];
    config.focusWindowName = 'blockage';
    config.focusWindow = [40, 95];
elseif strcmp(variant, 'teacher')
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = [1, 9, 17];
    config.targetCrossTrackSpacing = 34;
    config.clutterRate = 3;
    config.detectionProbability = 0.90;
    config.measurementNoiseStd = 6;
    config.birthProbability = 0.07;
    config.fovRange = 340;
    config.fovHalfAngleDeg = 95;
    config.sensorFovHeadingMode = 'scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 300;
    config.forceDelivery = true;
    config.linkMode = 'ideal';
    config.focusWindowName = 'teacher-handover';
    config.focusWindow = [30, 105];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.03, ...
        'minSingleFormationFraction', 0.55, ...
        'minMultiFormationFraction', 0.10, ...
        'minFocusHandovers', 12, ...
        'minCrossGroupCloseEncounterFraction', 0.15, ...
        'minFormationOwnershipEntropy', 0.85, ...
        'minBlockageFocusOverlapFraction', 0);
else
    config.forceDelivery = true;
    config.linkMode = 'ideal';
    config.focusWindowName = 'handover';
    config.focusWindow = [35, 95];
end
end

function config = configureM24(config, variant)
config.variant = variant;
config.simulationLength = 160;
config.regionLimits = [-800, 800; -800, 800];
config.formationCount = 4;
config.sensorsPerFormation = 6;
config.formationRadius = 35;
config.formationRadiusJitterFraction = 0.10;
config.formationRotationJitterDeg = 12;
config.sensorWaypointTimes = [1, 40, 80, 120, 160];
config.sensorCenterWaypoints = { ...
    [-520,-340,-120, 260, 520; -180,-120,  80, 200, 180], ...
    [-180,-120,  80, 200, 180;  520, 340, 120,-260,-520], ...
    [ 520, 340, 120,-260,-520;  180, 120, -80,-200,-180], ...
    [ 180, 120, -80,-200,-180; -520,-340,-120, 260, 520]};
config.targetGroupCount = 4;
config.targetsPerTargetGroup = 3;
config.targetBirthTimesByGroup = [1, 11, 21, 31];
config.targetDeathTimesByGroup = [160, 158, 154, 150];
config.targetRoutes = { ...
    [-720,-260,  30, 300, 720; -140,-100, -20,  80, 140], ...
    [ 140, 100,  20, -80,-140;  720, 260, -30,-300,-720], ...
    [ 720, 260, -30,-300,-720;  140, 100,  20, -80,-140], ...
    [-140,-100, -20,  80, 140; -720,-260,  30, 300, 720]};
config.targetCrossTrackSpacing = 58;
config.normalizeTargetRouteDuration = true;
config.ospaPositionCutoff = 120;
config.fovRange = 360;
config.fovHalfAngleDeg = 75;
config.commRange = 900;
config.edgeBudget = 30;
config.maxEdgeReplacementsPerStep = 2;
config.topologyFamily = 'projected-general';
if strcmp(variant, 'link')
    config.fovRange = 1200;
    config.forceDelivery = false;
    config.linkMode = 'correlated-blockage';
    config.focusWindowName = 'blockage';
    config.focusWindow = [91, 135];
elseif strcmp(variant, 'composite')
    config.forceDelivery = false;
    config.linkMode = 'correlated-blockage';
    config.focusWindowName = 'handover-and-blockage';
    config.focusWindow = [65, 135];
elseif strcmp(variant, 'teacher')
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = [1, 9, 17, 25];
    config.targetCrossTrackSpacing = 46;
    config.clutterRate = 4;
    config.detectionProbability = 0.88;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.06;
    config.fovRange = 340;
    config.fovHalfAngleDeg = 145;
    config.sensorFovHeadingMode = 'scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 300;
    config.edgeBudget = 29;
    config.forceDelivery = false;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 70, 90; ...
        3, 4, 95, 115; ...
        1, 4, 116, 135];
    config.focusWindowName = 'teacher-handover-and-blockage';
    config.focusWindow = [55, 135];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.05, ...
        'minSingleFormationFraction', 0.25, ...
        'minMultiFormationFraction', 0.35, ...
        'minFocusHandovers', 30, ...
        'minCrossGroupCloseEncounterFraction', 0.20, ...
        'minFormationOwnershipEntropy', 0.85, ...
        'minBlockageFocusOverlapFraction', 0.50);
elseif strcmp(variant, 'formation-fov')
    % Realistic directional sensing contract: all sensors in one formation
    % share a common boresight toward the monitored region. FoV is reported
    % as a 120-degree total angle (60-degree half angle), leaving a genuine
    % rear blind sector. The 300 m hard range limit remains part of the
    % hardware contract and keeps local target load below near-global
    % visibility in the smallest scene.
    config.regionLimits = [-650, 650; -650, 650];
    config.formationHeadingMode = 'fixed';
    config.sensorCenterWaypoints = buildRadialFormationWaypoints( ...
        config.formationCount, [340, 228, 136, 228, 340], ...
        deg2rad([0, 6, 18, 30, 38]));
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = [1, 9, 17, 25];
    config.targetRoutes = buildOpposedCorridorRoutes( ...
        config.targetGroupCount, 420, 60);
    config.targetCrossTrackSpacing = 40;
    config.clutterRate = 4;
    config.detectionProbability = 0.88;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.06;
    config.ospaPositionCutoff = 150;
    config.fovRange = 300;
    config.fovHalfAngleDeg = 60;
    config.fovTotalAngleDeg = 120;
    config.sensorFovHeadingMode = ...
        'formation-shared-scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 300;
    config.sensorHardwareProfile = ...
        'formation-shared-120deg-r300-q300-v1';
    config.observationSpaceLimits = [-1050, 1050; -1050, 1050];
    config.clutterSpatialProfile = ...
        'uniform-global-box2100-c4-v1';
    config.edgeBudget = 29;
    config.forceDelivery = false;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 70, 90; ...
        3, 4, 95, 115; ...
        1, 4, 116, 135];
    config.focusWindowName = ...
        'formation-fov-handover-and-blockage';
    config.focusWindow = [55, 135];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.15, ...
        'maxFocusBlackoutFraction', 0.005, ...
        'maxPerTargetBlackoutFraction', 0.23, ...
        'maxConsecutiveBlackoutSteps', 24, ...
        'minSingleFormationFraction', 0.07, ...
        'minMultiFormationFraction', 0.76, ...
        'maxFocusVisibleTargetsPerSensorTime', 13.6, ...
        'maxFocusVisibleTargetFractionPerSensorTime', 0.85, ...
        'minFocusHandovers', 30, ...
        'minCrossGroupCloseEncounterFraction', 0.20, ...
        'minFormationOwnershipEntropy', 0.99, ...
        'minBlockageFocusOverlapFraction', 0.50);
else
    config.forceDelivery = true;
    config.linkMode = 'ideal';
    config.focusWindowName = 'handover';
    config.focusWindow = [65, 95];
end
if ~config.forceDelivery && isempty(config.blockageWindows)
    config.blockageWindows = [1, 2, 91, 110; 3, 4, 116, 135];
end
end

function config = configureX36(config, loadMode)
config.variant = loadMode;
config.simulationLength = 160;
config.regionLimits = [-900, 900; -900, 900];
config.formationCount = 6;
config.sensorsPerFormation = 6;
config.formationRadius = 35;
config.formationRadiusJitterFraction = 0.10;
config.formationRotationJitterDeg = 15;
config.formationHeadingMode = 'fixed';
config.sensorWaypointTimes = [1, 40, 80, 120, 160];
radii = [650, 430, 260, 430, 650];
rotations = deg2rad([0, 6, 18, 30, 38]);
config.sensorCenterWaypoints = buildRadialFormationWaypoints( ...
    config.formationCount, radii, rotations);
if any(strcmp(loadMode, ...
        {'joint', 'matched', 'clean-scale', 'formation-fov', 'teacher'}))
    config.targetGroupCount = 6;
else
    config.targetGroupCount = 4;
end
config.targetsPerTargetGroup = 3;
config.targetBirthTimesByGroup = ...
    1 + 8 * (0:config.targetGroupCount-1);
config.targetDeathTimesByGroup = ...
    160 - 2 * (0:config.targetGroupCount-1);
config.targetRoutes = buildOpposedCorridorRoutes( ...
    config.targetGroupCount, 650, 70, config.formationCount);
config.targetCrossTrackSpacing = 55;
config.normalizeTargetRouteDuration = true;
config.ospaPositionCutoff = 150;
config.fovRange = 390;
config.fovHalfAngleDeg = 80;
config.commRange = 900;
config.edgeBudget = 45;
config.maxEdgeReplacementsPerStep = 3;
config.topologyFamily = 'projected-general';
config.forceDelivery = false;
config.linkMode = 'distance';
config.focusWindowName = 'central-overlap';
config.focusWindow = [60, 110];
if strcmp(loadMode, 'clean-scale')
    % Clean scale transfer: preserve the X36 routes, target load and
    % communication stress while scaling M24-hard's per-sensor sensing
    % envelope with the larger region.
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = ...
        1 + 6 * (0:config.targetGroupCount-1);
    config.targetDeathTimesByGroup = ...
        160 - 2 * (0:config.targetGroupCount-1);
    config.targetCrossTrackSpacing = 44;
    config.clutterRate = 4;
    config.detectionProbability = 0.88;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.06;
    config.fovRange = 390;
    config.fovHalfAngleDeg = 145;
    config.sensorFovHeadingMode = 'scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 340;
    config.edgeBudget = 44;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 60, 80; ...
        3, 4, 86, 106; ...
        5, 6, 112, 132];
    config.focusWindowName = 'clean-scale-handover-and-blockage';
    config.focusWindow = [50, 135];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.01, ...
        'minSingleFormationFraction', 0.05, ...
        'minMultiFormationFraction', 0.85, ...
        'minFocusHandovers', 60, ...
        'minCrossGroupCloseEncounterFraction', 0.80, ...
        'minFormationOwnershipEntropy', 0.90, ...
        'minBlockageFocusOverlapFraction', 0.50);
elseif strcmp(loadMode, 'formation-fov')
    % Same-hardware scale control: only the formation geometry and target
    % traffic density are calibrated. The 120-degree FoV, sensing range and
    % complete sensor-quality profile are frozen across M24/X36/X48.
    config.targetsPerTargetGroup = 4;
    config.sensorCenterWaypoints = buildRadialFormationWaypoints( ...
        config.formationCount, [552, 365, 221, 365, 552], ...
        deg2rad([0, 6, 18, 30, 38]));
    config.targetBirthTimesByGroup = ...
        1 + 6 * (0:config.targetGroupCount-1);
    config.targetDeathTimesByGroup = ...
        160 - 2 * (0:config.targetGroupCount-1);
    config.targetRoutes = buildOpposedCorridorRoutes( ...
        config.targetGroupCount, 640, 60);
    config.targetCrossTrackSpacing = 40;
    config.clutterRate = 4;
    config.detectionProbability = 0.88;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.06;
    config.fovRange = 300;
    config.fovHalfAngleDeg = 60;
    config.fovTotalAngleDeg = 120;
    config.sensorFovHeadingMode = ...
        'formation-shared-scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 300;
    config.sensorHardwareProfile = ...
        'formation-shared-120deg-r300-q300-v1';
    config.observationSpaceLimits = [-1050, 1050; -1050, 1050];
    config.clutterSpatialProfile = ...
        'uniform-global-box2100-c4-v1';
    config.edgeBudget = 44;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 60, 80; ...
        3, 4, 86, 106; ...
        5, 6, 112, 132];
    config.focusWindowName = ...
        'formation-fov-handover-and-blockage';
    config.focusWindow = [50, 135];
    config.scaleControlReferencePreset = 'm24-formation-fov';
    config.scaleControlMatchedMetrics = { ...
        'focusMeanVisibleTargetsPerSensorTime'};
    config.scaleControlDerivedMetrics = { ...
        'focusMeanVisibleSensorCount'};
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.15, ...
        'maxFocusBlackoutFraction', 0.005, ...
        'maxPerTargetBlackoutFraction', 0.23, ...
        'maxConsecutiveBlackoutSteps', 24, ...
        'minSingleFormationFraction', 0.07, ...
        'minMultiFormationFraction', 0.77, ...
        'maxFocusVisibleTargetsPerSensorTime', 13.6, ...
        'maxFocusVisibleTargetFractionPerSensorTime', 0.85, ...
        'minFocusHandovers', 65, ...
        'minCrossGroupCloseEncounterFraction', 0.80, ...
        'minFormationOwnershipEntropy', 0.99, ...
        'minBlockageFocusOverlapFraction', 0.50);
elseif strcmp(loadMode, 'matched')
    % Aggregate-observability-matched diagnostic. Approximate M24-hard's
    % single-/multi-formation visibility fractions while increasing
    % formation, sensor and target counts; clean-scale below is the
    % per-sensor-envelope transfer.
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = ...
        1 + 6 * (0:config.targetGroupCount-1);
    config.targetDeathTimesByGroup = ...
        160 - 2 * (0:config.targetGroupCount-1);
    config.targetCrossTrackSpacing = 44;
    config.clutterRate = 4;
    config.detectionProbability = 0.88;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.06;
    config.fovRange = 300;
    config.fovHalfAngleDeg = 75;
    config.sensorFovHeadingMode = 'scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 300;
    config.edgeBudget = 44;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 60, 80; ...
        3, 4, 86, 106; ...
        5, 6, 112, 132];
    config.focusWindowName = 'matched-scale-handover-and-blockage';
    config.focusWindow = [50, 135];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.02, ...
        'minSingleFormationFraction', 0.25, ...
        'minMultiFormationFraction', 0.60, ...
        'minFocusHandovers', 50, ...
        'minCrossGroupCloseEncounterFraction', 0.30, ...
        'minFormationOwnershipEntropy', 0.90, ...
        'minBlockageFocusOverlapFraction', 0.50);
elseif strcmp(loadMode, 'teacher')
    config.targetsPerTargetGroup = 4;
    config.targetBirthTimesByGroup = ...
        1 + 6 * (0:config.targetGroupCount-1);
    config.targetDeathTimesByGroup = ...
        160 - 2 * (0:config.targetGroupCount-1);
    config.targetCrossTrackSpacing = 44;
    config.clutterRate = 5;
    config.detectionProbability = 0.86;
    config.measurementNoiseStd = 7;
    config.birthProbability = 0.05;
    config.fovRange = 280;
    config.fovHalfAngleDeg = 60;
    config.sensorFovHeadingMode = 'scene-center';
    config.sensorQuality.enabled = true;
    config.sensorQuality.referenceRange = 250;
    config.edgeBudget = 44;
    config.linkMode = 'correlated-blockage';
    config.blockageWindows = [ ...
        1, 2, 60, 80; ...
        3, 4, 86, 106; ...
        5, 6, 112, 132];
    config.focusWindowName = 'teacher-scale-pressure';
    config.focusWindow = [50, 135];
    config.enforceDifficultyRequirements = true;
    config.difficultyRequirements = struct( ...
        'maxBlackoutFraction', 0.05, ...
        'minSingleFormationFraction', 0.25, ...
        'minMultiFormationFraction', 0.35, ...
        'minFocusHandovers', 50, ...
        'minCrossGroupCloseEncounterFraction', 0.30, ...
        'minFormationOwnershipEntropy', 0.90, ...
        'minBlockageFocusOverlapFraction', 0.50);
end
end

function config = configureX48(config, variant)
% Eight formations retain the six-sensor local geometry while increasing
% both network scale and the number of independently moving target groups.
% Target kinematics remain on the X36 scale so network growth is not
% confounded with a relaxed target-speed contract.
config.variant = variant;
config.simulationLength = 160;
config.regionLimits = [-1050, 1050; -1050, 1050];
config.formationCount = 8;
config.sensorsPerFormation = 6;
config.formationRadius = 35;
config.formationRadiusJitterFraction = 0.10;
config.formationRotationJitterDeg = 15;
config.formationHeadingMode = 'fixed';
config.sensorWaypointTimes = [1, 40, 80, 120, 160];
config.sensorCenterWaypoints = buildRadialFormationWaypoints( ...
    config.formationCount, [611, 419, 258, 419, 611], ...
    deg2rad([0, 6, 18, 30, 38]));
config.targetGroupCount = 8;
config.targetsPerTargetGroup = 4;
config.targetBirthTimesByGroup = ...
    1 + 5 * (0:config.targetGroupCount-1);
config.targetDeathTimesByGroup = ...
    160 - 2 * (0:config.targetGroupCount-1);
% Keep the expanded X48 corridor inside the same 14 m/s target-speed
% contract by spacing its intermediate route points more evenly.
config.targetRoutes = buildOpposedCorridorRoutes( ...
    config.targetGroupCount, 730, 60, config.formationCount, 0.39);
config.targetCrossTrackSpacing = 40;
config.normalizeTargetRouteDuration = true;
config.ospaPositionCutoff = 150;
config.clutterRate = 4;
config.detectionProbability = 0.88;
config.measurementNoiseStd = 7;
config.birthProbability = 0.06;
% Hardware parameters are identical to the M24/X36 formation-FoV scenes;
% only spatial geometry and target traffic are scale-calibrated.
config.fovRange = 300;
config.fovHalfAngleDeg = 60;
config.fovTotalAngleDeg = 120;
config.sensorFovHeadingMode = 'formation-shared-scene-center';
config.sensorQuality.enabled = true;
config.sensorQuality.referenceRange = 300;
config.sensorHardwareProfile = ...
    'formation-shared-120deg-r300-q300-v1';
config.observationSpaceLimits = [-1050, 1050; -1050, 1050];
config.clutterSpatialProfile = 'uniform-global-box2100-c4-v1';
config.commRange = 900;
config.edgeBudget = 59;
config.maxEdgeReplacementsPerStep = 4;
config.topologyFamily = 'projected-general';
config.forceDelivery = false;
config.linkMode = 'correlated-blockage';
config.blockageWindows = [ ...
    1, 2, 58, 78; ...
    3, 4, 82, 102; ...
    5, 6, 106, 126; ...
    7, 8, 128, 148];
config.focusWindowName = 'formation-fov-handover-and-blockage';
config.focusWindow = [50, 135];
config.scaleControlReferencePreset = 'm24-formation-fov';
config.scaleControlMatchedMetrics = { ...
    'focusMeanVisibleTargetsPerSensorTime'};
config.scaleControlDerivedMetrics = { ...
    'focusMeanVisibleSensorCount'};
config.enforceDifficultyRequirements = true;
config.difficultyRequirements = struct( ...
    'maxBlackoutFraction', 0.16, ...
    'maxFocusBlackoutFraction', 0.005, ...
    'maxPerTargetBlackoutFraction', 0.25, ...
    'maxConsecutiveBlackoutSteps', 26, ...
    'minSingleFormationFraction', 0.06, ...
    'minMultiFormationFraction', 0.77, ...
    'maxFocusVisibleTargetsPerSensorTime', 13.6, ...
    'maxFocusVisibleTargetFractionPerSensorTime', 0.85, ...
    'minFocusHandovers', 120, ...
    'minCrossGroupCloseEncounterFraction', 0.80, ...
    'minFormationOwnershipEntropy', 0.99, ...
    'minBlockageFocusOverlapFraction', 0.70);
end

function waypoints = buildRadialFormationWaypoints( ...
        formationCount, radii, rotations)
if numel(radii) ~= numel(rotations)
    error('Radial formation radii and rotations must have equal length.');
end
baseAngles = 2 * pi * (0:formationCount-1) / formationCount;
waypoints = cell(1, formationCount);
for groupIdx = 1:formationCount
    theta = baseAngles(groupIdx) + rotations;
    waypoints{groupIdx} = [radii .* cos(theta); radii .* sin(theta)];
end
end

function routes = buildOpposedCorridorRoutes( ...
        targetGroupCount, startRadius, normalOffset, angularGroupCount, ...
        intermediateFraction)
if nargin < 4 || isempty(angularGroupCount)
    angularGroupCount = targetGroupCount;
end
if nargin < 5 || isempty(intermediateFraction)
    intermediateFraction = 0.35;
end
if angularGroupCount < targetGroupCount
    error('angularGroupCount must not be smaller than targetGroupCount.');
end
if intermediateFraction <= 0 || intermediateFraction >= 1
    error('intermediateFraction must lie strictly between zero and one.');
end
baseAngles = 2 * pi * (0:angularGroupCount-1) / angularGroupCount;
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    theta = baseAngles(groupIdx);
    start = startRadius * [cos(theta); sin(theta)];
    finish = -start;
    normal = normalOffset * [-sin(theta); cos(theta)];
    routes{groupIdx} = [ ...
        start, intermediateFraction * start + normal, [0; 0], ...
        intermediateFraction * finish - normal, finish];
end
end

function merged = mergeStructRecursive(base, overrides)
merged = base;
fields = fieldnames(overrides);
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    overrideValue = overrides.(fieldName);
    if isfield(merged, fieldName) && isstruct(merged.(fieldName)) && ...
            isstruct(overrideValue)
        merged.(fieldName) = mergeStructRecursive( ...
            merged.(fieldName), overrideValue);
    else
        merged.(fieldName) = overrideValue;
    end
end
end

function validateConfig(config)
if config.formationCount < 2 || config.sensorsPerFormation < 2
    error('A dynamic-topology scene needs at least two nontrivial formations.');
end
if numel(config.sensorCenterWaypoints) ~= config.formationCount
    error('sensorCenterWaypoints must have one entry per formation.');
end
if numel(config.targetRoutes) ~= config.targetGroupCount
    error('targetRoutes must have one entry per target group.');
end
if numel(config.targetBirthTimesByGroup) ~= config.targetGroupCount || ...
        numel(config.targetDeathTimesByGroup) ~= config.targetGroupCount
    error('Target birth/death schedules must match targetGroupCount.');
end
if config.edgeBudget < config.formationCount - 1
    error('edgeBudget is too small to connect the formation-level graph.');
end
if size(config.regionLimits, 1) ~= 2 || ...
        size(config.regionLimits, 2) ~= 2 || ...
        any(config.regionLimits(:, 1) >= config.regionLimits(:, 2))
    error('regionLimits must be [xmin xmax; ymin ymax].');
end
if isfield(config, 'observationSpaceLimits') && ...
        ~isempty(config.observationSpaceLimits)
    limits = config.observationSpaceLimits;
    if size(limits, 1) ~= 2 || size(limits, 2) ~= 2 || ...
            any(~isfinite(limits(:))) || ...
            any(limits(:, 1) >= limits(:, 2))
        error(['observationSpaceLimits must be ', ...
            '[xmin xmax; ymin ymax].']);
    end
    if any(limits(:, 1) > config.regionLimits(:, 1)) || ...
            any(limits(:, 2) < config.regionLimits(:, 2))
        error('observationSpaceLimits must enclose regionLimits.');
    end
end
end
