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
    case {'x36', 'x36-topology'}
        config = configureX36(config, 'topology');
    case 'x36-joint'
        config = configureX36(config, 'joint');
    case {'x36-hard', 'x36-teacher'}
        config = configureX36(config, 'teacher');
    otherwise
        error('Unknown dynamic-topology scenario preset: %s', presetName);
end

config = mergeStructRecursive(config, overrides);
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
angles = 2 * pi * (0:5) / 6;
radii = [650, 430, 260, 430, 650];
rotations = deg2rad([0, 6, 18, 30, 38]);
config.sensorCenterWaypoints = cell(1, 6);
for groupIdx = 1:6
    theta = angles(groupIdx) + rotations;
    config.sensorCenterWaypoints{groupIdx} = ...
        [radii .* cos(theta); radii .* sin(theta)];
end
if any(strcmp(loadMode, {'joint', 'teacher'}))
    config.targetGroupCount = 6;
else
    config.targetGroupCount = 4;
end
config.targetsPerTargetGroup = 3;
config.targetBirthTimesByGroup = ...
    1 + 8 * (0:config.targetGroupCount-1);
config.targetDeathTimesByGroup = ...
    160 - 2 * (0:config.targetGroupCount-1);
config.targetRoutes = cell(1, config.targetGroupCount);
for groupIdx = 1:config.targetGroupCount
    theta = angles(groupIdx);
    start = 650 * [cos(theta); sin(theta)];
    finish = -start;
    normal = 70 * [-sin(theta); cos(theta)];
    config.targetRoutes{groupIdx} = [ ...
        start, 0.35 * start + normal, [0; 0], ...
        0.35 * finish - normal, finish];
end
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
if strcmp(loadMode, 'teacher')
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
end
