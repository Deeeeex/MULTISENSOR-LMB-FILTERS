function config = applyFormationFovSceneStyle(config, styleName)
% APPLYFORMATIONFOVSCENESTYLE Geometry overlays for formation-FoV scenes.
%
%   config = applyFormationFovSceneStyle(config, 'convoy')
%
% The overlay changes platform/target motion and the blockage pattern while
% preserving the registered sensor envelope (120 degree total FoV, 300 m
% range, detection/noise/quality profile and clutter field).  New styles are
% deliberately marked calibration-only until their observability and graph
% difficulty gates have been frozen on unopened seeds.

if nargin < 2 || isempty(styleName)
    error('A formation-FoV scene style is required.');
end
requiredFields = { ...
    'formationCount', 'targetGroupCount', 'simulationLength', ...
    'fovTotalAngleDeg', 'fovRange', 'sensorHardwareProfile'};
if ~isstruct(config) || ~all(isfield(config, requiredFields))
    error('Formation-FoV scene style requires a complete base config.');
end
if abs(config.fovTotalAngleDeg - 120) > 1e-12 || ...
        abs(config.fovRange - 300) > 1e-12
    error('Scene styles may not change the frozen 120-degree/300-m envelope.');
end

styleName = lower(strrep(strrep(char(styleName), '_', '-'), ' ', '-'));
config.sceneStyle = styleName;
config.sceneGeometryVersion = 'formation-fov-multistyle-v5';
config.sceneCalibrationStatus = ...
    'stress-only-v5';
config.formalValidationAuthorized = false;
config.trackingOutcomeAuthorized = false;
config.enforceDifficultyRequirements = false;
config.difficultyRequirements = struct();
config.scaleControlReferencePreset = '';
config.scaleControlMatchedMetrics = {};
config.scaleControlDerivedMetrics = {};
config.sensorWaypointTimes = [1, 40, 80, 120, 160];
config.normalizeTargetRouteDuration = true;
config.focusWindow = [45, 140];
config.forceDelivery = false;
config.linkMode = 'correlated-blockage';
% The reference tree is chosen from geometry observable at t=1 only.  It
% must not inspect the complete planned trajectory to select an edge that
% is known in advance to remain available throughout the episode.
config.formationBackboneMode = 'initial-geometry-mst';
config.requireStaticPhysicalAllTimes = true;
config.blockageScheduleMode = 'backbone-sequential';
config.targetAccelerationLimit = 1.0;
config.minimumSensorTargetSeparation = 0;

switch styleName
    case {'convoy', 'parallel-convoy'}
        config.sceneStyle = 'parallel-convoy';
        config.informationFlowStyle = ...
            'co-moving-offset-corridor-and-overtake';
        config.regionLimits = [-820, 820; -520, 520];
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-fixed';
        config.sensorFovFixedHeadingRadByFormation = ...
            deg2rad(30) * ones(1, config.formationCount);
        config.sensorCenterWaypoints = buildConvoyFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildConvoyTargetRoutes( ...
            config.targetGroupCount);
        config.targetCrossTrackSpacing = 20;
        config.minimumTargetSeparation = 14;
        config.minimumSensorTargetSeparation = 30;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'convoy');
        config.focusWindowName = 'convoy-overtake-and-link-blockage';
        config.sceneCalibrationStatus = ...
            'held-out-geometry-gate-frozen-v5';
        config.formalValidationAuthorized = true;
        config.enforceDifficultyRequirements = true;
        config.difficultyRequirements = ...
            getFormationFovMultistyleAbsoluteDifficultyRequirements(config);
    case {'crossing', 'orthogonal-crossing'}
        config.sceneStyle = 'orthogonal-crossing';
        config.informationFlowStyle = ...
            'multi-direction-intersection-and-transient-overlap';
        config.regionLimits = [-760, 760; -760, 760];
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = buildCrossingFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildCrossingTargetRoutes( ...
            config.targetGroupCount);
        config.targetCrossTrackSpacing = 32;
        config.minimumTargetSeparation = 2;
        config.requireStaticPhysicalAllTimes = false;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'crossing');
        config.focusWindowName = ...
            'orthogonal-intersection-and-link-blockage';
    case {'relay', 'linear-relay', 'corridor-relay'}
        config.sceneStyle = 'linear-relay';
        config.informationFlowStyle = ...
            'sequential-handover-along-a-sensor-chain';
        if config.formationCount <= 4
            config.regionLimits = [-720, 720; -430, 520];
        else
            config.regionLimits = [-980, 980; -430, 520];
        end
        config.formationHeadingMode = 'fixed';
        config.sensorFovHeadingMode = 'formation-shared-fixed';
        config.sensorFovFixedHeadingRadByFormation = ...
            -(pi / 2) * ones(1, config.formationCount);
        config.sensorCenterWaypoints = buildRelayFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildRelayTargetRoutes( ...
            config.targetGroupCount);
        config.targetCrossTrackSpacing = 28;
        config.minimumTargetSeparation = 9;
        config.minimumSensorTargetSeparation = 30;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'relay');
        config.focusWindowName = 'corridor-relay-and-link-blockage';
        config.sceneCalibrationStatus = ...
            'held-out-geometry-gate-frozen-v5';
        config.formalValidationAuthorized = true;
        config.enforceDifficultyRequirements = true;
        config.difficultyRequirements = ...
            getFormationFovMultistyleAbsoluteDifficultyRequirements(config);
    case {'merge-split', 'converge-diverge'}
        config.sceneStyle = 'merge-split';
        config.informationFlowStyle = ...
            'parallel-branches-merge-into-dense-corridor-then-split';
        config.regionLimits = [-950, 1050; -650, 650];
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = ...
            buildMergeSplitFormationWaypoints(config.formationCount);
        config.targetRoutes = ...
            buildMergeSplitTargetRoutes(config.targetGroupCount);
        % Keep every target stream phase-aligned with the moving formation
        % branches.  Inheriting the radial preset's staggered births made a
        % late stream appear almost inside a formation that had already
        % advanced along the corridor, creating sub-metre sensor--target
        % encounters unrelated to the intended information-flow problem.
        config.targetBirthTimesByGroup = ...
            ones(1, config.targetGroupCount);
        config.targetDeathTimesByGroup = ...
            config.simulationLength * ones(1, config.targetGroupCount);
        config.targetCrossTrackSpacing = 18;
        config.minimumTargetSeparation = 6;
        config.minimumSensorTargetSeparation = 30;
        config.targetAccelerationLimit = 2.0;
        config.requireStaticPhysicalAllTimes = false;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'merge-split');
        config.focusWindowName = 'formation-merge-and-branch-split';
        config = markExploratoryStyle(config);
    case {'curved-corridor', 'turning-corridor'}
        config.sceneStyle = 'curved-corridor';
        config.informationFlowStyle = ...
            'co-oriented-formations-follow-a-sustained-turn';
        config.regionLimits = [-900, 900; -950, 1050];
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = ...
            buildCurvedFormationWaypoints(config.formationCount);
        config.targetRoutes = ...
            buildCurvedTargetRoutes(config.targetGroupCount);
        config.targetCrossTrackSpacing = 26;
        config.minimumTargetSeparation = 8;
        config.minimumSensorTargetSeparation = 0;
        config.requireStaticPhysicalAllTimes = false;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'curved-corridor');
        config.focusWindowName = 'sustained-turn-and-fov-reorientation';
        config = markExploratoryStyle(config);
    otherwise
        error('Unknown formation-FoV scene style: %s', styleName);
end
end

function config = markExploratoryStyle(config)
% Keep new styles switchable while they remain development-only.
config.sceneCalibrationStatus = 'development-only';
config.formalValidationAuthorized = false;
config.trackingOutcomeAuthorized = false;
config.enforceDifficultyRequirements = false;
config.difficultyRequirements = struct();
config.scaleControlReferencePreset = '';
config.scaleControlMatchedMetrics = {};
config.scaleControlDerivedMetrics = {};
if isfield(config, 'sceneGeometryVersion')
    config = rmfield(config, 'sceneGeometryVersion');
end
if isfield(config, 'sceneContractSha256')
    config = rmfield(config, 'sceneContractSha256');
end
end

function waypoints = buildConvoyFormationWaypoints(formationCount)
if mod(formationCount, 2) ~= 0
    error('Convoy geometry requires two equally sized columns.');
end
laneCount = formationCount / 2;
% Sensor and target service corridors are interleaved rather than
% coincident.  The two lane grids share a 220 m pitch and are shifted by
% -55/+55 m, giving a 110 m cross-set offset while keeping the complete
% layout centred.  Adding a lane pair therefore preserves local geometry.
lanes = centeredValues(laneCount, 220) - 55;
% A 300 m front/rear-column spacing retains overlapping handoff support
% under the fixed 300 m sensing range without placing both columns on top
% of the same target platoon.
longitudinalColumns = [-150, 150];
progress = [-170, -85, 0, 95, 210];
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    laneIdx = mod(formationIdx - 1, numel(lanes)) + 1;
    columnIdx = floor((formationIdx - 1) / numel(lanes)) + 1;
    if columnIdx > numel(longitudinalColumns)
        error('Convoy geometry requires at most two formation columns.');
    end
    waypoints{formationIdx} = [ ...
        longitudinalColumns(columnIdx) + progress; ...
        lanes(laneIdx) * ones(1, numel(progress))];
end
end

function routes = buildConvoyTargetRoutes(targetGroupCount)
if mod(targetGroupCount, 2) ~= 0
    error('Convoy geometry requires two target cohorts per lane.');
end
laneCount = targetGroupCount / 2;
lanes = centeredValues(laneCount, 220) + 55;
% Targets travel in neighbouring monitored corridors rather than through
% the planar sensor rings.  The two same-lane route templates receive an
% 80 m longitudinal anchor offset; their synchronized runtime spacing also
% depends on the registered birth/death normalization and is safety-gated
% from generated trajectories.  A small lateral bend still creates a
% genuine overtake/handover without manufacturing a platform collision.
x = [-320, -120, 120, 380, 650];
routes = cell(1, targetGroupCount);
cohortOffsets = centeredValues( ...
    ceil(targetGroupCount / numel(lanes)), 80);
for groupIdx = 1:targetGroupCount
    startLaneIdx = mod(groupIdx - 1, numel(lanes)) + 1;
    cohortIdx = floor((groupIdx - 1) / numel(lanes)) + 1;
    y0 = lanes(startLaneIdx);
    if abs(y0) > 1e-12
        bend = -8 * sign(y0);
    else
        bend = 8;
    end
    y = y0 + [0, 0, bend, 0, 0];
    routes{groupIdx} = [x + cohortOffsets(cohortIdx); y];
end
end

function waypoints = buildCrossingFormationWaypoints(formationCount)
horizontalCount = ceil(formationCount / 2);
verticalCount = formationCount - horizontalCount;
horizontalLanes = centeredValues(horizontalCount, 210) + 70;
verticalLanes = centeredValues(verticalCount, 210) - 70;
% Horizontal formations clear the intersection before the vertical group
% reaches it.  The temporal staggering preserves genuine crossing traffic
% without allowing two sensor rings to occupy the same point.
horizontalProgress = [-560, -100, 360, 560, 560];
verticalProgress = [-560, -560, -360, 100, 560];
waypoints = cell(1, formationCount);
for formationIdx = 1:horizontalCount
    direction = 1 - 2 * mod(formationIdx, 2);
    progress = direction * horizontalProgress;
    waypoints{formationIdx} = [ ...
        progress; ...
        horizontalLanes(formationIdx) * ones(1, numel(progress))];
end
for localIdx = 1:verticalCount
    formationIdx = horizontalCount + localIdx;
    direction = 1 - 2 * mod(localIdx + 1, 2);
    progress = direction * verticalProgress;
    waypoints{formationIdx} = [ ...
        verticalLanes(localIdx) * ones(1, numel(progress)); ...
        progress];
end
end

function routes = buildCrossingTargetRoutes(targetGroupCount)
horizontalCount = ceil(targetGroupCount / 2);
verticalCount = targetGroupCount - horizontalCount;
horizontalLanes = centeredValues(horizontalCount, 115) + 35;
verticalLanes = centeredValues(verticalCount, 115) - 35;
% Crossing remains a deliberately hard stress geometry in v2.  Its target
% streams traverse the complete intersection while platform groups are
% temporally staggered to preserve separation.
progress = [-620, -310, 0, 310, 620];
routes = cell(1, targetGroupCount);
for groupIdx = 1:horizontalCount
    direction = 1 - 2 * mod(groupIdx, 2);
    routes{groupIdx} = [ ...
        direction * progress; ...
        horizontalLanes(groupIdx) * ones(1, numel(progress))];
end
for localIdx = 1:verticalCount
    groupIdx = horizontalCount + localIdx;
    direction = 1 - 2 * mod(localIdx + 1, 2);
    routes{groupIdx} = [ ...
        verticalLanes(localIdx) * ones(1, numel(progress)); ...
        direction * progress];
end
end

function waypoints = buildRelayFormationWaypoints(formationCount)
x = centeredValues(formationCount, 300);
y = 350;
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    waypoints{formationIdx} = repmat([x(formationIdx); y], 1, 5);
end
end

function routes = buildRelayTargetRoutes(targetGroupCount)
% The v2 lower lane at 80 m combined with the 24 m route bend and 42 m
% maximum within-group offset to create a range-only blind strip between
% adjacent 300 m-spaced formations.  Raising only the lower bound preserves
% the handover mechanism without moving sensors or changing the FoV.
lanes = linspace(100, 220, min(4, targetGroupCount));
if targetGroupCount <= 4
    progress = [-600, -300, 0, 300, 600];
else
    progress = [-900, -450, 0, 450, 900];
end
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    direction = 1 - 2 * mod(groupIdx, 2);
    laneIdx = mod(groupIdx - 1, numel(lanes)) + 1;
    lane = lanes(laneIdx);
    bend = 24 * (1 - 2 * mod(groupIdx, 2));
    routes{groupIdx} = [ ...
        direction * progress; ...
        lane + [0, bend, 0, -bend, 0]];
end
end

function waypoints = buildMergeSplitFormationWaypoints(formationCount)
% Parallel branches become a dense corridor before separating again.
progress = [-620, -310, 0, 310, 620];
longitudinalOffsets = centeredValues(formationCount, 70);
entryLanes = centeredValues( ...
    formationCount, 900 / max(formationCount - 1, 1));
denseLanes = centeredValues(formationCount, 90);
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    exitLane = splitExitLane(formationCount, formationIdx, 300, 100);
    y = [ ...
        entryLanes(formationIdx), ...
        0.55 * entryLanes(formationIdx), ...
        denseLanes(formationIdx), ...
        0.35 * denseLanes(formationIdx) + 0.65 * exitLane, ...
        exitLane];
    waypoints{formationIdx} = [ ...
        progress + longitudinalOffsets(formationIdx); y];
end
end

function routes = buildMergeSplitTargetRoutes(targetGroupCount)
% Targets stay ahead of the co-moving sensor formations so that the
% forward-looking 120-degree FoV, rather than an accidental rear blind
% sector, controls visibility.
% Maintain a physical lead over the sensor branches throughout the common
% 160-step traversal.  The earlier route started only 100 m ahead and the
% different branch offsets allowed the rear target stream to be overtaken
% during the first few samples.  A further 120 m lead keeps the traffic in
% the forward FoV without letting targets pass through a sensor ring.
progress = [-400, -90, 240, 570, 900];
longitudinalOffsets = centeredValues(targetGroupCount, 45);
entryLanes = centeredValues( ...
    targetGroupCount, 850 / max(targetGroupCount - 1, 1)) + 45;
denseLanes = centeredValues(targetGroupCount, 75) + 30;
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    exitLane = splitExitLane(targetGroupCount, groupIdx, 310, 95) + 35;
    y = [ ...
        entryLanes(groupIdx), ...
        0.55 * entryLanes(groupIdx), ...
        denseLanes(groupIdx), ...
        0.35 * denseLanes(groupIdx) + 0.65 * exitLane, ...
        exitLane];
    routes{groupIdx} = [ ...
        progress + longitudinalOffsets(groupIdx); y];
end
end

function lane = splitExitLane(count, index, baseOffset, spacing)
lowerCount = floor(count / 2);
if index <= lowerCount
    lane = -baseOffset - spacing * (lowerCount - index);
else
    lane = baseOffset + spacing * (index - lowerCount - 1);
end
end

function waypoints = buildCurvedFormationWaypoints(formationCount)
baseX = [-650, -360, -100, 100, 190];
baseY = [-410, -380, -210, 80, 400];
laneOffsets = centeredValues(formationCount, 160);
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    offset = laneOffsets(formationIdx);
    waypoints{formationIdx} = [ ...
        baseX; baseY + offset];
end
end

function routes = buildCurvedTargetRoutes(targetGroupCount)
% The target streams lead the formations through the turn.  This keeps
% them inside the forward FoV.  Their smooth common lane drift creates one
% interpretable information-ownership handover per stream without imposing
% the excessive speed and acceleration of a repeated weaving manoeuvre.
baseX = [-540, -250, 0, 180, 250];
baseY = [-390, -320, -90, 170, 440];
laneShift = [0, 25, 60, 95, 120];
laneOffsets = centeredValues(targetGroupCount, 160) + 80;
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    offset = laneOffsets(groupIdx);
    routes{groupIdx} = [ ...
        baseX; baseY + offset + laneShift];
end
end

function times = buildStyleBlockageTimes(formationCount, styleName)
% Pair identities are resolved from the actual t=1 reference tree by
% resolveDynamicTopologyBlockageWindows.  Only the causal time schedule is
% part of the scene definition here.
if formationCount <= 4
    windowCount = formationCount - 1;
else
    windowCount = ceil((formationCount - 1) / 2);
end
if strcmp(styleName, 'crossing')
    startRange = [54, 108];
else
    startRange = [58, 112];
end
starts = round(linspace(startRange(1), startRange(2), windowCount));
stops = starts + 20;
times = [reshape(starts, [], 1), reshape(stops, [], 1)];
end

function values = centeredValues(count, spacing)
if count <= 0
    values = zeros(1, 0);
else
    values = ((1:count) - (count + 1) / 2) * spacing;
end
end
