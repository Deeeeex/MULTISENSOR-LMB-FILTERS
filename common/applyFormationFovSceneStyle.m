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
config.sceneGeometryVersion = 'formation-fov-multistyle-v3';
config.sceneCalibrationStatus = ...
    'geometry-recalibration-in-progress-v3';
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
            'co-moving-local-overlap-and-overtake';
        config.regionLimits = [-820, 820; -520, 520];
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = buildConvoyFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildConvoyTargetRoutes( ...
            config.targetGroupCount);
        config.targetCrossTrackSpacing = 32;
        config.minimumTargetSeparation = 14;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'convoy');
        config.focusWindowName = 'convoy-overtake-and-link-blockage';
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
    otherwise
        error('Unknown formation-FoV scene style: %s', styleName);
end
end

function waypoints = buildConvoyFormationWaypoints(formationCount)
if formationCount <= 4
    lanes = [-100, 100];
else
    % Preserve the same 200 m local lane spacing when scale grows from two
    % to three lanes.  Scale is expressed by adding formations/traffic, not
    % by silently making X36's local sensing geometry denser.
    lanes = [-200, 0, 200];
end
% The fixed 300 m sensing range must bridge the handoff between a rear
% east-facing column and the front column.  A wider gap creates a long blind
% band that is a scene artifact rather than a routing challenge.
longitudinalColumns = [-175, 175];
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
if targetGroupCount <= 4
    lanes = [-100, 100];
else
    lanes = [-200, 0, 200];
end
% Targets begin just ahead of the rear formations and then overtake the
% front column.  This removes the long unavoidable rear-sector blackout in
% v1 while retaining directional handover and gentle lateral maneuvers.
x = [-320, -120, 120, 380, 650];
routes = cell(1, targetGroupCount);
cohortOffsets = centeredValues( ...
    ceil(targetGroupCount / numel(lanes)), 16);
for groupIdx = 1:targetGroupCount
    startLaneIdx = mod(groupIdx - 1, numel(lanes)) + 1;
    cohortIdx = floor((groupIdx - 1) / numel(lanes)) + 1;
    y0 = lanes(startLaneIdx);
    % Groups that reuse the same lane-change template receive a small fixed
    % sub-lane offset.  Without it, different labelled groups can nearly
    % collide even though within-group cross-track spacing is respected.
    if abs(y0) > 1e-12
        bend = -24 * sign(y0);
    else
        bend = 24;
    end
    y = y0 + cohortOffsets(cohortIdx) + [0, 0, bend, 0, 0];
    routes{groupIdx} = [x; y];
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
