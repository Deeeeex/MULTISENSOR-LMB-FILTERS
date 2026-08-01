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
config.sceneGeometryVersion = 'formation-fov-multistyle-v1';
config.sceneCalibrationStatus = ...
    'geometry-implemented-difficulty-gates-unfrozen';
config.formalValidationAuthorized = false;
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
config.formationBackboneMode = 'reliable-mst';
config.requireStaticPhysicalAllTimes = true;

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
        config.blockageWindows = buildStyleBlockages( ...
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
        config.blockageWindows = buildStyleBlockages( ...
            config.formationCount, 'crossing');
        config.focusWindowName = ...
            'orthogonal-intersection-and-link-blockage';
    case {'relay', 'linear-relay', 'corridor-relay'}
        config.sceneStyle = 'linear-relay';
        config.informationFlowStyle = ...
            'sequential-handover-along-a-sensor-chain';
        config.regionLimits = [-820, 820; -430, 520];
        config.formationHeadingMode = 'fixed';
        config.sensorFovHeadingMode = 'formation-shared-fixed';
        config.sensorFovFixedHeadingRadByFormation = ...
            -(pi / 2) * ones(1, config.formationCount);
        config.sensorCenterWaypoints = buildRelayFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildRelayTargetRoutes( ...
            config.targetGroupCount);
        config.targetCrossTrackSpacing = 28;
        config.blockageWindows = buildStyleBlockages( ...
            config.formationCount, 'relay');
        config.focusWindowName = 'corridor-relay-and-link-blockage';
    otherwise
        error('Unknown formation-FoV scene style: %s', styleName);
end
end

function waypoints = buildConvoyFormationWaypoints(formationCount)
if formationCount <= 4
    longitudinalOffsets = linspace(-360, 360, formationCount);
    lanes = [-160, 160];
else
    longitudinalOffsets = linspace(-450, 450, formationCount);
    lanes = [-180, 0, 180];
end
progress = [-170, -85, 0, 95, 210];
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    laneIdx = mod(formationIdx - 1, numel(lanes)) + 1;
    waypoints{formationIdx} = [ ...
        longitudinalOffsets(formationIdx) + progress; ...
        lanes(laneIdx) * ones(1, numel(progress))];
end
end

function routes = buildConvoyTargetRoutes(targetGroupCount)
lanes = linspace(-170, 170, min(4, targetGroupCount));
x = [-680, -340, 0, 340, 680];
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    startLaneIdx = mod(groupIdx - 1, numel(lanes)) + 1;
    finishLaneIdx = mod(groupIdx, numel(lanes)) + 1;
    y0 = lanes(startLaneIdx);
    y1 = lanes(finishLaneIdx);
    y = [y0, y0, 0.6 * y0 + 0.4 * y1, y1, y1];
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
if formationCount <= 4
    x = linspace(-480, 480, formationCount);
else
    x = linspace(-650, 650, formationCount);
end
y = 350;
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    waypoints{formationIdx} = repmat([x(formationIdx); y], 1, 5);
end
end

function routes = buildRelayTargetRoutes(targetGroupCount)
lanes = linspace(80, 220, min(4, targetGroupCount));
progress = [-680, -340, 0, 340, 680];
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

function windows = buildStyleBlockages(formationCount, styleName)
switch styleName
    case 'crossing'
        offset = ceil(formationCount / 2);
        left = 1:min(3, formationCount - offset);
        right = left + offset;
    otherwise
        if formationCount <= 4
            left = 1:(formationCount - 1);
            right = 2:formationCount;
        else
            left = 1:2:(formationCount - 1);
            right = left + 1;
        end
end
windowCount = numel(left);
starts = round(linspace(58, 112, windowCount));
stops = starts + 20;
windows = [reshape(left, [], 1), reshape(right, [], 1), ...
    reshape(starts, [], 1), reshape(stops, [], 1)];
end

function values = centeredValues(count, spacing)
if count <= 0
    values = zeros(1, 0);
else
    values = ((1:count) - (count + 1) / 2) * spacing;
end
end
