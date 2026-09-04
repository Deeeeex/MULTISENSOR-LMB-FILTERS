function config = applyFormationFovSceneStyle(config, styleName)
% APPLYFORMATIONFOVSCENESTYLE Geometry overlays for formation-FoV scenes.
%
%   config = applyFormationFovSceneStyle(config, 'convoy')
%
% The overlay changes platform/target motion and the blockage pattern while
% preserving the registered sensor envelope (120 degree total FoV, 300 m
% range, and detection/noise/quality profile).  A scale-normalized style may
% register one common observation/clutter support for both scales.  New styles
% are deliberately marked calibration-only until their observability and graph
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
    case {'zipper-merge', 'dynamic-zipper-merge', ...
            'dual-interchange-zipper'}
        if mod(config.formationCount, 2) ~= 0
            error('Zipper merge requires an even formation count.');
        end
        config.sceneStyle = 'zipper-merge';
        config.informationFlowStyle = [ ...
            'two-parallel-platoons-zipper-through-one-', ...
            'shared-bottleneck-and-split'];
        config.regionLimits = [-3000, 3000; -900, 900];
        config.observationSpaceLimits = [-3000, 3000; -900, 900];
        config.clutterSpatialProfile = ...
            'uniform-global-box6000x1800-c4-v1';
        config.commRange = 270;
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = ...
            buildZipperMergeFormationWaypoints(config.formationCount);
        [config.targetRoutes, ...
            config.targetRouteSourceFormationIds, ...
            config.targetRouteDestinationFormationIds, ...
            config.targetRouteHandoverFractions, ...
            config.targetRouteLaneSigns] = ...
            buildZipperMergeTargetRoutes( ...
                config.sensorCenterWaypoints, ...
                config.sensorWaypointTimes, config.simulationLength);
        config.targetBirthTimesByGroup = ...
            ones(1, config.targetGroupCount);
        config.targetDeathTimesByGroup = ...
            config.simulationLength * ones(1, config.targetGroupCount);
        config.normalizeTargetRouteDuration = false;
        config.targetCrossTrackSpacing = 18;
        config.minimumTargetSeparation = 8;
        config.minimumSensorTargetSeparation = 30;
        config.targetAccelerationLimit = 2.5;
        config.requireStaticPhysicalAllTimes = false;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = [48, 80; 96, 128];
        config.focusWindow = [35, 145];
        config.focusWindowName = ...
            'zipper-merge-reorder-split-and-paired-handoffs';
        config.taskTopologyCouplingContract = [ ...
            'paired-target-handoffs-during-one-sustained-', ...
            'geometric-tree-failure-episode-v1'];
        config = markExploratoryStyle(config);
    case {'target-overlap', 'target-group-overlap-split', ...
            'target-merge-split'}
        if mod(config.targetGroupCount, 2) ~= 0
            error(['Target-group overlap/split requires two equally ', ...
                'sized target cohorts.']);
        end
        config.sceneStyle = 'target-group-overlap-split';
        config.informationFlowStyle = [ ...
            'stationary-sensor-chain-two-target-cohorts-overlap-', ...
            'and-reseparate'];
        config.regionLimits = [-1040, 1040; -520, 620];
        config.formationHeadingMode = 'fixed';
        config.sensorFovHeadingMode = 'formation-shared-fixed';
        config.sensorFovFixedHeadingRadByFormation = ...
            -(pi / 2) * ones(1, config.formationCount);
        % Reuse the exact stationary relay formation motion.  The scene
        % changes only target information flow, not platform trajectories
        % or sensor hardware.
        config.sensorCenterWaypoints = buildRelayFormationWaypoints( ...
            config.formationCount);
        config.targetRoutes = buildTargetGroupOverlapSplitRoutes( ...
            config.targetGroupCount, config.formationCount);
        config.targetBirthTimesByGroup = ...
            ones(1, config.targetGroupCount);
        config.targetDeathTimesByGroup = ...
            config.simulationLength * ones(1, config.targetGroupCount);
        config.targetCrossTrackSpacing = 18;
        config.minimumTargetSeparation = 8;
        config.minimumSensorTargetSeparation = 30;
        config.targetAccelerationLimit = 1.5;
        config.requireStaticPhysicalAllTimes = true;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'target-group-overlap-split');
        config.focusWindow = [50, 125];
        config.focusWindowName = ...
            'target-cohort-overlap-and-reseparation';
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
    case {'braided-handover', 'scale-normalized-braided-handover'}
        if mod(config.formationCount, 2) ~= 0
            error('Braided handover requires an even formation count.');
        end
        config.sceneStyle = 'braided-handover';
        config.informationFlowStyle = ...
            'scale-normalized-local-handover-on-a-sparse-moving-chain';
        % Both scales share the same observation/clutter support.  The
        % occupied corridor grows with formation count inside this common
        % box, so X36 gains propagation distance rather than denser local
        % visibility.
        config.regionLimits = [-3000, 3000; -900, 900];
        config.observationSpaceLimits = [-3000, 3000; -900, 900];
        config.clutterSpatialProfile = ...
            'uniform-global-box6000x1800-c4-v1';
        % A common 270 m communication radius makes adjacent 180 m modules
        % physical while excluding their two-hop neighbours.  The 300 m,
        % 120-degree sensing footprint then leaves a short but genuine
        % ownership overlap for targets in the two offset service lanes.
        config.commRange = 270;
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = ...
            buildBraidedHandoverFormationWaypoints( ...
                config.formationCount);
        config.targetRoutes = buildBraidedHandoverTargetRoutes( ...
            config.targetGroupCount);
        config.targetBirthTimesByGroup = ...
            ones(1, config.targetGroupCount);
        config.targetDeathTimesByGroup = ...
            config.simulationLength * ones(1, config.targetGroupCount);
        config.targetCrossTrackSpacing = 18;
        config.minimumTargetSeparation = 8;
        config.minimumSensorTargetSeparation = 30;
        config.targetAccelerationLimit = 2.5;
        config.requireStaticPhysicalAllTimes = true;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'braided-handover');
        config.focusWindowName = ...
            'paired-braided-handover-on-sparse-chain';
        config = markExploratoryStyle(config);
    case {'formation-braid', 'dynamic-formation-braid', ...
            'information-coupled-formation-braid', ...
            'coupled-formation-braid', ...
            'temporal-coupled-formation-braid'}
        if mod(config.formationCount, 2) ~= 0
            error('Formation braid requires an even formation count.');
        end
        informationCoupled = any(strcmp(styleName, { ...
            'information-coupled-formation-braid', ...
            'coupled-formation-braid'}));
        temporalCoupled = strcmp( ...
            styleName, 'temporal-coupled-formation-braid');
        if temporalCoupled
            config.sceneStyle = 'temporal-coupled-formation-braid';
            config.informationFlowStyle = [ ...
                'moving-formation-relative-target-handoffs-', ...
                'aligned-with-initial-tree-cut-failures'];
        elseif informationCoupled
            config.sceneStyle = 'information-coupled-formation-braid';
            config.informationFlowStyle = [ ...
                'staggered-paired-overtakes-with-target-handoffs-', ...
                'covering-every-initial-chain-cut'];
        else
            config.sceneStyle = 'formation-braid';
            config.informationFlowStyle = [ ...
                'staggered-paired-overtakes-with-formation-neighbour-', ...
                'handover'];
        end
        config.regionLimits = [-3000, 3000; -900, 900];
        config.observationSpaceLimits = [-3000, 3000; -900, 900];
        config.clutterSpatialProfile = ...
            'uniform-global-box6000x1800-c4-v1';
        % Adjacent formation centres begin 180 m apart.  A common 270 m
        % radius keeps the local chain sparse while the staggered overtakes
        % replace its inter-pair bridges without disconnecting the physical
        % formation graph.
        config.commRange = 270;
        config.formationHeadingMode = 'motion';
        config.sensorFovHeadingMode = 'formation-shared-velocity';
        config.sensorCenterWaypoints = ...
            buildFormationBraidWaypoints(config.formationCount);
        if temporalCoupled
            [config.targetRoutes, ...
                config.targetRouteSourceFormationIds, ...
                config.targetRouteDestinationFormationIds, ...
                config.targetRouteHandoverFractions, ...
                config.targetRouteLaneSigns] = ...
                buildTemporallyCoupledFormationBraidTargetRoutes( ...
                    config.sensorCenterWaypoints, ...
                    config.sensorWaypointTimes, ...
                    config.simulationLength);
            config.taskTopologyCouplingContract = [ ...
                'actual-owner-side-transition-during-failed-', ...
                'initial-tree-cut-v2'];
            config.temporalCouplingLookbackSteps = 24;
            config.temporalCouplingLookaheadSteps = 36;
            config.temporalCouplingMinimumSupportFraction = 0.50;
            config.temporalCouplingMinimumAlignedTargetFraction = 0.50;
        elseif informationCoupled
            [config.targetRoutes, ...
                config.targetRouteSourceFormationIds, ...
                config.targetRouteDestinationFormationIds] = ...
                buildInformationCoupledFormationBraidTargetRoutes( ...
                    config.targetGroupCount);
            config.taskTopologyCouplingContract = [ ...
                'every-initial-chain-cut-has-an-adjacent-target-', ...
                'handoff-v1'];
        else
            config.targetRoutes = ...
                buildFormationBraidTargetRoutes( ...
                    config.targetGroupCount);
            config.targetRouteSourceFormationIds = ...
                1:config.targetGroupCount;
            config.targetRouteDestinationFormationIds = ...
                pairedFormationDestinations(config.targetGroupCount);
            config.taskTopologyCouplingContract = ...
                'paired-module-target-handoffs-v1';
        end
        config.targetBirthTimesByGroup = ...
            ones(1, config.targetGroupCount);
        config.targetDeathTimesByGroup = ...
            config.simulationLength * ones(1, config.targetGroupCount);
        config.targetCrossTrackSpacing = 18;
        config.minimumTargetSeparation = 8;
        config.minimumSensorTargetSeparation = 30;
        config.targetAccelerationLimit = 2.5;
        config.requireStaticPhysicalAllTimes = false;
        config.blockageWindows = zeros(0, 4);
        config.blockageWindowTimes = buildStyleBlockageTimes( ...
            config.formationCount, 'formation-braid');
        config.focusWindow = [40, 140];
        config.focusWindowName = ...
            'staggered-formation-neighbour-handover';
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

function waypoints = buildZipperMergeFormationWaypoints(formationCount)
% Two parallel platoons repeatedly enter a one-dimensional zipper chain.
%
% At the separated waypoints, odd/even formations occupy upper/lower road
% lanes at the same longitudinal stations.  At both bottlenecks, their
% longitudinal order becomes 1,2,...,F.  Same-lane edges of the initial
% ladder then become unavailable, while adjacent zipper neighbours keep
% the physical formation graph connected.  The same distances are used at
% M24 and X36; increasing scale only adds another local road module.
stationCount = formationCount / 2;
gridX = centeredValues(stationCount, 300);
zipperX = centeredValues(formationCount, 180);
progress = [-400, -200, 0, 200, 400];
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    stationIdx = ceil(formationIdx / 2);
    laneSign = 1 - 2 * mod(formationIdx, 2);
    midpointX = 0.5 * (gridX(stationIdx) + ...
        zipperX(formationIdx));
    relativeX = [ ...
        gridX(stationIdx), midpointX, zipperX(formationIdx), ...
        midpointX, gridX(stationIdx)];
    relativeY = laneSign * [110, 80, 50, 80, 110];
    waypoints{formationIdx} = [progress + relativeX; relativeY];
end
end

function [routes, sourceFormationIds, destinationFormationIds, ...
        handoverFractions, laneSigns] = buildZipperMergeTargetRoutes( ...
            formationWaypoints, waypointTimes, simulationLength)
% Pairwise target ownership changes are centred on the two bottlenecks.
formationCount = numel(formationWaypoints);
timeValues = 1:simulationLength;
normalizedTime = linspace(0, 1, simulationLength);
denseFormationRoutes = cell(1, formationCount);
for formationIdx = 1:formationCount
    denseFormationRoutes{formationIdx} = interpolateStyleRoute( ...
        waypointTimes, formationWaypoints{formationIdx}, timeValues);
end
sourceFormationIds = 1:formationCount;
destinationFormationIds = pairedFormationDestinations(formationCount);
handoverFractions = zeros(1, formationCount);
routeCandidates = cell(formationCount, 2);
for groupIdx = 1:formationCount
    if mod(groupIdx, 2) == 1
        transitionCentre = 0.35;
    else
        transitionCentre = 0.65;
    end
    handoverFractions(groupIdx) = transitionCentre;
    alpha = smoothStep01((normalizedTime - ...
        (transitionCentre - 0.25)) / 0.50);
    source = denseFormationRoutes{sourceFormationIds(groupIdx)};
    destination = ...
        denseFormationRoutes{destinationFormationIds(groupIdx)};
    movingReference = source .* (1 - alpha) + destination .* alpha;
    routeCandidates{groupIdx, 1} = movingReference + ...
        repmat([160; -230], 1, simulationLength);
    routeCandidates{groupIdx, 2} = movingReference + ...
        repmat([160; 230], 1, simulationLength);
end
[routes, laneSigns] = chooseTemporalServiceLanes( ...
    routeCandidates, denseFormationRoutes);
end

function routes = buildTargetGroupOverlapSplitRoutes( ...
        targetGroupCount, formationCount)
% Two target cohorts share the same monitored lanes near the middle of the
% episode, then exchange their separated service bands.  Group-specific
% longitudinal offsets prevent artificial target collisions while preserving
% genuine FoV overlap and label ambiguity.  Sensor formations remain fixed.
cohortSize = targetGroupCount / 2;
if formationCount <= 4
    progress = [-600, -300, 0, 300, 600];
else
    progress = [-850, -425, 0, 425, 850];
end
longitudinalOffsets = centeredValues(targetGroupCount, 50);
separatedLocalLanes = centeredValues(cohortSize, 30);
overlapLocalLanes = centeredValues(cohortSize, 45);
entryCenters = [100, 200];
exitCenters = [200, 100];
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    cohortIdx = 1 + (groupIdx > cohortSize);
    localIdx = mod(groupIdx - 1, cohortSize) + 1;
    entryLane = entryCenters(cohortIdx) + ...
        separatedLocalLanes(localIdx);
    overlapLane = 150 + overlapLocalLanes(localIdx);
    exitLane = exitCenters(cohortIdx) + ...
        separatedLocalLanes(localIdx);
    y = [ ...
        entryLane, ...
        0.55 * entryLane + 0.45 * overlapLane, ...
        overlapLane, ...
        0.45 * overlapLane + 0.55 * exitLane, ...
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

function waypoints = ...
        buildBraidedHandoverFormationWaypoints(formationCount)
% A repeated local module: adjacent formations are physical under the
% common 270 m communication range, while two-hop pairs are not.  Adding
% formations extends the chain without changing a module's geometry.
spacing = 180;
baseX = centeredValues(formationCount, spacing);
laneY = zeros(1, formationCount);
progress = [-360, -180, 0, 180, 360];
waypoints = cell(1, formationCount);
for formationIdx = 1:formationCount
    waypoints{formationIdx} = [ ...
        baseX(formationIdx) + progress; ...
        laneY(formationIdx) * ones(1, numel(progress))];
end
end

function routes = buildBraidedHandoverTargetRoutes(targetGroupCount)
% Every adjacent formation pair exchanges two target streams in opposite
% relative directions.  The streams remain in two offset service lanes:
% this prevents target/platform near-collisions while the longitudinal
% exchange still changes the observing formation exactly once.
if mod(targetGroupCount, 2) ~= 0
    error('Braided handover requires an even target-group count.');
end
spacing = 180;
formationX = centeredValues(targetGroupCount, spacing);
progress = [-360, -180, 0, 180, 360];
handoverPhase = [0, 0.25, 0.50, 0.75, 1.00];
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    if mod(groupIdx, 2) == 1
        destinationIdx = groupIdx + 1;
    else
        destinationIdx = groupIdx - 1;
    end
    deltaX = formationX(destinationIdx) - formationX(groupIdx);
    serviceLaneY = 110 * (1 - 2 * mod(groupIdx, 2));
    routes{groupIdx} = [ ...
        formationX(groupIdx) + 85 + progress + ...
            handoverPhase * deltaX; ...
        serviceLaneY * ones(1, numel(progress))];
end
end

function waypoints = buildFormationBraidWaypoints(formationCount)
% Paired formations exchange longitudinal order in separated passing lanes.
% The same local module is repeated as the number of formations grows.
spacing = 180;
baseX = centeredValues(formationCount, spacing);
progress = [-360, -180, 0, 180, 360];
normalizedTime = linspace(0, 1, numel(progress));
pairCount = formationCount / 2;
swapCentres = linspace(0.25, 0.75, pairCount);
waypoints = cell(1, formationCount);
for pairIdx = 1:pairCount
    leftIdx = 2 * pairIdx - 1;
    rightIdx = leftIdx + 1;
    fraction = min(max((normalizedTime - ...
        (swapCentres(pairIdx) - 0.25)) / 0.5, 0), 1);
    lateralBell = sin(pi * fraction);
    waypoints{leftIdx} = [ ...
        baseX(leftIdx) + progress + spacing * fraction; ...
        70 * lateralBell];
    waypoints{rightIdx} = [ ...
        baseX(rightIdx) + progress - spacing * fraction; ...
        -70 * lateralBell];
end
end

function routes = buildFormationBraidTargetRoutes(targetGroupCount)
% Targets retain the modular adjacent-pair handover but use service lanes
% outside the passing lanes.  The 170 m offset avoids platform encounters
% while remaining inside the frozen 300 m, 120-degree sensing envelope.
if mod(targetGroupCount, 2) ~= 0
    error('Formation braid requires an even target-group count.');
end

spacing = 180;
formationX = centeredValues(targetGroupCount, spacing);
progress = [-360, -180, 0, 180, 360];
handoverPhase = [0, 0.25, 0.50, 0.75, 1.00];
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    if mod(groupIdx, 2) == 1
        destinationIdx = groupIdx + 1;
    else
        destinationIdx = groupIdx - 1;
    end
    deltaX = formationX(destinationIdx) - formationX(groupIdx);
    serviceLaneY = 170 * (1 - 2 * mod(groupIdx, 2));
    routes{groupIdx} = [ ...
        formationX(groupIdx) + 85 + progress + ...
            handoverPhase * deltaX; ...
        serviceLaneY * ones(1, numel(progress))];
end
end

function destinations = pairedFormationDestinations(formationCount)
if mod(formationCount, 2) ~= 0
    error('Paired formation destinations require an even count.');
end
destinations = zeros(1, formationCount);
destinations(1:2:end) = 2:2:formationCount;
destinations(2:2:end) = 1:2:(formationCount - 1);
end

function [routes, sourceFormationIds, destinationFormationIds] = ...
        buildInformationCoupledFormationBraidTargetRoutes(targetGroupCount)
% Couple target handoffs to the chain cuts that a fixed formation tree uses.
%
% The original formation-braid scene pairs both platform overtakes and target
% handoffs as (1,2), (3,4), ... .  The bridge between two such modules can
% therefore fail without carrying task-relevant target information.  This
% variant assigns one target cohort to every adjacent chain cut.  The final
% cohort returns across the last cut so that the target load remains one
% cohort per formation at every scale.
if targetGroupCount < 2
    error('Information-coupled formation braid needs two target groups.');
end
spacing = 180;
formationX = centeredValues(targetGroupCount, spacing);
progress = [-360, -180, 0, 180, 360];
handoverPhase = [0, 0.25, 0.50, 0.75, 1.00];
sourceFormationIds = 1:targetGroupCount;
destinationFormationIds = [2:targetGroupCount, targetGroupCount - 1];
routes = cell(1, targetGroupCount);
for groupIdx = 1:targetGroupCount
    sourceIdx = sourceFormationIds(groupIdx);
    destinationIdx = destinationFormationIds(groupIdx);
    deltaX = formationX(destinationIdx) - formationX(sourceIdx);
    serviceLaneY = 170 * (1 - 2 * mod(groupIdx, 2));
    % The final return stream and the preceding forward stream both end at
    % F-1.  Leaving them in the same +170 m lane makes their target groups
    % coalesce at the route endpoint.  A separate +240 m service lane is the
    % smallest scale-invariant offset that passes the existing target and
    % sensor--target separation gates at M24, X36, and X48 while retaining a
    % registered formation handover under the frozen 120-degree/300-m FoV.
    if groupIdx == targetGroupCount
        serviceLaneY = 240;
    end
    routes{groupIdx} = [ ...
        formationX(sourceIdx) + 85 + progress + ...
            handoverPhase * deltaX; ...
        serviceLaneY * ones(1, numel(progress))];
end
end

function [routes, sourceFormationIds, destinationFormationIds, ...
        handoverFractions, laneSigns] = ...
        buildTemporallyCoupledFormationBraidTargetRoutes( ...
            formationWaypoints, waypointTimes, simulationLength)
% Build handoffs relative to the moving formations, not their t=1 centres.
%
% A target cohort starts in front of its registered source formation and
% gradually moves to the corresponding position in front of the destination
% formation.  Transitions across the bridges between paired overtakes are
% centred between the two neighbouring swap phases.  This is the same
% scale-free rule for M24, X36 and X48; no scale-specific route is fitted.
formationCount = numel(formationWaypoints);
if formationCount < 2 || mod(formationCount, 2) ~= 0
    error(['Temporally coupled formation braid requires a positive ', ...
        'even formation count.']);
end
waypointCount = size(formationWaypoints{1}, 2);
for formationIdx = 1:formationCount
    if ~isequal(size(formationWaypoints{formationIdx}), ...
            [2, waypointCount])
        error('Formation waypoint shapes must agree.');
    end
end

if numel(waypointTimes) ~= waypointCount || simulationLength < 2
    error('Temporal route times do not match the formation waypoints.');
end
timeValues = 1:simulationLength;
normalizedTime = linspace(0, 1, simulationLength);
denseFormationRoutes = cell(1, formationCount);
for formationIdx = 1:formationCount
    denseFormationRoutes{formationIdx} = interpolateStyleRoute( ...
        waypointTimes, formationWaypoints{formationIdx}, timeValues);
end
pairCount = formationCount / 2;
swapCentres = linspace(0.25, 0.75, pairCount);
sourceFormationIds = 1:formationCount;
destinationFormationIds = [2:formationCount, formationCount - 1];
handoverFractions = zeros(1, formationCount);
routeCandidates = cell(formationCount, 2);
for groupIdx = 1:formationCount
    sourceIdx = sourceFormationIds(groupIdx);
    destinationIdx = destinationFormationIds(groupIdx);
    if sourceIdx == formationCount
        transitionCentre = min(0.88, swapCentres(end) + 0.10);
    elseif mod(sourceIdx, 2) == 1
        transitionCentre = swapCentres((sourceIdx + 1) / 2);
    else
        leftPair = sourceIdx / 2;
        transitionCentre = 0.5 * ( ...
            swapCentres(leftPair) + swapCentres(leftPair + 1));
    end
    handoverFractions(groupIdx) = transitionCentre;
    transitionWidth = 0.48;
    alpha = smoothStep01((normalizedTime - ...
        (transitionCentre - transitionWidth / 2)) / transitionWidth);
    source = denseFormationRoutes{sourceIdx};
    destination = denseFormationRoutes{destinationIdx};
    movingReference = source .* (1 - alpha) + destination .* alpha;
    % The two lane candidates are symmetric in the world-frame service
    % corridor.  A deterministic joint selector below chooses the
    % assignment with the largest clearance from every formation centre
    % and every other target
    % cohort.  It uses geometry only and is identical at every scale.
    % All formation-braid platforms move eastward; a fixed world-frame
    % service corridor avoids amplifying the platforms' heading curvature
    % into unrealistic target acceleration.  The 180/170 m offset remains
    % inside the frozen 300 m, 120-degree sensor envelope while clearing the
    % neighbouring 180 m-spaced formation centres.
    forwardOffset = 180;
    laneMagnitude = 170;
    routeCandidates{groupIdx, 1} = movingReference + ...
        repmat([forwardOffset; -laneMagnitude], 1, simulationLength);
    routeCandidates{groupIdx, 2} = movingReference + ...
        repmat([forwardOffset; laneMagnitude], 1, simulationLength);
end
[routes, laneSigns] = chooseTemporalServiceLanes( ...
    routeCandidates, denseFormationRoutes);
end

function [routes, laneSigns] = chooseTemporalServiceLanes( ...
        routeCandidates, formationRoutes)
formationCount = size(routeCandidates, 1);
assignmentCount = 2 ^ formationCount;
bestScore = -inf;
bestMeanClearance = -inf;
bestAssignment = ones(1, formationCount);
for assignmentCode = 0:(assignmentCount - 1)
    assignment = 1 + bitget(assignmentCode, 1:formationCount);
    selected = cell(1, formationCount);
    for groupIdx = 1:formationCount
        selected{groupIdx} = ...
            routeCandidates{groupIdx, assignment(groupIdx)};
    end
    [score, meanClearance] = temporalLaneSafetyScore( ...
        selected, formationRoutes);
    if score > bestScore + 1e-9 || ...
            (abs(score - bestScore) <= 1e-9 && ...
             meanClearance > bestMeanClearance)
        bestScore = score;
        bestMeanClearance = meanClearance;
        bestAssignment = assignment;
    end
end
routes = cell(1, formationCount);
laneSigns = zeros(1, formationCount);
for groupIdx = 1:formationCount
    routes{groupIdx} = ...
        routeCandidates{groupIdx, bestAssignment(groupIdx)};
    laneSigns(groupIdx) = 2 * bestAssignment(groupIdx) - 3;
end
end

function [score, meanClearance] = temporalLaneSafetyScore( ...
        routes, formationRoutes)
% A 96 m centre clearance covers the 38.5 m jittered sensor ring, the
% 27 m outer target offset and the registered 30 m sensor-target margin.
% A 62 m inter-route clearance covers two 27 m cohort half-widths and the
% registered 8 m target-target margin.
formationClearances = [];
for groupIdx = 1:numel(routes)
    for formationIdx = 1:numel(formationRoutes)
        formationClearances(end+1) = min(sqrt(sum( ... %#ok<AGROW>
            (routes{groupIdx} - formationRoutes{formationIdx}).^2, 1)));
    end
end
routeClearances = [];
for leftIdx = 1:numel(routes)-1
    for rightIdx = leftIdx+1:numel(routes)
        routeClearances(end+1) = min(sqrt(sum( ... %#ok<AGROW>
            (routes{leftIdx} - routes{rightIdx}).^2, 1)));
    end
end
score = min([formationClearances - 96, routeClearances - 62]);
meanClearance = mean([formationClearances, routeClearances]);
end

function values = interpolateStyleRoute(times, waypoints, queryTimes)
values = zeros(size(waypoints, 1), numel(queryTimes));
for dimensionIdx = 1:size(waypoints, 1)
    values(dimensionIdx, :) = interp1( ...
        times, waypoints(dimensionIdx, :), queryTimes, 'pchip');
end
end

function values = smoothStep01(values)
values = min(max(values, 0), 1);
values = values .* values .* (3 - 2 * values);
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
