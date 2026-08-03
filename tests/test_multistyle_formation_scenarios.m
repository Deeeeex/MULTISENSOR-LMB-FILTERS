function test_multistyle_formation_scenarios()
% TEST_MULTISTYLE_FORMATION_SCENARIOS Geometry-diversity contracts.

presets = { ...
    'm24-formation-fov-convoy', ...
    'm24-formation-fov-crossing', ...
    'm24-formation-fov-relay', ...
    'x36-formation-fov-convoy', ...
    'x36-formation-fov-crossing', ...
    'x36-formation-fov-relay'};
expectedNodes = [24, 24, 24, 36, 36, 36];
expectedTargets = [16, 16, 16, 24, 24, 24];
expectedStyles = { ...
    'parallel-convoy', 'orthogonal-crossing', 'linear-relay', ...
    'parallel-convoy', 'orthogonal-crossing', 'linear-relay'};
expectedContractSha256 = { ...
    '2ac1026f99d2d9048cd63b0d5be0ecd475e6a863f414826c6a2fb46cc8fc9181', ...
    '9f4331e492f044aa566be6f8b3adc1c4cb99e207aa9a1f6a72aaab76bd4770b0', ...
    '4f1f9173e8208f6b8f792442e9d6610e98f8ad992957c4c08671d1ef6deb4e76', ...
    '7b529e5554dfc9581aa5b715a3a81fe6b86266a2966015b258841d2e50447bc7', ...
    'df71200a4e5e62c3ee00e7cd0b1864bed1d99db7a0c349d8771b9943b9e7a3ec', ...
    'b00829d37cd78e5f1e4a2050b8e80c0154864a05f2dc01e89d8b50dc5f45d8c3'};

hardwareFields = { ...
    'fovHalfAngleDeg', 'fovTotalAngleDeg', 'fovRange', ...
    'detectionProbability', 'measurementNoiseStd', ...
    'sensorQuality', 'sensorHardwareProfile', ...
    'observationSpaceLimits', 'clutterRate', ...
    'clutterSpatialProfile'};
baseByScale = { ...
    buildDynamicTopologyScenarioConfig('m24-formation-fov'), ...
    buildDynamicTopologyScenarioConfig('x36-formation-fov')};

for presetIdx = 1:numel(presets)
    rng(41);
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    assert(config.numberOfSensors == expectedNodes(presetIdx));
    assert(config.numberOfTargets == expectedTargets(presetIdx));
    assert(strcmp(config.sceneStyle, expectedStyles{presetIdx}));
    assert(strcmp(config.sceneGeometryVersion, ...
        'formation-fov-multistyle-v4'));
    assert(strcmp(config.sceneCalibrationStatus, ...
        'geometry-recalibration-in-progress-v4'));
    assert(~config.formalValidationAuthorized);
    assert(~config.trackingOutcomeAuthorized);
    assert(strcmp(config.sceneContractSha256, ...
        expectedContractSha256{presetIdx}));
    assert(strcmp(config.sceneContractSha256, ...
        computeFormationFovMultistyleSceneContractSha256(config)));
    assert(~config.enforceDifficultyRequirements);
    assert(isempty(fieldnames(config.difficultyRequirements)));
    assert(config.fovTotalAngleDeg == 120);
    assert(config.fovHalfAngleDeg == 60);
    assert(config.fovRange == 300);
    assert(strncmp(config.sensorFovHeadingMode, ...
        'formation-shared-', numel('formation-shared-')));

    scaleIdx = 1 + (config.numberOfSensors == 36);
    base = baseByScale{scaleIdx};
    for fieldIdx = 1:numel(hardwareFields)
        fieldName = hardwareFields{fieldIdx};
        assert(isequaln(config.(fieldName), base.(fieldName)));
    end

    [sensors, ~] = generateMultiFormationTrajectories(config);
    [targets, ~] = generateCorridorTargetTrajectories(config);
    headings = buildSensorFovHeadingSchedule(config, sensors);
    graphs = buildDynamicTopologyGraphs(config, sensors);
    assert(graphs.backboneSelectionTime == 1);
    assert(size(graphs.formationBackbonePairs, 1) == ...
        config.formationCount - 1);
    assertBackboneUsesInitialGeometryOnly(config, sensors, graphs);
    resolvedBlockages = ...
        resolveDynamicTopologyBlockageWindows(config, graphs);
    assert(size(resolvedBlockages, 1) == ...
        size(config.blockageWindowTimes, 1));
    assertResolvedBlockagesHitReference( ...
        config, graphs, resolvedBlockages);
    [pDropByEdge, linkMetadata] = ...
        buildDynamicTopologyLinkSchedule(config, graphs);
    assert(isequal(linkMetadata.blockageWindows, ...
        resolvedBlockages));
    assertResolvedBlockageScheduleEffective( ...
        config, graphs, resolvedBlockages, pDropByEdge);
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graphs);
    assert(validation.isValid);
    assert(validation.targetStatesWellFormed);
    assert(validation.targetActivityMatchesContract);
    if config.requireStaticPhysicalAllTimes
        assert(validation.staticPhysicalViolationCount == 0);
        assert(graphs.staticAllTimePhysical);
    else
        assert(validation.staticPhysicalViolationCount > 0);
        assert(~graphs.staticAllTimePhysical);
    end
    assert(validation.staticEdgeCount == config.edgeBudget);
    assert(maximumSpeed(sensors) <= config.sensorSpeedLimit + 1e-9);
    assert(maximumSpeed(targets) <= config.targetSpeedLimit + 1e-9);
    assert(validation.maxTargetAcceleration <= ...
        config.targetAccelerationLimit + 1e-9);
    assert(validation.minimumTargetSeparation + 1e-9 >= ...
        config.minimumTargetSeparation);
    assert(validation.minimumSensorTargetSeparation + 1e-9 >= ...
        config.minimumSensorTargetSeparation);
    if presetIdx == 1
        assertTargetKinematicContractsFailClosed( ...
            config, sensors, targets, graphs);
        assertPartialNanTargetStateFailsClosed( ...
            config, sensors, targets, graphs);
        assertAllNanActiveGapFailsClosed( ...
            config, sensors, targets, graphs);
        assertSensorTargetTrajectoryMutationFailsClosed( ...
            config, sensors, targets, graphs);
    end
    assert(all(isfinite(headings(:))));
    assertFormationSharedHeadings(config, headings);

    centers = formationCenters(config, sensors);
    displacements = squeeze(centers(:, end, :) - centers(:, 1, :));
    switch config.sceneStyle
        case 'parallel-convoy'
            assert(all(displacements(1, :) > 300));
            assert(max(abs(displacements(2, :))) < 1e-9);
            assert(max(abs(wrapAngle( ...
                headings(:, round(config.simulationLength / 2)) - ...
                deg2rad(30)))) < 1e-12);
            assertConvoySafetyGeometry(config);
        case 'orthogonal-crossing'
            assert(all(sqrt(sum(displacements.^2, 1)) > 900));
            groupHeading = headings( ...
                1:config.sensorsPerFormation:end, ...
                round(config.simulationLength / 2));
            assert(maximumCircularSeparation(groupHeading) > pi / 2);
        case 'linear-relay'
            assert(max(abs(displacements(:))) < 1e-9);
            assert(max(abs(wrapAngle(headings(:) + pi / 2))) < 1e-12);
        otherwise
            error('Unexpected scene style in test.');
    end
end

testVersionedPresetsRejectOverrides();
testExplicitBlockagesMayRepeatPairs();
testFullInputGeneration(presets);
testHeadingModesFailClosed();
fprintf('test_multistyle_formation_scenarios passed\n');
end

function assertConvoySafetyGeometry(config)
laneCount = config.formationCount / 2;
assert(laneCount == floor(laneCount));
expectedSensorLanes = ...
    ((1:laneCount) - (laneCount + 1) / 2) * 220 - 55;
expectedTargetLanes = ...
    ((1:laneCount) - (laneCount + 1) / 2) * 220 + 55;
sensorLanes = zeros(1, laneCount);
targetLanes = zeros(1, laneCount);
for laneIdx = 1:laneCount
    sensorLanes(laneIdx) = ...
        config.sensorCenterWaypoints{laneIdx}(2, 1);
    targetLanes(laneIdx) = config.targetRoutes{laneIdx}(2, 1);
end
assert(max(abs(sensorLanes - expectedSensorLanes)) < 1e-12);
assert(max(abs(targetLanes - expectedTargetLanes)) < 1e-12);
crossSetLaneSeparations = abs(bsxfun(@minus, ...
    sensorLanes(:), targetLanes(:).'));
assert(min(crossSetLaneSeparations(:)) == 110);
assert(config.targetCrossTrackSpacing == 20);
assert(config.minimumSensorTargetSeparation == 30);
assert(all(abs(config.sensorFovFixedHeadingRadByFormation - ...
    deg2rad(30)) < 1e-12));
frontColumn = config.sensorCenterWaypoints{laneCount + 1}(1, :);
rearColumn = config.sensorCenterWaypoints{1}(1, :);
assert(all(abs(frontColumn - rearColumn - 300) < 1e-12));
for laneIdx = 1:laneCount
    secondCohort = laneCount + laneIdx;
    % Freeze the raw route-template anchor offset.  Generated synchronized
    % spacing is separately covered by the trajectory-level separation gate.
    assert(all(abs(config.targetRoutes{secondCohort}(1, :) - ...
        config.targetRoutes{laneIdx}(1, :) - 80) < 1e-12));
end
end

function assertResolvedBlockagesHitReference( ...
        config, graphs, windows)
pairs = sortrows(graphs.formationBackbonePairs);
for windowIdx = 1:size(windows, 1)
    pair = sort(windows(windowIdx, 1:2));
    assert(any(all(bsxfun(@eq, pairs, pair), 2)));
    left = find(config.sensorGroupIds == pair(1));
    right = find(config.sensorGroupIds == pair(2));
    assert(any(any(graphs.staticAdjacency(left, right))));
end
end

function assertResolvedBlockageScheduleEffective( ...
        config, graphs, windows, pDropByEdge)
controlConfig = config;
controlConfig.blockageScheduleMode = 'explicit';
controlConfig.blockageWindows = zeros(0, 4);
[controlDropByEdge, ~] = ...
    buildDynamicTopologyLinkSchedule(controlConfig, graphs);
for windowIdx = 1:size(windows, 1)
    pair = windows(windowIdx, 1:2);
    left = find(config.sensorGroupIds == pair(1));
    right = find(config.sensorGroupIds == pair(2));
    for timeIdx = windows(windowIdx, 3):windows(windowIdx, 4)
        staticPhysical = graphs.staticAdjacency(left, right) & ...
            graphs.physicalAdjacency(left, right, timeIdx);
        assert(any(staticPhysical(:)), [ ...
            'A registered blockage lacked a physical reference edge ', ...
            'during part of its window.']);
        currentDrop = pDropByEdge(left, right, timeIdx);
        controlDrop = controlDropByEdge(left, right, timeIdx);
        increment = currentDrop(staticPhysical) - ...
            controlDrop(staticPhysical);
        assert(all(abs(increment - 0.65) < 1e-12), [ ...
            'A registered blockage did not add the expected drop ', ...
            'penalty throughout its window.']);
    end
end
end

function assertBackboneUsesInitialGeometryOnly(config, sensors, graphs)
counterfactual = sensors;
timeCount = config.simulationLength;
phase = linspace(0, 2 * pi, timeCount - 1);
for sensorIdx = 1:numel(counterfactual)
    groupIdx = config.sensorGroupIds(sensorIdx);
    counterfactual{sensorIdx}(1, 2:end) = ...
        counterfactual{sensorIdx}(1, 2:end) + ...
        2000 * groupIdx + 300 * sin(phase + sensorIdx);
    counterfactual{sensorIdx}(2, 2:end) = ...
        counterfactual{sensorIdx}(2, 2:end) - ...
        1700 * groupIdx + 250 * cos(phase + sensorIdx);
end
% Disable only the post-selection all-time-physical assertion.  The
% counterfactual deliberately destroys future links; the property under
% test is that the t=1 tree itself remains unchanged.
counterfactualConfig = config;
counterfactualConfig.requireStaticPhysicalAllTimes = false;
altered = buildDynamicTopologyGraphs( ...
    counterfactualConfig, counterfactual);
assert(isequal(graphs.initialAdjacency, altered.initialAdjacency));
assert(isequal(graphs.staticAdjacency, altered.staticAdjacency));
assert(isequal(graphs.formationBackbonePairs, ...
    altered.formationBackbonePairs));
end

function assertTargetKinematicContractsFailClosed( ...
        config, sensors, targets, graphs)
impossibleAcceleration = config;
impossibleAcceleration.targetAccelerationLimit = -1e-12;
assertScenarioValidationFailure(impossibleAcceleration, sensors, ...
    targets, graphs, 'target-acceleration');

impossibleSeparation = config;
impossibleSeparation.minimumTargetSeparation = inf;
assertScenarioValidationFailure(impossibleSeparation, sensors, ...
    targets, graphs, 'target-separation');

impossibleCrossSetSeparation = config;
impossibleCrossSetSeparation.minimumSensorTargetSeparation = inf;
assertScenarioValidationFailure(impossibleCrossSetSeparation, sensors, ...
    targets, graphs, 'sensor-target-separation');
end

function assertPartialNanTargetStateFailsClosed( ...
        config, sensors, targets, graphs)
malformedTargets = targets;
activeTime = find(all(isfinite(malformedTargets{1}), 1), 1, 'first');
malformedTargets{1}(1, activeTime) = NaN;
assertScenarioValidationFailure(config, sensors, ...
    malformedTargets, graphs, 'target-state-column-integrity');
end

function assertAllNanActiveGapFailsClosed( ...
        config, sensors, targets, graphs)
malformedTargets = targets;
activeTime = find(all(isfinite(malformedTargets{1}), 1), 1, 'first');
malformedTargets{1}(:, activeTime) = NaN;
assertScenarioValidationFailure(config, sensors, ...
    malformedTargets, graphs, 'target-activity-contract');
end

function assertSensorTargetTrajectoryMutationFailsClosed( ...
        config, sensors, targets, graphs)
collidingSensors = sensors;
activeTime = find(all(isfinite(targets{1}(1:2, :)), 1), ...
    1, 'first');
collidingSensors{1}(1:2, activeTime) = ...
    targets{1}(1:2, activeTime);
assertScenarioValidationFailure(config, collidingSensors, ...
    targets, graphs, 'sensor-target-separation');
end

function assertScenarioValidationFailure( ...
        config, sensors, targets, graphs, expectedToken)
failed = false;
try
    validateDynamicTopologyScenario(config, sensors, targets, graphs);
catch errorInfo
    failed = ~isempty(strfind(errorInfo.message, expectedToken)); %#ok<STREMP>
end
assert(failed, 'Expected scenario validation failure: %s', expectedToken);
end

function testVersionedPresetsRejectOverrides()
assertThrowsMessage(@() buildDynamicTopologyScenarioConfig( ...
    'm24-formation-fov-convoy', struct('fovRange', 900)), ...
    'do not accept runtime overrides');
assertThrowsMessage(@() buildDynamicTopologyScenarioConfig( ...
    'x36-formation-fov-relay', struct( ...
        'formalValidationAuthorized', true)), ...
    'do not accept runtime overrides');
end

function testExplicitBlockagesMayRepeatPairs()
rng(67);
config = buildDynamicTopologyScenarioConfig('d12-formation-fov');
[sensors, ~] = generateMultiFormationTrajectories(config);
graphs = buildDynamicTopologyGraphs(config, sensors);
[~, metadata] = buildDynamicTopologyLinkSchedule(config, graphs);
assert(isequal(metadata.blockageWindows, config.blockageWindows));
pairs = sort(metadata.blockageWindows(:, 1:2), 2);
assert(sum(all(bsxfun(@eq, pairs, [1, 2]), 2)) == 2);
end

function testFullInputGeneration(presets)
for presetIdx = 1:numel(presets)
    inputs = generateDynamicTopologyScenarioInputs( ...
        presets{presetIdx}, 61);
    assert(inputs.validation.isValid);
    assert(strcmp(inputs.config.sceneGeometryVersion, ...
        'formation-fov-multistyle-v4'));
    assert(isequal(inputs.linkMetadata.blockageWindows, ...
        resolveDynamicTopologyBlockageWindows( ...
            inputs.config, inputs.graphData)));
    assert(isequal(size(inputs.commConfig.pDropByEdge), [ ...
        inputs.config.numberOfSensors, ...
        inputs.config.numberOfSensors, ...
        inputs.config.simulationLength]));
    assertTrackingFailsClosed(inputs);
end
end

function assertTrackingFailsClosed(inputs)
assert(isfield(inputs.model, ...
    'dynamicTopologyTrackingOutcomeGate'));
assertTrackingGateError(@() runEventTriggeredDistributedLmbFilter( ...
    inputs.model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, struct()));

tampered = inputs.model;
tampered.dynamicTopologyScenario.config.trackingOutcomeAuthorized = true;
assertTrackingGateError(@() ...
    assertDynamicTopologyTrackingOutcomeAuthorized(tampered));

missingGate = inputs.model;
missingGate.dynamicTopologyScenario.config = rmfield( ...
    missingGate.dynamicTopologyScenario.config, ...
    'trackingOutcomeAuthorized');
assertTrackingGateError(@() ...
    assertDynamicTopologyTrackingOutcomeAuthorized(missingGate));

strippedVersion = inputs.model;
strippedVersion.dynamicTopologyScenario.config.sceneGeometryVersion = ...
    'legacy-scene';
strippedVersion.dynamicTopologyScenario.config. ...
    trackingOutcomeAuthorized = true;
assertTrackingGateError(@() ...
    assertDynamicTopologyTrackingOutcomeAuthorized(strippedVersion));

missingScenario = rmfield(inputs.model, 'dynamicTopologyScenario');
assertTrackingGateError(@() ...
    assertDynamicTopologyTrackingOutcomeAuthorized(missingScenario));

missingEnvelope = rmfield(inputs.model, ...
    'dynamicTopologyTrackingOutcomeGate');
assertTrackingGateError(@() ...
    assertDynamicTopologyTrackingOutcomeAuthorized(missingEnvelope));
end

function assertTrackingGateError(callback)
failed = false;
try
    callback();
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'DynamicTopology:TrackingOutcomeNotAuthorized');
end
assert(failed, 'Calibration-only scene did not fail closed at tracking.');
end

function assertThrowsMessage(callback, expectedText)
threw = false;
try
    callback();
catch errorInfo
    threw = ~isempty(strfind(errorInfo.message, expectedText)); %#ok<STREMP>
end
assert(threw, 'Expected error containing: %s', expectedText);
end

function assertFormationSharedHeadings(config, headings)
for formationIdx = 1:config.formationCount
    members = find(config.sensorGroupIds == formationIdx);
    reference = headings(members(1), :);
    differences = wrapAngle(bsxfun(@minus, ...
        headings(members, :), reference));
    assert(max(abs(differences(:))) < 1e-12);
end
end

function centers = formationCenters(config, sensors)
centers = zeros(2, config.simulationLength, config.formationCount);
for formationIdx = 1:config.formationCount
    members = find(config.sensorGroupIds == formationIdx);
    for sensorIdx = reshape(members, 1, [])
        centers(:, :, formationIdx) = centers(:, :, formationIdx) + ...
            sensors{sensorIdx}(1:2, :);
    end
    centers(:, :, formationIdx) = ...
        centers(:, :, formationIdx) / numel(members);
end
end

function speed = maximumSpeed(trajectories)
speed = 0;
for trajectoryIdx = 1:numel(trajectories)
    velocity = trajectories{trajectoryIdx}(3:4, :);
    finiteColumns = all(isfinite(velocity), 1);
    if any(finiteColumns)
        speed = max(speed, max(sqrt(sum( ...
            velocity(:, finiteColumns).^2, 1))));
    end
end
end

function separation = maximumCircularSeparation(angles)
separation = 0;
for leftIdx = 1:numel(angles)
    for rightIdx = leftIdx+1:numel(angles)
        separation = max(separation, abs(wrapAngle( ...
            angles(leftIdx) - angles(rightIdx))));
    end
end
end

function values = wrapAngle(values)
values = atan2(sin(values), cos(values));
end

function testHeadingModesFailClosed()
config = buildDynamicTopologyScenarioConfig( ...
    'm24-formation-fov-relay');
rng(43);
[sensors, ~] = generateMultiFormationTrajectories(config);

missing = rmfield(config, 'sensorFovFixedHeadingRadByFormation');
failedClosed = false;
try
    buildSensorFovHeadingSchedule(missing, sensors);
catch errorInfo
    failedClosed = ~isempty(strfind( ...
        errorInfo.message, 'sensorFovFixedHeadingRadByFormation')); %#ok<STREMP>
end
assert(failedClosed);

stationaryVelocity = config;
stationaryVelocity.sensorFovHeadingMode = 'formation-shared-velocity';
failedClosed = false;
try
    buildSensorFovHeadingSchedule(stationaryVelocity, sensors);
catch errorInfo
    failedClosed = ~isempty(strfind( ...
        errorInfo.message, 'requires platform motion')); %#ok<STREMP>
end
assert(failedClosed);
end
