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
        'formation-fov-multistyle-v1'));
    assert(strcmp(config.sceneCalibrationStatus, ...
        'geometry-implemented-difficulty-gates-unfrozen'));
    assert(~config.formalValidationAuthorized);
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
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graphs);
    assert(validation.isValid);
    if config.requireStaticPhysicalAllTimes
        assert(validation.staticPhysicalViolationCount == 0);
        assert(graphs.staticAllTimePhysical);
    else
        assert(validation.staticPhysicalViolationCount > 0);
        assert(~graphs.staticAllTimePhysical);
    end
    assert(validation.staticEdgeCount <= config.edgeBudget);
    assert(maximumSpeed(sensors) <= config.sensorSpeedLimit + 1e-9);
    assert(maximumSpeed(targets) <= config.targetSpeedLimit + 1e-9);
    assert(all(isfinite(headings(:))));
    assertFormationSharedHeadings(config, headings);

    centers = formationCenters(config, sensors);
    displacements = squeeze(centers(:, end, :) - centers(:, 1, :));
    switch config.sceneStyle
        case 'parallel-convoy'
            assert(all(displacements(1, :) > 300));
            assert(max(abs(displacements(2, :))) < 1e-9);
            assert(max(abs(wrapAngle( ...
                headings(:, round(config.simulationLength / 2))))) < 1e-6);
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

testHeadingModesFailClosed();
fprintf('test_multistyle_formation_scenarios passed\n');
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
