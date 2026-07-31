function test_formation_fov_scenarios()
% TEST_FORMATION_FOV_SCENARIOS Focused realistic-FoV scene contract tests.

presets = { ...
    'm24-formation-fov', ...
    'x36-formation-fov', ...
    'x48-formation-fov'};
expectedSensors = [24, 36, 48];
expectedTargets = [16, 24, 32];
validatedMetrics = repmat(struct(), 1, numel(presets));
configs = cell(1, numel(presets));

for presetIdx = 1:numel(presets)
    rng(41);
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    configs{presetIdx} = config;
    [sensorTrajectories, ~] = ...
        generateMultiFormationTrajectories(config);
    [targetTrajectories, ~] = ...
        generateCorridorTargetTrajectories(config);
    graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
    validation = validateDynamicTopologyScenario( ...
        config, sensorTrajectories, targetTrajectories, graphData);
    assert(validation.isValid);
    assert(config.numberOfSensors == expectedSensors(presetIdx));
    assert(config.numberOfTargets == expectedTargets(presetIdx));
    assert(config.fovHalfAngleDeg == 75);
    assert(config.fovTotalAngleDeg == 150);
    assert(strcmp(config.sensorFovHeadingMode, ...
        'formation-shared-scene-center'));
    assert(validation.staticEdgeCount <= config.edgeBudget);
    assert(validation.staticPhysicalViolationCount == 0);

    headings = buildSensorFovHeadingSchedule( ...
        config, sensorTrajectories);
    assert(all(isfinite(headings(:))));
    assertFormationHeadingsShared( ...
        config, sensorTrajectories, headings);
    validatedMetrics(presetIdx) = validation.difficulty;

    scan = scanFormationFovSensingGeometry( ...
        presets{presetIdx}, 41, config.fovRange, ...
        config.sensorQuality.referenceRange);
    assertScannerMatchesValidation( ...
        scan.records, validation.difficulty);
end

matchedFields = { ...
    'focusMeanVisibleTargetsPerSensorTime'};
derivedFields = {'focusMeanVisibleSensorCount'};
reference = validatedMetrics(1);
for presetIdx = 2:numel(presets)
    config = configs{presetIdx};
    assert(strcmp(config.scaleControlReferencePreset, ...
        'm24-formation-fov'));
    assert(isequal(config.scaleControlMatchedMetrics, matchedFields));
    assert(isequal(config.scaleControlDerivedMetrics, derivedFields));
    for fieldIdx = 1:numel(matchedFields)
        fieldName = matchedFields{fieldIdx};
        relativeError = abs( ...
            validatedMetrics(presetIdx).(fieldName) / ...
            reference.(fieldName) - 1);
        assert(relativeError <= 0.03);
    end
end
hardwareFields = { ...
    'fovHalfAngleDeg', 'fovTotalAngleDeg', 'fovRange', ...
    'sensorFovHeadingMode', 'sensorFovPointingCenter', ...
    'detectionProbability', 'measurementNoiseStd', ...
    'sensorQuality', 'sensorHardwareProfile', ...
    'observationSpaceLimits', 'clutterRate', ...
    'clutterSpatialProfile'};
for presetIdx = 2:numel(configs)
    for fieldIdx = 1:numel(hardwareFields)
        fieldName = hardwareFields{fieldIdx};
        assert(isequaln(configs{1}.(fieldName), ...
            configs{presetIdx}.(fieldName)));
    end
end
assert(validatedMetrics(1).focusMeanExpectedTargetDetectionCount > ...
    validatedMetrics(2).focusMeanExpectedTargetDetectionCount);
assert(validatedMetrics(2).focusMeanExpectedTargetDetectionCount > ...
    validatedMetrics(3).focusMeanExpectedTargetDetectionCount);

invalidLimits = configs{1}.regionLimits;
invalidLimits(1, :) = [-100, 100];
failedClosed = false;
try
    buildDynamicTopologyScenarioConfig('m24-formation-fov', ...
        struct('observationSpaceLimits', invalidLimits));
catch errorInfo
    failedClosed = ~isempty(strfind( ...
        errorInfo.message, 'must enclose')); %#ok<STREMP>
end
assert(failedClosed);

config = rmfield(config, 'sensorGroupIds');
failedClosed = false;
try
    buildSensorFovHeadingSchedule(config, sensorTrajectories);
catch errorInfo
    failedClosed = ~isempty(strfind( ...
        errorInfo.message, 'sensorGroupIds')); %#ok<STREMP>
end
assert(failedClosed);
fprintf('test_formation_fov_scenarios passed\n');
end

function assertFormationHeadingsShared( ...
        config, sensorTrajectories, headings)
for groupIdx = 1:config.formationCount
    groupSensors = find(config.sensorGroupIds == groupIdx);
    referenceHeading = headings(groupSensors(1), :);
    differences = bsxfun(@minus, ...
        headings(groupSensors, :), referenceHeading);
    wrappedDifferences = atan2(sin(differences), cos(differences));
    assert(max(abs(wrappedDifferences(:))) < 1e-12);

    centerPositions = zeros(2, config.simulationLength);
    for sensorIdx = reshape(groupSensors, 1, [])
        centerPositions = centerPositions + ...
            sensorTrajectories{sensorIdx}(1:2, :);
    end
    centerPositions = centerPositions / numel(groupSensors);
    expectedHeading = atan2( ...
        config.sensorFovPointingCenter(2) - centerPositions(2, :), ...
        config.sensorFovPointingCenter(1) - centerPositions(1, :));
    errorAngle = atan2(sin(referenceHeading - expectedHeading), ...
        cos(referenceHeading - expectedHeading));
    assert(max(abs(errorAngle)) < 1e-12);
end
end

function assertScannerMatchesValidation(scan, validation)
fields = { ...
    'focusMeanVisibleSensorCount', ...
    'focusMeanExpectedTargetDetectionCount', ...
    'focusMeanVisibleTargetsPerSensorTime', ...
    'blackoutFraction', ...
    'singleFormationFraction', ...
    'multiFormationFraction', ...
    'formationOwnershipEntropy'};
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    assert(abs(scan.(fieldName) - validation.(fieldName)) < 1e-10);
end
assert(scan.focusHandovers == validation.focusHandovers);
end
