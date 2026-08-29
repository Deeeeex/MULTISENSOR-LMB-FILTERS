function test_target_group_overlap_split_scenarios()
% TEST_TARGET_GROUP_OVERLAP_SPLIT_SCENARIOS Isolated target-flow geometry.

presets = { ...
    'm24-formation-fov-target-overlap', ...
    'x36-formation-fov-target-overlap'};
relayPresets = { ...
    'm24-formation-fov-relay', ...
    'x36-formation-fov-relay'};
expectedSensors = [24, 36];
expectedTargets = [16, 24];

for presetIdx = 1:numel(presets)
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    relay = buildDynamicTopologyScenarioConfig(relayPresets{presetIdx});
    assert(strcmp(config.sceneStyle, 'target-group-overlap-split'));
    assert(strcmp(config.sceneCalibrationStatus, 'development-only'));
    assert(~config.formalValidationAuthorized && ...
        ~config.trackingOutcomeAuthorized);
    assert(config.numberOfSensors == expectedSensors(presetIdx));
    assert(config.numberOfTargets == expectedTargets(presetIdx));
    assert(config.fovTotalAngleDeg == 120 && config.fovRange == 300);
    assert(strcmp(config.formationHeadingMode, 'fixed'));
    assert(strcmp(config.sensorFovHeadingMode, ...
        'formation-shared-fixed'));
    assert(isequal(config.sensorCenterWaypoints, ...
        relay.sensorCenterWaypoints));

    rng(41);
    [sensorTrajectories, ~] = ...
        generateMultiFormationTrajectories(config);
    [targetTrajectories, ~] = ...
        generateCorridorTargetTrajectories(config);
    graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
    validation = validateDynamicTopologyScenario( ...
        config, sensorTrajectories, targetTrajectories, graphData);
    assert(validation.isValid);
    assert(validation.minimumTargetSeparation + 1e-9 >= ...
        config.minimumTargetSeparation);
    assert(validation.minimumSensorTargetSeparation + 1e-9 >= ...
        config.minimumSensorTargetSeparation);
    assert(validation.staticPhysicalViolationCount == 0);

    maximumSensorSpeed = max(cellfun(@(state) ...
        max(sqrt(sum(state(3:4, :).^2, 1))), ...
        sensorTrajectories));
    assert(maximumSensorSpeed <= 1e-12);

    groupCenters = targetGroupCenters(config, targetTrajectories);
    cohortSize = config.targetGroupCount / 2;
    firstCohort = 1:cohortSize;
    secondCohort = cohortSize + (1:cohortSize);
    yDifference = mean(groupCenters(2, firstCohort, :), 2) - ...
        mean(groupCenters(2, secondCohort, :), 2);
    yDifference = reshape(yDifference, 1, []);
    assert(abs(yDifference(1)) >= 80);
    assert(abs(yDifference(end)) >= 80);
    assert(yDifference(1) * yDifference(end) < 0);
    assert(min(abs(yDifference(70:91))) <= 3);
end

fprintf('test_target_group_overlap_split_scenarios passed\n');
end

function centers = targetGroupCenters(config, targetTrajectories)
centers = zeros(2, config.targetGroupCount, config.simulationLength);
for groupIdx = 1:config.targetGroupCount
    targetIndices = find(config.targetGroupIds == groupIdx);
    positions = zeros(2, numel(targetIndices), config.simulationLength);
    for localIdx = 1:numel(targetIndices)
        positions(:, localIdx, :) = reshape( ...
            targetTrajectories{targetIndices(localIdx)}(1:2, :), ...
            2, 1, config.simulationLength);
    end
    centers(:, groupIdx, :) = mean(positions, 2);
end
end
