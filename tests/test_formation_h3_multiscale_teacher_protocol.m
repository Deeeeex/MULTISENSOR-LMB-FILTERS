function test_formation_h3_multiscale_teacher_protocol()
% TEST_FORMATION_H3_MULTISCALE_TEACHER_PROTOCOL Frozen split and scale test.

protocol = getFormationH3MultiscaleTeacherProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-h3-multiscale-teacher-protocol-v1'));
assert(isequal(protocol.trainingSeeds, [211, 223]));
assert(isequal(protocol.developmentSeeds, 227));
assert(isequal(protocol.snapshotTimes, [60, 72]));
assert(protocol.horizonSteps == 3);
assert(protocol.filterSeedOffset == 100000);
assert(~protocol.featuresUseTruth);
assert(~protocol.featuresUseFutureMeasurements);
assert(protocol.teacherTargetsUseTruth);
assert(protocol.teacherTargetsUseFutureMeasurements);
assert(protocol.openedSentinelDevelopmentOnly);
assert(~protocol.finalModelTrainingAuthorized);
assert(~protocol.validationClaimAllowed);
assert(isempty(intersect(protocol.allSeeds, ...
    protocol.finalValidationSeedsReserved)));

hardwareFields = { ...
    'fovHalfAngleDeg', 'fovTotalAngleDeg', 'fovRange', ...
    'sensorFovHeadingMode', 'detectionProbability', ...
    'measurementNoiseStd', 'sensorQuality', ...
    'sensorHardwareProfile', 'observationSpaceLimits', ...
    'clutterRate', 'clutterSpatialProfile'};
configs = cell(1, numel(protocol.presets));
for presetIdx = 1:numel(protocol.presets)
    configs{presetIdx} = buildDynamicTopologyScenarioConfig( ...
        protocol.presets{presetIdx});
    formationCount = protocol.expectedFormationCounts(presetIdx);
    assert(configs{presetIdx}.formationCount == formationCount);
    assert(configs{presetIdx}.sensorsPerFormation == 6);
    assert(configs{presetIdx}.numberOfSensors == 6 * formationCount);
    assert(protocol.expectedLocalActionCounts(presetIdx) == ...
        1 + 3 * formationCount);
    assert(protocol.expectedPairActionCounts(presetIdx) == ...
        1 + formationCount * (formationCount - 1) / 2);
    assert(protocol.expectedAugmentedActionCounts(presetIdx) == ...
        protocol.expectedLocalActionCounts(presetIdx) + ...
        protocol.expectedPairActionCounts(presetIdx) - 1);
    assert(max(protocol.snapshotTimes) + ...
        protocol.horizonSteps - 1 <= ...
        configs{presetIdx}.simulationLength);
end
for presetIdx = 2:numel(configs)
    for fieldIdx = 1:numel(hardwareFields)
        fieldName = hardwareFields{fieldIdx};
        assert(isequaln(configs{1}.(fieldName), ...
            configs{presetIdx}.(fieldName)));
    end
end
assert(strcmp(configs{1}.scaleControlReferencePreset, ...
    'm24-formation-fov'));
assert(nargin('generateFormationH3ReferenceStateCaches') == 4);
assert(nargin('runFormationH3MultiscaleTeacherShard') == 3);
fprintf('test_formation_h3_multiscale_teacher_protocol passed\n');
end
