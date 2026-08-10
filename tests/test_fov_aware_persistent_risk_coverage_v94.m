function test_fov_aware_persistent_risk_coverage_v94()
% TEST_FOV_AWAREPERSISTENTRISKCOVERAGEV94 Frozen paired-baseline contract.

protocol = getFovAwarePersistentRiskCoverageV94Protocol();
assert(strcmp(protocol.receiverMode, 'fov-aware-censored'));
assert(strcmp(protocol.baselineName, ...
    'matched-static-fixed-ccw-full-payload'));
assert(protocol.horizonSteps == 3);
assert(protocol.interventionDurationSteps == 3);
assert(protocol.minimumMeanGainPercent == 5);
assert(protocol.requiredAnchorPassCount == 4);
assert(numel(protocol.cases) == 2);
assert(isequal(protocol.cases(1).anchorTimes, [104, 124]));
assert(isequal(protocol.cases(2).anchorTimes, [72, 100]));
assert(isequal(protocol.cases(2).cacheGenerationTimes, [72, 100, 128]));
assert(all([protocol.cases.expectedNodeCount] == [24, 36]));
assert(all([protocol.cases.expectedFormationCount] == [4, 6]));
assert(~protocol.fullEpisodeTrackingAuthorized);
assert(~protocol.validationClaimAllowed);

fprintf('test_fov_aware_persistent_risk_coverage_v94 passed.\n');
end
