function protocol = getFormationH3MultiscaleTeacherProtocol()
% GETFORMATIONH3MULTISCALETEACHERPROTOCOL Frozen v13 sentinel design.

protocol = struct();
protocol.id = 'formation-h3-multiscale-teacher-sentinel-v1';
protocol.contractVersion = ...
    'formation-h3-multiscale-teacher-protocol-v1';
protocol.presets = { ...
    'd12-formation-fov', ...
    'm24-formation-fov', ...
    'x36-formation-fov'};
protocol.expectedFormationCounts = [2, 4, 6];
protocol.expectedLocalActionCounts = [7, 13, 19];
protocol.expectedPairActionCounts = [2, 7, 16];
protocol.expectedAugmentedActionCounts = [8, 19, 34];
protocol.trainingSeeds = [211, 223];
protocol.developmentSeeds = 227;
protocol.allSeeds = [ ...
    protocol.trainingSeeds, protocol.developmentSeeds];
protocol.snapshotTimes = [60, 72];
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.pairTrustWeight = 0.30;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v13', 'multiscale_teacher_sentinel', 'cache');
protocol.shardRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v13', 'multiscale_teacher_sentinel', 'shards');
protocol.minimumPositiveStateCountPerScale = 2;
protocol.minimumStrongStateCountM24X36 = 1;
protocol.strongGainThresholdPercent = 3;
protocol.minimumMeanOracleGainM24X36Percent = 2;
protocol.minimumDevelopmentMeanGainPearson = 0.20;
protocol.minimumDevelopmentTop3CaptureFraction = 0.50;
protocol.minimumDevelopmentSafeSelectionFraction = 0.80;
protocol.featuresUseTruth = false;
protocol.featuresUseFutureMeasurements = false;
protocol.teacherTargetsUseTruth = true;
protocol.teacherTargetsUseFutureMeasurements = true;
protocol.finalValidationSeedsReserved = [251, 257, 263, 269, 271];
protocol.openedSentinelDevelopmentOnly = true;
protocol.finalModelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'Seeds 211 and 223 are sentinel training states and seed 227 ', ...
    'is sentinel development. Times 60 and 72 are opened by this ', ...
    'protocol. The sentinel may select dataset scale, feature, and ', ...
    'model direction only. Seeds 251, 257, 263, 269, and 271 remain ', ...
    'reserved and must not be opened before the final policy is frozen.'];
end
