function protocol = getFormationH3EventConditionedProtocol()
% GETFORMATIONH3EVENTCONDITIONEDPROTOCOL Frozen M24 timing falsifier.

protocol = struct();
protocol.id = 'formation-h3-event-conditioned-sentinel-v1';
protocol.contractVersion = ...
    'formation-h3-event-conditioned-protocol-v1';
protocol.presets = {'m24-formation-fov'};
protocol.expectedFormationCounts = 4;
protocol.expectedLocalActionCounts = 13;
protocol.expectedPairActionCounts = 7;
protocol.expectedAugmentedActionCounts = 19;
protocol.trainingSeeds = [211, 223];
protocol.developmentSeeds = 227;
protocol.allSeeds = [ ...
    protocol.trainingSeeds, protocol.developmentSeeds];
protocol.candidateTimes = 40:4:136;
protocol.snapshotTimes = protocol.candidateTimes;
protocol.selectedStateCount = 2;
protocol.minimumSelectedTimeSeparation = 16;
protocol.eventScoreTailFraction = 0.34;
protocol.eventScoreTailWeight = 0.5;
protocol.eventScoreContractVersion = ...
    'formation-h3-observable-event-score-v1';
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.pairTrustWeight = 0.30;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.manifestRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', ...
    'manifests');
protocol.shardRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'shards');
protocol.minimumPositiveStateCountM24 = 2;
protocol.minimumStrongStateCountM24 = 1;
protocol.strongGainThresholdPercent = 3;
protocol.minimumMeanOracleGainM24Percent = 2;
protocol.featuresUseTruth = false;
protocol.featuresUseFutureMeasurements = false;
protocol.eventSelectionUsesTruth = false;
protocol.eventSelectionUsesFutureMeasurements = false;
protocol.teacherTargetsUseTruth = true;
protocol.teacherTargetsUseFutureMeasurements = true;
protocol.finalValidationSeedsReserved = [251, 257, 263, 269, 271];
protocol.openedSentinelDevelopmentOnly = true;
protocol.finalModelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'This v14 falsifier may use only seeds 211, 223, and 227. ', ...
    'Two times per seed are selected from the fixed-reference ', ...
    'trajectory by a predecision posterior/link score with no truth, ', ...
    'future measurements, or future link outcomes. The v13 action ', ...
    'bank and six-target gate remain unchanged. X36 and final seeds ', ...
    '251, 257, 263, 269, and 271 remain unopened.'];
end
