function protocol = getFormationTemporalSuspensionProbeProtocol()
% GETFORMATIONTEMPORALSUSPENSIONPROBEPROTOCOL Frozen v29 probe.

protocol = struct();
protocol.id = 'formation-temporal-cross-edge-suspension-v29-v1';
protocol.contractVersion = ...
    'formation-temporal-suspension-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.primarySnapshotTime = 72;
protocol.openedSnapshotTimes = [60, 72, 104, 124];
protocol.horizonSteps = 3;
protocol.interventionDurationSteps = 1;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.dominantWeight = 0.70;
protocol.residualWeight = 0.05;
protocol.maximumActionCount = 256;
protocol.referenceSupportThreshold = 0.10;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.34;
protocol.retentionReceiverTailWeight = 0.50;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.maximumIncomingCountForOutcomeEnumeration = 4;
protocol.oneStepDisagreementHardGateEnabled = false;
protocol.rollingWindowLength = 3;
protocol.selectedRollingConnectivityRequired = true;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.cacheProtocolId = ...
    'formation-h3-event-conditioned-sentinel-v1';
protocol.cacheGenerationGitCommit = ...
    'c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53';
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v29', 'temporal_cross_edge_suspension');
protocol.primaryEvaluationMode = ...
    'one-step-suspension-two-step-reference-h3';
protocol.physicalProjectionRequired = true;
protocol.rowStochasticWeightsRequired = true;
protocol.currentPosteriorRetentionProjectionRequired = true;
protocol.meanTrackingGainThresholdPercent = 2.0;
protocol.minimumFormationGainThresholdPercent = 0.0;
protocol.worstSensorGainThresholdPercent = 0.0;
protocol.windowConsensusGainThresholdPercent = 0.0;
protocol.minimumAttemptedByteSavingPercent = 0.0;
protocol.minimumStrongOpenedStateCount = 3;
protocol.minimumOpenedStateCountForLearning = 4;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.finalModelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.finalValidationSeedsReserved = [251, 257, 263, 269, 271];
protocol.evidenceBoundary = [ ...
    'v29 removes selected registered cross-formation payloads for one ', ...
    'step and then executes the fixed reference for two steps. Candidate ', ...
    'subsets use no truth or future outcomes and must pass reference-', ...
    'relative existence retention, expected-cardinality, physical, ', ...
    'row-stochastic, lower-message-count, and selected rolling-B3 ', ...
    'constraints. One-step posterior disagreement is diagnostic only ', ...
    'because v27/v28 falsified it as a recursive safety certificate. ', ...
    'The primary screen reuses only seed-211 t=72; other M24 states, ', ...
    'X36, X48, and reserved seeds remain sealed until its gate passes.'];
end
