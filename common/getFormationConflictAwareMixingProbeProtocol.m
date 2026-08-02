function protocol = getFormationConflictAwareMixingProbeProtocol()
% GETFORMATIONCONFLICTAWAREMIXINGPROBEPROTOCOL Frozen v28 probe.

protocol = struct();
protocol.id = 'formation-conflict-aware-selective-mixing-v28-v1';
protocol.contractVersion = ...
    'formation-conflict-aware-mixing-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.primarySnapshotTime = 72;
protocol.openedSnapshotTimes = [60, 72, 104, 124];
protocol.horizonSteps = 3;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.dominantWeight = 0.70;
protocol.residualWeight = 0.05;
protocol.pairResidualWeight = 0.025;
protocol.dampedReferenceWeight = 0.025;
protocol.pairActionModes = { ...
    'pair-redistribute', 'pair-add-low', 'pair-add-reference'};
protocol.includeReferenceDampActions = true;
protocol.maximumActionCount = 128;
protocol.referenceSupportThreshold = 0.10;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.34;
protocol.retentionReceiverTailWeight = 0.50;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.maximumExpectedDisagreementIncreaseFraction = 0.01;
protocol.maximumIncomingCountForOutcomeEnumeration = 4;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.cacheProtocolId = ...
    'formation-h3-event-conditioned-sentinel-v1';
protocol.cacheGenerationGitCommit = ...
    'c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53';
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v28', 'conflict_aware_selective_mixing');
protocol.primaryEvaluationMode = 'persistent-action-h3';
protocol.messageCountParityRequired = true;
protocol.physicalProjectionRequired = true;
protocol.combinedOneStepStrongConnectivityRequired = true;
protocol.formationStrongConnectivityRequired = true;
protocol.rowStochasticWeightsRequired = true;
protocol.currentPosteriorSafetyProjectionRequired = true;
protocol.meanTrackingGainThresholdPercent = 2.0;
protocol.minimumFormationGainThresholdPercent = 0.0;
protocol.worstSensorGainThresholdPercent = 0.0;
protocol.windowConsensusGainThresholdPercent = 0.0;
protocol.maximumAttemptedPayloadIncreasePercent = 5.0;
protocol.minimumStrongOpenedStateCount = 3;
protocol.minimumOpenedStateCountForLearning = 4;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.finalModelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.finalValidationSeedsReserved = [251, 257, 263, 269, 271];
protocol.evidenceBoundary = [ ...
    'v28 uses current-posterior KLA compatibility only to construct ', ...
    'pairwise candidates, then requires reference-relative existence ', ...
    'retention, expected-cardinality, disagreement, physical, message, ', ...
    'and connectivity guards before a tracking continuation may run. ', ...
    'Thresholds are frozen before the v28 outcome screen. The first ', ...
    'screen reuses only the already-opened seed-211 t=72 M24 state. ', ...
    'Other M24 states, X36, X48, and reserved seeds remain sealed until ', ...
    'the preregistered primary gate passes.'];
end
