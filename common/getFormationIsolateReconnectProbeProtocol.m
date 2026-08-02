function protocol = getFormationIsolateReconnectProbeProtocol()
% GETFORMATIONISOLATERECONNECTPROBEPROTOCOL Frozen v30 development probe.

protocol = struct();
protocol.id = 'formation-retention-debt-v30-v1';
protocol.contractVersion = ...
    'formation-retention-debt-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.primarySnapshotTime = 72;
protocol.openedSnapshotTimes = [60, 72, 104, 124];
protocol.horizonSteps = 3;
protocol.isolationDurationSteps = 1;
protocol.recoveryDurationSteps = 2;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.maximumFormationCount = 8;
protocol.dominantWeight = 0.70;
protocol.residualWeight = 0.05;
protocol.referenceOrientation = 'counter-clockwise';
protocol.maximumIncomingCountForOutcomeEnumeration = 4;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.referenceSupportThreshold = 0.05;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.25;
protocol.retentionReceiverTailWeight = 0.50;
protocol.retentionDebtOnFraction = 0.02;
protocol.retentionDebtOffFraction = 0.01;
protocol.minimumReferenceFormationExpectedCardinality = 1.0;
protocol.maximumControlRouteEvaluations = ...
    2 * protocol.maximumFormationCount + 1;
protocol.expectedPrimarySelectedFormationIds = [2, 3, 4];
protocol.recoveryReferenceFallbackRequired = true;
protocol.recoveryOneStepStrongConnectivityRequired = false;
protocol.recoveryCurrentPosteriorRequired = true;
protocol.recoveryCurrentLinkReliabilityRequired = true;
protocol.meanTrackingGainThresholdPercent = 2.0;
protocol.minimumFormationGainThresholdPercent = 0.0;
protocol.worstSensorGainThresholdPercent = 0.0;
protocol.windowConsensusGainThresholdPercent = 0.0;
protocol.terminalConsensusGainThresholdPercent = 0.0;
protocol.minimumAttemptedByteSavingPercent = 0.0;
protocol.selectedRollingConnectivityRequired = true;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.cacheProtocolId = ...
    'formation-h3-event-conditioned-sentinel-v1';
protocol.cacheGenerationGitCommit = ...
    'c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53';
protocol.sourceV29ScreenCommit = ...
    '7e6e61bcc6efc8d85de5dc4c6e3f152dac06d481';
protocol.sourceCacheSha256 = ...
    ['60dfbf2615181cde046af15f42bba37c415ea0034cb7ce536', ...
     '85b79042bfaf762'];
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v30', 'causal_isolate_reconnect');
protocol.openedDevelopmentEvidenceOnly = true;
protocol.additionalOpenedStateScreenAuthorized = false;
protocol.finalModelTrainingAuthorized = false;
protocol.gnnTrainingAuthorized = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'v30 reuses only the already-opened seed-211 t=72 M24 state. ', ...
    'At every selected step, the controller estimates each formation''s ', ...
    'counterfactual expected-cardinality recovery after withholding its ', ...
    'registered cross input. A two-threshold hysteresis rule requests ', ...
    'suspensions, and the joint action is projected through the frozen ', ...
    'reference-relative existence and rolling-B3 constraints. The ', ...
    'controller reads only the current posterior, physical graph, link ', ...
    'reliability, and selected-topology history; it reads no truth or ', ...
    'future outcome. Offline truth scores the H=3 development outcome ', ...
    'only. Other M24 states, GNN training, X36, X48, and reserved ', ...
    'validation remain sealed until the primary strong gate passes.'];
end
