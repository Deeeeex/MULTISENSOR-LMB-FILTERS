function protocol = getFormationStaggeredRecoveryProbeProtocol()
% GETFORMATIONSTAGGEREDRECOVERYPROBEPROTOCOL V35 causal recovery probe.

protocol = struct();
protocol.id = 'formation-staggered-recovery-v35-v1';
protocol.contractVersion = ...
    'formation-staggered-recovery-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.anchorTime = 72;
protocol.staggerTime = 73;
protocol.reconnectTime = 74;
protocol.horizonSteps = 3;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.maximumFormationCount = 8;
protocol.dominantWeight = 0.70;
protocol.residualWeight = 0.05;
protocol.minimumSuspensionAgeSteps = 1;
protocol.minimumRetainedDebtCoverageFraction = 0.80;
protocol.minimumIncumbentDisagreementImprovementFraction = 0.0025;
protocol.minimumMessageSavingCount = 1;
protocol.maximumIncomingCountForOutcomeEnumeration = 4;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.referenceSupportThreshold = 0.05;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.25;
protocol.retentionReceiverTailWeight = 0.50;
protocol.maximumControlRouteEvaluations = ...
    3 * protocol.maximumFormationCount + 1;
protocol.expectedBaseSelectedFormationIds = [2, 3, 4];
protocol.expectedMatureFormationIds = [2, 3, 4];
protocol.expectedReleaseOrder = [3, 4, 2];
protocol.expectedCandidateRetainedFormationIds = {[2, 4]};
protocol.expectedSelectedFormationIds = [2, 4];
protocol.expectedCandidateSafetyMask = true;
protocol.expectedCandidateEligibilityMask = true;
protocol.expectedReferenceFallback = false;
protocol.expectedStaggeredReleaseUsed = true;
protocol.expectedRuntimeSelectedFormationIdsByTime = { ...
    [2, 3, 4], [2, 4], 3};
protocol.expectedRuntimeReleasedFormationIdsByTime = { ...
    zeros(1, 0), 3, zeros(1, 0)};
protocol.expectedRuntimeStaggeredReleaseMask = ...
    logical([0, 1, 0]);
protocol.expectedRetainedDebtCoverageFraction = ...
    0.89138853069654445;
protocol.expectedIncumbentNetworkRisk = ...
    1.9796200101269477;
protocol.expectedSelectedNetworkRisk = ...
    1.9648362762801086;
protocol.expectedDisagreementImprovementFraction = ...
    0.0074679654535776407;
protocol.expectedSelectedRetentionRisk = ...
    0.00085488857989502755;
protocol.sourceMetricTolerance = 5e-12;
protocol.meanTrackingGainThresholdPercent = 2.0;
protocol.minimumFormationGainThresholdPercent = 0.0;
protocol.worstSensorGainThresholdPercent = 0.0;
protocol.windowConsensusGainThresholdPercent = 0.0;
protocol.terminalConsensusGainThresholdPercent = 0.0;
protocol.minimumAttemptedByteSavingPercent = 0.0;
protocol.sourceV30ScreenGenerationCommit = ...
    '985120f7945205357a1643574d38e62feaf14063';
protocol.sourceV30AuditCommit = '33c4114';
protocol.sourceCacheSha256 = ...
    ['60dfbf2615181cde046af15f42bba37c415ea0034cb7ce536', ...
     '85b79042bfaf762'];
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v35', 'staggered_recovery');
protocol.trackingOutcomeRerunAuthorized = true;
protocol.additionalM24StateAuthorized = false;
protocol.gnnTrainingAuthorized = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'v35 reuses only the frozen v30 M24 seed-211 t=72 trajectory. ', ...
    'At t=73 it ranks previously suspended formations by current ', ...
    'truth-free retention debt, proposes nested low-debt releases ', ...
    'while preserving at least 80 percent of the incumbent positive ', ...
    'debt, and accepts only an exact label-safe, rolling-B3-safe route ', ...
    'that improves one-round disagreement by at least 0.25 percent ', ...
    'relative to continued suspension. Truth and future outcomes are ', ...
    'sealed while a full source-only H=3 rollout verifies the causal ', ...
    '[2,3,4] to [2,4] to [3] runtime trace. Only then may a clean ', ...
    'preflight authorize one paired H=3 rerun. ', ...
    'No additional M24 state, GNN, X36, X48, or validation is opened.'];
end
