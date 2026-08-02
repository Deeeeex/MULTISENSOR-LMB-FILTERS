function protocol = getFormationAlternativeGatewayProbeProtocol()
% GETFORMATIONALTERNATEGATEWAYPROBEPROTOCOL V32 source-only probe.

protocol = struct();
protocol.id = 'formation-label-compatible-gateway-v32-v1';
protocol.contractVersion = ...
    'formation-alternative-gateway-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.anchorTime = 72;
protocol.reconnectTime = 74;
protocol.horizonSteps = 3;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.maximumFormationCount = 8;
protocol.dominantWeight = 0.70;
protocol.residualWeight = 0.05;
protocol.formationOrientation = 'counter-clockwise';
protocol.maximumGatewayCandidates = 16;
protocol.maximumGatewayCutChanges = 1;
protocol.compatibilitySourceWeight = 0.05;
protocol.compatibilityNoveltyWeight = 0.10;
protocol.compatibilityPrecisionPenaltyWeight = 0.10;
protocol.maximumIncomingCountForOutcomeEnumeration = 4;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.referenceSupportThreshold = 0.05;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.25;
protocol.retentionReceiverTailWeight = 0.50;
protocol.minimumDisagreementImprovementFraction = 0.0025;
protocol.maximumControlRouteEvaluations = ...
    1 + protocol.maximumGatewayCandidates;
protocol.expectedCandidateBankSha256 = ...
    'd5a53c158fa368a7fcb913e8847c359489c938cec8979191a228e5fbb00221d8';
protocol.expectedCandidateCount = 12;
protocol.expectedCandidateSafetyMask = false(1, 12);
protocol.expectedCandidateEligibilityMask = false(1, 12);
protocol.expectedReferenceFallback = true;
protocol.referenceFallbackRequired = true;
protocol.messageCountParityRequired = true;
protocol.formationGraphParityRequired = true;
protocol.residualWeightParityRequired = true;
protocol.selectedRollingConnectivityRequired = true;
protocol.currentPosteriorRequired = true;
protocol.currentLinkReliabilityRequired = true;
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
    'formation_value_v32', 'label_compatible_gateway');
protocol.openedDevelopmentEvidenceOnly = true;
protocol.trackingOutcomeRerunAuthorized = true;
protocol.additionalM24StateAuthorized = false;
protocol.gnnTrainingAuthorized = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'v32 may replay only the already-opened v30 controller trajectory ', ...
    'through t=74. It keeps the registered formation-level cycle, ', ...
    'dominant route, residual weight, and directed message count fixed, ', ...
    'and ranks at most sixteen reference-anchored sensor-gateway ', ...
    'realizations with at most one formation cut change, using only ', ...
    'current label-wise KLA compatibility and current link reliability. ', ...
    'Exact one-round disagreement and reference-relative existence ', ...
    'retention may project this proposal bank, but target truth and ', ...
    'later outcomes may not be read. A tracking rerun is permitted only ', ...
    'after a clean frozen preflight selects a nonreference route that ', ...
    'passes every safety, parity, rolling-B3, and 0.25-percent ', ...
    'disagreement-improvement gate. No other M24 state, GNN, X36, or ', ...
    'X48 is opened by this probe.'];
end
