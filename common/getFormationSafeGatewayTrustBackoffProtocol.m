function protocol = getFormationSafeGatewayTrustBackoffProtocol()
% GETFORMATIONSAFEGATEWAYTRUSTBACKOFFPROTOCOL V33 source-only probe.

protocol = struct();
protocol.id = 'formation-safe-gateway-trust-backoff-v33-v1';
protocol.contractVersion = ...
    'formation-safe-gateway-trust-backoff-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.anchorTime = 72;
protocol.reconnectTime = 74;
protocol.horizonSteps = 3;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.maximumFormationCount = 8;
protocol.dominantWeight = 0.70;
protocol.referenceResidualWeight = 0.05;
protocol.alternativeResidualWeightGrid = [0.0125, 0.025, 0.0375];
protocol.maximumPromisingGatewayCandidates = 16;
protocol.expectedPromisingCandidateIndices = [8, 10, 12];
protocol.expectedRouteSafetyMask = false(3, 3);
protocol.expectedRouteEligibilityMask = false(3, 3);
protocol.expectedReferenceFallback = true;
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
    protocol.maximumPromisingGatewayCandidates * ...
        numel(protocol.alternativeResidualWeightGrid);
protocol.messageCountParityRequired = true;
protocol.formationGraphParityRequired = true;
protocol.dominantRouteParityRequired = true;
protocol.maximumAlternativeResidualWeight = ...
    protocol.referenceResidualWeight;
protocol.selectedRollingConnectivityRequired = true;
protocol.currentPosteriorRequired = true;
protocol.currentLinkReliabilityRequired = true;
protocol.sourceV30ScreenGenerationCommit = ...
    '985120f7945205357a1643574d38e62feaf14063';
protocol.sourceV30AuditCommit = '33c4114';
protocol.sourceV32PreflightGenerationCommit = ...
    '90a3d2ca499b2fa0cd834a55cb3095a6cf3479c5';
protocol.sourceV32EvidenceCommit = '3ef9082';
protocol.sourceV32CandidateBankSha256 = ...
    'd5a53c158fa368a7fcb913e8847c359489c938cec8979191a228e5fbb00221d8';
protocol.sourceCacheSha256 = ...
    ['60dfbf2615181cde046af15f42bba37c415ea0034cb7ce536', ...
     '85b79042bfaf762'];
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v33', 'safe_gateway_trust_backoff');
protocol.openedDevelopmentEvidenceOnly = true;
protocol.trackingOutcomeRerunAuthorized = true;
protocol.additionalM24StateAuthorized = false;
protocol.gnnTrainingAuthorized = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'v33 reuses only the frozen v32 radius-one gateway bank and the ', ...
    'already-opened v30 causal t=74 state. Structural candidates are ', ...
    'preselected only when their frozen weight-0.05 one-round ', ...
    'disagreement improvement is at least 0.25 percent. For those ', ...
    'routes, only the two changed cross inputs are evaluated at the ', ...
    'preregistered lower weights 0.0125, 0.025, and 0.0375; all other ', ...
    'edges, weights, messages, and the formation-level cycle remain ', ...
    'fixed. Exact current-state label retention and disagreement may ', ...
    'select a route, but target truth and later outcomes may not be ', ...
    'read. A paired tracking rerun is authorized only after a clean ', ...
    'preflight finds a nonreference route that passes every safety, ', ...
    'parity, rolling-B3, and disagreement gate. Other M24 states, GNN, ', ...
    'X36, X48, and validation remain closed.'];
end
