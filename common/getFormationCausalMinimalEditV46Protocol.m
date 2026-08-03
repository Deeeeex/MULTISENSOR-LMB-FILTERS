function protocol = getFormationCausalMinimalEditV46Protocol()
% GETFORMATIONCAUSALMINIMALEDITV46PROTOCOL Frozen structural development gate.

parent = getFormationIndexEquivariantRouteV43DevelopmentProtocol();
payload = struct();
payload.id = 'formation-causal-minimal-edit-v46-development-v1';
payload.contractVersion = ...
    'formation-causal-minimal-edit-v46-protocol-v1';
payload.designParentCommit = ...
    'f17c29382c27d891d3a4859e473186a5f74c1fbb';
payload.designParentDecision = ...
    'v45-fixed-registered-backbone-rejected-before-tracking';
payload.parentRouteProtocolId = parent.id;
payload.parentRouteProtocolCanonicalSha256 = parent.canonicalSha256;
payload.registeredPresets = parent.registeredPresets;
payload.registeredSeeds = parent.registeredSeeds;
payload.registeredCartesianCaseCount = ...
    parent.registeredCartesianCaseCount;
payload.primaryArms = { ...
    'v46-repaired-reference-a70-e05', ...
    'v46-repaired-sync-all-b4-e20-mc'};
payload.primaryArmCount = numel(payload.primaryArms);
payload.formationStaggeredArmDeferred = true;
payload.projectionObjective = { ...
    'minimum-removed-registered-pairs', ...
    'minimum-added-nonregistered-pairs', ...
    'maximum-current-bottleneck-reliability', ...
    'maximum-current-total-log-reliability', ...
    'minimum-current-total-distance', ...
    'physical-uid-lexicographic-tie-break'};
payload.projectionExactnessScope = [ ...
    'candidate-topology-exhaustive-within-cap-conditional-on-', ...
    'frozen-v43-realized-assignment'];
payload.projectionCertificateScope = [ ...
    'cross-backbone-connectivity-and-distinct-receiver-', ...
    'matching-only'];
payload.fullV43RouteCompositionRequired = true;
payload.maximumEnumeratedProjectionCandidates = 1e6;
payload.oracleFormationCountLimit = 6;
payload.exactDistinctReceiverMatchingRequired = true;
payload.currentPhysicalFormationConnectivityRequired = true;
payload.noSilentFallback = true;
payload.sensorArrayPermutationEquivarianceRequired = true;
payload.groupRelabelEquivarianceRequired = true;
payload.formationCountDevelopmentLimit = 8;
payload.dominantWeight = 0.70;
payload.referenceResidualWeight = 0.05;
payload.period = 4;
payload.activeResidualWeight = 0.20;
payload.dutyLayer = 'all';
payload.phasePattern = 'synchronized';
payload.maximumIncomingCount = 2;
payload.referenceMessageCountRule = ...
    'two-directed-inputs-per-receiver';
payload.candidateAlignedMessageSavingFraction = 3 / 8;
payload.everyActualRollingB4SensorUnionStrongRequired = true;
payload.everyActualRollingB4FormationUnionStrongRequired = true;
payload.everyAlignedCycleSavingExactRequired = true;
payload.dynamicMatrixProductReauditRequired = true;
payload.perEdgeMassEquivalenceClaimAllowed = false;
payload.full32FullHorizonRoutePreflightRequired = true;
payload.crossingStressIncludedInStructuralGate = true;
payload.realFilterDirectedRuntimeSmokeRequired = true;
payload.physicalUidPairedDeliveryUniformsRequired = true;
payload.runnerSelfAuthorizationAllowed = false;
payload.trackingAdvanceGatePresent = false;
payload.routeReceivesCurrentPageOnly = true;
payload.posteriorUsedByProjection = false;
payload.truthUsedByProjection = false;
payload.measurementUsedByProjection = false;
payload.futurePageUsedByProjection = false;
payload.realizedDeliveryUniformsUsedByProjection = false;
payload.trackingOutcomeScored = false;
payload.trackingOutcomeAuthorized = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '01516bc9b58437b4c7436c1f160706cb313a6886b55b15f758f84fd78b64f848';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationCausalMinimalEditV46:UnregisteredDrift', ...
        ['The v46 structural protocol changed without a version ', ...
         'update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end
