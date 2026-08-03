function protocol = ...
    getFormationReliableKlaIndexEquivarianceV42Protocol()
% GETFORMATIONRELIABLEKLAINDEXEQUIVARIANCEV42PROTOCOL
% Frozen diagnostic for arbitrary formation-block/node ordering effects.

base = getReliableKlaWindowContractionProtocol();
payload = struct();
payload.id = 'formation-reliable-kla-index-equivariance-v42-v1';
payload.contractVersion = ...
    'formation-reliable-kla-index-equivariance-protocol-v1';
payload.baseProtocolId = base.id;
payload.baseProtocolCanonicalSha256 = base.canonicalSha256;
payload.registeredPresets = { ...
    'm24-formation-fov', 'x36-formation-fov'};
payload.registeredSeeds = [41, 43, 47, 53];
payload.registeredCartesianCaseCount = 8;
payload.subsetRunAllowedForSmoke = true;
payload.registeredBatchCompletenessRequiredForRegisteredConclusion = true;
payload.permutationFamily = ...
    'all-cyclic-complete-formation-block-node-coordinate-orders';
payload.baselineShift = 0;
payload.fullFormationBlockPermutationFamilyCovered = false;
payload.withinFormationSensorPermutationCovered = false;
payload.formationLabelsChanged = false;
payload.physicalFormationMembershipChanged = false;
payload.sensorGeometryChanged = false;
payload.linkReliabilityChanged = false;
payload.posteriorChanged = false;
payload.previousRouteHistoryChanged = false;
payload.coordinateRestoreRule = ...
    'restored-new-to-old-rows-and-columns';
payload.candidateIdentityRule = ...
    'compare-by-unchanged-physical-formation-label';
payload.referenceAdjacencyEqualityRequired = true;
payload.candidateAdjacencyEqualityRequired = true;
payload.candidateAvailabilityEqualityRequired = true;
payload.adaptiveHorizonEqualityRequired = true;
payload.fusionWeightAbsoluteTolerance = 1e-12;
payload.meanSquareFactorAbsoluteTolerance = 1e-12;
payload.bestCandidateTieAbsoluteTolerance = 1e-12;
payload.testedCyclicBlockOrderEquivarianceRequiredForFurtherAudit = true;
payload.physicalActionInterpretationAuthorized = false;
payload.failureInterpretation = [ ...
    'classify-reference-route,candidate-construction,adaptive-horizon,', ...
    'or-certificate-index-order-confounding;', ...
    'no-physical-formation-value-claim'];
payload.passInterpretation = [ ...
    'tested-cyclic-block-order-confound-not-detected;', ...
    'tracking-value-still-not-established'];
payload.fullPlannedSensorGeometryMaterialized = true;
payload.fullPlannedLinkProbabilityScheduleMaterialized = true;
payload.formalRuntimeObservableBoundaryPassed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.posteriorUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.trackingOutcomeScored = false;
payload.developmentEvidenceOnly = true;
payload.m24TrackingAuthorized = false;
payload.x36TrackingAuthorized = false;
payload.validationClaimAllowed = false;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '7572ebcc50e3080ac4e67a3e6133f52720463248535a15c85e65e0dbbed80ed4';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationKlaIndexEquivariance:UnregisteredDrift', [ ...
        'The v42 protocol changed without a version update: ', ...
        'actual=%s expected=%s.'], actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end
