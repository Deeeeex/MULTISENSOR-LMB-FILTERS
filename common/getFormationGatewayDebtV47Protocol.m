function protocol = getFormationGatewayDebtV47Protocol()
% GETFORMATIONGATEWAYDEBTV47PROTOCOL Frozen causal gateway-debt design.

parent = getFormationCausalMinimalEditV46Protocol();
payload = struct();
payload.id = 'formation-gateway-debt-v47-development-v1';
payload.contractVersion = 'formation-gateway-debt-v47-protocol-v1';
payload.designParentCommit = ...
    'c58913116eb4401d687eedad42900c1b7fdaea63';
payload.parentProtocolId = parent.id;
payload.parentProtocolCanonicalSha256 = parent.canonicalSha256;
payload.primaryArms = { ...
    'v46-repaired-reference-a70-e05', ...
    'v47-repaired-gateway-debt-b4-e20'};
payload.period = 4;
payload.dominantWeight = 0.70;
payload.referenceResidualWeight = 0.05;
payload.maximumActiveResidualWeight = 0.20;
payload.residualQuotaRule = ...
    'floor-phase-cumulative-n-over-four';
payload.exactNoFallbackMessageSavingFraction = 3 / 8;
payload.dominantLayerAlwaysAttempted = true;
payload.crossFormationResidualPriority = true;
payload.crossServiceHorizonRule = ...
    'ceil-current-cross-residual-count-over-minimum-phase-quota';
payload.deadlineFirstProjection = true;
payload.deliveryAgeScoreWeight = 0.45;
payload.posteriorDisagreementScoreWeight = 0.35;
payload.currentReliabilityScoreWeight = 0.20;
payload.scoreWeightSum = payload.deliveryAgeScoreWeight + ...
    payload.posteriorDisagreementScoreWeight + ...
    payload.currentReliabilityScoreWeight;
payload.posteriorDisagreementDefinition = ...
    'truth-free-labelwise-existence-and-normalized-spatial-moment';
payload.residualWeightRule = ...
    'reference-weight-divided-by-current-class-duty-rate-capped-at-e20';
payload.referenceFallbackOnMatureRollingFormationFailure = true;
payload.observableContractVersion = ...
    'topology-policy-observable-input-v4-physical-uid-delivery-history';
payload.currentPosteriorUsed = true;
payload.pastSelectedTopologyUsed = true;
payload.pastSuccessfulDeliveryUsed = true;
payload.currentLinkReliabilityUsed = true;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.futurePageUsed = false;
payload.futureOutcomeUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.perEdgeMassEquivalenceClaimAllowed = false;
payload.trackingOutcomeScored = false;
payload.trackingOutcomeAuthorized = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '5416e417627a0670a9a913c38850962aa5e1f36d9188def3915ff2ddbbb5c382';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationGatewayDebtV47:UnregisteredDrift', ...
        ['The V47 gateway-debt protocol changed without a version ', ...
         'update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end
