function test_formation_index_equivariant_route_v43_protocol()
protocol = getFormationIndexEquivariantRouteV43DevelopmentProtocol();
assert(strcmp(protocol.id, ...
    'formation-index-equivariant-route-v43-development-v1'));
assert(strcmp(protocol.canonicalSha256, ...
    '4eae77976aa8b5b6cd2722b09a3909dbf8b2bce45eb2e464a54ed45a1c84aa03'));
assert(strcmp(protocol.methodCommit, ...
    '30580c794f19573f6101457c405861da8853c35a'));
assert(numel(protocol.registeredPresets) == 8);
assert(numel(protocol.registeredSeeds) == 4);
assert(protocol.registeredCartesianCaseCount == 32);
assert(numel(protocol.registeredCases) == 32);
assert(protocol.caseInputIdentitiesFrozen);
assert(numel(unique({protocol.registeredCases.caseId})) == 32);
assert(all(cellfun(@(value) numel(value) == 64, ...
    {protocol.registeredCases.inputCanonicalSha256})));
assert(all(cellfun(@(value) numel(value) == 64, ...
    {protocol.registeredCases.physicalUidRegistryCanonicalSha256})));
assert(protocol.sameDirectedMessageCountRequired);
assert(protocol.sameCrossFormationMessageCountRequired);
assert(protocol.sameFusionWeightsRequired);
assert(protocol.protocolWeightsMustBeApplied);
assert(protocol.scaleMedianTargetHorizonRatioMaximum == 0.50);
assert(~protocol.crossingIncludedInScaleGate);
assert(protocol.harmfulSparseActionMustFallbackReference);
assert(protocol.officialCleanTrackedSourceRequired);
assert(protocol.postHocExecutableFreezeAuditRequired);
assert(~protocol.runnerSelfAuthorizationAllowed);
assert(~protocol.trackingAdvanceGatePresent);
assert(protocol.fullPlannedSensorGeometryMaterialized);
assert(protocol.fullPlannedLinkProbabilityScheduleMaterialized);
assert(~protocol.formalRuntimeObservableBoundaryPassed);
assert(~protocol.runtimePhysicalUidRegistryIntegrated);
assert(~protocol.posteriorUsed && ~protocol.truthUsed);
assert(~protocol.trackingOutcomeScored);
assert(~protocol.trackingOutcomeAuthorized);
assert(~protocol.validationClaimAllowed);
assert(protocol.developmentEvidenceOnly);
fprintf('PASS: v43 route development protocol tests\n');
end
