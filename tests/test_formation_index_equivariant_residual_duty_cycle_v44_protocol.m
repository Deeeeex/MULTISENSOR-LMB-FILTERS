function test_formation_index_equivariant_residual_duty_cycle_v44_protocol()
% Frozen V44 protocol and claim-boundary contracts.

protocol = ...
    getFormationIndexEquivariantResidualDutyCycleV44Protocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-index-equivariant-residual-duty-cycle-v44-protocol-v1'));
assert(strcmp(protocol.canonicalSha256, ...
    '6b26fcec9238df591a6c721229911e9328daf7ad822315539604cdcb7f2a2339'));
assert(protocol.registeredCartesianCaseCount == 32);
assert(protocol.candidateCount == 11);
assert(isequal([protocol.candidates.ordinal], 1:11));
assert(isequal(unique([protocol.candidates.period]), [2, 3, 4]));
assert(all(strcmp({protocol.candidates.dutyLayer}, 'all')));
assert(all(ismember({protocol.candidates.phasePattern}, ...
    {'synchronized', 'formation-staggered'})));
assert(protocol.minimumActiveSelfWeight == 0.10);
assert(protocol.minimumPeriodMessageSavingFraction == 0.25);
assert(protocol.everyStartPhaseRequired);
assert(protocol.everyRollingPeriodUnionMustEqualV43);
assert(protocol.referenceReproductionRequired);
assert(protocol.crossingIncludedInAdvanceGate);
assert(~protocol.runnerSelfAuthorizationAllowed);
assert(protocol.postHocExecutableFreezeAuditRequired);
assert(~protocol.formalRuntimeObservableBoundaryPassed);
assert(~protocol.runtimePhysicalUidRegistryIntegrated);
assert(~protocol.posteriorUsed && ~protocol.truthUsed && ...
    ~protocol.measurementUsed && ~protocol.futureOutcomeUsed && ...
    ~protocol.realizedDeliveryUniformsUsed && ...
    ~protocol.trackingOutcomeScored && ...
    ~protocol.trackingOutcomeAuthorized && ...
    ~protocol.validationClaimAllowed && ...
    protocol.developmentEvidenceOnly);
fprintf('PASS: V44 residual duty-cycle protocol tests\n');
end
