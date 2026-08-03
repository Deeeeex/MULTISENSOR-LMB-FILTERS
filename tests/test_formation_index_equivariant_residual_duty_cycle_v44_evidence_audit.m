function test_formation_index_equivariant_residual_duty_cycle_v44_evidence_audit()
audit = ...
    auditFormationIndexEquivariantResidualDutyCycleV44Evidence();
assert(audit.passed);
assert(audit.postHocExecutableFreezeAuditPassed);
assert(audit.routeDevelopmentAdvancePassed);
assert(audit.caseRebuildMatched && audit.gateRebuildMatched);
assert(audit.parentBindingPassed && audit.executableManifestPassed);
assert(strcmp(audit.runnerRecommendedCandidateId, ...
    'formation-all-b2-e20-sp'));
assert(any(strcmp(audit.passingMassMatchedCandidateIds, ...
    'sync-all-b4-e20-mc')));
assert(any(strcmp(audit.passingMassMatchedCandidateIds, ...
    'formation-all-b4-e20-mc')));
assert(~audit.trackingAdvancePassed);
assert(~audit.validationClaimAllowed);
assert(numel(audit.canonicalSha256) == 64);
fprintf('PASS: V44 committed evidence audit test\n');
end
