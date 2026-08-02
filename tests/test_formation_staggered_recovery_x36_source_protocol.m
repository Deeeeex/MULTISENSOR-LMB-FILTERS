function test_formation_staggered_recovery_x36_source_protocol()
% TEST_FORMATIONSTAGGEREDRECOVERYX36SOURCEPROTOCOL V37 contract.

protocol = getFormationStaggeredRecoveryX36SourceProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-staggered-recovery-x36-source-protocol-v1'));
assert(strcmp(protocol.presetName, 'x36-formation-fov'));
assert(protocol.seed == 211);
assert(isequal(protocol.anchorTimes, [72, 100, 128]));
assert(isequal(protocol.blockageFormationPairs, ...
    [1, 2; 3, 4; 5, 6]));
assert(protocol.expectedNodeCount == 36);
assert(protocol.expectedFormationCount == 6);
assert(protocol.expectedTargetCount == 24);
assert(protocol.expectedFovTotalAngleDeg == 120);
assert(protocol.expectedFovRange == 300);
assert(protocol.cacheGenerationAuthorized);
assert(protocol.sourceOnlyControlAuditAuthorized);
assert(~protocol.trackingOutcomeRerunAuthorized);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.x48OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('auditFormationStaggeredRecoveryX36V37Scene') == 1);

fprintf('PASS: v37 X36 source protocol tests\n');
end
