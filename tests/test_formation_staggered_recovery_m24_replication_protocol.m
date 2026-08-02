function test_formation_staggered_recovery_m24_replication_protocol()
% TEST_FORMATIONSTAGGEREDRECOVERYM24REPLICATIONPROTOCOL V36 contract.

protocol = ...
    getFormationStaggeredRecoveryM24ReplicationProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-staggered-recovery-m24-replication-protocol-v1'));
assert(isequal(protocol.anchorTimes, [60, 104, 124]));
assert(isequal(protocol.expectedInitialFormationIdsByState, ...
    {[2, 4], [1, 2, 4], [2, 3, 4]}));
assert(isequal(protocol.expectedInitialActionIndices, [11, 12, 15]));
assert(isequal( ...
    protocol.expectedRuntimeSelectedFormationIdsByState{1}, ...
    {[2, 4], [2, 4], zeros(1, 0)}));
assert(isequal( ...
    protocol.expectedRuntimeSelectedFormationIdsByState{2}, ...
    {[1, 2, 4], [1, 2], 4}));
assert(isequal( ...
    protocol.expectedRuntimeSelectedFormationIdsByState{3}, ...
    {[2, 3, 4], [2, 3], 4}));
assert(isequal( ...
    protocol.expectedRuntimeReferenceFallbackCountByState{1}, ...
    [0, 0, 1]));
assert(numel(protocol.expectedCacheSha256) == 3);
assert(protocol.minimumStrongReplicationStateCount == 2);
assert(protocol.minimumMedianMeanGainPercent == 2);
assert(protocol.minimumStateMeanGainPercent == -1);
assert(protocol.minimumPositiveTerminalStateCount == 2);
assert(protocol.trackingOutcomeRerunAuthorized);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36SourceOnlyProtocolAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.x48OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('auditFormationStaggeredRecoveryM24V36Preflight') == 1);
assert(nargin('runFormationStaggeredRecoveryM24V36Screen') == 1);

fprintf('PASS: v36 M24 replication protocol tests\n');
end
