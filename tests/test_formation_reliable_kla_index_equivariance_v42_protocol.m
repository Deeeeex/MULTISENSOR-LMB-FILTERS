function test_formation_reliable_kla_index_equivariance_v42_protocol()
protocol = getFormationReliableKlaIndexEquivarianceV42Protocol();
assert(strcmp(protocol.id, ...
    'formation-reliable-kla-index-equivariance-v42-v1'));
assert(numel(protocol.canonicalSha256) == 64);
assert(strcmp(protocol.permutationFamily, ...
    'all-cyclic-complete-formation-block-node-coordinate-orders'));
assert(protocol.registeredCartesianCaseCount == 8);
assert(protocol.subsetRunAllowedForSmoke);
assert(protocol. ...
    registeredBatchCompletenessRequiredForRegisteredConclusion);
assert(protocol. ...
    testedCyclicBlockOrderEquivarianceRequiredForFurtherAudit);
assert(~protocol.fullFormationBlockPermutationFamilyCovered);
assert(~protocol.withinFormationSensorPermutationCovered);
assert(~protocol.physicalActionInterpretationAuthorized);
assert(~protocol.formalRuntimeObservableBoundaryPassed);
assert(~protocol.m24TrackingAuthorized);
assert(~protocol.x36TrackingAuthorized);
assert(~protocol.validationClaimAllowed);
fprintf('Formation reliable-KLA v42 protocol tests passed.\n');
end
