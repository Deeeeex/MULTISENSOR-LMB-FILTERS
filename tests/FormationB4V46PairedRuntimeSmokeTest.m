function FormationB4V46PairedRuntimeSmokeTest()
% FORMATIONB4V46PAIREDRUNTIMESMOKETEST Eight-step paired runtime gate.

first = runFormationB4V46PairedRuntimeSmoke();
second = runFormationB4V46PairedRuntimeSmoke();
assert(strcmp(first.canonicalSha256, second.canonicalSha256));
assert(strcmp(first.contractVersion, ...
    'formation-b4-v46-paired-runtime-smoke-v1'));
assert(first.nodeCount == 9 && first.timeCount == 8);
assert(first.deliverySeed == 46008);
assert(strcmp(first.deliveryUniformContractVersion, ...
    'physical-uid-directed-delivery-uniforms-v1'));
assertSha256(first.deliveryUniformCanonicalTensorSha256);
assertSha256(first.deliveryUniformMetadataCanonicalSha256);
assertSha256(first.physicalIdentityRegistryCanonicalSha256);
assert(isequal(first.expectedRepairTriggeredByTime, ...
    logical([0, 0, 0, 0, 1, 1, 1, 1])));
assert(numel(first.arms) == 2);
assert(isequal(first.arms(1).messageCountByTime, ...
    18 * ones(1, 8)));
assert(isequal(first.arms(2).messageCountByTime, ...
    [18, 9, 9, 9, 18, 9, 9, 9]));
assert(isequal(first.arms(1).repairTriggeredByTime, ...
    first.expectedRepairTriggeredByTime));
assert(isequal(first.arms(2).repairTriggeredByTime, ...
    first.expectedRepairTriggeredByTime));
assert(first.referenceAttemptedMessageCount == 144);
assert(first.candidateAttemptedMessageCount == 90);
assert(abs(first.attemptedMessageSavingFraction - 3 / 8) < 1e-12);
assert(first.exactEightStepSavingFractionPassed);
assert(first.candidateAttemptedSubsetOfReference);
assert(first.commonAttemptedPhysicalUidPairedDeliveryPassed);
assert(first.commonAttemptedMessageCount == 90);
assert(first.measurementCellsAllEmpty);
assert(~first.posteriorOutputRead && ~first.truthGeneratedOrRead);
assert(~first.trackingOutcomeScored && ...
    ~first.trackingOutcomeAuthorized && ...
    ~first.validationClaimAllowed && first.developmentEvidenceOnly);
for armIdx = 1:numel(first.arms)
    arm = first.arms(armIdx);
    assert(arm.deliveryRecomputedExactly);
    assert(arm.everyRollingB4SensorUnionStrong);
    assert(~arm.posteriorUsedByPolicy && ~arm.truthUsedByPolicy && ...
        ~arm.futureOutcomeUsedByPolicy && ...
        ~arm.realizedDeliveryUniformsUsedByPolicy && ...
        ~arm.trackingOutcomeScored);
    assert(numel(arm.routeCanonicalSha256ByTime) == 8);
    assert(numel(arm.fusionWeightCanonicalSha256ByTime) == 8);
    assert(numel(arm.observableContractCanonicalSha256ByTime) == 8);
    assertSha256(arm.canonicalSha256);
end
assertSha256(first.canonicalSha256);
fprintf('PASS: FormationB4V46 paired runtime smoke tests\n');
end

function assertSha256(value)
assert(ischar(value) && isrow(value) && numel(value) == 64 && ...
    all(ismember(lower(value), '0123456789abcdef')));
end
