function FormationB4V47PairedRuntimeSmokeTest()
% FORMATIONB4V47PAIREDRUNTIMESMOKETEST Repeat the real-filter V47 gate.

first = runFormationB4V47PairedRuntimeSmoke();
second = runFormationB4V47PairedRuntimeSmoke();
assert(strcmp(first.canonicalSha256, second.canonicalSha256));
assert(first.candidateAttemptedSubsetOfReference);
assert(first.commonAttemptedPhysicalUidPairedDeliveryPassed);
assert(first.exactEightStepSavingFractionPassed);
assert(first.candidateDeliveryHistoryContractPassed);
assert(first.candidatePosteriorDebtSignalUsed);
assert(first.attemptedMessageSavingFraction == 3 / 8);
assert(~first.truthGeneratedOrRead);
assert(~first.trackingOutcomeScored);
assert(~first.validationClaimAllowed);
fprintf('PASS: FormationB4V47 paired runtime smoke tests\n');
end
