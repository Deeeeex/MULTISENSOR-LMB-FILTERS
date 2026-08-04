function test_formation_b4_v47_structural_screen()
% Short deterministic screen covers same-budget and claim-boundary logic.

result = runFormationB4V47StructuralScreen(struct( ...
    'presets', {{'m24-formation-fov'}}, ...
    'seeds', 41, 'timeIndices', 1:8, ...
    'horizons', [4, 8], 'progressEvery', 8));
assert(result.caseCount == 1);
current = result.cases(1);
assert(current.syncAttemptedMessageCount == 10 * current.nodeCount);
assert(current.candidateAttemptedMessageCount == ...
    current.syncAttemptedMessageCount);
assert(current.fullReferenceAttemptedMessageCount == ...
    16 * current.nodeCount);
assert(current.candidateSameBudgetAsSynchronizedB4);
assert(current.candidateExactNoFallbackSavingPassed);
assert(current.candidateAttemptedMessageSavingFraction == 3 / 8);
assert(current.candidateEveryAttemptedRollingB4Strong);
assert(current.posteriorSignalHeldNeutral);
assert(current.adaptiveRouteDependsOnPastDeliveries);
assert(current.postHocFixedRouteMeanSquareDiagnosticOnly);
assert(~current.closedLoopMeanSquareContractionCertified);
assert(~result.trackingOutcomeScored);
assert(~result.validationClaimAllowed);
fprintf('PASS: FormationB4V47 structural screen tests\n');
end
