function FormationB4V49PairedRuntimeSmokeTest()
% Check only the method-relevant behavior of the real-filter smoke.

result = runFormationB4V49PairedRuntimeSmoke();
assert(isequal(result.expectedMessageCountByTime, ...
    [24, 12, 12, 12, 24, 12, 12, 12]));
assert(result.reference.totalAttemptedMessageCount == 120);
assert(result.candidate.totalAttemptedMessageCount == 120);
assert(result.commonAttemptedMessageCount == 112);
assert(result.commonDeliveryMatched);
assert(result.phaseOneRoutesDiffer);
assert(result.nonburstRoutesAndWeightsMatch);
assert(isequal(result.candidate.cycleSelectedByTime, ...
    logical([1, 0, 0, 0, 1, 0, 0, 0])));
assert(result.reference.attemptedStrongWindowCount == 5);
assert(result.candidate.attemptedStrongWindowCount == 5);
assert(result.reference.deliveredStrongWindowCount == 0);
assert(result.candidate.deliveredStrongWindowCount == 5);
assert(result.measurementsEmpty && ~result.trackingOutcomeScored);
fprintf('PASS: FormationB4V49 paired runtime smoke test\n');
end
