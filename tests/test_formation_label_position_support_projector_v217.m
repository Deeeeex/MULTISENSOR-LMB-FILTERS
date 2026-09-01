function test_formation_label_position_support_projector_v217()
% The frozen 99% position guard keeps supported and rejects disjoint labels.

template = struct( ...
    'candidateIndex', 0, ...
    'receiverCompatibilityMinimum', 0);
candidates = repmat(template, 1, 3);
candidates(1).candidateIndex = 11;
candidates(1).receiverCompatibilityMinimum = 0.80;
candidates(2).candidateIndex = 12;
candidates(2).receiverCompatibilityMinimum = 0.01;
candidates(3).candidateIndex = 13;
candidates(3).receiverCompatibilityMinimum = 1e-4;

[projected, details] = ...
    projectFormationLabelCandidatesByPositionSupportV217( ...
        candidates, struct());
assert(abs(details.minimumCompatibilityFloor - 0.01) < 1e-12);
assert(isequal(details.eligibleMask, [true, true, false]));
assert(isequal([projected.candidateIndex], [11, 12]));
assert(details.rejectedCandidateCount == 1);
assert(~details.thresholdFittedToTrackingOutcomes);
assert(~details.trackingValueCertified);
assert(~details.truthUsed && ~details.futureInformationUsed);

fprintf('test_formation_label_position_support_projector_v217 passed\n');
end
