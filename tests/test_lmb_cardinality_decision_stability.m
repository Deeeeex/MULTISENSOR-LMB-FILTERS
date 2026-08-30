function test_lmb_cardinality_decision_stability()
% TEST_LMB_CARDINALITY_DECISION_STABILITY Focused MAP certificate check.

reference = receiverDistribution(0.90);
safeCandidate = receiverDistribution(0.85);
unsafeCandidate = receiverDistribution(0.40);
[safeRisk, safe] = computeLmbCardinalityDecisionStability( ...
    {reference}, {safeCandidate}, 1);
[unsafeRisk, unsafe] = computeLmbCardinalityDecisionStability( ...
    {reference}, {unsafeCandidate}, 1);
assert(safeRisk == 0);
assert(safe.mapCardinalityCertified);
assert(~safe.mapCardinalityChanged);
assert(unsafeRisk > 0);
assert(~unsafe.mapCardinalityCertified);
assert(unsafe.mapCardinalityChanged);
assert(unsafe.referenceMapCardinality == 1);
assert(unsafe.candidateMapCardinality == 0);
fprintf('test_lmb_cardinality_decision_stability passed.\n');
end

function distribution = receiverDistribution(existence)
distribution = struct();
distribution.probability = 1;
distribution.summary = {struct( ...
    'labels', [1; 1], 'existence', existence)};
end

