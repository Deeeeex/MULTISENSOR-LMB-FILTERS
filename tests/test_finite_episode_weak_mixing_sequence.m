function test_finite_episode_weak_mixing_sequence()
% Boolean support, chronological order, success, failure, and bias oracle.

cycle = [0.5, 0.5, 0.0; 0.0, 0.5, 0.5; 0.5, 0.0, 0.5];
history = repmat(cycle, 1, 1, 6);
success = auditFiniteEpisodeWeakMixingSequence( ...
    history, [2, 4], struct('verifyAllProductsDirectly', true));
assert(success(2).everyProductSupportScrambling);
assert(success(2).everyProductHasPositiveColumn);
assert(success(2).everyProductSupportPositive);
assert(success(2).everyFloatingSupportMatchesBoolean);
assert(success(2).productTreeDirectComparisonPassed);
assert(success(2).everyWindowComparedDirectly);

identityHistory = repmat(eye(3), 1, 1, 4);
failure = auditFiniteEpisodeWeakMixingSequence( ...
    identityHistory, 4, ...
    struct('verifyAllProductsDirectly', true));
assert(~failure.everyProductSupportScrambling);
assert(~failure.everyProductHasPositiveColumn);
assert(~failure.everyProductSupportPositive);

bias = ones(3, 1) * [0.9, 0.05, 0.05];
biasAudit = auditFiniteEpisodeWeakMixingSequence( ...
    reshape(bias, 3, 3, 1), 1, ...
    struct('verifyAllProductsDirectly', true));
assert(biasAudit.worstDobrushin < 1e-12);
assert(biasAudit.worstMaximumCenteredRowL2 < 1e-12);
assert(biasAudit.worstDistanceToUniformAveraging > 1);
assert(biasAudit.worstMaximumColumnSumError > 1);
assert(biasAudit.worstInfluenceWeightL1Imbalance > 1);

first = [0.7, 0.3, 0.0; 0.0, 0.6, 0.4; 0.2, 0.0, 0.8];
second = [0.5, 0.0, 0.5; 0.4, 0.6, 0.0; 0.0, 0.2, 0.8];
noncommutative = cat(3, first, second);
ordered = auditFiniteEpisodeWeakMixingSequence( ...
    noncommutative, 2, ...
    struct('verifyAllProductsDirectly', true));
expected = second * first;
reversed = first * second;
assert(max(abs(expected(:) - reversed(:))) > 1e-3);
assert(strcmp(ordered. ...
        firstSegmentTreeChronologicalProductCanonicalSha256, ...
    computeCanonicalValueSha256(expected)));
assert(~strcmp(ordered. ...
        firstSegmentTreeChronologicalProductCanonicalSha256, ...
    computeCanonicalValueSha256(reversed)));

pages = zeros(4, 4, 7);
for currentTime = 1:7
    raw = zeros(4);
    for rowIdx = 1:4
        raw(rowIdx, rowIdx) = 1 + 0.1 * currentTime;
        raw(rowIdx, mod(rowIdx + currentTime - 1, 4) + 1) = ...
            raw(rowIdx, mod(rowIdx + currentTime - 1, 4) + 1) + ...
            0.2 + 0.03 * rowIdx;
        raw(rowIdx, mod(rowIdx + 2 * currentTime - 1, 4) + 1) = ...
            raw(rowIdx, mod(rowIdx + 2 * currentTime - 1, 4) + 1) + ...
            0.1 + 0.01 * currentTime;
    end
    pages(:, :, currentTime) = raw ./ sum(raw, 2);
end
multiRange = auditFiniteEpisodeWeakMixingSequence( ...
    pages, [2, 3, 5, 7], ...
    struct('verifyAllProductsDirectly', true));
assert(all([multiRange.productTreeDirectComparisonPassed]));
assert(all([multiRange.everyWindowComparedDirectly]));
assert(all([multiRange.everyFloatingSupportMatchesBoolean]));

assertErrorId(@() auditFiniteEpisodeWeakMixingSequence( ...
    cat(3, eye(2), [1, -0.1; 0, 1.1]), 2), ...
    'FiniteEpisodeWeakMixing:InvalidInput');
fprintf('PASS: finite-episode weak-mixing sequence tests\n');
end

function assertErrorId(callback, expectedIdentifier)
failed = false;
actualIdentifier = '';
try
    callback();
catch errorInfo
    actualIdentifier = errorInfo.identifier;
    failed = strcmp(actualIdentifier, expectedIdentifier);
end
assert(failed, 'Expected %s, received %s.', ...
    expectedIdentifier, actualIdentifier);
end
