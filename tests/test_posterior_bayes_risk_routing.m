function test_posterior_bayes_risk_routing()
% TEST_POSTERIOR_BAYES_RISK_ROUTING Focused v10 objective tests.

testSingleSummaryRisk();
testExpectedOutcomeRisk();
testOutcomeLabelUniverse();
testFormationModeObjectives();
testReceiverTailPreventsMasking();
testInvalidCovarianceRejected();
testInvalidProbabilityRejected();
fprintf('test_posterior_bayes_risk_routing passed\n');
end

function testSingleSummaryRisk()
cutoff = 10;
empty = makeSummary([1; 1], 0, zeros(2));
certain = makeSummary([1; 1], 1, zeros(2));
uncertain = makeSummary([1; 1], 0.9, 100 * eye(2));
[emptyRisk, emptyDetails] = ...
    computeLmbPosteriorSummaryBayesRisk( ...
        empty, struct('positionCutoff', cutoff));
certainRisk = computeLmbPosteriorSummaryBayesRisk( ...
    certain, struct('positionCutoff', cutoff));
uncertainRisk = computeLmbPosteriorSummaryBayesRisk( ...
    uncertain, struct('positionCutoff', cutoff));
assert(emptyRisk == 0);
assert(certainRisk == 0);
assert(abs(uncertainRisk - 0.9) < 1e-12);
assert(~emptyDetails.truthUsed);
assert(~emptyDetails.multiObjectEospaExact);
assert(emptyDetails.pointDecisionRiskUpperBound);
assert(emptyDetails.restrictedDecisionRiskUpperBound);

labels = [1, 1; 1, 2];
ordered = makeSummary(labels, [0.9, 0.6], ...
    cat(3, eye(2), 4 * eye(2)));
permuted = makeSummary(labels(:, [2, 1]), [0.6, 0.9], ...
    cat(3, 4 * eye(2), eye(2)));
orderedRisk = computeLmbPosteriorSummaryBayesRisk( ...
    ordered, struct('positionCutoff', cutoff));
permutedRisk = computeLmbPosteriorSummaryBayesRisk( ...
    permuted, struct('positionCutoff', cutoff, ...
        'labelUniverse', labels));
assert(abs(orderedRisk - permutedRisk) < 1e-12);
end

function testExpectedOutcomeRisk()
low = makeSummary([1; 1], 1, zeros(2));
high = makeSummary([1; 1], 0.6, zeros(2));
distribution = makeDistribution([0.25, 0.75], {low, high});
[risk, details] = computeExpectedLmbPosteriorBayesRisk( ...
    distribution, struct('positionCutoff', 10));
assert(abs(risk - 0.3) < 1e-12);
assert(abs(details.riskVariance - 0.03) < 1e-12);
assert(~details.truthUsed);
end

function testOutcomeLabelUniverse()
first = makeSummary([1; 1], 0.6, zeros(2));
second = makeSummary([1; 2], 0.8, zeros(2));
distribution = makeDistribution([0.25, 0.75], {first, second});
[risk, details] = computeExpectedLmbPosteriorBayesRisk( ...
    distribution, struct('positionCutoff', 10));
assert(isequal(size(details.labelUniverse), [2, 2]));
assert(max(abs(details.outcomeRisk - [0.2, 0.1])) < 1e-12);
assert(abs(risk - 0.125) < 1e-12);
assert(all(cellfun(@(item) item.labelCount == 2, ...
    details.outcomeDetails)));
end

function testFormationModeObjectives()
groupIds = [1, 1, 2, 2];
referenceHighRisk = makeSummary([1; 1], 0.9, 100 * eye(2));
referenceLowRisk = makeSummary([1; 1], 0.9, zeros(2));
good = makeSummary([1; 1], 1, zeros(2));
bad = makeSummary([1; 1], 0.5, 100 * eye(2));
referenceModes = { ...
    makeDistribution(1, {referenceHighRisk}), ...
    makeDistribution(1, {referenceHighRisk}), ...
    makeDistribution(1, {referenceLowRisk}), ...
    makeDistribution(1, {referenceLowRisk})};
candidateModes = { ...
    makeDistribution(1, {good}), ...
    makeDistribution(1, {good}), ...
    makeDistribution(1, {bad}), ...
    makeDistribution(1, {bad})};
[objectives, details] = ...
    computeFormationModePosteriorBayesRiskObjectives( ...
        {referenceModes, candidateModes}, groupIds, struct( ...
            'positionCutoff', 10, ...
            'receiverTailFraction', 0.5, ...
            'receiverTailWeight', 0.5));
assert(isequal(size(objectives), [2, 2]));
assert(all(objectives(:, 1) == 0));
assert(objectives(1, 2) > 0);
assert(objectives(2, 2) < 0);
assert(details.formationRiskByMode(1, 2) < ...
    details.formationRiskByMode(1, 1));
assert(~details.truthUsed);
end

function testReceiverTailPreventsMasking()
groupIds = [1, 1, 1];
reference = makeSummary([1; 1], 0.6, zeros(2));
good = makeSummary([1; 1], 1, zeros(2));
harmful = makeSummary([1; 1], 0.5, zeros(2));
referenceModes = repmat( ...
    {makeDistribution(1, {reference})}, 1, 3);
candidateModes = { ...
    makeDistribution(1, {good}), ...
    makeDistribution(1, {good}), ...
    makeDistribution(1, {harmful})};
meanObjective = ...
    computeFormationModePosteriorBayesRiskObjectives( ...
        {referenceModes, candidateModes}, groupIds, struct( ...
            'positionCutoff', 10, ...
            'receiverTailFraction', 1 / 3, ...
            'receiverTailWeight', 0));
tailObjective = ...
    computeFormationModePosteriorBayesRiskObjectives( ...
        {referenceModes, candidateModes}, groupIds, struct( ...
            'positionCutoff', 10, ...
            'receiverTailFraction', 1 / 3, ...
            'receiverTailWeight', 1));
assert(meanObjective(1, 2) > 0);
assert(tailObjective(1, 2) < 0);
end

function testInvalidCovarianceRejected()
invalid = makeSummary([1; 1], 0.9, [1, 2; 0, 1]);
failed = false;
try
    computeLmbPosteriorSummaryBayesRisk(invalid, struct());
catch
    failed = true;
end
assert(failed);
end

function testInvalidProbabilityRejected()
summary = makeSummary([1; 1], 0.9, zeros(2));
invalid = makeDistribution([0.4, 0.4], {summary, summary});
failed = false;
try
    computeExpectedLmbPosteriorBayesRisk(invalid, struct());
catch
    failed = true;
end
assert(failed);
end

function summary = makeSummary(labels, existence, covariance)
labels = reshape(labels, 2, []);
existence = reshape(existence, 1, []);
if isempty(covariance)
    covariance = zeros(2, 2, 0);
elseif ndims(covariance) == 2 && size(labels, 2) == 1
    covariance = reshape(covariance, 2, 2, 1);
end
summary = struct( ...
    'contractVersion', 'lmb-disagreement-summary-v1', ...
    'labels', labels, ...
    'existence', existence, ...
    'positionMean', zeros(2, size(labels, 2)), ...
    'positionCovariance', covariance);
end

function distribution = makeDistribution(probability, summaries)
distribution = struct( ...
    'probability', reshape(probability, 1, []), ...
    'summary', {summaries});
end
