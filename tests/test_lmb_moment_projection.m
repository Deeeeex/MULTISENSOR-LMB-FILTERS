function test_lmb_moment_projection()
% TEST_LMB_MOMENT_PROJECTION Verify canonical projection and solve jitter.

rng(20270710);
model = generateMultisensorModel( ...
    2, [1, 1], [0.9, 0.9], [3, 3], 'GA', 'LBP');
stateDimension = model.xDimension;

single = makeRandomObject(model, 1, 1, 0.8, 1);
single.mu{1} = (1:stateDimension)';
single.Sigma{1}(1, 2) = single.Sigma{1}(1, 2) + 1e-12;
expectedSingleCovariance = ...
    (single.Sigma{1} + single.Sigma{1}') / 2;
[singleMean, singleCovariance] = ...
    projectLmbObjectMoments(single, stateDimension);
assert(isequaln(singleMean, single.mu{1}));
assert(isequaln(singleCovariance, expectedSingleCovariance));
assertProjectionIsIdempotent(single, stateDimension);

weighted = makeRandomObject(model, 2, 1, 0.75, 3);
weighted.w = [2, 3, 5];
[expectedMean, expectedCovariance] = manualMoments( ...
    weighted, weighted.w / sum(weighted.w), stateDimension);
[actualMean, actualCovariance] = ...
    projectLmbObjectMoments(weighted, stateDimension);
assert(norm(actualMean - expectedMean, inf) <= 10 * eps);
assert(norm(actualCovariance - expectedCovariance, inf) <= 100 * eps);
assertProjectionIsIdempotent(weighted, stateDimension);

invalidWeights = makeRandomObject(model, 3, 1, 0.7, 4);
invalidWeights.w = [0, NaN, -1, Inf];
[expectedMean, expectedCovariance] = manualMoments( ...
    invalidWeights, ones(1, 4) / 4, stateDimension);
[actualMean, actualCovariance] = ...
    projectLmbObjectMoments(invalidWeights, stateDimension);
assert(norm(actualMean - expectedMean, inf) <= 10 * eps);
assert(norm(actualCovariance - expectedCovariance, inf) <= 100 * eps);
assertProjectionIsIdempotent(invalidWeights, stateDimension);

partiallyValidWeights = makeRandomObject(model, 3, 2, 0.7, 2);
partiallyValidWeights.w = [NaN, 2];
[expectedMean, expectedCovariance] = manualMoments( ...
    partiallyValidWeights, [0, 1], stateDimension);
[actualMean, actualCovariance, normalizedWeights] = ...
    projectLmbObjectMoments(partiallyValidWeights, stateDimension);
assert(isequaln(normalizedWeights, [0, 1]));
assert(isequaln(actualMean, expectedMean));
assert(isequaln(actualCovariance, expectedCovariance));
assertProjectionIsIdempotent(partiallyValidWeights, stateDimension);

componentOrderFixture = makeRandomObject(model, 3, 2, 0.7, 2);
componentOrderFixture.w = [0.1, 0.9];
componentOrderFixture.mu = {zeros(2, 1), zeros(2, 1)};
componentOrderFixture.Sigma = { ...
    [2, 1e-20; 1e-20 + eps(1e-20), 3], ...
    [4, 1; 1 + eps(1), 5]};
firstCanonical = (componentOrderFixture.Sigma{1} + ...
    componentOrderFixture.Sigma{1}') / 2;
secondCanonical = (componentOrderFixture.Sigma{2} + ...
    componentOrderFixture.Sigma{2}') / 2;
expectedOrderCovariance = 0.1 * firstCanonical + ...
    0.9 * secondCanonical;
expectedOrderCovariance = ...
    (expectedOrderCovariance + expectedOrderCovariance') / 2;
[~, actualOrderCovariance] = ...
    projectLmbObjectMoments(componentOrderFixture, 2);
assert(isequaln(actualOrderCovariance, expectedOrderCovariance));
assertProjectionIsIdempotent(componentOrderFixture, 2);

for componentCount = 1:20
    object = makeRandomObject( ...
        model, 4, componentCount, 0.6, componentCount);
    object.w = 10 * rand(1, componentCount);
    assertProjectionIsIdempotent(object, stateDimension);
end

poorlyConditioned = makeRandomObject(model, 5, 1, 0.9, 1);
poorlyConditioned.mu{1} = zeros(stateDimension, 1);
poorlyConditioned.Sigma{1} = diag([1e20, 1, 1, 1]);
[~, projectedPoorCovariance] = ...
    projectLmbObjectMoments(poorlyConditioned, stateDimension);
assert(isequaln(projectedPoorCovariance, poorlyConditioned.Sigma{1}));
assertProjectionIsIdempotent(poorlyConditioned, stateDimension);
[regularizedPoorCovariance, poorJitter] = ...
    regularizeCovarianceForSolve(projectedPoorCovariance);
assert(poorJitter == 0);
assert(isequaln(regularizedPoorCovariance, projectedPoorCovariance));
[regularizedPoorAgain, secondPoorJitter] = ...
    regularizeCovarianceForSolve(regularizedPoorCovariance);
assert(secondPoorJitter == 0);
assert(isequaln(regularizedPoorAgain, regularizedPoorCovariance));
assert(regularizedPoorAgain(2, 2) == 1);

singularCovariance = diag([4, 1, 0, 0]);
[regularizedSingular, singularJitter] = ...
    regularizeCovarianceForSolve(singularCovariance);
assert(singularJitter > 0);
[~, cholFlag] = chol(regularizedSingular);
assert(cholFlag == 0);
[regularizedSingularAgain, repeatedJitter] = ...
    regularizeCovarianceForSolve(regularizedSingular);
assert(repeatedJitter == 0);
assert(isequaln(regularizedSingularAgain, regularizedSingular));

testFusionEquivalence(model);
fprintf('test_lmb_moment_projection passed\n');
end

function assertProjectionIsIdempotent(object, stateDimension)
[firstMean, firstCovariance] = ...
    projectLmbObjectMoments(object, stateDimension);
projected = object;
projected.numberOfGmComponents = 1;
projected.w = 1;
projected.mu = {firstMean};
projected.Sigma = {firstCovariance};
[secondMean, secondCovariance] = ...
    projectLmbObjectMoments(projected, stateDimension);
assert(isequaln(firstMean, secondMean));
assert(isequaln(firstCovariance, secondCovariance));
end

function testFusionEquivalence(model)
sourceOne = [ ...
    makeRandomObject(model, 10, 1, 0.8, 4), ...
    makeRandomObject(model, 10, 2, 0.65, 5)];
sourceTwo = [ ...
    makeRandomObject(model, 10, 1, 0.75, 4), ...
    makeRandomObject(model, 10, 3, 0.7, 2)];
sourceOne(1).w = [0, NaN, -1, Inf];
sourceOne(1).Sigma{1} = diag([1e20, 1, 1, 1]);
sourceOne(2).w = [0.1, 0.9, 0, 0, 0];
sourceOne(2).mu{1} = zeros(model.xDimension, 1);
sourceOne(2).mu{2} = zeros(model.xDimension, 1);
sourceOne(2).Sigma{1} = eye(model.xDimension);
sourceOne(2).Sigma{1}(1:2, 1:2) = ...
    [2, 1e-20; 1e-20 + eps(1e-20), 3];
sourceOne(2).Sigma{2} = eye(model.xDimension);
sourceOne(2).Sigma{2}(1:2, 1:2) = ...
    [4, 1; 1 + eps(1), 5];

fullFusion = fuseLmbPosteriorsByLabel( ...
    {sourceOne, sourceTwo}, [0.4, 0.6], model, [0.55, 0.45]);
projectedOne = compressLmbPosterior(sourceOne, model, 0);
projectedTwo = compressLmbPosterior(sourceTwo, model, 0);
projectedFusion = fuseLmbPosteriorsByLabel( ...
    {projectedOne, projectedTwo}, [0.4, 0.6], model, [0.55, 0.45]);

assert(numel(fullFusion) == numel(projectedFusion));
expectedLabels = [10, 10, 10; 1, 2, 3];
assert(isequal([fullFusion.birthTime; fullFusion.birthLocation], ...
    expectedLabels));
assert(isequal([projectedFusion.birthTime; ...
    projectedFusion.birthLocation], expectedLabels));
for objectIdx = 1:numel(fullFusion)
    assert(fullFusion(objectIdx).birthTime == ...
        projectedFusion(objectIdx).birthTime);
    assert(fullFusion(objectIdx).birthLocation == ...
        projectedFusion(objectIdx).birthLocation);
    assert(isequaln(fullFusion(objectIdx).r, ...
        projectedFusion(objectIdx).r));
    assert(isequaln(fullFusion(objectIdx).mu{1}, ...
        projectedFusion(objectIdx).mu{1}));
    assert(isequaln(fullFusion(objectIdx).Sigma{1}, ...
        projectedFusion(objectIdx).Sigma{1}));
end
end

function object = makeRandomObject( ...
    model, birthTime, birthLocation, existence, componentCount)
object = model.object;
object(1).birthTime = birthTime;
object(1).birthLocation = birthLocation;
object(1).r = existence;
object(1).numberOfGmComponents = componentCount;
object(1).w = rand(1, componentCount) + 0.1;
object(1).mu = cell(1, componentCount);
object(1).Sigma = cell(1, componentCount);
for componentIdx = 1:componentCount
    object(1).mu{componentIdx} = randn(model.xDimension, 1);
    factor = randn(model.xDimension);
    covariance = factor * factor' + 0.25 * eye(model.xDimension);
    object(1).Sigma{componentIdx} = ...
        (covariance + covariance') / 2;
end
object(1).trajectoryLength = 0;
object(1).trajectory = zeros(model.xDimension, 0);
object(1).timestamps = zeros(1, 0);
end

function [mu, covariance] = manualMoments( ...
    object, weights, stateDimension)
mu = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    mu = mu + weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    componentCovariance = ...
        (object.Sigma{componentIdx} + object.Sigma{componentIdx}') / 2;
    delta = object.mu{componentIdx} - mu;
    covariance = covariance + weights(componentIdx) * ...
        (componentCovariance + delta * delta');
end
covariance = (covariance + covariance') / 2;
end
