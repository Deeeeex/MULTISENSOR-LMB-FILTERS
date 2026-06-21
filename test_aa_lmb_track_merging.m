function test_aa_lmb_track_merging()
% TEST_AA_LMB_TRACK_MERGING - Regression tests for Bernoulli-AA spatial weights.

clc;
setPath;

fprintf('Test 1: AA spatial mixture is weighted by local existence probability\n');
model = buildModel([0.5, 0.5], [0.5, 0.5], 2);
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.95, 0.05), model);
assert(abs(objects(1).r - 0.5) < 1e-12);
assert(abs(objects(1).w(1) - 0.95) < 1e-12);
assert(abs(objects(1).w(2) - 0.05) < 1e-12);

fprintf('Test 2: low-existence component cannot dominate through spatial weight alone\n');
model = buildModel([0.2, 0.8], [0.5, 0.5], 2);
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.90, 0.05), model);
expectedFirstWeight = (0.2 * 0.90) / (0.2 * 0.90 + 0.8 * 0.05);
assert(abs(objects(1).w(1) - expectedFirstWeight) < 1e-12);
assert(objects(1).mu{1}(1) == 0);

fprintf('Test 3: AA consumes target-wise weights before branch weights\n');
model = buildModel([0.1, 0.9], [0.1, 0.9], 2);
model.aaTargetWiseWeights = [0.9, 0.1];
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.80, 0.80), model);
assert(abs(objects(1).r - 0.80) < 1e-12);
assert(abs(objects(1).w(1) - 0.9) < 1e-12);
assert(objects(1).mu{1}(1) == 0);

fprintf('Test 4: strict AA uses the same weights for existence and spatial density\n');
model = buildModel([0.9, 0.1], [0.1, 0.9], 2);
model.aaSensorWeights = [0.6, 0.4];
model.aaFusionWeightMode = 'strict';
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.20, 0.80), model);
expectedExistence = 0.6 * 0.20 + 0.4 * 0.80;
expectedFirstWeight = (0.6 * 0.20) / (0.6 * 0.20 + 0.4 * 0.80);
assert(abs(objects(1).r - expectedExistence) < 1e-12);
assert(abs(objects(1).w(2) - expectedFirstWeight) < 1e-12);
assert(objects(1).mu{2}(1) == 0);

fprintf('Test 5: hybrid spatial-KLA AA keeps AA existence and emits one Gaussian\n');
model = buildModel([0.5, 0.5], [0.5, 0.5], 2);
model.aaSpatialFusionMode = 'kla';
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.20, 0.80), model);
assert(abs(objects(1).r - 0.50) < 1e-12);
assert(objects(1).numberOfGmComponents == 1);
assert(abs(objects(1).w - 1) < 1e-12);
assert(abs(objects(1).mu{1}(1) - 50) < 1e-9);

fprintf('AA-LMB track-merging tests passed.\n');
end

function model = buildModel(spatialWeights, existenceWeights, maxComponents)
model = struct();
model.numberOfSensors = 2;
model.xDimension = 4;
model.maximumNumberOfGmComponents = maxComponents;
model.aaSensorWeights = ones(1, 2) / 2;
model.aaSpatialWeights = spatialWeights;
model.aaExistenceWeights = existenceWeights;
end

function distributions = buildTwoSensorObjectSet(r1, r2)
distributions = cell(1, 2);
distributions{1} = buildObject(r1, [0; 0; 0; 0]);
distributions{2} = buildObject(r2, [100; 0; 0; 0]);
end

function objects = buildObject(existence, state)
objects = struct( ...
    'r', existence, ...
    'numberOfGmComponents', 1, ...
    'w', 1, ...
    'mu', {{state}}, ...
    'Sigma', {{eye(4)}});
end
