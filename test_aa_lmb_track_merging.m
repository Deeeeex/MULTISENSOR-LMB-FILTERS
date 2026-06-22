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

fprintf('Test 6: optional KLA spatial existence gate is target-wise\n');
model = buildModel([0.5, 0.5], [0.5, 0.5], 2);
model.aaSpatialFusionMode = 'kla';
model.aaKlaSpatialExistencePower = 1.0;
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.20, 0.80), model);
assert(abs(objects(1).r - 0.50) < 1e-12);
assert(objects(1).numberOfGmComponents == 1);
assert(abs(objects(1).mu{1}(1) - 80) < 1e-9);

fprintf('Test 7: label-uncertainty KLA matches ordinary KLA for identical posteriors\n');
model = buildModel([0.5, 0.5], [0.5, 0.5], 2);
model.aaSpatialFusionMode = 'kla';
model.useAaLabelUncertaintyFusion = true;
objects = aaLmbTrackMerging(buildObjectSet([0.80, 0.80], {[20; 0; 0; 0], [20; 0; 0; 0]}), model);
assert(abs(objects(1).mu{1}(1) - 20) < 1e-9);
assert(max(abs(diag(objects(1).Sigma{1}) - 1)) < 1e-8);

fprintf('Test 8: label-uncertainty KLA downweights spatial outlier by overlap\n');
model = buildModel([1/3, 1/3, 1/3], [1/3, 1/3, 1/3], 2);
model.aaSpatialFusionMode = 'kla';
model.useAaLabelUncertaintyFusion = true;
objects = aaLmbTrackMerging(buildObjectSet( ...
    [0.90, 0.90, 0.90], {[0; 0; 0; 0], [1; 0; 0; 0], [100; 0; 0; 0]}), model);
assert(objects(1).mu{1}(1) < 5);
assert(objects(1).Sigma{1}(1, 1) > 0.5);

fprintf('Test 9: label support tempering lowers weak label odds without hard deletion\n');
model = buildModel([1/3, 1/3, 1/3], [1/3, 1/3, 1/3], 2);
model.aaSpatialFusionMode = 'kla';
model.useAaLabelUncertaintyFusion = true;
model.useAaLabelExistenceTempering = true;
objects = aaLmbTrackMerging(buildObjectSet( ...
    [0.80, 0.05, 0.05], {[0; 0; 0; 0], [0; 0; 0; 0], [0; 0; 0; 0]}), model);
assert(objects(1).r > 0);
assert(objects(1).r < mean([0.80, 0.05, 0.05]));

fprintf('Test 10: label-uncertainty inflation can be used without overlap reweighting\n');
model = buildModel([0.5, 0.5], [0.5, 0.5], 2);
model.aaSpatialFusionMode = 'kla';
model.useAaLabelUncertaintyFusion = true;
model.useAaLabelSpatialOverlapWeights = false;
model.useAaLabelUncertaintyInflation = true;
model.useAaLabelExistenceTempering = false;
objects = aaLmbTrackMerging(buildTwoSensorObjectSet(0.80, 0.80), model);
assert(abs(objects(1).mu{1}(1) - 50) < 1e-9);
assert(objects(1).Sigma{1}(1, 1) > 1000);
assert(abs(objects(1).Sigma{1}(2, 2) - 1) < 1e-8);

fprintf('Test 11: single supported label posterior is not suppressed by overlap logic\n');
model = buildModel([1/3, 1/3, 1/3], [1/3, 1/3, 1/3], 2);
model.aaSpatialFusionMode = 'kla';
model.useAaLabelUncertaintyFusion = true;
model.useAaLabelExistenceTempering = false;
objects = aaLmbTrackMerging(buildObjectSet( ...
    [0.80, 0.00, 0.00], {[0; 0; 0; 0], [100; 0; 0; 0], [200; 0; 0; 0]}), model);
assert(abs(objects(1).r - mean([0.80, 0.00, 0.00])) < 1e-12);
assert(abs(objects(1).mu{1}(1)) < 1e-9);
assert(abs(objects(1).Sigma{1}(1, 1) - 1) < 1e-8);

fprintf('AA-LMB track-merging tests passed.\n');
end

function model = buildModel(spatialWeights, existenceWeights, maxComponents)
model = struct();
model.numberOfSensors = numel(spatialWeights);
model.xDimension = 4;
model.maximumNumberOfGmComponents = maxComponents;
model.aaSensorWeights = ones(1, model.numberOfSensors) / model.numberOfSensors;
model.aaSpatialWeights = spatialWeights;
model.aaExistenceWeights = existenceWeights;
end

function distributions = buildTwoSensorObjectSet(r1, r2)
distributions = buildObjectSet([r1, r2], {[0; 0; 0; 0], [100; 0; 0; 0]});
end

function distributions = buildObjectSet(existences, states)
distributions = cell(1, numel(existences));
for s = 1:numel(existences)
    distributions{s} = buildObject(existences(s), states{s});
end
end

function objects = buildObject(existence, state)
objects = struct( ...
    'r', existence, ...
    'numberOfGmComponents', 1, ...
    'w', 1, ...
    'mu', {{state}}, ...
    'Sigma', {{eye(4)}});
end
