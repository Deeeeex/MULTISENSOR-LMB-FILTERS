function test_additional_attempts_appendix()
% TEST_ADDITIONAL_ATTEMPTS_APPENDIX - Smoke and unit tests for appendix-B comparison helpers

clc;
setPath;
addpath('RUN/GA');

fprintf('Test 1: legacy association ambiguity config is ignored by core weights\n');
[gaWeights, ~, debug] = computeAdaptiveFusionWeights( ...
    buildSimpleDistributions(), cell(3, 1), buildAmbiguityModel(), 1, ...
    struct('associationAmbiguityScore', [0.95, 0.70, 0.30]), struct());
fprintf('  weights: %s\n', mat2str(gaWeights, 4));
assert(~isfield(debug, 'associationAmbiguityScore'));
assert(max(abs(gaWeights - ones(1, 3) / 3)) < 1e-12);

fprintf('Test 2: posterior-structure comparison smoke test\n');
[reportPath, summary] = runMultisensorFilters_formation_4plus4_PosteriorStructureCompare(1, 1, true, false);
assert(isempty(reportPath));
assert(isfield(summary, 'consensus'));
assert(isfield(summary.consensus, 'ospaBase'));
assert(isfield(summary.consensus, 'ospaAdaptive'));
assert(isfield(summary.consensus, 'cardAdaptive'));

fprintf('Test 3: association ambiguity comparison smoke test\n');
[reportPath, summary] = runMultisensorFilters_formation_4plus4_AssociationAmbiguityCompare(1, 1, true, false);
assert(isempty(reportPath));
assert(isfield(summary, 'consensus'));
assert(isfield(summary.consensus, 'ospaBase'));
assert(isfield(summary.consensus, 'ospaAdaptive'));
assert(isfield(summary.consensus, 'cardAdaptive'));

fprintf('Additional appendix comparison tests passed.\n');
end

function model = buildAmbiguityModel()
model = struct();
model.numberOfSensors = 3;
model.xDimension = 4;
model.gaSensorWeights = ones(1, 3) / 3;
model.aaSensorWeights = model.gaSensorWeights;
model.gaSpatialWeights = model.gaSensorWeights;
model.aaSpatialWeights = model.gaSensorWeights;
model.gaExistenceWeights = model.gaSensorWeights;
model.aaExistenceWeights = model.gaSensorWeights;
model.gaTopologyWeights = ones(1, 3);
model.aaTopologyWeights = ones(1, 3);
model.adaptiveFusion = struct( ...
    'enabled', true, ...
    'useCovariance', false, ...
    'useLinkQuality', false, ...
    'useNIS', false, ...
    'useHistory', false, ...
    'useCardinalityConsensus', false, ...
    'useExistenceConfidence', false, ...
    'useAssociationAmbiguity', true, ...
    'associationAmbiguityMinScore', 0.2, ...
    'associationAmbiguityPower', 1.0, ...
    'useDecoupledKla', false, ...
    'emaAlpha', 0.0, ...
    'minWeight', 0.0);
end

function distributions = buildSimpleDistributions()
distributions = cell(1, 3);
template = buildObjectArray([0, 0; 10, 0], [0.9, 0.2]);
for idx = 1:3
    distributions{idx} = template;
end
end

function objects = buildObjectArray(positionArray, existenceArray)
numObjects = size(positionArray, 1);
template = struct( ...
    'r', 0.0, ...
    'numberOfGmComponents', 1, ...
    'w', 1, ...
    'mu', {{}}, ...
    'Sigma', {{}});
objects = repmat(template, 1, numObjects);
for idx = 1:numObjects
    state = [positionArray(idx, 1); positionArray(idx, 2); 0; 0];
    sigma = diag([4, 4, 1, 1]);
    objects(idx).r = existenceArray(idx);
    objects(idx).mu = {state};
    objects(idx).Sigma = {sigma};
end
end
