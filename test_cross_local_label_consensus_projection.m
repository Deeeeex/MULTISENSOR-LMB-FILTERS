function test_cross_local_label_consensus_projection()
% TEST_CROSS_LOCAL_LABEL_CONSENSUS_PROJECTION - Regression for estimate-level label consensus.

clc;
setPath;

fprintf('Test 1: projection gives every sensor the same median-cardinality label set\n');
stateEstimatesBySensor = {
    buildEstimate({state(0, 0), state(10, 0)}, [1, 1; 1, 2]), ...
    buildEstimate({state(1, 0), state(11, 0)}, [2, 1; 2, 2]), ...
    buildEstimate({state(0.5, 0), state(9, 0), state(40, 0)}, [3, 1; 3, 2; 3, 3])};
model = struct('ospaParameters', struct('eC', 5));
projected = applyCrossLocalLabelConsensusProjection(stateEstimatesBySensor, model);
for sensorIdx = 1:numel(projected)
    assert(numel(projected{sensorIdx}.mu{1}) == 2);
    assert(isequal(projected{sensorIdx}.labels{1}, projected{1}.labels{1}));
    assert(norm(projected{sensorIdx}.mu{1}{1} - projected{1}.mu{1}{1}) < 1e-12);
    assert(norm(projected{sensorIdx}.mu{1}{2} - projected{1}.mu{1}{2}) < 1e-12);
end
assert(abs(projected{1}.mu{1}{1}(1) - 0.5) < 1e-12);
assert(abs(projected{1}.mu{1}{2}(1) - 10.0) < 1e-12);

fprintf('Test 2: all-empty estimates remain empty\n');
emptyEstimates = {buildEstimate({}, zeros(2, 0)), buildEstimate({}, zeros(2, 0))};
projected = applyCrossLocalLabelConsensusProjection(emptyEstimates, model);
assert(isempty(projected{1}.mu{1}));
assert(isempty(projected{2}.mu{1}));

fprintf('Test 3: reference-only mode copies the medoid estimate without barycenter averaging\n');
model.adaptiveFusion = struct('crossLocalConsensusProjectionMode', 'reference-only');
projected = applyCrossLocalLabelConsensusProjection(stateEstimatesBySensor, model);
for sensorIdx = 1:numel(projected)
    assert(numel(projected{sensorIdx}.mu{1}) == 2);
    assert(abs(projected{sensorIdx}.mu{1}{1}(1) - 0.0) < 1e-12);
    assert(abs(projected{sensorIdx}.mu{1}{2}(1) - 10.0) < 1e-12);
end

fprintf('Cross-local label-consensus projection tests passed.\n');
end

function estimate = buildEstimate(muCells, labels)
estimate = struct();
estimate.mu = {muCells};
estimate.Sigma = {repmat({eye(4)}, 1, numel(muCells))};
estimate.labels = {labels};
estimate.objects = struct([]);
end

function x = state(px, py)
x = [px; py; 0; 0];
end
