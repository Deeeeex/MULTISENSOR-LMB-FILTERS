function test_lmb_label_lifecycle_thresholds()
% TEST_LMB_LABEL_LIFECYCLE_THRESHOLDS - Regression for decoupled output/pruning thresholds.

clc;
setPath;

fprintf('Test 1: default lifecycle keeps legacy single-threshold behavior\n');
model = buildModel(0.18);
objects = buildObjects([0.25, 0.10, 0.005], [12, 12, 12]);
[outputObjects, recursionObjects, discardedObjects, thresholds] = ...
    applyLmbLabelLifecycleThresholds(objects, model);
assert(abs(thresholds.output - 0.18) < 1e-12);
assert(abs(thresholds.pruning - 0.18) < 1e-12);
assert(numel(outputObjects) == 1);
assert(numel(recursionObjects) == 1);
assert(numel(discardedObjects) == 2);

fprintf('Test 2: decoupled pruning retains mature sub-output labels for recursion\n');
model = buildModel(0.18);
model.labelPruningThreshold = 0.01;
[outputObjects, recursionObjects, discardedObjects, thresholds] = ...
    applyLmbLabelLifecycleThresholds(objects, model);
assert(abs(thresholds.pruning - 0.01) < 1e-12);
assert(thresholds.pruningMinTrajectoryLength == 1);
assert(numel(outputObjects) == 1);
assert(numel(recursionObjects) == 2);
assert([recursionObjects.birthLocation] == [1, 2]);
assert(numel(discardedObjects) == 1);
assert(discardedObjects.birthLocation == 3);

fprintf('Test 3: decoupled pruning does not retain newborn sub-output clutter\n');
model = buildModel(0.18);
model.labelPruningThreshold = 0.01;
newbornObjects = buildObjects([0.25, 0.10, 0.005], [0, 0, 0]);
[outputObjects, recursionObjects, discardedObjects, thresholds] = ...
    applyLmbLabelLifecycleThresholds(newbornObjects, model);
assert(thresholds.pruningMinTrajectoryLength == 1);
assert(numel(outputObjects) == 1);
assert(numel(recursionObjects) == 1);
assert(recursionObjects.birthLocation == 1);
assert(isempty(discardedObjects));

fprintf('Test 4: adaptiveFusion pruning settings override model defaults\n');
model = buildModel(0.18);
model.labelPruningThreshold = 0.10;
model.adaptiveFusion = struct( ...
    'labelPruningThreshold', 0.03, ...
    'labelPruningMinTrajectoryLength', 0);
[~, recursionObjects, ~, thresholds] = applyLmbLabelLifecycleThresholds(objects, model);
assert(abs(thresholds.pruning - 0.03) < 1e-12);
assert(thresholds.pruningMinTrajectoryLength == 0);
assert(numel(recursionObjects) == 2);

fprintf('Test 5: last-output protection keeps only recently output labels\n');
model = buildModel(0.18);
model.labelPruningThreshold = 0.01;
model.labelPruningProtectionMode = 'last-output';
model.labelPruningMaxOutputGap = 1;
historyObjects = buildObjects([0.25, 0.10, 0.09, 0.08], [8, 8, 8, 8]);
historyObjects(2).lastOutputTime = 4;
historyObjects(3).lastOutputTime = 3;
historyObjects(4).lastOutputTime = -Inf;
[outputObjects, recursionObjects, ~, thresholds] = ...
    applyLmbLabelLifecycleThresholds(historyObjects, model, 5);
assert(strcmp(thresholds.protectionMode, 'last-output'));
assert(thresholds.maxOutputGap == 1);
assert(numel(outputObjects) == 1);
assert(numel(recursionObjects) == 2);
assert([recursionObjects.birthLocation] == [1, 2]);

fprintf('LMB label lifecycle tests passed.\n');
end

function model = buildModel(existenceThreshold)
model = struct();
model.existenceThreshold = existenceThreshold;
model.minimumTrajectoryLength = 10;
end

function objects = buildObjects(existences, trajectoryLengths)
template = struct( ...
    'birthLocation', 0, ...
    'birthTime', 1, ...
    'r', 0, ...
    'numberOfGmComponents', 1, ...
    'w', 1, ...
    'mu', {{zeros(4, 1)}}, ...
    'Sigma', {{eye(4)}}, ...
    'trajectoryLength', 0, ...
    'lastOutputTime', -Inf, ...
    'trajectory', [], ...
    'timestamps', []);
objects = repmat(template, 1, numel(existences));
for idx = 1:numel(existences)
    objects(idx).birthLocation = idx;
    objects(idx).r = existences(idx);
    objects(idx).trajectoryLength = trajectoryLengths(idx);
end
end
