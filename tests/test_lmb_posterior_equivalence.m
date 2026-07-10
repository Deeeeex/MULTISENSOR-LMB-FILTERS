function test_lmb_posterior_equivalence()
% TEST_LMB_POSTERIOR_EQUIVALENCE Verify compact output-level comparisons.

model = generateMultisensorModel( ...
    2, [1, 1], [0.9, 0.9], [3, 3], 'GA', 'LBP');
first = makeObject(model, 4, 1, 0.75, 2);
second = makeObject(model, 4, 2, 0.65, 1);
snapshot = snapshotLmbPosterior([second, first], model);
assert(isequal(fieldnames(snapshot), ...
    {'labels'; 'r'; 'means'; 'covariances'}));
assert(isequal(snapshot.labels, [4, 4; 1, 2]));
assert(isequal(size(snapshot.means), [model.xDimension, 2]));
assert(isequal(size(snapshot.covariances), ...
    [model.xDimension, model.xDimension, 2]));

permuted = permuteSnapshot(snapshot, [2, 1]);
permutationReport = compareLmbPosteriorSnapshots( ...
    {snapshot}, {permuted});
assert(permutationReport.snapshotCount == 1);
assert(permutationReport.comparisonCount == 2);
assert(permutationReport.labelSetMismatchCount == 0);
assert(permutationReport.missingLabelCount == 0);
assert(permutationReport.exactMatch);

existencePerturbed = snapshot;
existencePerturbed.r(1) = existencePerturbed.r(1) + 0.125;
existenceReport = compareLmbPosteriorSnapshots( ...
    {snapshot}, {existencePerturbed});
assert(abs(existenceReport.maxAbsR - 0.125) <= eps);
assert(existenceReport.maxAbsMu == 0);
assert(existenceReport.maxAbsSigma == 0);
assert(~existenceReport.exactMatch);

meanPerturbed = snapshot;
meanPerturbed.means(2, 1) = meanPerturbed.means(2, 1) + 0.25;
meanReport = compareLmbPosteriorSnapshots({snapshot}, {meanPerturbed});
assert(meanReport.maxAbsR == 0);
assert(meanReport.maxAbsMu == 0.25);
assert(meanReport.maxAbsSigma == 0);
assert(~meanReport.exactMatch);

covariancePerturbed = snapshot;
covariancePerturbed.covariances(1, 1, 2) = ...
    covariancePerturbed.covariances(1, 1, 2) + 0.5;
covarianceReport = compareLmbPosteriorSnapshots( ...
    {snapshot}, {covariancePerturbed});
assert(covarianceReport.maxAbsR == 0);
assert(covarianceReport.maxAbsMu == 0);
assert(covarianceReport.maxAbsSigma == 0.5);
assert(~covarianceReport.exactMatch);

labelPerturbed = snapshot;
labelPerturbed.labels(:, 1) = [99; 99];
labelReport = compareLmbPosteriorSnapshots({snapshot}, {labelPerturbed});
assert(labelReport.labelSetMismatchCount == 1);
assert(labelReport.missingLabelCount == 2);
assert(labelReport.missingFromLeftCount == 1);
assert(labelReport.missingFromRightCount == 1);
assert(labelReport.comparisonCount == 1);
assert(~labelReport.exactMatch);

emptySnapshot = snapshotLmbPosterior(model.object, model);
assert(isempty(emptySnapshot.labels));
assert(isempty(emptySnapshot.r));
assert(isempty(emptySnapshot.means));
assert(isempty(emptySnapshot.covariances));
emptyReport = compareLmbPosteriorSnapshots( ...
    {emptySnapshot}, {emptySnapshot});
assert(emptyReport.snapshotCount == 1);
assert(emptyReport.comparisonCount == 0);
assert(emptyReport.exactMatch);

missingSnapshotReport = compareLmbPosteriorSnapshots( ...
    {snapshot}, {[]});
assert(missingSnapshotReport.missingSnapshotCount == 1);
assert(missingSnapshotReport.labelSetMismatchCount == 1);
assert(missingSnapshotReport.missingLabelCount == 2);
assert(~missingSnapshotReport.exactMatch);
assertThrows(@() compareLmbPosteriorSnapshots( ...
    {snapshot}, {snapshot, snapshot}));

testCodecFusionEquivalence(model);
testFilterSnapshotCapture();
fprintf('test_lmb_posterior_equivalence passed\n');
end

function testCodecFusionEquivalence(model)
sourceOne = [ ...
    makeObject(model, 10, 1, 0.80, 2), ...
    makeObject(model, 10, 2, 0.65, 2)];
sourceTwo = [ ...
    makeObject(model, 10, 1, 0.75, 3), ...
    makeObject(model, 10, 3, 0.70, 1)];

asymmetric = eye(model.xDimension);
asymmetric(1, 2) = 1e-12;
asymmetric(2, 1) = 1e-12 + eps(1e-12);
sourceOne(1).Sigma{1} = asymmetric;
sourceOne(2).Sigma{1} = diag([1e20, 1, 1, 1]);
sourceOne(2).Sigma{2} = diag([1e18, 2, 2, 2]);
assert(~isequal(sourceOne(1).Sigma{1}, sourceOne(1).Sigma{1}'));
assert(rcond(sourceOne(2).Sigma{1}) < 1e-15);

fullInputs = cell(1, 2);
projectedInputs = cell(1, 2);
sources = {sourceOne, sourceTwo};
for sourceIdx = 1:2
    receiverIdx = 3 - sourceIdx;
    fullMetadata = makeMetadata( ...
        2, sourceIdx, receiverIdx, 7);
    fullBytes = encodeLmbWireMessage( ...
        sources{sourceIdx}, fullMetadata, model);
    fullInputs{sourceIdx} = decodeLmbWireMessage(fullBytes, model);

    projected = compressLmbPosterior(sources{sourceIdx}, model, 0);
    projectedMetadata = makeMetadata( ...
        1, sourceIdx, receiverIdx, 7);
    projectedBytes = encodeLmbWireMessage( ...
        projected, projectedMetadata, model);
    projectedInputs{sourceIdx} = ...
        decodeLmbWireMessage(projectedBytes, model);
    assert(numel(projectedBytes) < numel(fullBytes));
end

spatialWeights = [0.4, 0.6];
existenceWeights = [0.55, 0.45];
fullFusion = fuseLmbPosteriorsByLabel( ...
    fullInputs, spatialWeights, model, existenceWeights);
projectedFusion = fuseLmbPosteriorsByLabel( ...
    projectedInputs, spatialWeights, model, existenceWeights);
fullSnapshot = snapshotLmbPosterior(fullFusion, model);
projectedSnapshot = snapshotLmbPosterior(projectedFusion, model);
assert(isequal(fullSnapshot.labels, [10, 10, 10; 1, 2, 3]));
codecReport = compareLmbPosteriorSnapshots( ...
    {fullSnapshot}, {projectedSnapshot});
assert(codecReport.snapshotCount == 1);
assert(codecReport.comparisonCount == 3);
assert(codecReport.labelSetMismatchCount == 0);
assert(codecReport.missingLabelCount == 0);
assert(codecReport.maxExistenceResidual == 0);
assert(codecReport.maxMeanResidual == 0);
assert(codecReport.maxCovarianceResidual == 0);
assert(codecReport.exactMatch);
end

function testFilterSnapshotCapture()
model = generateMultisensorModel( ...
    2, [1, 1], [0.9, 0.9], [3, 3], 'GA', 'LBP');
model.simulationLength = 100;
[~, measurements, ~, sensorTrajectories] = ...
    generateMultisensorGroundTruth(model);
measurements = measurements(:, 1:2);
for sensorIdx = 1:numel(sensorTrajectories)
    sensorTrajectories{sensorIdx} = sensorTrajectories{sensorIdx}(:, 1:2);
end
neighborMap = {[1, 2], [1, 2]};
commConfig = struct( ...
    'pDropBySensor', [0, 0], ...
    'forceDelivery', true);
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'capturePosteriorSnapshots', true);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
assert(isfield(diagnostics, 'posteriorSnapshots'));
assert(isequal(size(diagnostics.posteriorSnapshots), [2, 2]));
missingCapturedSnapshots = cellfun( ...
    @isempty, diagnostics.posteriorSnapshots);
assert(~any(missingCapturedSnapshots(:)));
selfReport = compareLmbPosteriorSnapshots( ...
    diagnostics.posteriorSnapshots, diagnostics.posteriorSnapshots);
assert(selfReport.snapshotCount == 4);
assert(selfReport.labelSetMismatchCount == 0);
assert(selfReport.exactMatch);
for snapshotIdx = 1:numel(diagnostics.posteriorSnapshots)
    captured = diagnostics.posteriorSnapshots{snapshotIdx};
    assert(isequal(fieldnames(captured), ...
        {'labels'; 'r'; 'means'; 'covariances'}));
end

triggerConfig = rmfield(triggerConfig, 'capturePosteriorSnapshots');
[~, noCaptureDiagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
assert(~noCaptureDiagnostics.triggerConfig.capturePosteriorSnapshots);
assert(~isfield(noCaptureDiagnostics, 'posteriorSnapshots'));

invalidCaptureConfig = triggerConfig;
invalidCaptureConfig.capturePosteriorSnapshots = [true, false];
assertThrows(@() runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, invalidCaptureConfig));
end

function metadata = makeMetadata(eventType, sender, receiver, timeIndex)
metadata = struct( ...
    'eventType', eventType, ...
    'sender', sender, ...
    'receiver', receiver, ...
    'timeIndex', timeIndex);
end

function permuted = permuteSnapshot(snapshot, order)
permuted = snapshot;
permuted.labels = snapshot.labels(:, order);
permuted.r = snapshot.r(order);
permuted.means = snapshot.means(:, order);
permuted.covariances = snapshot.covariances(:, :, order);
end

function object = makeObject( ...
    model, birthTime, birthLocation, existence, componentCount)
object = model.object;
object(1).birthTime = birthTime;
object(1).birthLocation = birthLocation;
object(1).r = existence;
object(1).numberOfGmComponents = componentCount;
object(1).w = (1:componentCount) / sum(1:componentCount);
object(1).mu = cell(1, componentCount);
object(1).Sigma = cell(1, componentCount);
for componentIdx = 1:componentCount
    object(1).mu{componentIdx} = ...
        componentIdx * (1:model.xDimension)' + birthLocation;
    factor = diag(componentIdx + (1:model.xDimension));
    object(1).Sigma{componentIdx} = factor * factor';
end
object(1).trajectoryLength = 2;
object(1).trajectory = ones(model.xDimension, 2);
object(1).timestamps = 1:2;
end

function assertThrows(callback)
didThrow = false;
try
    callback();
catch
    didThrow = true;
end
assert(didThrow);
end
