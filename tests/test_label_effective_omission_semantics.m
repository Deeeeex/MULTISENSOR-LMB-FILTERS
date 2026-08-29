function test_label_effective_omission_semantics()
% TEST_LABEL_EFFECTIVE_OMISSION_SEMANTICS Verify three-state label handling.

model = buildModel();
object = buildObject([20; 0; 0; 0], 0.8);
emptyPosterior = object([]);
label = [object.birthTime; object.birthLocation];
weights = [0.5, 0.5];
config = buildMixtureAwareKlaReferenceConfig(struct( ...
    'missingLabelFusionMode', 'fov-aware-censored', ...
    'fovAwareMissingLabelMinimumFovFraction', 0.5, ...
    'fovAwareMissingLabelMinimumExpectedDetectionProbability', 0.1));
baseDetails = struct( ...
    'sourceIndices', [1, 2], ...
    'isSelf', [true, false], ...
    'isStale', [false, false], ...
    'currentTime', 1, ...
    'eventType', [2, 2]);

participatingDetails = baseDetails;
participatingDetails.explicitLabelOmissionRegistered = [false, true];
participatingDetails.explicitOmittedLabels = {zeros(2, 0), zeros(2, 0)};
[participatingResult, participatingDiagnostics] = ...
    fuseLmbPosteriorsByLabel( ...
        {object, object}, weights, model, weights, ...
        participatingDetails, config);
assert(numel(participatingResult) == 1);
assert(abs(participatingResult.r - object.r) <= 1e-10);
assert(participatingDiagnostics.explicitLabelAbstentionSourceCount == 0);

naturallyAbsentDetails = participatingDetails;
[naturallyAbsentResult, naturallyAbsentDiagnostics] = ...
    fuseLmbPosteriorsByLabel( ...
        {object, emptyPosterior}, weights, model, weights, ...
        naturallyAbsentDetails, config);
assert(numel(naturallyAbsentResult) == 1);
assert(naturallyAbsentResult.r > 0);
assert(naturallyAbsentResult.r < object.r);
assert(naturallyAbsentDiagnostics.observableCensoredSourceCount == 1);
assert(naturallyAbsentDiagnostics.explicitLabelAbstentionSourceCount == 0);

omittedDetails = baseDetails;
omittedDetails.explicitLabelOmissionRegistered = [false, true];
omittedDetails.explicitOmittedLabels = {zeros(2, 0), label};
[omittedResult, omittedDiagnostics] = fuseLmbPosteriorsByLabel( ...
    {object, emptyPosterior}, weights, model, weights, ...
    omittedDetails, config);
assert(numel(omittedResult) == 1);
assert(abs(omittedResult.r - object.r) <= 1e-10);
assert(omittedDiagnostics.observableCensoredSourceCount == 0);
assert(omittedDiagnostics.explicitLabelAbstentionSourceCount == 1);

didRejectContradiction = false;
try
    fuseLmbPosteriorsByLabel( ...
        {object, object}, weights, model, weights, ...
        omittedDetails, config);
catch exception
    didRejectContradiction = strcmp( ...
        exception.identifier, 'LmbKla:PayloadAlsoMarkedOmitted');
end
assert(didRejectContradiction);

didRejectUnregisteredSet = false;
unregisteredDetails = baseDetails;
unregisteredDetails.explicitLabelOmissionRegistered = [false, false];
unregisteredDetails.explicitOmittedLabels = {zeros(2, 0), label};
try
    fuseLmbPosteriorsByLabel( ...
        {object, emptyPosterior}, weights, model, weights, ...
        unregisteredDetails, config);
catch exception
    didRejectUnregisteredSet = strcmp( ...
        exception.identifier, ...
        'LmbKla:UnregisteredExplicitOmittedLabels');
end
assert(didRejectUnregisteredSet);

fprintf('test_label_effective_omission_semantics passed\n');
end

function model = buildModel()
model = struct();
model.numberOfSensors = 2;
model.xDimension = 4;
model.detectionProbability = [0.9, 0.9];
model.Q = {eye(2), eye(2)};
model.sensorMotionEnabled = true;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = [60, 60];
model.sensorFovRange = [100, 100];
model.sensorFovHeadingRad = [0; 0];
model.sensorTrajectories = {zeros(4, 1), zeros(4, 1)};
model.sensorQuality = struct('enabled', false);
end

function object = buildObject(mean, existence)
object = struct( ...
    'birthTime', 1, ...
    'birthLocation', 1, ...
    'r', existence, ...
    'numberOfGmComponents', 1, ...
    'w', 1, ...
    'mu', {{mean}}, ...
    'Sigma', {{eye(4)}});
end
