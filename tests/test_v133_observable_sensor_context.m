function test_v133_observable_sensor_context()
% TEST_V133_OBSERVABLE_SENSOR_CONTEXT Current-only FoV model boundary.

sensorCount = 2;
currentTime = 2;
model = generateMultisensorModel( ...
    sensorCount, [0, 0], [0.9, 0.8], [2, 3], 'GA', 'LBP');
model.sensorMotionEnabled = true;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = [60, 55];
model.sensorFovRange = [100, 90];
model.sensorFovHeadingRad = [pi, 0; -pi / 2, pi / 2];
model.sensorTrajectories = { ...
    [0, 1; 0, 2; 1, 1; 0, 0], ...
    [10, 11; 0, 3; 1, 1; 0, 0]};
model.sensorQuality = struct('enabled', false);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', [1, 2]));
positions = [1, 11; 2, 3];
model = attachCurrentObservableSensorGeometry( ...
    model, positions, currentTime, sensorCount);

context = buildContext(model, positions, currentTime, sensorCount);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig(struct( ...
    'topologyPolicySensorObservationEnabled', true, ...
    'missingLabelFusionMode', 'fov-aware-censored', ...
    'missingNeighborWeightMode', 'renormalize'));
observable = buildObservableTopologyPolicyContext(context);
contract = observable.observableInputContract;
assert(contract.passed);
assert(contract.currentSensorObservationEnabled);
assert(contract.currentSensorObservationRestricted);
assert(strcmp(contract.contractVersion, ...
    'topology-policy-observable-input-v5-current-sensor-observation'));
assert(~isfield(observable.model, 'sensorTrajectories'));
assert(~isfield(observable.model, 'sensorFovHeadingRad'));
assert(isequal(observable.model.sensorObservation.currentPosition, ...
    positions));
assert(isequal(observable.model.sensorObservation. ...
    currentFovHeadingRad, [0, pi / 2]));
assert(strcmp(observable.triggerConfig.missingLabelFusionMode, ...
    'fov-aware-censored'));

materialized = materializeObservableSensorFusionModel( ...
    observable.model, currentTime);
assert(isequal(materialized.sensorFovHeadingRad(:, currentTime), ...
    [0; pi / 2]));
assert(isequal(materialized.sensorTrajectories{1}(1:2, currentTime), ...
    positions(:, 1)));
assert(~materialized.observableSensorGeometryUsesFuturePage);

fprintf('PASS: V133 current sensor-observation context\n');
end

function context = buildContext(model, positions, currentTime, sensorCount)
emptyPosterior = repmat({model.object}, 1, sensorCount);
context = struct();
context.localPosteriorBySensor = emptyPosterior;
context.model = model;
context.commConfig = struct('pDropByEdge', zeros(sensorCount));
context.triggerConfig = struct();
context.currentTime = currentTime;
context.previousAdjacency = false(sensorCount);
context.previousAdjacencyHistory = false(sensorCount, sensorCount, 1);
context.previousAdjacencyHistoryCount = 1;
context.previousAdjacencyHistoryTimes = 1;
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
context.previousAdjacencyHistorySource = ...
    'selected-validated-topology';
context.baseAdjacency = false(sensorCount);
context.physicalAdjacency = false(sensorCount);
context.edgeScores = zeros(sensorCount);
context.edgeBudget = 0;
context.directedMessageBudget = 0;
context.positions = positions;
context.localInnovationHistory = zeros(sensorCount, 1);
context.localAssociationConfidenceHistory = zeros(sensorCount, 1);
context.localNisNormHistory = zeros(sensorCount, 1);
context.localNisDeviationHistory = zeros(sensorCount, 1);
context.localUpdateHistoryTimes = currentTime;
end
