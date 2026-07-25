function test_dynamic_topology_scenarios()
% TEST_DYNAMIC_TOPOLOGY_SCENARIOS Scenario, safety and KLA reference tests.

testScenarioPresets();
testScheduledBirths();
testTimeVaryingDropAccounting();
testInfeasiblePhysicalGraphFailsClosed();
testMixtureAwareReferenceBoundary();
testD12OracleCallbackSmoke();
fprintf('test_dynamic_topology_scenarios passed\n');
end

function testScenarioPresets()
names = {'r8-legacy', 'd12-handover', ...
    'm24-handover', 'm24-link', 'm24-composite', ...
    'x36-topology', 'x36-joint'};
expectedSensors = [8, 12, 24, 24, 24, 36, 36];
for presetIdx = 1:numel(names)
    rng(7);
    config = buildDynamicTopologyScenarioConfig(names{presetIdx});
    [sensors, ~] = generateMultiFormationTrajectories(config);
    [targets, ~] = generateCorridorTargetTrajectories(config);
    graphs = buildDynamicTopologyGraphs(config, sensors);
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graphs);
    assert(validation.isValid);
    assert(config.numberOfSensors == expectedSensors(presetIdx));
    assert(validation.staticEdgeCount <= config.edgeBudget);
    assert(validation.staticPhysicalViolationCount == 0);
end

rng(7);
config = buildDynamicTopologyScenarioConfig('d12-handover');
[sensors, ~] = generateMultiFormationTrajectories(config);
graphs = buildDynamicTopologyGraphs(config, sensors);
assert(size(graphs.candidateAdjacency, 3) == 48);
for candidateIdx = 1:size(graphs.candidateAdjacency, 3)
    assert(nnz(triu( ...
        graphs.candidateAdjacency(:, :, candidateIdx), 1)) == 14);
end
end

function testScheduledBirths()
inputs = generateDynamicTopologyScenarioInputs('d12-handover', 7);
model = inputs.model;
objects = model.object;
objects = lmbPredictionStep(objects, model, 1);
assert(numel(objects) == 3);
objects = lmbPredictionStep(objects, model, 2);
assert(numel(objects) == 3);
for timeIdx = 3:11
    objects = lmbPredictionStep(objects, model, timeIdx);
end
assert(numel(objects) == 6);
birthTimes = sort([objects.birthTime]);
assert(isequal(birthTimes, [1, 1, 1, 11, 11, 11]));
end

function testTimeVaryingDropAccounting()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
for birthIdx = 1:numel(model.birthParameters)
    model.birthParameters(birthIdx).r = 0.9;
end
measurements = repmat({{}}, 2, 2);
neighborMap = {[1, 2], [1, 2]};
pDrop = zeros(2, 2, 2);
pDrop(:, :, 2) = 1;
commConfig = struct( ...
    'pDropBySensor', [0, 0], ...
    'pDropByEdge', pDrop, ...
    'linkUniforms', 0.5 * ones(2, 2, 2));
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, [], neighborMap, commConfig, triggerConfig);
assert(diagnostics.summary.attemptCount == 4);
assert(diagnostics.summary.deliveryCount == 2);
assert(diagnostics.summary.attemptedPayloadBytes > ...
    diagnostics.summary.payloadBytes);
assert(diagnostics.summary.payloadBytes > 0);
end

function testInfeasiblePhysicalGraphFailsClosed()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
model.sensorCommRange = 1;
measurements = repmat({{}}, 2, 2);
sensorTrajectories = { ...
    repmat([0; 0; 0; 0], 1, 2), ...
    repmat([100; 0; 0; 0], 1, 2)};
neighborMap = {[1, 2], [1, 2]};
commConfig = struct('forceDelivery', true, 'pDropBySensor', [0, 0]);
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'dynamicTopologyEnabled', true, ...
    'dynamicTopologyEdgeBudget', 1);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
assert(all(diagnostics.topologyUndirectedEdgeCount == 0));
assert(all(~diagnostics.topologyFeasible));
assert(diagnostics.summary.attemptCount == 0);
end

function testMixtureAwareReferenceBoundary()
model = generateMultisensorModel( ...
    2, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
left = makeObject(model, 1, 1, 0.8, ...
    [-2; 0; 0; 0], 2 * eye(4));
right = makeObject(model, 1, 1, 0.8, ...
    [2; 0; 0; 0], 3 * eye(4));
config = buildMixtureAwareKlaReferenceConfig();
details = struct('eventType', [0, 2]);
fused = fuseLmbPosteriorsByLabel( ...
    {left, right}, [0.5, 0.5], model, [0.5, 0.5], ...
    details, config);
expectedCovariance = inv( ...
    0.5 * inv(left.Sigma{1}) + 0.5 * inv(right.Sigma{1}));
expectedMean = expectedCovariance * ( ...
    0.5 * (left.Sigma{1} \ left.mu{1}) + ...
    0.5 * (right.Sigma{1} \ right.mu{1}));
assert(fused.numberOfGmComponents == 1);
assert(norm(fused.mu{1} - expectedMean) < 1e-10);
assert(norm(fused.Sigma{1} - expectedCovariance, 'fro') < 1e-10);

bimodal = makeObject(model, 1, 2, 0.8, ...
    [-8; 0; 0; 0], eye(4));
bimodal.numberOfGmComponents = 2;
bimodal.w = [0.5, 0.5];
bimodal.mu = {[-8; 0; 0; 0], [8; 0; 0; 0]};
bimodal.Sigma = {eye(4), eye(4)};
fixedPoint = fuseLmbPosteriorsByLabel( ...
    {bimodal, bimodal}, [0.5, 0.5], model, [0.5, 0.5], ...
    details, config);
assert(fixedPoint.numberOfGmComponents >= 2);
[meanVector, covariance] = objectMoments(fixedPoint);
[referenceMean, referenceCovariance] = objectMoments(bimodal);
assert(norm(meanVector - referenceMean) < 1e-6);
assert(norm(covariance - referenceCovariance, 'fro') < 1e-3);
assert(abs(fixedPoint.r - bimodal.r) < 1e-3);
end

function testD12OracleCallbackSmoke()
inputs = generateDynamicTopologyScenarioInputs('d12-handover', 13);
inputs.measurements = inputs.measurements(:, 1);
inputs.sensorTrajectories = cellfun( ...
    @(x) x(:, 1), inputs.sensorTrajectories, ...
    'UniformOutput', false);
inputs.commConfig.pDropByEdge = ...
    inputs.commConfig.pDropByEdge(:, :, 1);
inputs.commConfig.linkUniforms = ...
    inputs.commConfig.linkUniforms(:, :, 1);
config = buildMixtureAwareKlaReferenceConfig();
config.dynamicTopologyEnabled = true;
config.dynamicTopologyEdgeBudget = inputs.config.edgeBudget;
config.topologyPolicyName = 'oracle-consensus';
config.topologyPolicyFcn = ...
    @(context) selectD12TopologyPolicy(context, 'oracle-consensus');
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    inputs.model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, config);
assert(diagnostics.topologyFeasible(1));
assert(diagnostics.topologyUndirectedEdgeCount(1) == ...
    inputs.config.edgeBudget);
assert(isfinite(diagnostics.topologyPolicyCandidateIndex(1)));
assert(isfinite(diagnostics.topologyPolicyObjective(1)));

uncachedConfig = config;
uncachedConfig.topologyOracleFusionCacheEnabled = false;
[~, uncachedDiagnostics] = runEventTriggeredDistributedLmbFilter( ...
    inputs.model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, uncachedConfig);
assert(uncachedDiagnostics.topologyPolicyCandidateIndex(1) == ...
    diagnostics.topologyPolicyCandidateIndex(1));
assert(abs(uncachedDiagnostics.topologyPolicyObjective(1) - ...
    diagnostics.topologyPolicyObjective(1)) < 1e-10);
end

function object = makeObject(model, birthTime, birthLocation, r, mu, Sigma)
object = model.birthParameters(1);
object.birthTime = birthTime;
object.birthLocation = birthLocation;
object.r = r;
object.numberOfGmComponents = 1;
object.w = 1;
object.mu = {mu};
object.Sigma = {Sigma};
end

function [meanVector, covariance] = objectMoments(object)
weights = reshape(object.w, 1, []);
weights = weights / sum(weights);
meanVector = zeros(size(object.mu{1}));
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(size(object.Sigma{1}));
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
end
