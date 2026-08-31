function test_topology_policy_observable_context()
% TEST_TOPOLOGY_POLICYOBSERVABLECONTEXT Filter callback isolation.

global topologyPolicyObservableContextTestState;
cleanup = onCleanup(@clearTestState); %#ok<NASGU>
sensorCount = 2;
timeCount = 2;
model = generateMultisensorModel( ...
    sensorCount, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
model.dynamicTopologyScenario = struct( ...
    'config', struct( ...
        'sensorGroupIds', [1, 2], ...
        'targetCount', 99), ...
    'targetTrajectories', {{ones(4, timeCount)}}, ...
    'groundTruthRfs', struct('cardinality', [7, 8]), ...
    'graphData', struct('physicalAdjacency', ...
        true(sensorCount, sensorCount, timeCount)));
model.targetTrajectories = {2 * ones(4, timeCount)};
model.futureModelLeak = struct('groundTruth', 42);
measurements = repmat({{}}, sensorCount, timeCount);
sensorTrajectories = { ...
    [0, 1; 0, 0; 0, 0; 0, 0], ...
    [10, 11; 0, 0; 0, 0; 0, 0]};
neighborMap = {[1, 2], [1, 2]};
pDropByEdge = zeros(sensorCount, sensorCount, timeCount);
pDropByEdge(:, :, 1) = [0, 0.25; 0.25, 0];
pDropByEdge(:, :, 2) = [0, 0.75; 0.75, 0];
linkUniforms = 0.5 * ones(sensorCount, sensorCount, timeCount);
linkUniforms(:, :, 1) = [0.5, 0.10; 0.10, 0.5];
linkUniforms(:, :, 2) = [0.5, 0.90; 0.90, 0.5];
commConfig = struct( ...
    'pDropByEdge', pDropByEdge, ...
    'linkUniforms', linkUniforms, ...
    'nestedTruth', struct( ...
        'targetTrajectories', {{zeros(4, timeCount)}}));
baseConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'dynamicTopologyEnabled', true, ...
    'dynamicTopologyEdgeBudget', 1, ...
    'topologyPolicyFcn', @observablePolicyCallback, ...
    'nestedLeak', struct( ...
        'groundTruth', 1, ...
        'linkUniforms', linkUniforms), ...
    'futurePolicyData', ones(1, timeCount));

topologyPolicyObservableContextTestState = struct( ...
    'mode', 'observable', ...
    'pDropByEdge', pDropByEdge, ...
    'linkUniforms', linkUniforms, ...
    'sensorTrajectories', {sensorTrajectories}, ...
    'captured', {cell(1, timeCount)});
observableConfig = baseConfig;
observableConfig.topologyPolicyObservableContextOnly = true;
rng(1701);
[~, observableDiagnostics] = ...
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, observableConfig);
assert(all(observableDiagnostics. ...
    topologyPolicyObservableContextOnly));
for currentTime = 1:timeCount
    captured = topologyPolicyObservableContextTestState. ...
        captured{currentTime};
    assert(isstruct(captured) && isscalar(captured));
    contract = captured.observableInputContract;
    assert(contract.passed && contract.enforced);
    assert(strcmp(contract.contractVersion, ...
        'topology-policy-observable-input-v2'));
    assert(contract.currentTime == currentTime);
    assert(isempty(contract.forbiddenFieldPaths));
    assert(isequal(observableDiagnostics. ...
        topologyPolicyObservableInputContract{currentTime}, ...
        contract));
end
assert(~observableDiagnostics.delivered(1, 2, 1));
assert(~observableDiagnostics.delivered(2, 1, 1));
assert(observableDiagnostics.delivered(1, 2, 2));
assert(observableDiagnostics.delivered(2, 1, 2));

topologyPolicyObservableContextTestState.mode = 'posterior-history';
topologyPolicyObservableContextTestState.captured = ...
    cell(1, timeCount);
historyConfig = observableConfig;
historyConfig.topologyPolicyLocalPosteriorHistoryEnabled = true;
historyConfig.topologyPolicyLocalPosteriorHistoryDepth = 1;
rng(1701);
runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, historyConfig);
for currentTime = 1:timeCount
    captured = topologyPolicyObservableContextTestState. ...
        captured{currentTime};
    contract = captured.observableInputContract;
    assert(contract.localPosteriorHistoryPresent);
    assert(contract.pastLocalPosteriorHistoryOnly);
    assert(contract.localPosteriorHistoryAligned);
    assert(contract.localPosteriorHistorySchemaRestricted);
    assert(contract.localPosteriorHistoryHasNoFutureTimestamps);
    expectedCount = double(currentTime > 1);
    assert(captured.previousLocalPosteriorHistoryCount == ...
        expectedCount);
    assert(numel(captured.previousLocalPosteriorHistory) == ...
        expectedCount);
    if expectedCount > 0
        assert(isequal(captured.previousLocalPosteriorHistoryTimes, ...
            currentTime - 1));
        assert(numel(captured.previousLocalPosteriorHistory{1}) == ...
            sensorCount);
    end
end

topologyPolicyObservableContextTestState.mode = 'legacy';
topologyPolicyObservableContextTestState.captured = ...
    cell(1, timeCount);
legacyConfig = baseConfig;
rng(1701);
[~, legacyDiagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, legacyConfig);
assert(~any(legacyDiagnostics. ...
    topologyPolicyObservableContextOnly));
assert(all(cellfun(@isempty, legacyDiagnostics. ...
    topologyPolicyObservableInputContract)));
assert(isequal(legacyDiagnostics.delivered, ...
    observableDiagnostics.delivered));
for currentTime = 1:timeCount
    captured = topologyPolicyObservableContextTestState. ...
        captured{currentTime};
    assert(isfield(captured, 'sensorTrajectories'));
    assert(isfield(captured.model, 'sensorTrajectories'));
    assert(isfield(captured.model, 'targetTrajectories'));
    assert(isfield(captured.model.dynamicTopologyScenario, ...
        'targetTrajectories'));
    assert(isfield(captured.model.dynamicTopologyScenario.config, ...
        'targetCount'));
    assert(isfield(captured.commConfig, 'linkUniforms'));
    assert(size(captured.commConfig.pDropByEdge, 3) == timeCount);
    assert(size(captured.commConfig.linkUniforms, 3) == timeCount);
    assert(isfield(captured.triggerConfig, 'topologyPolicyFcn'));
    assert(isfield(captured.triggerConfig.nestedLeak, ...
        'groundTruth'));
    assert(~isfield(captured, 'observableInputContract'));
end
assertFuturePosteriorRejected(captured, timeCount);
assertTriggerContainerRejected(captured);
assertPosteriorSideChannelRejected(captured, timeCount);
assertHistoryContainerRejected(captured, timeCount);

fprintf('PASS: topology-policy observable-context tests\n');
end

function assertFuturePosteriorRejected(context, currentTime)
object = context.model.birthParameters(1);
object.birthTime = 1;
object.timestamps = [1, currentTime + 1];
object.numberOfGmComponents = 1;
object.w = 1;
object.mu = {zeros(context.model.xDimension, 1)};
object.Sigma = {eye(context.model.xDimension)};
context.currentTime = currentTime;
context.localPosteriorBySensor{1} = object;
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:InvalidPosterior');
end
assert(failed);
end

function assertTriggerContainerRejected(context)
context.triggerConfig.payloadExistenceThreshold = ...
    struct('targetTruth', 1);
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:InvalidTriggerConfig');
end
assert(failed);
end

function assertPosteriorSideChannelRejected(context, currentTime)
object = context.model.birthParameters(1);
object.birthTime = currentTime;
object.oracle = @() 42;
object.dropDrawAlias = 0.123;
context.currentTime = currentTime;
context.localPosteriorBySensor{1} = object;
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:InvalidPosterior');
end
assert(failed);
end

function assertHistoryContainerRejected(context, currentTime)
context.currentTime = currentTime;
context.localInnovationHistory = containers.Map( ...
    {'oracle'}, {@() 42});
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:ContractViolation');
end
assert(failed);
end

function [adjacency, details] = observablePolicyCallback(context)
global topologyPolicyObservableContextTestState;
currentTime = context.currentTime;
if ismember(topologyPolicyObservableContextTestState.mode, { ...
        'observable', 'posterior-history'})
    assert(isfield(context, 'observableInputContract'));
    contract = context.observableInputContract;
    assert(contract.passed);
    assert(contract.targetTruthAbsent);
    assert(contract.fullDynamicScenarioAbsent);
    assert(contract.modelStaticFusionInputsOnly);
    assert(contract.modelObjectTemplateEmpty);
    assert(contract.triggerConfigStaticFusionInputsOnly);
    assert(contract.triggerConfigPrimitiveValuesOnly);
    assert(contract.linkUniformsAbsent);
    assert(contract.futurePDropPagesAbsent);
    assert(contract.sensorTrajectoriesAbsent);
    assert(contract.currentPositionsOnly);
    assert(contract.currentPosteriorPresent);
    assert(contract.posteriorSchemaRestricted);
    assert(contract.posteriorTrajectoryFieldsAbsent);
    assert(contract.contextHasNoFunctionHandles);
    assert(contract.contextUsesPrimitiveContainersOnly);
    assert(contract.causalArraySchemaRestricted);
    assert(contract.currentPosteriorHasNoFutureTimestamps);
    assert(contract.pastTopologyHistoryOnly);
    assert(contract.nonfutureUpdateHistoryOnly);
    assert(contract.modelScenarioGroupIdsOnly);
    observableData = rmfield(context, 'observableInputContract');
    assert(~containsForbiddenField(observableData));
    assert(~isfield(context, 'sensorTrajectories'));
    assert(~isfield(context.model, 'sensorTrajectories'));
    assert(~isfield(context.model, 'targetTrajectories'));
    assert(isequal(fieldnames(context.model), { ...
        'xDimension'; 'existenceThreshold'; 'object'; ...
        'ospaParameters'; ...
        'dynamicTopologyScenario'}));
    assert(~contract.currentSensorObservationEnabled);
    assert(contract.currentSensorObservationRestricted);
    assert(isequal(fieldnames( ...
        context.model.dynamicTopologyScenario), {'config'}));
    assert(isequal(fieldnames( ...
        context.model.dynamicTopologyScenario.config), ...
        {'sensorGroupIds'}));
    assert(isequal(context.model.dynamicTopologyScenario. ...
        config.sensorGroupIds, [1, 2]));
    assert(isequal(fieldnames(context.commConfig), ...
        {'pDropByEdge'}));
    assert(ndims(context.commConfig.pDropByEdge) == 2);
    assert(isequal(context.commConfig.pDropByEdge, ...
        topologyPolicyObservableContextTestState. ...
            pDropByEdge(:, :, currentTime)));
    assert(~isfield(context.triggerConfig, 'topologyPolicyFcn'));
    assert(~isfield(context.triggerConfig, 'futurePolicyData'));
    assert(~isfield(context.triggerConfig, 'nestedLeak'));
    posteriorFields = { ...
        'birthLocation'; 'birthTime'; 'r'; ...
        'numberOfGmComponents'; 'w'; 'mu'; 'Sigma'; ...
        'associationEntropy'; 'detectionAssociationEntropy'; ...
        'detectionAssociationMass'; 'associationAmbiguity'; ...
        'associationConfidence'};
    for sensorIdx = 1:numel(context.localPosteriorBySensor)
        assert(isequal(sort(fieldnames( ...
            context.localPosteriorBySensor{sensorIdx})), ...
            sort(posteriorFields)));
    end
    assert(isequal(context.positions, cell2mat(cellfun( ...
        @(trajectory) trajectory(1:2, currentTime), ...
        topologyPolicyObservableContextTestState.sensorTrajectories, ...
        'UniformOutput', false))));
    assert(all(context.previousAdjacencyHistoryTimes < currentTime));
    assert(all(context.localUpdateHistoryTimes <= currentTime));
    if strcmp(topologyPolicyObservableContextTestState.mode, ...
            'posterior-history')
        assert(isfield(context, 'previousLocalPosteriorHistory'));
    end
else
    assert(~isfield(context, 'observableInputContract'));
end
topologyPolicyObservableContextTestState.captured{currentTime} = ...
    context;
adjacency = logical([0, 1; 1, 0]);
details = struct('truthUsed', false);
end

function result = containsForbiddenField(value)
result = false;
if iscell(value)
    for valueIdx = 1:numel(value)
        if containsForbiddenField(value{valueIdx})
            result = true;
            return;
        end
    end
    return;
end
if ~isstruct(value)
    return;
end
names = fieldnames(value);
for fieldIdx = 1:numel(names)
    name = lower(names{fieldIdx});
    forbidden = any(strcmp(name, { ...
        'groundtruth', 'groundtruthrfs', 'truth', ...
        'trackingtruth', 'trackingtruthrfs', ...
        'targettrajectories', 'sensortrajectories', ...
        'linkuniforms', 'measurements', 'topologypolicyfcn'})) || ...
        strncmp(name, 'future', 6) || ...
        ~isempty(strfind(name, 'groundtruth')) || ... %#ok<STREMP>
        ~isempty(strfind(name, 'targettrajector')); %#ok<STREMP>
    if forbidden
        result = true;
        return;
    end
end
for valueIdx = 1:numel(value)
    for fieldIdx = 1:numel(names)
        if containsForbiddenField( ...
                value(valueIdx).(names{fieldIdx}))
            result = true;
            return;
        end
    end
end
end

function clearTestState()
global topologyPolicyObservableContextTestState;
topologyPolicyObservableContextTestState = [];
end
