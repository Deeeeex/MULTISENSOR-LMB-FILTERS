function test_topology_policy_physical_identity_context()
% TEST_TOPOLOGY_POLICYPHYSICALIDENTITYCONTEXT Runtime-bound UID context.

global topologyPolicyPhysicalIdentityTestState;
cleanup = onCleanup(@clearTestState); %#ok<NASGU>
sensorCount = 2;
timeCount = 2;
model = generateMultisensorModel( ...
    sensorCount, [0, 0], [0.9, 0.9], [3, 3], 'GA', 'LBP');
sceneConfig = struct();
sceneConfig.formationCount = 1;
sceneConfig.sensorsPerFormation = sensorCount;
sceneConfig.sensorCenterWaypoints = {[0; 0]};
sceneConfig.sensorGroupIds = [1, 1];
identity = buildDynamicTopologyPhysicalIdentityRegistry(sceneConfig);
sceneConfig.sensorPhysicalUids = identity.sensorPhysicalUids;
sceneConfig.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
sceneConfig.sensorLocalRoleUidsByFormation = ...
    identity.sensorLocalRoleUidsByFormation;
sceneConfig.physicalIdentityRegistryCanonicalSha256 = ...
    identity.canonicalSha256;
model.dynamicTopologyScenario = struct('config', sceneConfig);

measurements = repmat({{}}, sensorCount, timeCount);
sensorTrajectories = { ...
    [0, 1; 0, 0; 0, 0; 0, 0], ...
    [10, 11; 0, 0; 0, 0; 0, 0]};
neighborMap = {[1, 2], [1, 2]};
commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(sensorCount, sensorCount, timeCount), ...
    'linkUniforms', 0.5 * ones(sensorCount, sensorCount, timeCount));
triggerConfig = struct( ...
    'eventPolicy', 'alwaysHeavy', ...
    'linkGateEnabled', false, ...
    'dynamicTopologyEnabled', true, ...
    'dynamicTopologyEdgeBudget', 1, ...
    'topologyPolicyFcn', @capturePhysicalIdentityPolicy, ...
    'topologyPolicyObservableContextOnly', true, ...
    'topologyPolicyPhysicalIdentityEnabled', true);

topologyPolicyPhysicalIdentityTestState = struct( ...
    'captured', {cell(1, timeCount)});
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
for currentTime = 1:timeCount
    captured = topologyPolicyPhysicalIdentityTestState. ...
        captured{currentTime};
    assert(isequal(captured.sensorPhysicalUids, ...
        identity.sensorPhysicalUids));
    assert(isequal(captured.formationPhysicalUidsBySensor, ...
        identity.formationPhysicalUidsBySensor));
    assert(strcmp(captured.physicalIdentityRegistryCanonicalSha256, ...
        identity.canonicalSha256));
    contract = captured.observableInputContract;
    assert(contract.passed && contract.physicalIdentityPresent);
    assert(contract.physicalIdentitySchemaRestricted);
    assert(strcmp(contract.contractVersion, ...
        'topology-policy-observable-input-v3-physical-uid'));
    assert(contract.targetTruthAbsent && contract.linkUniformsAbsent);
    assert(contract.futurePDropPagesAbsent);
    assert(contract.directedTopologyRuntimeSemanticsPresent);
    assert(~contract.topologyDirectedEnabled);
    assert(isempty(contract.topologyDirectedMessageBudget));
    assert(strcmp(contract.physicalIdentityRegistryCanonicalSha256, ...
        identity.canonicalSha256));
    assert(isequal(diagnostics. ...
        topologyPolicyObservableInputContract{currentTime}, contract));
end

assertIncompleteTupleRejected( ...
    topologyPolicyPhysicalIdentityTestState.captured{1});
assertMembershipTamperRejected( ...
    topologyPolicyPhysicalIdentityTestState.captured{1});
assertRuntimeRegistryTamperRejected( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);
assertObservableBoundaryRequired(triggerConfig, model);

fprintf('PASS: topology-policy physical-identity context tests\n');
end

function [adjacency, details] = capturePhysicalIdentityPolicy(context)
global topologyPolicyPhysicalIdentityTestState;
topologyPolicyPhysicalIdentityTestState.captured{context.currentTime} = ...
    context;
adjacency = logical([0, 1; 1, 0]);
details = struct('truthUsed', false, 'futureOutcomeUsed', false);
end

function assertIncompleteTupleRejected(context)
context = rmfield(context, { ...
    'observableInputContract', ...
    'formationPhysicalUidsBySensor'});
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:IncompletePhysicalIdentity');
end
assert(failed);
end

function assertMembershipTamperRejected(context)
context = rmfield(context, 'observableInputContract');
context.formationPhysicalUidsBySensor(2) = ...
    context.formationPhysicalUidsBySensor(2) + 101;
failed = false;
try
    buildObservableTopologyPolicyContext(context);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ObservableTopologyContext:InvalidPhysicalIdentity');
end
assert(failed);
end

function assertRuntimeRegistryTamperRejected( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, triggerConfig)
model.dynamicTopologyScenario.config.sensorPhysicalUids(1) = ...
    model.dynamicTopologyScenario.config.sensorPhysicalUids(1) + 1;
failed = false;
try
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, triggerConfig);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'DynamicTopology:PhysicalIdentityRegistryDrift');
end
assert(failed);
end

function assertObservableBoundaryRequired(triggerConfig, model)
triggerConfig.topologyPolicyObservableContextOnly = false;
failed = false;
try
    resolveTriggerConfigForPhysicalIdentityTest(triggerConfig, model);
catch errorInfo
    failed = ~isempty(strfind(errorInfo.message, ... %#ok<STREMP>
        'observable-only topology context boundary'));
end
assert(failed);
end

function resolved = resolveTriggerConfigForPhysicalIdentityTest(config, model)
% Exercise the private resolver through a one-step filter invocation.
measurements = repmat({{}}, 2, 1);
trajectories = {zeros(4, 1), [10; 0; 0; 0]};
neighborMap = {[1, 2], [1, 2]};
commConfig = struct('forceDelivery', true);
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, trajectories, neighborMap, commConfig, config);
resolved = diagnostics; %#ok<NASGU>
end

function clearTestState()
global topologyPolicyPhysicalIdentityTestState;
topologyPolicyPhysicalIdentityTestState = [];
end
