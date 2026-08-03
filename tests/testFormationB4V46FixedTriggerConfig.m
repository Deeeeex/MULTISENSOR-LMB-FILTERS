function testFormationB4V46FixedTriggerConfig()
% Exact two-arm registration and V45-config parity for V46.

protocol = getFormationCausalMinimalEditV46Protocol();
arms = protocol.primaryArms;
parents = {'v43-reference-a70-e05', ...
    'v44-sync-all-b4-e20-mc'};
policyNames = { ...
    'selectFormationB4V46ReferenceRuntimePolicy', ...
    'selectFormationB4V46SynchronizedRuntimePolicy'};
nodeCount = 36;
configs = cell(1, numel(arms));

for armIdx = 1:numel(arms)
    [config, details] = ...
        buildFormationB4V46FixedTriggerConfig( ...
            arms{armIdx}, nodeCount);
    [parentConfig, parentDetails] = ...
        buildFormationB4V45FixedTriggerConfig( ...
            parents{armIdx}, nodeCount);
    configs{armIdx} = config;

    assert(strcmp(config.eventPolicy, 'alwaysHeavy'));
    assert(config.dynamicTopologyEnabled);
    assert(islogical(config.topologyDirectedEnabled) && ...
        isscalar(config.topologyDirectedEnabled) && ...
        config.topologyDirectedEnabled);
    assert(config.topologyDirectedMessageBudget == ...
        2 * nodeCount);
    assert(config.topologyPolicyObservableContextOnly);
    assert(config.topologyPolicyPhysicalIdentityEnabled);
    assert(strcmp(config.topologyPolicyName, ...
        policyNames{armIdx}));
    assert(isa(config.topologyPolicyFcn, 'function_handle'));
    assert(strcmp(func2str(config.topologyPolicyFcn), ...
        policyNames{armIdx}));

    assert(strcmp(details.contractVersion, ...
        'formation-b4-v46-fixed-trigger-config-v1'));
    assert(strcmp(details.armId, arms{armIdx}));
    assert(details.armOrdinal == armIdx);
    assert(strcmp(details.parentArmId, parents{armIdx}));
    assert(strcmp(details.parentConfigCanonicalSha256, ...
        parentDetails.canonicalSha256));
    assert(strcmp(details.protocolId, protocol.id));
    assert(strcmp(details.protocolCanonicalSha256, ...
        protocol.canonicalSha256));
    assert(details.nodeCount == nodeCount);
    assert(details.parentConfigReusedWithoutRelaxation);
    assert(details.policyFunctionHandleExcluded);
    assert(~details.overridesAccepted);
    assert(~containsFunctionHandle(details));

    actualParent = rmfield(parentConfig, { ...
        'topologyPolicyName', 'topologyPolicyFcn'});
    actualV46 = rmfield(config, { ...
        'topologyPolicyName', 'topologyPolicyFcn'});
    assert(isequaln(actualV46, actualParent));

    fingerprint = details;
    fingerprint = rmfield(fingerprint, { ...
        'allowedCrossArmConfigDifferenceFields', ...
        'parentConfigReusedWithoutRelaxation', ...
        'policyFunctionHandleExcluded', ...
        'overridesAccepted', 'canonicalSha256'});
    assert(strcmp(computeCanonicalValueSha256(fingerprint), ...
        details.canonicalSha256));
end

left = rmfield(configs{1}, { ...
    'topologyPolicyName', 'topologyPolicyFcn'});
right = rmfield(configs{2}, { ...
    'topologyPolicyName', 'topologyPolicyFcn'});
assert(isequaln(left, right));

assertErrorId(@() buildFormationB4V46FixedTriggerConfig( ...
    'v44-formation-all-b4-e20-mc', nodeCount), ...
    'FormationB4V46TriggerConfig:UnknownArm');
assertErrorId(@() buildFormationB4V46FixedTriggerConfig( ...
    arms{1}, 0), 'FormationB4V46TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V46FixedTriggerConfig( ...
    arms{1}, int32(nodeCount)), ...
    'FormationB4V46TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V46FixedTriggerConfig( ...
    arms(1), nodeCount), ...
    'FormationB4V46TriggerConfig:InvalidInput');

fprintf('PASS: FormationB4V46 fixed trigger config tests\n');
end

function found = containsFunctionHandle(value)
if isa(value, 'function_handle')
    found = true;
elseif isstruct(value)
    found = false;
    names = fieldnames(value);
    for fieldIdx = 1:numel(names)
        if containsFunctionHandle(value.(names{fieldIdx}))
            found = true;
            return;
        end
    end
elseif iscell(value)
    found = false;
    for cellIdx = 1:numel(value)
        if containsFunctionHandle(value{cellIdx})
            found = true;
            return;
        end
    end
else
    found = false;
end
end

function assertErrorId(callable, expectedId)
thrown = false;
try
    callable();
catch errorInfo
    thrown = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Unexpected error identifier: %s', errorInfo.identifier);
end
assert(thrown, 'Expected error was not thrown.');
end
