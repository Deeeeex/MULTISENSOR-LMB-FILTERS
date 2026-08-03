function testFormationB4V45FixedTriggerConfig()
% Frozen construction and cross-arm parity for the V45 trigger configs.

armIds = { ...
    'v43-reference-a70-e05', ...
    'v44-sync-all-b4-e20-mc', ...
    'v44-formation-all-b4-e20-mc'};
policyNames = { ...
    'selectFormationB4V45ReferenceRuntimePolicy', ...
    'selectFormationB4V45SynchronizedRuntimePolicy', ...
    'selectFormationB4V45FormationStaggeredRuntimePolicy'};
nodeCount = 36;
configs = cell(1, numel(armIds));
detailsByArm = cell(1, numel(armIds));
for armIdx = 1:numel(armIds)
    [config, details] = buildFormationB4V45FixedTriggerConfig( ...
        armIds{armIdx}, nodeCount);
    configs{armIdx} = config;
    detailsByArm{armIdx} = details;

    assert(strcmp(config.eventPolicy, 'alwaysHeavy'));
    assert(config.dynamicTopologyEnabled);
    assert(config.topologyDirectedEnabled);
    assert(config.topologyDirectedMessageBudget == 2 * nodeCount);
    assert(config.topologyPolicyObservableContextOnly);
    assert(config.topologyPolicyPhysicalIdentityEnabled);
    assert(strcmp(config.topologyPolicyName, policyNames{armIdx}));
    assert(isa(config.topologyPolicyFcn, 'function_handle'));
    assert(strcmp(func2str(config.topologyPolicyFcn), ...
        policyNames{armIdx}));

    assert(strcmp(details.contractVersion, ...
        'formation-b4-v45-fixed-trigger-config-v1'));
    assert(strcmp(details.armId, armIds{armIdx}));
    assert(details.armOrdinal == armIdx);
    assert(details.nodeCount == nodeCount);
    assert(~details.overridesAccepted);
    assert(details.policyFunctionHandleExcluded);
    assert(isequal(details.allowedCrossArmConfigDifferenceFields, ...
        {'topologyPolicyName', 'topologyPolicyFcn'}));
    assert(~containsFunctionHandle(details));
    assert(isSha256(details.canonicalSha256));

    [repeatConfig, repeatDetails] = ...
        buildFormationB4V45FixedTriggerConfig( ...
            armIds{armIdx}, nodeCount);
    assert(isequaln(config, repeatConfig));
    assert(isequaln(details, repeatDetails));
end

allowedDifferences = {'topologyPolicyName', 'topologyPolicyFcn'};
sharedReference = rmfield(configs{1}, allowedDifferences);
for armIdx = 2:numel(configs)
    sharedCandidate = rmfield(configs{armIdx}, allowedDifferences);
    assert(isequaln(sharedReference, sharedCandidate));
    assert(isequal(fieldnames(configs{1}), fieldnames(configs{armIdx})));
    assert(~strcmp(configs{1}.topologyPolicyName, ...
        configs{armIdx}.topologyPolicyName));
    assert(~isequal(configs{1}.topologyPolicyFcn, ...
        configs{armIdx}.topologyPolicyFcn));
end

base = buildMixtureAwareKlaReferenceConfig();
common = configs{1};
common = rmfield(common, [allowedDifferences, { ...
    'dynamicTopologyEnabled', 'topologyDirectedEnabled', ...
    'topologyDirectedMessageBudget', ...
    'topologyPolicyObservableContextOnly', ...
    'topologyPolicyPhysicalIdentityEnabled'}]);
assert(isequaln(base, common));

assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    'unregistered-arm', 24), ...
    'FormationB4V45TriggerConfig:UnknownArm');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig('', 24), ...
    'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, 0), 'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, -1), 'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, 1.5), 'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, NaN), 'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, Inf), 'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, [24, 36]), ...
    'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    armIds{1}, single(24)), ...
    'FormationB4V45TriggerConfig:InvalidInput');
assertErrorId(@() buildFormationB4V45FixedTriggerConfig( ...
    42, 24), 'FormationB4V45TriggerConfig:InvalidInput');

fprintf('PASS: FormationB4V45 fixed trigger-config tests\n');
end

function found = containsFunctionHandle(value)
found = isa(value, 'function_handle');
if found
    return;
end
if iscell(value)
    for valueIdx = 1:numel(value)
        if containsFunctionHandle(value{valueIdx})
            found = true;
            return;
        end
    end
elseif isstruct(value)
    names = fieldnames(value);
    for valueIdx = 1:numel(value)
        for fieldIdx = 1:numel(names)
            if containsFunctionHandle(value(valueIdx).(names{fieldIdx}))
                found = true;
                return;
            end
        end
    end
end
end

function valid = isSha256(value)
valid = ischar(value) && isrow(value) && numel(value) == 64 && ...
    all(ismember(lower(value), '0123456789abcdef'));
end

function assertErrorId(handle, expected)
failed = false;
try
    handle();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expected);
end
assert(failed);
end
