function [config, details] = ...
    buildFormationB4V45FixedTriggerConfig(armId, nodeCount)
% BUILDFORMATIONB4V45FIXEDTRIGGERCONFIG Frozen three-arm V45 runtime.
%
% No caller overrides are accepted.  The three returned trigger configs
% differ only in topologyPolicyName and topologyPolicyFcn; every fusion,
% communication and observable-input setting is shared exactly.

if nargin ~= 2 || ~ischar(armId) || ~isrow(armId) || ...
        ~isa(nodeCount, 'double') || ~isreal(nodeCount) || ...
        ~isscalar(nodeCount) || ~isfinite(nodeCount) || ...
        nodeCount < 1 || nodeCount ~= round(nodeCount) || ...
        nodeCount > flintmax
    error('FormationB4V45TriggerConfig:InvalidInput', ...
        ['An exact registered arm identifier and a positive exactly ', ...
         'represented integer node count are required.']);
end

switch armId
    case 'v43-reference-a70-e05'
        armOrdinal = 1;
        policyFunctionName = ...
            'selectFormationB4V45ReferenceRuntimePolicy';
        policyFunction = ...
            @selectFormationB4V45ReferenceRuntimePolicy;
    case 'v44-sync-all-b4-e20-mc'
        armOrdinal = 2;
        policyFunctionName = ...
            'selectFormationB4V45SynchronizedRuntimePolicy';
        policyFunction = ...
            @selectFormationB4V45SynchronizedRuntimePolicy;
    case 'v44-formation-all-b4-e20-mc'
        armOrdinal = 3;
        policyFunctionName = ...
            'selectFormationB4V45FormationStaggeredRuntimePolicy';
        policyFunction = ...
            @selectFormationB4V45FormationStaggeredRuntimePolicy;
    otherwise
        error('FormationB4V45TriggerConfig:UnknownArm', ...
            'Only the three frozen V45 arm identifiers are allowed.');
end

assertSimplePolicyHandle(policyFunction, policyFunctionName);

config = buildMixtureAwareKlaReferenceConfig();
config.eventPolicy = 'alwaysHeavy';
config.dynamicTopologyEnabled = true;
config.topologyDirectedEnabled = true;
config.topologyDirectedMessageBudget = 2 * nodeCount;
config.topologyPolicyObservableContextOnly = true;
config.topologyPolicyPhysicalIdentityEnabled = true;
config.topologyPolicyName = policyFunctionName;
config.topologyPolicyFcn = policyFunction;

configSnapshot = rmfield(config, 'topologyPolicyFcn');
configSnapshot.topologyPolicyFunctionName = policyFunctionName;
fingerprintPayload = struct();
fingerprintPayload.contractVersion = ...
    'formation-b4-v45-fixed-trigger-config-v1';
fingerprintPayload.armId = armId;
fingerprintPayload.armOrdinal = armOrdinal;
fingerprintPayload.nodeCount = nodeCount;
fingerprintPayload.configSnapshot = configSnapshot;

details = fingerprintPayload;
details.allowedCrossArmConfigDifferenceFields = { ...
    'topologyPolicyName', 'topologyPolicyFcn'};
details.policyFunctionHandleExcluded = true;
details.overridesAccepted = false;
details.canonicalSha256 = ...
    computeCanonicalValueSha256(fingerprintPayload);
end

function assertSimplePolicyHandle(handle, expectedName)
identity = functions(handle);
if ~isa(handle, 'function_handle') || ...
        ~isstruct(identity) || ~isscalar(identity) || ...
        ~isfield(identity, 'type') || ...
        ~strcmp(identity.type, 'simple') || ...
        ~strcmp(func2str(handle), expectedName)
    error('FormationB4V45TriggerConfig:InvalidPolicyHandle', ...
        'A frozen V45 policy did not resolve to its simple wrapper.');
end
end
