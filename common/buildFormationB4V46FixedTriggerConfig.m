function [config, details] = ...
    buildFormationB4V46FixedTriggerConfig(armId, nodeCount)
% BUILDFORMATIONB4V46FIXEDTRIGGERCONFIG Frozen two-arm V46 runtime.
%
% Each configuration starts from its exact V45 parent and changes only the
% public topology-policy wrapper.  Fusion, communication, directed-2N and
% observable-input settings are inherited byte-for-byte.

if nargin ~= 2 || ~ischar(armId) || ~isrow(armId) || ...
        ~isa(nodeCount, 'double') || ~isreal(nodeCount) || ...
        ~isscalar(nodeCount) || ~isfinite(nodeCount) || ...
        nodeCount < 1 || nodeCount ~= round(nodeCount) || ...
        nodeCount > flintmax
    error('FormationB4V46TriggerConfig:InvalidInput', ...
        'A frozen V46 arm and positive exactly represented N are required.');
end

protocol = getFormationCausalMinimalEditV46Protocol();
switch armId
    case 'v46-repaired-reference-a70-e05'
        armOrdinal = 1;
        parentArmId = 'v43-reference-a70-e05';
        policyFunctionName = ...
            'selectFormationB4V46ReferenceRuntimePolicy';
        policyFunction = @selectFormationB4V46ReferenceRuntimePolicy;
    case 'v46-repaired-sync-all-b4-e20-mc'
        armOrdinal = 2;
        parentArmId = 'v44-sync-all-b4-e20-mc';
        policyFunctionName = ...
            'selectFormationB4V46SynchronizedRuntimePolicy';
        policyFunction = @selectFormationB4V46SynchronizedRuntimePolicy;
    otherwise
        error('FormationB4V46TriggerConfig:UnknownArm', ...
            'Only the two frozen V46 primary arms are allowed.');
end
if ~strcmp(protocol.primaryArms{armOrdinal}, armId)
    error('FormationB4V46TriggerConfig:ProtocolDrift', ...
        'The V46 primary-arm registry changed or reordered.');
end

[config, parentDetails] = ...
    buildFormationB4V45FixedTriggerConfig(parentArmId, nodeCount);
config.topologyPolicyName = policyFunctionName;
config.topologyPolicyFcn = policyFunction;
assertSimplePolicyHandle(policyFunction, policyFunctionName);

configSnapshot = rmfield(config, 'topologyPolicyFcn');
configSnapshot.topologyPolicyFunctionName = policyFunctionName;
fingerprintPayload = struct();
fingerprintPayload.contractVersion = ...
    'formation-b4-v46-fixed-trigger-config-v1';
fingerprintPayload.armId = armId;
fingerprintPayload.armOrdinal = armOrdinal;
fingerprintPayload.parentArmId = parentArmId;
fingerprintPayload.parentConfigCanonicalSha256 = ...
    parentDetails.canonicalSha256;
fingerprintPayload.protocolId = protocol.id;
fingerprintPayload.protocolCanonicalSha256 = protocol.canonicalSha256;
fingerprintPayload.nodeCount = nodeCount;
fingerprintPayload.configSnapshot = configSnapshot;

details = fingerprintPayload;
details.allowedCrossArmConfigDifferenceFields = { ...
    'topologyPolicyName', 'topologyPolicyFcn'};
details.parentConfigReusedWithoutRelaxation = true;
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
    error('FormationB4V46TriggerConfig:InvalidPolicyHandle', ...
        'A frozen V46 policy did not resolve to its simple wrapper.');
end
end
