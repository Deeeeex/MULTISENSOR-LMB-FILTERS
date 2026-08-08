function [config, details] = ...
    buildFormationB4V49FixedTriggerConfig(armId, nodeCount)
% BUILDFORMATIONB4V49FIXEDTRIGGERCONFIG Freeze the V49 runtime arm.

if nargin ~= 2 || ~ischar(armId) || ~isrow(armId) || ...
        ~isa(nodeCount, 'double') || ~isreal(nodeCount) || ...
        ~isscalar(nodeCount) || ~isfinite(nodeCount) || ...
        nodeCount < 1 || nodeCount ~= round(nodeCount)
    error('FormationB4V49TriggerConfig:InvalidInput', ...
        'The frozen V49 arm and a positive integer N are required.');
end
protocol = getFormationB4V49RuntimeProtocol();
if ~strcmp(armId, protocol.candidateArmId)
    error('FormationB4V49TriggerConfig:UnknownArm', ...
        'Only the frozen V49 feasible-cycle runtime arm is allowed.');
end

[config, ~] = buildFormationB4V46FixedTriggerConfig( ...
    protocol.referenceArmId, nodeCount);
policyFunctionName = ...
    'selectFormationB4V49SynchronizedRuntimePolicy';
policyFunction = @selectFormationB4V49SynchronizedRuntimePolicy;
config.topologyPolicyName = policyFunctionName;
config.topologyPolicyFcn = policyFunction;
assertSimplePolicyHandle(policyFunction, policyFunctionName);

details = struct();
details.contractVersion = ...
    'formation-b4-v49-fixed-trigger-config-v1';
details.armId = armId;
details.parentArmId = protocol.referenceArmId;
details.nodeCount = nodeCount;
details.parentConfigReusedWithoutRelaxation = true;
details.overridesAccepted = false;
end

function assertSimplePolicyHandle(handle, expectedName)
identity = functions(handle);
if ~isa(handle, 'function_handle') || ...
        ~isstruct(identity) || ~isscalar(identity) || ...
        ~isfield(identity, 'type') || ...
        ~strcmp(identity.type, 'simple') || ...
        ~strcmp(func2str(handle), expectedName)
    error('FormationB4V49TriggerConfig:InvalidPolicyHandle', ...
        'The V49 public policy did not resolve to a simple wrapper.');
end
end
