function config = buildRepeatedMultiGatewayHandoverV89TriggerConfig( ...
        armId, nodeCount)
% BUILDREPEATEDMULTIGATEWAYHANDOVERV89TRIGGERCONFIG Frozen paired arms.

protocol = getRepeatedMultiGatewayHandoverV89Protocol();
if nargin ~= 2 || ~ischar(armId) || ~isrow(armId) || ...
        ~any(strcmp(armId, {protocol.referenceArmId, ...
            protocol.candidateArmId})) || ...
        ~isscalar(nodeCount) || ~isfinite(nodeCount) || ...
        ~any(nodeCount == protocol.expectedNodeCounts)
    error('RepeatedMultiGatewayV89:InvalidTriggerConfigInput', ...
        'V89 requires one registered arm and an M24/X36 node count.');
end

config = buildMixtureAwareKlaReferenceConfig();
config.eventPolicy = 'alwaysHeavy';
config.dynamicTopologyEnabled = true;
config.topologyDirectedEnabled = true;
config.topologyDirectedMessageBudget = 2 * nodeCount;
config.topologyPolicyObservableContextOnly = true;
config.topologyPolicyHistoryDepth = 2;
if strcmp(armId, protocol.referenceArmId)
    config.topologyPolicyName = protocol.referencePolicyName;
    config.topologyPolicyFcn = ...
        @selectRepeatedMultiGatewayHandoverV89ReferencePolicy;
else
    config.topologyPolicyName = protocol.candidatePolicyName;
    config.topologyPolicyFcn = ...
        @selectRepeatedMultiGatewayHandoverV89RuntimePolicy;
end
end
