function config = buildStaticVsDynamicRoutingV91TriggerConfig( ...
        armId, nodeCount)
% BUILDSTATICVSDYNAMICROUTINGV91TRIGGERCONFIG Matched routing arms.

protocol = getStaticVsDynamicRoutingV91Protocol();
if nargin ~= 2 || ~ischar(armId) || ~isrow(armId) || ...
        ~any(strcmp(armId, {protocol.dynamicArmId, ...
            protocol.staticArmId})) || ...
        ~isscalar(nodeCount) || ~isfinite(nodeCount) || ...
        ~any(nodeCount == protocol.expectedNodeCounts)
    error('StaticVsDynamicV91:InvalidTriggerConfigInput', ...
        'V91 requires one registered arm and an M24/X36 node count.');
end

config = buildMixtureAwareKlaReferenceConfig();
config.eventPolicy = 'alwaysHeavy';
config.dynamicTopologyEnabled = true;
config.topologyDirectedEnabled = true;
config.topologyDirectedMessageBudget = 2 * nodeCount;
config.topologyPolicyObservableContextOnly = true;
config.topologyPolicyHistoryDepth = 2;
config.missingLabelFusionMode = protocol.receiverMode;
if strcmp(armId, protocol.dynamicArmId)
    config.topologyPolicyName = protocol.dynamicPolicyName;
    config.topologyPolicyFcn = ...
        @selectStaticVsDynamicRoutingV91DynamicPolicy;
else
    config.topologyPolicyName = protocol.staticPolicyName;
    config.topologyPolicyFcn = ...
        @selectStaticVsDynamicRoutingV91StaticPolicy;
end
end
