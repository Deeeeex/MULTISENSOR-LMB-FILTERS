function [adjacency, details] = ...
    selectOnlinePositiveNetPropagationV100Policy(context)
% SELECTONLINEPOSITIVENETPROPAGATIONV100POLICY V99 rule over H=6.

[adjacency, details] = ...
    selectOnlinePositiveNetAddressablePayloadV99Policy(context);
protocol = getOnlinePositiveNetPropagationV100Protocol();
details.contractVersion = ...
    'online-positive-net-propagation-v100-policy-v1';
details.armId = protocol.outcomePolicyName;
details.protocolId = protocol.id;
end
