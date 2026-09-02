function config = buildCausalGatewayEmbeddingV250Config( ...
        scenario, requestedAssignment)
% BUILDCAUSALGATEWAYEMBEDDINGV250CONFIG Persistent H=3 gateway action.

protocol = getCausalGatewayEmbeddingV250Protocol();
if nargin < 2 || ~isnumeric(requestedAssignment) || ...
        size(requestedAssignment, 2) ~= 4 || ...
        any(~isfinite(requestedAssignment(:)))
    error('CausalGatewayEmbeddingV250:InvalidConfigInput', ...
        'V250 requires one finite K-by-4 gateway assignment.');
end
config = buildCausalMinimumFormationBackboneV242Config(scenario);
assignment = requestedAssignment;
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @(context) ...
    selectCausalGatewayEmbeddingV250Policy(context, assignment);
end
