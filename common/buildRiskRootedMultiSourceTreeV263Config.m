function config = buildRiskRootedMultiSourceTreeV263Config(scenario)
% BUILDRISKROOTEDMULTISOURCETREEV263CONFIG Exact-budget continuation.

protocol = getRiskRootedMultiSourceTreeV263Protocol();
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~isfield(scenario, 'presetName') || ...
        ~ismember(scenario.presetName, protocol.allowedPresets) || ...
        ~isfield(scenario, 'numberOfSensors') || ...
        ~isfield(scenario, 'sensorGroupIds')
    error('RiskRootedMultiSourceV263:InvalidConfigInput', ...
        'V263 requires its registered M24 formation-braid scene.');
end
formationCount = numel(unique(scenario.sensorGroupIds));
minimumMessages = scenario.numberOfSensors + ...
    2 * (formationCount - 1);
config = buildCausalMinimumFormationBackboneV242Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectRiskRootedMultiSourceTreeV263Policy;
config.topologyDirectedMessageBudget = minimumMessages;
end
