function config = buildParetoSafeResidualAugmentationV246Config(scenario)
% BUILDPARETOSAFERESIDUALAUGMENTATIONV246CONFIG V242 plus bounded residuals.

protocol = getParetoSafeResidualAugmentationV246Protocol();
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~isfield(scenario, 'presetName') || ...
        ~ismember(scenario.presetName, protocol.allowedPresets) || ...
        ~isfield(scenario, 'numberOfSensors') || ...
        ~isfield(scenario, 'sensorGroupIds')
    error('ParetoSafeResidualV246:InvalidConfigInput', ...
        'V246 requires one registered information-coupled braid scene.');
end
formationCount = numel(unique(scenario.sensorGroupIds));
minimumMessages = scenario.numberOfSensors + 2 * (formationCount - 1);
residualQuota = ceil(formationCount / 2);
config = buildCausalMinimumFormationBackboneV242Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = ...
    @selectParetoSafeResidualAugmentationV246Policy;
config.topologyDirectedMessageBudget = minimumMessages + residualQuota;
end
