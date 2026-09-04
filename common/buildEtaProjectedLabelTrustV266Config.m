function config = buildEtaProjectedLabelTrustV266Config(scenario)
% BUILDETAPROJECTEDLABELTRUSTV266CONFIG V265 with projected trust.

protocol = getEtaProjectedLabelTrustV266Protocol();
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~isfield(scenario, 'presetName') || ...
        ~ismember(scenario.presetName, protocol.allowedPresets)
    error('EtaProjectedLabelTrustV266:InvalidConfigInput', ...
        'V266 requires its registered M24 formation-braid scene.');
end
config = buildLabelSelectiveRiskShortcutV265Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectEtaProjectedLabelTrustV266Policy;
config.topologyPolicyLabelInputRouteSourceWeight = protocol.sourceWeight;
config.topologyPolicyLabelInputRouteSourceWeightGrid = ...
    protocol.sourceWeightGrid;
end
