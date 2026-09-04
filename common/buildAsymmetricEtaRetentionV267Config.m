function config = buildAsymmetricEtaRetentionV267Config(scenario)
% BUILDASYMMETRICETARETENTIONV267CONFIG V266 plus support protection.

protocol = getAsymmetricEtaRetentionV267Protocol();
config = buildEtaProjectedLabelTrustV266Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectAsymmetricEtaRetentionV267Policy;
config.topologyPolicyLabelInputRouteMaximumSubMapReferenceLogOddsDrop = ...
    protocol.maximumSubMapReferenceLogOddsDrop;
end
