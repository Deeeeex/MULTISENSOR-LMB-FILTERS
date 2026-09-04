function config = buildOneShotPacketEpisodeV270Config(scenario)
% BUILDONESHOTPACKETEPISODEV270CONFIG V269 with an episode latch.

protocol = getOneShotPacketEpisodeV270Protocol();
config = buildExistenceMarginBeneficiaryV269Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectOneShotPacketEpisodeV270Policy;
config. ...
    topologyPolicySourcePreservingLabelPacketOneShotPerEpisodeEnabled = ...
    true;
end
