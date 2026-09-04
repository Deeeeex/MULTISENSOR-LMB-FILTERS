function config = buildSourcePreservingLabelPacketV268Config(scenario)
% BUILDSOURCEPRESERVINGLABELPACKETV268CONFIG V267 plus relay-only packet.

protocol = getSourcePreservingLabelPacketV268Protocol();
config = buildAsymmetricEtaRetentionV267Config(scenario);
config.topologyPolicyName = protocol.policyName;
config.topologyPolicyFcn = @selectSourcePreservingLabelPacketV268Policy;
config.topologyPolicySourcePreservingLabelPacketEnabled = true;
config.topologyPolicySourcePreservingLabelPacketMaximumAge = ...
    protocol.packetMaximumAge;
config.topologyPolicySourcePreservingLabelPacketMetadataScalars = ...
    protocol.packetMetadataScalars;
end
