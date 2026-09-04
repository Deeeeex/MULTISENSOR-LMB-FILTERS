function [adjacency, details] = ...
        selectSourcePreservingLabelPacketV268Policy(context)
% SELECTSOURCEPRESERVINGLABELPACKETV268POLICY Keep source provenance.

[adjacency, details] = selectAsymmetricEtaRetentionV267Policy(context);
protocol = getSourcePreservingLabelPacketV268Protocol();
details.contractVersion = ...
    'source-preserving-label-packet-v268-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v242-backbone-with-source-preserving-label-packet';
details.backboneMode = details.mode;
details.sourcePreservingPacketRouteEnabled = true;
details.packetMaximumAge = protocol.packetMaximumAge;
details.packetMetadataScalars = protocol.packetMetadataScalars;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'source-preserving-label-packet-v268-schedule-v1';
schedule.sourcePreservingPacketRouteEnabled = true;
schedule.packetMaximumAge = protocol.packetMaximumAge;
schedule.packetMetadataScalars = protocol.packetMetadataScalars;
details.scheduleCertificate = schedule;
end
