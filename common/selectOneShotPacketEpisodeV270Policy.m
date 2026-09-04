function [adjacency, details] = ...
        selectOneShotPacketEpisodeV270Policy(context)
% SELECTONESHOTPACKETEPISODEV270POLICY Keep V269 causal schedule.

[adjacency, details] = selectExistenceMarginBeneficiaryV269Policy(context);
protocol = getOneShotPacketEpisodeV270Protocol();
details.contractVersion = 'one-shot-packet-episode-v270-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v269-beneficiary-with-one-shot-packet-episode';
details.backboneMode = details.mode;
details.oneShotPerEpisodeEnabled = true;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'one-shot-packet-episode-v270-schedule-v1';
schedule.oneShotPerEpisodeEnabled = true;
details.scheduleCertificate = schedule;
end
