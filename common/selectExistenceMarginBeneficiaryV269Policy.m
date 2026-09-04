function [adjacency, details] = ...
        selectExistenceMarginBeneficiaryV269Policy(context)
% SELECTEXISTENCEMARGINBENEFICIARYV269POLICY Keep graph; change recipient.

[adjacency, details] = selectSourcePreservingLabelPacketV268Policy(context);
protocol = getExistenceMarginBeneficiaryV269Protocol();
details.contractVersion = ...
    'existence-margin-beneficiary-v269-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v268-packet-with-existence-margin-beneficiary';
details.backboneMode = details.mode;
details.packetBeneficiaryMode = protocol.beneficiaryMode;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'existence-margin-beneficiary-v269-schedule-v1';
schedule.packetBeneficiaryMode = protocol.beneficiaryMode;
details.scheduleCertificate = schedule;
end
