function [adjacency, details] = ...
        selectAsymmetricEtaRetentionV267Policy(context)
% SELECTASYMMETRICETARETENTIONV267POLICY V266 with sub-MAP protection.

[adjacency, details] = selectEtaProjectedLabelTrustV266Policy(context);
protocol = getAsymmetricEtaRetentionV267Protocol();
details.contractVersion = ...
    'asymmetric-eta-retention-v267-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v242-backbone-with-asymmetric-eta-retention';
details.backboneMode = details.mode;
details.maximumSubMapReferenceLogOddsDrop = ...
    protocol.maximumSubMapReferenceLogOddsDrop;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'asymmetric-eta-retention-v267-schedule-v1';
schedule.maximumSubMapReferenceLogOddsDrop = ...
    protocol.maximumSubMapReferenceLogOddsDrop;
details.scheduleCertificate = schedule;
end
