function [adjacency, details] = ...
        selectEtaProjectedLabelTrustV266Policy(context)
% SELECTETAPROJECTEDLABELTRUSTV266POLICY Project V265 label trust by eta.

[adjacency, details] = ...
    selectLabelSelectiveRiskShortcutV265Policy(context);
protocol = getEtaProjectedLabelTrustV266Protocol();
details.contractVersion = ...
    'eta-projected-label-trust-v266-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'v242-backbone-with-eta-projected-label-trust';
details.backboneMode = details.mode;
details.labelShortcutSourceWeight = protocol.sourceWeight;
details.labelShortcutSourceWeightGrid = protocol.sourceWeightGrid;
details.labelShortcutWeightMode = 'bounded-proportional-share';

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'eta-projected-label-trust-v266-schedule-v1';
schedule.labelShortcutSourceWeight = protocol.sourceWeight;
schedule.labelShortcutSourceWeightGrid = protocol.sourceWeightGrid;
schedule.labelShortcutWeightMode = 'bounded-proportional-share';
details.scheduleCertificate = schedule;
end
