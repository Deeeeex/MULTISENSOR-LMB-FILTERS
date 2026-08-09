function protocol = getFormationB4V53RuntimeProtocol()
% GETFORMATIONB4V53RUNTIMEPROTOCOL Exact selective cross-pulse control.

base = getFormationB4V51RuntimeProtocol();
protocol = base;
protocol.id = ...
    'formation-b4-v53-exact-selective-cross-pulse-runtime-v1';
protocol.contractVersion = ...
    'formation-b4-v53-runtime-protocol-v1';
protocol.candidateArmId = ...
    'v53-exact-selective-cross-pulse-b4-e20';
protocol.primaryArms = { ...
    protocol.referenceArmId, protocol.candidateArmId};
protocol.retentionDebtOffFraction = 0.01;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.minimumReferenceFormationExpectedCardinality = 1.0;
protocol.referenceSupportThreshold = 0.10;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.34;
protocol.retentionReceiverTailWeight = 0.50;
protocol.maximumIncomingCountForOutcomeEnumeration = 2;
protocol.maximumConsecutiveDeferredPulses = 1;
protocol.deferCrossFormationResidualOnly = true;
protocol.localResidualTimingUnchanged = true;
protocol.exactFusionCounterfactualUsed = true;
protocol.v51ExistenceGapProxyRetained = false;
protocol.v52GlobalPulseTimingRetained = false;
protocol.fullPosteriorFusionUnchanged = true;
protocol.posteriorUsedForRoutingOnly = true;
protocol.validationClaimAllowed = false;
protocol.developmentEvidenceOnly = true;
end
