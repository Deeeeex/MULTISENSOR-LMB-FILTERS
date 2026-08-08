function protocol = getFormationB4V52RuntimeProtocol()
% GETFORMATIONB4V52RUNTIMEPROTOCOL Fixed-budget pulse timing.

base = getFormationB4V51RuntimeProtocol();
protocol = struct();
protocol.id = ...
    'formation-b4-v52-counterfactual-pulse-timing-runtime-v1';
protocol.contractVersion = ...
    'formation-b4-v52-runtime-protocol-v1';
protocol.referenceArmId = base.referenceArmId;
protocol.parentRouteArmId = base.parentRouteArmId;
protocol.candidateArmId = ...
    'v52-counterfactual-pulse-timing-b4-e20';
protocol.primaryArms = { ...
    protocol.referenceArmId, protocol.candidateArmId};
protocol.observableContractVersion = base.observableContractVersion;
protocol.period = base.period;
protocol.dominantWeight = base.dominantWeight;
protocol.referenceResidualWeight = base.referenceResidualWeight;
protocol.activeResidualWeight = base.activeResidualWeight;
protocol.pulseHistoryDepth = base.period;
protocol.bootstrapTime = 1;
protocol.decisionPhases = [1, 2, 3];
protocol.hardDeadlinePhase = 4;
protocol.maximumIncomingCountForOutcomeEnumeration = 2;
protocol.minimumDisagreementImprovementFraction = 0.0025;
protocol.minimumCardinalityGainFraction = 0.005;
protocol.maximumDisagreementRegressionForCardinality = 0.0025;
protocol.maximumRetentionRisk = 0.01;
protocol.minimumFormationMeanCardinalityChange = -0.05;
protocol.minimumSupportedLabelRetentionRatio = 0.80;
protocol.maximumDecisionThresholdCrossingCount = 0;
protocol.referenceSupportThreshold = 0.10;
protocol.decisionExistenceThreshold = 0.50;
protocol.retentionReceiverTailFraction = 0.34;
protocol.retentionReceiverTailWeight = 0.50;
protocol.scheduledDirectedEdgesPerPeriod = '5N';
protocol.exactlyOneCompletePulsePerPeriod = true;
protocol.fullPosteriorFusionUnchanged = true;
protocol.posteriorUsedForPulseTimingOnly = true;
protocol.truthUsed = false;
protocol.futureOutcomeUsed = false;
protocol.validationClaimAllowed = false;
protocol.developmentEvidenceOnly = true;
end
