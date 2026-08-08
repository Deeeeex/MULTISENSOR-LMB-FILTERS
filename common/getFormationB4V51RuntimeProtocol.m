function protocol = getFormationB4V51RuntimeProtocol()
% GETFORMATIONB4V51RUNTIMEPROTOCOL Retention-gated V50 cycle pulses.

base = getFormationB4V50RuntimeProtocol();
protocol = base;
protocol.id = 'formation-b4-v51-retention-gated-reference-runtime-v1';
protocol.contractVersion = ...
    'formation-b4-v51-runtime-protocol-v1';
protocol.parentRouteArmId = base.referenceArmId;
protocol.candidateArmId = ...
    'v51-retention-gated-sync-b4-e20';
protocol.primaryArms = {protocol.referenceArmId, protocol.candidateArmId};
protocol.retentionDebtOnFraction = 0.02;
protocol.retentionDebtAggregationMeanWeight = 0.5;
protocol.retentionDebtAggregationTailWeight = 0.5;
protocol.maximumConsecutiveDeferredPulses = 1;
protocol.pulseHistoryDepth = 4;
protocol.temporalConnectivityPulseCount = 2;
protocol.temporalConnectivityHorizonSteps = 8;
protocol.deferCrossFormationResidualOnly = true;
protocol.posteriorAwareCycleSelectionRetained = false;
protocol.v50NoValueFallbackObservedFraction = 1;
protocol.referenceFallbackOnMissingHistory = true;
protocol.sameOrLowerScheduledMessageCountVsReference = true;
protocol.fullPosteriorFusionUnchanged = true;
protocol.posteriorUsedForRoutingOnly = true;
protocol.validationClaimAllowed = false;
protocol.developmentEvidenceOnly = true;
end
