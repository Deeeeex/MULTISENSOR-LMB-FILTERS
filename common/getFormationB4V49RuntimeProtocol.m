function protocol = getFormationB4V49RuntimeProtocol()
% GETFORMATIONB4V49RUNTIMEPROTOCOL Frozen non-scoring runtime boundary.

design = getFormationB4V49FeasibleCycleCompletionProtocol();
payload = struct();
payload.id = 'formation-b4-v49-feasible-cycle-runtime-smoke-v1';
payload.contractVersion = 'formation-b4-v49-runtime-protocol-v1';
payload.designProtocolId = design.id;
payload.referenceArmId = 'v46-repaired-sync-all-b4-e20-mc';
payload.candidateArmId = 'v49-feasible-cycle-sync-b4-e20';
payload.primaryArms = {payload.referenceArmId, payload.candidateArmId};
payload.observableContractVersion = ...
    'topology-policy-observable-input-v3-physical-uid';
payload.graphOnlyRouteContextContractVersion = ...
    'formation-b4-v49-graph-only-route-context-v1';
payload.period = design.period;
payload.dominantWeight = design.dominantWeight;
payload.referenceResidualWeight = design.referenceResidualWeight;
payload.activeResidualWeight = design.activeResidualWeight;
payload.minimumRelativeImprovementVsIncumbent = ...
    design.minimumRelativeImprovementVsIncumbent;
payload.maximumCanonicalCycleEnumerationCount = ...
    design.maximumCanonicalCycleEnumerationCount;
payload.cycleSelectionMode = design.defaultCycleSelectionMode;
payload.maximumExactProposalEvaluations = ...
    design.maximumExactProposalEvaluations;
payload.pulsePhaseMode = design.runtimePulsePhaseMode;
payload.scheduledDirectedEdgeCountsByPhase = ...
    {'2N', 'N', 'N', 'N'};
payload.scheduledDirectedEdgesPerPeriod = '5N';
payload.sameScheduledDirectedEdgeCountVsReference = true;
payload.sameAttemptedPosteriorMessageCountRequiresNoOutage = true;
payload.sameAttemptedPosteriorMessageCountRequiresGateDisabled = true;
payload.sameAttemptedPosteriorMessageCountRequiresAlwaysHeavy = true;
payload.physicalUidPairedDeliveryUniformsRequired = true;
payload.deliveredRollingWindowConnectivityDescriptiveOnly = true;
payload.deliveredRollingWindowSafetyClaimAllowed = false;
payload.posteriorNoninterferenceTestRequired = true;
payload.currentGraphOnlySelectionRequired = true;
payload.routeSelectionOnlyOnAbsolutePhaseOne = true;
payload.routeDisseminationImplemented = false;
payload.atomicCommitImplemented = false;
payload.trackingOutcomeScored = false;
payload.trackingOutcomeAuthorized = false;
payload.sameTotalByteClaimAllowed = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
protocol = payload;
end
