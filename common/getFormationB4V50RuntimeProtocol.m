function protocol = getFormationB4V50RuntimeProtocol()
% GETFORMATIONB4V50RUNTIMEPROTOCOL Posterior-aware feasible-cycle arm.

base = getFormationB4V49RuntimeProtocol();
protocol = struct();
protocol.id = 'formation-b4-v50-posterior-aware-cycle-runtime-v1';
protocol.contractVersion = ...
    'formation-b4-v50-runtime-protocol-v1';
protocol.referenceArmId = base.referenceArmId;
protocol.candidateArmId = ...
    'v50-posterior-aware-cycle-sync-b4-e20';
protocol.primaryArms = { ...
    protocol.referenceArmId, protocol.candidateArmId};
protocol.observableContractVersion = base.observableContractVersion;
protocol.period = base.period;
protocol.dominantWeight = base.dominantWeight;
protocol.referenceResidualWeight = base.referenceResidualWeight;
protocol.activeResidualWeight = base.activeResidualWeight;
protocol.maximumCanonicalCycleEnumerationCount = ...
    base.maximumCanonicalCycleEnumerationCount;
protocol.maximumPosteriorProposalEvaluations = 8;
protocol.posteriorEdgeScoreMode = 'transfer';
protocol.formationMeanWeight = 0.5;
protocol.formationLowerQuartileWeight = 0.5;
protocol.worstFormationWeight = 0.5;
protocol.networkMeanWeight = 0.5;
protocol.minimumPosteriorObjectiveImprovement = 0;
protocol.maximumWorstFormationUtilityRegression = 0.05;
protocol.structuralNoWorseTolerance = 1e-12;
protocol.scheduledDirectedEdgeCountsByPhase = {'2N', 'N', 'N', 'N'};
protocol.scheduledDirectedEdgesPerPeriod = '5N';
protocol.routeSelectionOnlyOnAbsolutePhaseOne = true;
protocol.fullPosteriorFusionUnchanged = true;
protocol.posteriorUsedForRoutingOnly = true;
protocol.truthUsed = false;
protocol.futureOutcomeUsed = false;
protocol.validationClaimAllowed = false;
protocol.developmentEvidenceOnly = true;
end
