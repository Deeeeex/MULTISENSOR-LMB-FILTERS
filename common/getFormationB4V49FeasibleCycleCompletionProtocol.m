function protocol = getFormationB4V49FeasibleCycleCompletionProtocol()
% GETFORMATIONB4V49FEASIBLECYCLECOMPLETIONPROTOCOL Claim boundary.

payload = struct();
payload.id = 'formation-b4-v49-feasible-cycle-completion-development-v1';
payload.contractVersion = ...
    'formation-b4-v49-feasible-cycle-completion-protocol-v1';
payload.period = 4;
payload.dominantWeight = 0.70;
payload.referenceResidualWeight = 0.05;
payload.activeResidualWeight = 0.20;
payload.minimumRelativeImprovementVsIncumbent = 0.01;
payload.maximumCanonicalCycleEnumerationCount = 10000;
payload.cycleEnumerationWorstCaseExponential = true;
payload.defaultCycleSelectionMode = 'reliability-proposal-topk';
payload.maximumExactProposalEvaluations = 3;
payload.exactContractionEnumerationDevelopmentOracleAvailable = true;
payload.runtimePulsePhaseMode = 'fixed-phase-one';
payload.bestOfFourPulseDevelopmentOracleAvailable = true;
payload.incumbentExactFallbackRequired = true;
payload.toleranceTieUsesIncumbent = true;
payload.dominantLayerBitwiseEqualityRequired = true;
payload.referenceDirectedMessageCountPerStep = '2N';
payload.synchronizedPosteriorMessagesPerPeriod = '5N';
payload.fullReferencePosteriorMessagesPerPeriod = '8N';
payload.currentPhysicalPageRequired = true;
payload.currentLinkProbabilityPageRequired = true;
payload.currentGeometryPageRequired = true;
payload.posteriorAllowed = false;
payload.posteriorSummaryAllowed = false;
payload.deliveryAcknowledgmentAllowed = false;
payload.measurementAllowed = false;
payload.truthAllowed = false;
payload.futurePageAllowed = false;
payload.realizedDeliveryUniformAllowed = false;
payload.graphCertificateStandaloneTrackingObjective = false;
payload.frozenCurrentPagePropagationProxyOnly = true;
payload.timeVaryingNoWorseCertified = false;
payload.trackingSafetyClaimAllowed = false;
payload.routeDisseminationImplemented = false;
payload.atomicCommitImplemented = false;
payload.sameTotalByteClaimAllowed = false;
payload.developmentEvidenceOnly = true;
actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    'a397b0a83c925497776cce45153be30c68139c8ddaced09b0931754753d36bc9';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationB4V49Protocol:UnregisteredDrift', ...
        ['The V49 cycle-completion protocol changed without a ', ...
         'version update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end
