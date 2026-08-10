function protocol = getReferenceRecoveryReplayV76Protocol()
% GETREFERENCERECOVERYREPLAYV76PROTOCOL Frozen source-only KLA replay.

v75 = getReplacementInnovationEnergyV75Protocol();
cases = v75.cases;
cases(1).directSafeFormationIds = 3;
cases(2).directSafeFormationIds = 4;

protocol = struct();
protocol.id = 'reference-recovery-replay-v76-v1';
protocol.contractVersion = ...
    'reference-recovery-replay-v76-protocol-v1';
protocol.baseProtocolId = v75.id;
protocol.cases = cases;
protocol.cacheRoot = v75.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v76', 'reference_recovery_replay');
protocol.fusionConfig = buildMixtureAwareKlaReferenceConfig();
protocol.totalFusionRounds = 3;
protocol.candidateRounds = 1;
protocol.referenceRecoveryRounds = 2;
protocol.expectedDeliveryWeighting = true;
protocol.currentLinkPageReusedAcrossVirtualRounds = true;
protocol.disagreementTailFraction = 0.25;
protocol.requireMonotoneMeanArmGapContraction = true;
protocol.requireMonotoneTailArmGapContraction = true;
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V76 starts from the two opened current posterior states. The ', ...
    'reference arm uses the current physical-tree route for all three ', ...
    'virtual fusion rounds. The candidate arm uses only the V75 ', ...
    'direct-safe formation replacements in round one and the identical ', ...
    'reference route for rounds two and three. Every round preserves ', ...
    'the registered directed weights and formal mixture-aware heavy ', ...
    'fusion. Current link reliability scales weights deterministically; ', ...
    'no packet draw, prediction, target motion, new measurement, truth, ', ...
    'future link page, or tracking outcome is used. This diagnoses KLA ', ...
    'information-flow recovery only, not closed-loop tracking recovery.'];
end
