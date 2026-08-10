function protocol = getRepeatedMultiGatewayHandoverV89Protocol()
% GETREPEATEDMULTIGATEWAYHANDOVERV89PROTOCOL Repeated full-episode route.

source = getBraidedHandoverOpportunityV84Protocol();
protocol = struct();
protocol.id = 'repeated-multi-gateway-handover-v89-v1';
protocol.contractVersion = ...
    'repeated-multi-gateway-handover-v89-protocol-v1';
protocol.sourceProtocolId = source.id;
protocol.sourceFindingCommit = '6e84f9a';
protocol.presets = source.presets;
protocol.allSeeds = source.allSeeds;
protocol.snapshotTimes = source.snapshotTimes;
protocol.expectedNodeCounts = source.expectedNodeCounts;
protocol.expectedFormationCounts = source.expectedFormationCounts;
protocol.expectedFormationSize = 6;
protocol.sourceWeight = source.sourceWeight;
protocol.dominantWeight = 0.70;
protocol.minimumSenderNoveltyFraction = ...
    source.minimumSenderNoveltyFraction;
protocol.minimumCandidateAssociationSupport = ...
    source.positiveSupportThreshold;
protocol.maximumGatewayCount = inf;
protocol.requireFullFormationBroadcastPhysicality = true;
protocol.minimumActionableTimeFraction = 0.10;
protocol.minimumMultiGatewayTimeCount = 1;
protocol.minimumDistinctCoveredFormationFraction = 0.50;
protocol.period = 3;
protocol.activationStartTime = min(source.snapshotTimes);
protocol.activationEndTime = max(source.snapshotTimes);
protocol.referenceArmId = 'v89-current-physical-reference';
protocol.candidateArmId = 'v89-repeated-multi-gateway-handover';
protocol.referencePolicyName = ...
    'repeated-multi-gateway-v89-reference';
protocol.candidatePolicyName = ...
    'repeated-multi-gateway-v89-candidate';
protocol.minimumFullEpisodeMeanGainPercent = 5.0;
protocol.minimumFocusWindowMeanGainPercent = 5.0;
protocol.minimumWorstSensorGainPercent = 0.0;
protocol.minimumFormationGainPercent = 0.0;
protocol.minimumConsensusGainPercent = 0.0;
protocol.minimumAttemptedByteSavingPercent = -1.0;
protocol.cacheRoot = source.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v89', 'multi_gateway_handover_coverage');
protocol.fullEpisodeOutputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v89', 'multi_gateway_handover_full_episode');
protocol.deliverySeedOffset = 89000000;
protocol.filterSeedOffset = 89500000;
protocol.routeExecutionAuthorized = true;
protocol.trackingOutcomeScoringAuthorized = true;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V89 uses only the current local LMB posteriors, current detection-', ...
    'association support, current physical links and current link ', ...
    'reliability to nominate at most one gateway per receiver formation. ', ...
    'The frozen runtime cadence is acquire, full-formation broadcast, ', ...
    'then current physical-tree reference. Every executed row preserves ', ...
    'the reference message count and positive-weight multiset and passes ', ...
    'rolling-B3, otherwise that formation or round falls back to the ', ...
    'current reference. Truth, future measurements, realized future link ', ...
    'outcomes and tracking scores are unavailable to the policy. Paired ', ...
    'tracking is read only after both arms are frozen and executed with ', ...
    'the same scene, measurements, delivery uniforms and filter seed. ', ...
    'This is opened development evidence, not validation or training.'];
end
