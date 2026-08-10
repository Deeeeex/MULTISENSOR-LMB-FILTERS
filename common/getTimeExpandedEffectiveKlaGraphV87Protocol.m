function protocol = getTimeExpandedEffectiveKlaGraphV87Protocol()
% GETTIMEEXPANDEDEFFECTIVEKLAGRAPHV87PROTOCOL Two-hop source probe.

source = getTemporalAcquireBroadcastV86Protocol();
protocol = struct();
protocol.id = 'time-expanded-effective-kla-graph-v87-v1';
protocol.contractVersion = ...
    'time-expanded-effective-kla-graph-v87-protocol-v1';
protocol.sourceProtocolId = source.id;
protocol.sourceFindingCommit = 'bbee874';
protocol.cases = source.cases;
protocol.sourceWeight = source.sourceWeight;
protocol.dominantWeight = source.dominantWeight;
protocol.positiveSupportThreshold = 0.20;
protocol.decisionExistenceThreshold = 0.50;
protocol.minimumExistenceSurvivalRatio = 0.05;
protocol.minimumDownstreamExistenceNetFraction = 0;
protocol.minimumStateAlignmentNetFraction = 0;
protocol.maximumSupportedHarmRatio = 1.0;
protocol.requireZeroSupportedDownwardCrossings = true;
protocol.fusionConfig = buildMixtureAwareKlaReferenceConfig();
protocol.cacheRoot = source.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v87', 'time_expanded_effective_kla_primary');
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V87 uses only the opened current posteriors, current association ', ...
    'support, current physical graph, current link reliability and the ', ...
    'registered reference weights at the two V86 primary states. It ', ...
    'composes two mixture-aware KLA rounds without prediction or future ', ...
    'measurements and scores label existence survival plus supported ', ...
    'position alignment. Truth, V86 outcomes, route execution, tracking ', ...
    'scoring and model training are not read. This is source-only opened ', ...
    'development evidence and cannot support validation claims.'];
end
