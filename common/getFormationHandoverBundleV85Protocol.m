function protocol = getFormationHandoverBundleV85Protocol()
% GETFORMATIONHANDOVERBUNDLEV85PROTOCOL Formation-level source gate.

v84 = getBraidedHandoverOpportunityV84Protocol();
h3 = getBraidedHandoverH3V84Protocol();
protocol = struct();
protocol.id = 'formation-handover-bundle-v85-v1';
protocol.contractVersion = ...
    'formation-handover-bundle-v85-protocol-v1';
protocol.baseProtocolId = v84.id;
protocol.cases = h3.cases;
protocol.presets = v84.presets;
protocol.allSeeds = v84.allSeeds;
protocol.horizonSteps = 3;
protocol.filterSeedOffset = v84.filterSeedOffset;
protocol.sourceWeight = v84.sourceWeight;
protocol.positiveSupportThreshold = v84.positiveSupportThreshold;
protocol.decisionExistenceThreshold = v84.decisionExistenceThreshold;
protocol.fusionConfig = v84.fusionConfig;
protocol.requireSourceFormationChange = true;
protocol.requireCurrentNovelSupport = true;
protocol.minimumEdgeNoveltyPrefilterFraction = 0.0001;
protocol.minimumSenderNoveltyFraction = ...
    protocol.minimumEdgeNoveltyPrefilterFraction;
protocol.minimumBundleLocalNetFraction = 0.01;
protocol.minimumBundleSenderNoveltyFraction = 0.005;
protocol.minimumChangedReceiverCount = 2;
protocol.maximumProtectedHarmRatio = 1.0;
protocol.requireZeroSupportedDownwardCrossings = true;
protocol.cacheRoot = v84.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v85', 'formation_handover_bundle');
protocol.routeConstructionAuthorized = true;
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V85 reuses only the two already opened V84 primary anchors. ', ...
    'Candidate edges use current local LMB posteriors, current ', ...
    'association support, current physical links and reliability, and ', ...
    'registered past topology. Receiver-first mixture-aware KLA scores ', ...
    'each row; safe positive rows are aggregated by receiver formation ', ...
    'before the unchanged 1% net and 0.5% novelty gates are applied. ', ...
    'The selected bundle must preserve exact message and row-weight ', ...
    'parity plus rolling-B3. Truth, future measurements, route ', ...
    'execution, tracking outcomes, and model training remain closed.'];
end
