function protocol = getSignedCrossFormationOpportunityV67Protocol()
% GETSIGNEDCROSSFORMATIONOPPORTUNITYV67PROTOCOL Source-only relay diagnostic.

relay = getInfluenceAwareDecisionBreadthV66SceneDiscoveryProtocol();
protocol = struct();
protocol.id = 'signed-cross-formation-opportunity-v67-relay-v1';
protocol.contractVersion = ...
    'signed-cross-formation-opportunity-v67-protocol-v1';
protocol.baseProtocolId = relay.id;
protocol.presets = relay.presets;
protocol.allSeeds = relay.allSeeds;
protocol.snapshotTimes = relay.snapshotTimes;
protocol.cacheRoot = relay.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v67', 'signed_relay_opportunity');
protocol.minimumMaterialPressureFraction = 0.01;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V67 reuses the frozen V66 relay reference caches and separates ', ...
    'receiver-supported quarantine pressure from sender-supported ', ...
    'transport-retention pressure. It reads current posterior and ', ...
    'registered topology only. It does not authorize a routing action, ', ...
    'tracking scoring, model training, or validation.'];
end
