function protocol = getTrackingAlignedLayeredLabelHeadroomV62Protocol()
% GETTRACKINGALIGNEDLAYEREDLABELHEADROOMV62PROTOCOL Temporal-label gate.

protocol = struct();
protocol.id = 'tracking-aligned-layered-label-headroom-v62-v1';
protocol.contractVersion = ...
    'tracking-aligned-layered-label-headroom-v62-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.currentTime = 104;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.policyName = 'layered-formation-label-effective-kla-h3-v62';
protocol.scheduleTimes = 104:106;
protocol.formationSchedule = {[1, 2, 4], [1, 2], 4};
protocol.positiveSupportThreshold = 0.20;
protocol.highExistenceThreshold = 0.50;
protocol.minimumMeanGainPercent = 5;
protocol.minimumWorstSensorGainPercent = 0;
protocol.minimumFormationGainPercent = 0;
protocol.minimumWindowConsensusGainPercent = 0;
protocol.minimumTerminalConsensusGainPercent = 0;
protocol.minimumAttemptedByteSavingPercent = 0;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v58', 'leverage_attribution', ...
    'cache_old_formation_fov');
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v62', 'layered_label_headroom');
protocol.trackingOutcomeScoringAuthorized = true;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V62 opens only the already-developed M24 radial seed-211 state at ', ...
    'time 104. The historical observable formation schedule fixes where ', ...
    'and when cross-formation input is restricted; current association ', ...
    'support selects complete label mixtures within those carrier edges. ', ...
    'No X36, model-training, held-out or validation claim is authorized.'];
end
