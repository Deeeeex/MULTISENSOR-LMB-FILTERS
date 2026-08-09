function protocol = getTrackingAlignedReceiverLabelHeadroomV61Protocol()
% GETTRACKINGALIGNEDRECEIVERLABELHEADROOMV61PROTOCOL First label-action gate.

protocol = struct();
protocol.id = 'tracking-aligned-receiver-label-headroom-v61-v1';
protocol.contractVersion = ...
    'tracking-aligned-receiver-label-headroom-v61-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.currentTime = 104;
protocol.expectedNodeCount = 24;
protocol.expectedFormationCount = 4;
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.policyName = 'label-conditioned-effective-kla-h3-v61';
protocol.positiveSupportThreshold = 0.20;
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
    'tracking_aligned_v61', 'receiver_label_headroom');
protocol.trackingOutcomeScoringAuthorized = true;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V61 opens only the already-developed M24 radial seed-211 state at ', ...
    'time 104 as a positive-control label-action headroom experiment. ', ...
    'Candidate bundles are fixed from current observable posteriors before ', ...
    'H=3 truth scoring, preserve complete transmitted Gaussian mixtures, ', ...
    'and pay compact synopsis bytes. No X36, model-training, held-out or ', ...
    'validation claim is authorized.'];
end
