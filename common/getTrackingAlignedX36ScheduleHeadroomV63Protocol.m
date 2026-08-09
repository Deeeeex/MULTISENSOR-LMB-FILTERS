function protocol = getTrackingAlignedX36ScheduleHeadroomV63Protocol()
% GETTRACKINGALIGNEDX36SCHEDULEHEADROOMV63PROTOCOL Current-source X36 gate.

protocol = struct();
protocol.id = 'tracking-aligned-x36-schedule-headroom-v63-v1';
protocol.contractVersion = ...
    'tracking-aligned-x36-schedule-headroom-v63-protocol-v1';
protocol.presetName = 'x36-formation-fov';
protocol.presets = {protocol.presetName};
protocol.seed = 211;
protocol.allSeeds = protocol.seed;
protocol.anchorTimes = [72, 100, 128];
protocol.snapshotTimes = protocol.anchorTimes;
protocol.expectedNodeCount = 36;
protocol.expectedFormationCount = 6;
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.referencePolicyName = 'formation-h3-fixed-ccw-reference-v1';
protocol.outcomePolicyName = ...
    'x36-control-plane-data-plane-schedule-h3-v63';
protocol.positiveSupportThreshold = 0.20;
protocol.minimumMeanGainPercent = 5;
protocol.minimumWorstSensorGainPercent = 0;
protocol.minimumFormationGainPercent = 0;
protocol.minimumWindowConsensusGainPercent = 0;
protocol.minimumTerminalConsensusGainPercent = 0;
protocol.minimumAttemptedByteSavingPercent = 0;
protocol.blockageFormationPairs = [1, 2; 3, 4; 5, 6];
protocol.legacyFormationSchedules = { ...
    {[3, 5, 6], 5, 2}, ...
    {[1, 2, 5, 6], [1, 2, 4, 5], [4, 6]}, ...
    {[1, 2, 4, 6], [1, 3, 4], [3, 6]}};
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v63', 'x36_reference_cache');
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v63', 'x36_schedule_headroom');
protocol.trackingOutcomeScoringAuthorized = true;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V63 reuses only the opened X36 formation-FoV seed-211 anchors at ', ...
    'times 72, 100 and 128. Candidate schedules are generated from ', ...
    'current posterior association support and frozen before H=3 truth ', ...
    'scoring. No model-training, held-out or validation claim is authorized.'];
end
