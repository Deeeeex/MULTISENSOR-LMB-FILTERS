function protocol = getFormationStaggeredRecoveryX36SourceProtocol()
% GETFORMATIONSTAGGEREDRECOVERYX36SOURCEPROTOCOL Frozen v37 source gate.

protocol = struct();
protocol.id = ...
    'formation-staggered-recovery-x36-source-v37-v1';
protocol.contractVersion = ...
    'formation-staggered-recovery-x36-source-protocol-v1';
protocol.presetName = 'x36-formation-fov';
protocol.presets = {protocol.presetName};
protocol.seed = 211;
protocol.allSeeds = protocol.seed;
protocol.snapshotTimes = [72, 100, 128];
protocol.anchorTimes = protocol.snapshotTimes;
protocol.blockageFormationPairs = [1, 2; 3, 4; 5, 6];
protocol.horizonSteps = 3;
protocol.filterSeedOffset = 100000;
protocol.expectedNodeCount = 36;
protocol.expectedFormationCount = 6;
protocol.expectedTargetCount = 24;
protocol.expectedFovTotalAngleDeg = 120;
protocol.expectedFovRange = 300;
protocol.expectedSensorHardwareProfile = ...
    'formation-shared-120deg-r300-q300-v1';
protocol.expectedSensorFovHeadingMode = ...
    'formation-shared-scene-center';
protocol.referencePresetName = 'm24-formation-fov';
protocol.maximumFocusLoadRelativeErrorPercent = 1.0;
protocol.maximumFocusBlackoutFraction = 0.005;
protocol.maximumPerTargetBlackoutFraction = 0.23;
protocol.maximumConsecutiveBlackoutSteps = 24;
protocol.minimumFocusHandovers = 65;
protocol.minimumOwnershipEntropy = 0.99;
protocol.minimumMultiFormationFraction = 0.77;
protocol.sourceM24ScreenGenerationCommit = ...
    '1fe5090e4180621f6a5843f262fe84a1be7ba700';
protocol.sourceM24ScreenSha256 = ...
    'af314a31799db440a245026246f7690cc6bf1ce028e141bdbcb0f840910abae9';
protocol.sourceM24AuditCommit = ...
    'f485bb544dea37269d3dd8276a7f001712255f30';
protocol.meanTrackingGainThresholdPercent = 2.0;
protocol.minimumFormationGainThresholdPercent = 0.0;
protocol.worstSensorGainThresholdPercent = 0.0;
protocol.windowConsensusGainThresholdPercent = 0.0;
protocol.terminalConsensusGainThresholdPercent = 0.0;
protocol.minimumAttemptedByteSavingPercent = 0.0;
protocol.minimumStrongStateCount = 2;
protocol.minimumMedianMeanGainPercent = 2.0;
protocol.minimumStateMeanGainPercent = -1.0;
protocol.minimumPositiveTerminalStateCount = 2;
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v37', 'x36_source', 'cache');
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v37', 'x36_source');
protocol.cacheGenerationAuthorized = true;
protocol.sourceOnlyControlAuditAuthorized = true;
protocol.trackingOutcomeRerunAuthorized = false;
protocol.gnnTrainingAuthorized = false;
protocol.x36OutcomeOpeningAuthorized = false;
protocol.x48OutcomeOpeningAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.evidenceBoundary = [ ...
    'V37 opens only the registered X36 formation-FoV seed-211 ', ...
    'reference trajectory at t=72, 100, and 128. These anchors were ', ...
    'fixed before any X36 posterior was generated and cover the three ', ...
    'registered blockage-pair windows symmetrically. Scene geometry ', ...
    'and every controller decision are audited without tracking truth ', ...
    'or future outcomes. X36 tracking remains sealed until the exact ', ...
    'initial controls and complete H=3 runtime traces are frozen and ', ...
    'reproduced from a later clean commit. GNN, X48, reserved seeds, ', ...
    'and validation remain sealed.'];
end
