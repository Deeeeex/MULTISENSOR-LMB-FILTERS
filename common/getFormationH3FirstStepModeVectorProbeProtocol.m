function protocol = getFormationH3FirstStepModeVectorProbeProtocol()
% GETFORMATIONH3FIRSTSTEPMODEVECTORPROBEPROTOCOL Debt-aware prefix probe.

protocol = struct();
protocol.id = 'formation-h3-first-step-mode-vector-radius2-v1';
protocol.contractVersion = ...
    'formation-h3-first-step-mode-vector-probe-protocol-v1';
protocol.presetName = 'm24-formation-fov';
protocol.seed = 211;
protocol.currentTime = 72;
protocol.horizonSteps = 3;
protocol.modeCount = 4;
protocol.formationCount = 4;
protocol.referenceModeVector = [1, 1, 1, 1];
protocol.centerModeVector = protocol.referenceModeVector;
protocol.maximumHammingDistance = 2;
protocol.expectedCandidateCount = 67;
protocol.expectedActionCount = 256;
protocol.referenceActionIndex = 1;
protocol.reproductionModeVector = [1, 1, 3, 1];
protocol.reproductionActionIndex = 9;
protocol.expectedReproductionTargets = [ ...
    5.988075559537221, 0, -0.001412614870235341, ...
    -11.485696835958140, 0.1671421447540851, ...
    0.1747650221374080];
protocol.reproductionTolerancePercent = 5e-6;
protocol.strongGainPercent = 3;
protocol.constraintScalesPercent = [3, 0.1, 0.1, 3, 1, 1];
protocol.interventionBankType = 'formation-exhaustive-mode-vector';
protocol.cacheRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v14', 'event_conditioned_sentinel', 'cache');
protocol.cacheProtocolId = ...
    'formation-h3-event-conditioned-sentinel-v1';
protocol.cacheGenerationGitCommit = ...
    'c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53';
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'formation_value_v22', 'first_step_mode_vector_radius2');
protocol.actionSelectionUsesTruth = true;
protocol.actionSelectionUsesFutureMeasurements = true;
protocol.runtimePolicyUsesTruth = false;
protocol.openedTrainingMechanismProbeOnly = true;
protocol.finalModelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.finalValidationSeedsReserved = [251, 257, 263, 269, 271];
protocol.evidenceBoundary = [ ...
    'All 67 first-step mode vectors within Hamming distance two of ', ...
    'reference are fixed before execution. The opened seed-211/time-72 ', ...
    'outcomes may identify debt-aware prefixes for a subsequent teacher ', ...
    'beam only; they cannot validate a selector or support M24/X36/', ...
    'final-seed claims.'];
end
