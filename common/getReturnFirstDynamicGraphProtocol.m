function protocol = getReturnFirstDynamicGraphProtocol()
% GETRETURNFIRSTDYNAMICGRAPHPROTOCOL Frozen return-first redesign gates.
%
% This protocol separates an action-space headroom test from learning.
% Privileged current-risk arms may diagnose whether the exact rolling-safe
% feasible set contains useful actions, but they can never be reported as
% deployable policies or used as held-out evidence.

protocol = struct();
protocol.id = ...
    'return-first-rolling-safe-graph-value-v1';
protocol.evidenceSplit = 'development';
protocol.scenarioPresets = { ...
    'm24-hard', 'x36-clean-scale'};
protocol.screeningSeeds = 7;
protocol.confirmationSeeds = [11, 17];
protocol.continuationStartTime = 75;
protocol.continuationEndTime = 77;
protocol.sourceWeight = 0.50;
protocol.weightToken = 50;
protocol.connectivityWindowLength = 3;
protocol.backboneMode = 'fixed-balanced-cycle';
protocol.baselineArm = 'directed-fixed-index-w50';
protocol.matchedBackboneArm = ...
    'directed-fixed-cycle-w50';
protocol.baselineArms = { ...
    protocol.baselineArm, protocol.matchedBackboneArm};
protocol.armNames = { ...
    'local', ...
    protocol.baselineArm, ...
    protocol.matchedBackboneArm, ...
    'rolling-safe-analytic-w50', ...
    'oracle-rolling-safe-current-w50', ...
    'oracle-rolling-safe-minimax-w50'};
protocol.privilegedArmPatterns = { ...
    '^oracle-rolling-safe-current-', ...
    '^oracle-rolling-safe-minimax-'};
protocol.minimumMeanTrackingGainFraction = 0.05;
protocol.maximumWorstNodeRegressionFraction = 0;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireMeanTrackingBetterThanLocal = true;
protocol.requireWorstNodeNoWorseThanLocal = true;
protocol.requireGainAgainstEveryBaseline = true;
protocol.requireWorstNodeNoWorseThanEveryBaseline = true;
protocol.requireAllMatureRollingWindowsStrong = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.requireZeroSafetyEmergency = true;
protocol.requireZeroPolicyRepair = true;
protocol.requireM24HeadroomBeforeLearning = true;
protocol.requireX36HeadroomBeforeLearning = true;
protocol.primaryLearningTarget = ...
    'paired-h3-realized-tracking-return';
protocol.rejectedPrimaryTarget = ...
    'exact-current-risk-teacher-graph-identity';
protocol.plannedCritic = ...
    'permutation-invariant-candidate-graph-value';
protocol.plannedSearch = ...
    'interaction-aware-beam-search-plus-exact-rolling-b3-projector';
protocol.featuresUseTruth = false;
protocol.returnLabelsUseFutureOutcome = true;
protocol.privilegedHeadroomUsesTruth = true;
protocol.privilegedHeadroomDeployable = false;
protocol.heldoutClaimAllowed = false;
end
