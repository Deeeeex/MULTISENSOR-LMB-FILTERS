function protocol = getBackbonePreservingResidualProtocol()
% GETBACKBONEPRESERVINGRESIDUALPROTOCOL Additive residual action-space gate.

protocol = struct();
protocol.id = ...
    'backbone-preserving-additive-residual-routing-v1';
protocol.evidenceSplit = 'development';
protocol.scenarioPresets = { ...
    'm24-hard', 'x36-clean-scale'};
protocol.screeningSeeds = 7;
protocol.confirmationSeeds = [11, 17];
protocol.continuationStartTime = 75;
protocol.continuationEndTime = 77;
protocol.dominantSourceWeight = 0.70;
protocol.residualSourceWeight = 0.05;
protocol.selfWeightWithDistinctResidual = 0.25;
protocol.connectivityWindowLength = 3;
protocol.dominantBackboneMode = 'fixed-index-star';
protocol.residualBackboneMode = ...
    'fixed-balanced-cycle';
protocol.localArm = 'local';
protocol.fixedIndexW50Arm = ...
    'directed-fixed-index-w50';
protocol.fixedIndexW70Arm = ...
    'directed-fixed-index-w70';
protocol.matchedResidualStaticArm = ...
    'backbone-residual-static-a70-e05';
protocol.analyticArm = ...
    'backbone-residual-analytic-a70-e05';
protocol.currentOracleArm = ...
    'oracle-backbone-residual-current-a70-e05';
protocol.minimaxOracleArm = ...
    'oracle-backbone-residual-minimax-a70-e05';
protocol.baselineArms = { ...
    protocol.fixedIndexW50Arm, ...
    protocol.fixedIndexW70Arm, ...
    protocol.matchedResidualStaticArm};
protocol.armNames = { ...
    protocol.localArm, ...
    protocol.fixedIndexW50Arm, ...
    protocol.fixedIndexW70Arm, ...
    protocol.matchedResidualStaticArm, ...
    protocol.analyticArm, ...
    protocol.currentOracleArm, ...
    protocol.minimaxOracleArm};
protocol.maximumDirectedMessageMultiplier = 2;
protocol.minimumMeanTrackingGainFraction = 0.05;
protocol.maximumWorstNodeRegressionFraction = 0;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireMeanTrackingBetterThanLocal = true;
protocol.requireWorstNodeNoWorseThanLocal = true;
protocol.requireAllMatureRollingWindowsStrong = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.requireZeroSafetyEmergency = true;
protocol.requireZeroPolicyRepair = true;
protocol.privilegedHeadroomUsesTruth = true;
protocol.privilegedHeadroomDeployable = false;
protocol.featuresUseTruth = false;
protocol.returnLabelsUseFutureOutcome = true;
protocol.returnDataGenerationAuthorized = false;
protocol.criticTrainingAuthorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.heldoutClaimAllowed = false;
end
