function protocol = getBackboneResidualSplicedCycleProtocol()
% GETBACKBONERESIDUALSPLICEDCYCLEPROTOCOL Frozen M24 cycle gate.

base = getBackbonePreservingResidualProtocol('v1-e05');
predecessor = getBackboneResidualPolicySequenceProtocol();
protocol = struct();
protocol.id = ...
    'backbone-residual-spliced-strong-cycle-m24-h3-v1';
protocol.baseProtocolId = base.id;
protocol.predecessorProtocolId = predecessor.id;
protocol.presetName = 'm24-hard';
protocol.seed = 7;
protocol.continuationStartTime = 75;
protocol.continuationEndTime = 77;
protocol.horizonTimes = 75:77;
protocol.dominantWeight = base.dominantSourceWeight;
protocol.residualWeight = base.residualSourceWeight;
protocol.staticClockwiseArm = ...
    'backbone-residual-spliced-cycle-cw-a70-e05';
protocol.staticCounterClockwiseArm = ...
    'backbone-residual-spliced-cycle-ccw-a70-e05';
protocol.currentOracleArm = ...
    'oracle-backbone-residual-spliced-cycle-current-a70-e05';
protocol.baselineArms = { ...
    'directed-fixed-index-w70', ...
    'backbone-residual-static-a70-e05', ...
    'backbone-residual-analytic-a70-e05', ...
    protocol.staticClockwiseArm, ...
    protocol.staticCounterClockwiseArm};
protocol.armNames = [{ 'local'}, ...
    protocol.baselineArms, {protocol.currentOracleArm}];
protocol.maximumDirectedMessageMultiplier = 2;
protocol.crossFormationEdgesPerStepEqualsFormationCount = true;
protocol.residualGraphInstantaneouslySensorStrong = true;
protocol.residualGraphInDegree = 1;
protocol.residualGraphOutDegree = 1;
protocol.minimumMeanTrackingGainFraction = 0.05;
protocol.maximumWorstNodeRegressionFraction = 0;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireAllSelectedRollingWindowsStrong = true;
protocol.requireZeroRepair = true;
protocol.requireZeroPayloadEmergency = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.privilegedHeadroomUsesTruth = true;
protocol.deployable = false;
protocol.headroomGatePassed = false;
protocol.currentOracleEospa = ...
    18.0567158147012;
protocol.currentOracleWorstSensorEospa = ...
    34.6314120330557;
protocol.minimumBaselineMeanGainFraction = ...
    0.0393626021384249;
protocol.minimumBaselineWorstNodeGainFraction = ...
    -0.0000411226708800069;
protocol.attemptedByteDeviationFraction = ...
    0.00112457419531605;
protocol.constraintEligible = true;
protocol.fixedResidualWeightRejected = true;
protocol.jointEdgeAndTrustWeightHeadroomRequired = true;
protocol.registeredCrossResidualWeightGrid = ...
    [0.05, 0.10, 0.20, 0.25];
protocol.minimumSelfWeight = 0.05;
protocol.returnDataGenerationAuthorized = false;
protocol.criticTrainingAuthorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.heldoutClaimAllowed = false;
end
