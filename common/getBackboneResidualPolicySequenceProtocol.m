function protocol = getBackboneResidualPolicySequenceProtocol()
% GETBACKBONERESIDUALPOLICYSEQUENCEPROTOCOL Frozen M24 S/C sequence gate.

base = getBackbonePreservingResidualProtocol('v1-e05');
protocol = struct();
protocol.id = ...
    'backbone-preserving-residual-policy-sequence-m24-h3-v1';
protocol.baseProtocolId = base.id;
protocol.presetName = 'm24-hard';
protocol.seed = 7;
protocol.anchorTime = 75;
protocol.horizonTimes = 75:77;
protocol.sequenceLength = 3;
protocol.sequenceMasks = dec2bin(0:7, 3);
protocol.dominantWeight = base.dominantSourceWeight;
protocol.residualWeight = base.residualSourceWeight;
protocol.payloadToleranceFraction = ...
    base.maximumAttemptedByteDeviationFraction;
protocol.minimumMeanTrackingGainFraction = 0.05;
protocol.maximumWorstNodeRegressionFraction = 0;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireAllSelectedRollingWindowsStrong = true;
protocol.requireZeroRepair = true;
protocol.requireZeroPayloadEmergency = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.maskSelectionUsesFutureOutcome = true;
protocol.currentStepUsesTruth = true;
protocol.deployable = false;
protocol.headroomGatePassed = false;
protocol.constraintEligibleMasks = { ...
    '011', '101', '111'};
protocol.bestSequenceMask = '101';
protocol.bestSequenceEospa = 18.0414464472558;
protocol.bestSequenceWorstSensorEospa = ...
    34.6311036710960;
protocol.bestSequenceMinimumBaselineGainFraction = ...
    0.0401749494977313;
protocol.bestSequenceAttemptedByteDeviationFraction = ...
    0.000196289314091528;
protocol.staticCurrentTimingSearchRejected = true;
protocol.actionSpaceExpansionRequired = true;
protocol.returnDataGenerationAuthorized = false;
protocol.criticTrainingAuthorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.heldoutClaimAllowed = false;
end
