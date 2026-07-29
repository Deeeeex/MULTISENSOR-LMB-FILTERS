function protocol = getBackboneResidualReturnHeadroomProtocol()
% GETBACKBONERESIDUALRETURNHEADROOMPROTOCOL Frozen M24 H=3 gate.

base = getBackbonePreservingResidualProtocol('v1-e05');
protocol = struct();
protocol.id = ...
    'backbone-preserving-residual-return-headroom-m24-h3-v1';
protocol.baseProtocolId = base.id;
protocol.presetName = 'm24-hard';
protocol.seed = 7;
protocol.currentTime = 75;
protocol.horizonSteps = 3;
protocol.horizonTimes = 75:77;
protocol.dominantWeight = base.dominantSourceWeight;
protocol.residualWeight = base.residualSourceWeight;
protocol.payloadToleranceFraction = ...
    base.maximumAttemptedByteDeviationFraction;
protocol.observableActionCodes = 60:78;
protocol.scheduledRoots = 1:4;
protocol.scheduledOrientations = { ...
    'clockwise', 'counter-clockwise'};
protocol.scheduledTemporalPhases = 0:2;
protocol.currentTeacherMaximumBanCount = 2;
protocol.maximumDistinctCandidateCount = 64;
protocol.continuationMode = 'scheduled-burst';
protocol.continuationRootFormation = 1;
protocol.continuationOrientation = 'clockwise';
protocol.continuationTemporalPhase = 0;
protocol.filterSeedOffset = 100000;
protocol.minimumMeanTrackingGainFraction = 0.05;
protocol.maximumWorstNodeRegressionFraction = 0;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireAllSelectedRollingWindowsStrong = true;
protocol.requireZeroFirstActionRepair = true;
protocol.requireZeroPayloadEmergency = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.proposalFeaturesUseTruth = false;
protocol.currentTeacherProposalUsesTruth = true;
protocol.bestCandidateSelectionUsesFutureOutcome = true;
protocol.deployable = false;
protocol.returnDataGenerationAuthorized = false;
protocol.criticTrainingAuthorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.heldoutClaimAllowed = false;
protocol.headroomGatePassed = false;
protocol.bestCandidateIndex = 38;
protocol.bestCandidateEospa = 18.9375438300753;
protocol.bestCandidateMinimumBaselineGainFraction = ...
    -0.0074984295871556;
protocol.bestCandidateWorstNodeGainFraction = 0;
protocol.bestCandidateMaximumByteDeviationFraction = ...
    0.000458195772325794;
protocol.singleFirstActionSearchRejected = true;
protocol.dynamicPolicySequenceSearchRequired = true;
end
