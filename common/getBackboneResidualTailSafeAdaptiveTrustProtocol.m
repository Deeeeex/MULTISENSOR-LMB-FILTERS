function protocol = ...
    getBackboneResidualTailSafeAdaptiveTrustProtocol()
% GETBACKBONERESIDUALTAILSAFEADAPTIVETRUSTPROTOCOL Frozen M24 tail gate.

predecessor = getBackboneResidualAdaptiveTrustProtocol();
protocol = struct();
protocol.id = ...
    'backbone-residual-tail-safe-adaptive-trust-m24-h3-v1';
protocol.predecessorProtocolId = predecessor.id;
protocol.presetName = predecessor.presetName;
protocol.seed = predecessor.seed;
protocol.continuationStartTime = ...
    predecessor.continuationStartTime;
protocol.continuationEndTime = ...
    predecessor.continuationEndTime;
protocol.horizonTimes = predecessor.horizonTimes;
protocol.dominantWeight = predecessor.dominantWeight;
protocol.localResidualWeight = ...
    predecessor.localResidualWeight;
protocol.crossResidualWeightGrid = ...
    predecessor.crossResidualWeightGrid;
protocol.minimumSelfWeight = ...
    predecessor.minimumSelfWeight;
protocol.armName = [ ...
    'oracle-backbone-residual-spliced-cycle-', ...
    'tail-safe-adaptive-current-a70'];
protocol.baselineSourceRelativePath = ...
    predecessor.baselineSourceRelativePath;
protocol.baselineSourceSha256 = ...
    predecessor.baselineSourceSha256;
protocol.baselineModes = predecessor.baselineModes;
protocol.maximumDirectedMessageMultiplier = ...
    predecessor.maximumDirectedMessageMultiplier;
protocol.expectedCrossFormationEdgeCount = ...
    predecessor.expectedCrossFormationEdgeCount;
protocol.minimumMeanTrackingGainFraction = ...
    predecessor.minimumMeanTrackingGainFraction;
protocol.maximumWorstNodeRegressionFraction = ...
    predecessor.maximumWorstNodeRegressionFraction;
protocol.maximumAttemptedByteDeviationFraction = ...
    predecessor.maximumAttemptedByteDeviationFraction;
protocol.requireAllSelectedRollingWindowsStrong = true;
protocol.requireZeroRepair = true;
protocol.requireZeroPayloadEmergency = true;
protocol.requireZeroTopologyInfeasibility = true;
protocol.referenceMode = ...
    'matched-static-residual';
protocol.referenceArm = ...
    predecessor.tailSafeReferenceArm;
protocol.referenceResidualWeight = ...
    predecessor.tailSafeReferenceResidualWeight;
protocol.requireNonnegativeReferenceAdvantage = ...
    predecessor.requireNonnegativeReferenceAdvantage;
protocol.referenceAdvantageTolerance = 1e-12;
protocol.referenceGuardMode = ...
    'all-receivers';
protocol.failClosedOnNoCompleteSafeCycle = true;
protocol.privilegedHeadroomUsesTruth = true;
protocol.deployable = false;
protocol.evidencePending = false;
protocol.headroomGatePassed = false;
protocol.strictAllReceiverGuardRejected = true;
protocol.smokeFeasibilityPassed = false;
protocol.smokeFallbackUsed = true;
protocol.smokeTopologyInfeasibleRate = 1;
protocol.smokeCrossFormationEdgeCount = 0;
protocol.smokeEospa = 19.9529;
protocol.smokeWorstSensorEospa = 42.6925;
protocol.smokeSourceSha256 = ...
    '4c324f8b4270ff00f1edbcd2e4686f3408f2a20882e6070c6196e8b8e7ddbfb6';
protocol.returnDataGenerationAuthorized = false;
protocol.criticTrainingAuthorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.heldoutClaimAllowed = false;
end
