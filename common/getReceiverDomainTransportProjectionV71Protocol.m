function protocol = getReceiverDomainTransportProjectionV71Protocol()
% GETRECEIVERDOMAINTRANSPORTPROJECTIONV71PROTOCOL Frozen projection design.

v70 = getReceiverDomainNormalizedOpportunityV70Protocol();
v69 = getSignedMergeSplitOpportunityV69Protocol();
protocol = struct();
protocol.id = 'receiver-domain-transport-projection-v71-v1';
protocol.contractVersion = ...
    'receiver-domain-transport-projection-v71-protocol-v1';
protocol.baseProtocolId = v70.id;
protocol.referencePolicyName = v70.referencePolicyName;
protocol.presets = v70.presets;
protocol.allSeeds = v70.allSeeds;
protocol.anchorTimes = [80, 52];
protocol.anchorFormationIds = {[3], [4, 5]};
protocol.cacheRoot = v70.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v71', 'receiver_domain_transport_projection');
protocol.sourceWeight = v69.sourceWeight;
protocol.decisionExistenceThreshold = v69.decisionExistenceThreshold;
protocol.maximumNominatedFormationCount = 12;
protocol.maximumCandidateBundleEvaluations = 2^12 - 1;
protocol.referenceRecoverySteps = 2;
protocol.requireExactMessageCountParity = true;
protocol.requirePerReceiverMessageCountParity = true;
protocol.requirePerReceiverWeightMultisetParity = true;
protocol.requireCurrentPhysicality = true;
protocol.requireZeroSupportedDownwardCrossings = true;
protocol.requireRollingB3SensorConnectivity = true;
protocol.requireRollingB3FormationConnectivity = true;
protocol.routeConstructionAuthorized = true;
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V71 converts frozen V70 transport nominations into one current ', ...
    'route. Each affected receiver replaces at most one registered ', ...
    'cross-formation residual sender while retaining the same row ', ...
    'weight and message count. Candidate formation bundles are selected ', ...
    'by total receiver-domain net mass and must satisfy current ', ...
    'physicality plus rolling-B3 sensor and formation connectivity. ', ...
    'The projection reads no truth, future measurement, future link ', ...
    'outcome, tracking outcome, or learned prediction.'];
end
