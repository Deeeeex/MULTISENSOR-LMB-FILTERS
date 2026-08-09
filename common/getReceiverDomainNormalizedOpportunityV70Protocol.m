function protocol = getReceiverDomainNormalizedOpportunityV70Protocol()
% GETRECEIVERDOMAINNORMALIZEDOPPORTUNITYV70PROTOCOL Frozen source design.

v69 = getSignedMergeSplitOpportunityV69Protocol();
protocol = struct();
protocol.id = 'receiver-domain-normalized-opportunity-v70-v1';
protocol.contractVersion = ...
    'receiver-domain-normalized-opportunity-v70-protocol-v1';
protocol.baseProtocolId = v69.id;
protocol.referencePolicyName = v69.referencePolicyName;
protocol.presets = v69.presets;
protocol.allSeeds = v69.allSeeds;
protocol.snapshotTimes = v69.snapshotTimes;
protocol.cacheRoot = v69.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v70', 'receiver_domain_opportunity');
protocol.positiveSupportThreshold = v69.positiveSupportThreshold;
protocol.horizonSteps = v69.horizonSteps;
protocol.minimumLocalQuarantineNetFraction = 0.01;
protocol.minimumLocalAlternativeNetFraction = 0.01;
% Existing opened V66 development rows put the weak convoy event below
% 0.10 after receiver-domain normalization, while all four strong radial
% rows remain above it.  The threshold is frozen before the V69
% merge-split source states are scored under the new normalization.
protocol.minimumLocalRobustMarginExposure = 0.10;
protocol.maximumUsefulLossToRescueRatio = 1.0;
protocol.requireZeroSupportedDownwardCrossings = true;
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V70 re-scores the already frozen V69 M24/X36 current states. ', ...
    'A formation action is normalized only by the reference existence ', ...
    'mass of its affected receivers; appending unrelated formations ', ...
    'therefore cannot dilute the local decision. Current posterior, ', ...
    'current observation support, current physical links, and registered ', ...
    'topology history are allowed. Route execution, tracking outcomes, ', ...
    'future measurements, truth, model training, and validation remain ', ...
    'closed.'];
end
