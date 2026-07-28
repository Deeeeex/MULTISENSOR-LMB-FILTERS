function protocol = getRollingSafeJointActionCriticProtocol()
% GETROLLINGSAFEJOINTACTIONCRITICPROTOCOL Frozen joint-action study gates.

rollout = getRollingSafeRolloutProtocol();
protocol = struct();
protocol.id = ...
    'm24-rolling-safe-joint-action-critic-v1';
protocol.representationContractVersion = ...
    'rolling-safe-joint-action-features-v2';
protocol.preflightContractVersion = ...
    'rolling-safe-joint-action-representation-preflight-v2';
protocol.rolloutProtocolId = rollout.id;
protocol.datasetContractVersion = ...
    ['rolling-safe-joint-action-state-v3-', ...
     'selected-delivered-history'];
protocol.presetName = rollout.presetName;
protocol.datasetSeeds = rollout.datasetSeeds;
protocol.snapshotTimes = rollout.continuationStartTime + ...
    (0:(size(rollout.actionCodeSequences, 2) - 1));
protocol.actionCodeSequences = ...
    rollout.actionCodeSequences;
protocol.sourceWeight = rollout.sourceWeight;
protocol.payloadToleranceFraction = ...
    rollout.payloadToleranceFraction;
protocol.behaviorCacheSha256 = ...
    rollout.behaviorCacheSha256;
protocol.behaviorCacheSetSha256 = ...
    rollout.behaviorCacheSetSha256;
protocol.featureContextMode = 'raw';
protocol.maximumRelabellingDifference = 1e-12;
protocol.minimumRegisteredActionDistance = 1e-6;
protocol.maximumAuditRangeViolationFraction = 0.05;
protocol.maximumAuditStandardizedMagnitude = 8.0;
protocol.relabellingsPerBlock = 3;
protocol.syntheticScaleSensorFormationPairs = [8, 4; 12, 6];

% Candidate proposal gate.
protocol.maximumCandidateCount = 32;
protocol.minimumPrivilegedAdmissibleGain = 0.05;
protocol.minimumTopKAdmissibleCaptureFraction = 0.80;
protocol.minimumCandidateOracleAggregateGain = 0.07;
protocol.maximumAttemptedByteDeviationFraction = 0.02;
protocol.requireTailNondegradation = true;
protocol.requireNoRepair = true;
protocol.requireNoEmergencyFallback = true;
protocol.requireNoInfeasibility = true;
protocol.requireRollingB3Pass = true;

% Frozen critic gate.
protocol.maximumGainRegret = 0.02;
protocol.minimumCandidateOracleGainRetention = 0.75;
protocol.minimumCriticAggregateGain = 0.05;
end
