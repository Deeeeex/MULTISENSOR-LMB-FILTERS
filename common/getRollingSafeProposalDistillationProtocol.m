function protocol = getRollingSafeProposalDistillationProtocol()
% GETROLLINGSAFEPROPOSALDISTILLATIONPROTOCOL Expanded M24 teacher data.
%
% The behavior trajectory is fully truth-free: posterior-analytic action
% code 80 runs from t=75 through t=100. Privileged current-risk graphs are
% attached offline at every predecision state for proposal distillation.

rollout = getRollingSafeRolloutProtocol();
protocol = struct();
protocol.id = ...
    'm24-rolling-safe-proposal-distillation-truthfree-behavior-v1';
protocol.datasetContractVersion = ...
    'rolling-safe-proposal-distillation-m24-v1';
protocol.datasetVariant = ...
    'proposal-distillation-state-v1';
protocol.presetName = rollout.presetName;
protocol.datasetSeeds = rollout.datasetSeeds;
protocol.continuationStartTime = ...
    rollout.continuationStartTime;
protocol.continuationEndTime = 100;
protocol.snapshotTimes = ...
    protocol.continuationStartTime:protocol.continuationEndTime;
protocol.behaviorActionCode = 80;
protocol.actionCodeSequences = ...
    protocol.behaviorActionCode * ones( ...
        numel(protocol.datasetSeeds), ...
        numel(protocol.snapshotTimes));
protocol.sourceWeight = rollout.sourceWeight;
protocol.payloadToleranceFraction = ...
    rollout.payloadToleranceFraction;
protocol.filterSeedOffset = rollout.filterSeedOffset;
protocol.featureContextMode = 'raw';
protocol.behaviorCacheSha256 = ...
    rollout.behaviorCacheSha256;
protocol.behaviorCacheSetSha256 = ...
    rollout.behaviorCacheSetSha256;
protocol.expectedBlockCount = ...
    numel(protocol.datasetSeeds) * ...
        numel(protocol.snapshotTimes);
protocol.privilegedTargetActionCodes = [0, 90, 91, 92];
protocol.minimumTargetsPerBlock = 3;
protocol.maximumTargetsPerBlock = 4;
protocol.behaviorTruthUsed = false;
protocol.behaviorFutureOutcomeUsed = false;
protocol.featureTruthUsed = false;
protocol.featureFutureOutcomeUsed = false;
protocol.targetTruthUsed = true;
protocol.targetFutureOutcomeUsed = false;
protocol.deployable = false;
protocol.developmentOnly = true;
protocol.validationClaimAllowed = false;
end
