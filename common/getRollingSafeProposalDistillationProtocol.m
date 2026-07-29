function protocol = getRollingSafeProposalDistillationProtocol()
% GETROLLINGSAFEPROPOSALDISTILLATIONPROTOCOL Expanded M24 teacher data.
%
% The behavior trajectory is fully truth-free: posterior-analytic action
% code 80 runs from t=75 through t=83. Privileged current-risk graphs are
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
protocol.continuationEndTime = 83;
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
protocol.expectedTotalTargetCount = 210;
protocol.targetActionCodes = [0, 90, 91, 92];
protocol.expectedTargetCountByActionCode = [54, 54, 51, 51];
protocol.expectedUniqueTargetGraphCount = 210;
protocol.expectedUniqueBehaviorGraphCount = 54;
protocol.expectedUniqueStateFeatureCount = 54;
protocol.expectedFeatureColumnCount = 42;
protocol.expectedCandidateEdgeCount = 432;
protocol.expectedFeatureContractSha256 = ...
    ['b4ac6e96d37bf8124f9a6073bff512b', ...
     'd8b53160174d090b61e151c1e8fd3322a'];
protocol.datasetPath = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'datasets', ...
    ['rolling_safe_proposal_distillation_', ...
     'm24_hard_t75_t83_v1.mat']);
protocol.datasetSha256 = ...
    ['ce815847e6e540e379934e2f09bbdb4f', ...
     '461cc2b4c7dc4ac918f35ff56feccf06'];
protocol.datasetGenerationCommit = ...
    '30d5ca3fcfe8a46aa37583c7a55cebe426c8e752';
protocol.sourceShardSetSha256 = ...
    ['5faa374e11428c663db949bc0db30d6de', ...
     '71f3b5d1ca75c8dedf1c2e210fa9409'];
protocol.auditContractVersion = ...
    'rolling-safe-proposal-distillation-audit-m24-v1';
protocol.auditPath = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', ...
    'proposal_distillation_dataset_preflight_m24.mat');
protocol.auditSha256 = ...
    ['a57ebdfdda8d840e94f9525dd12076963', ...
     '0c5124506ad4dc6d3d39a3d79f31ec8'];
protocol.auditGenerationCommit = ...
    'd13365b05e161984a5d1c1747c3861fea1cce0c1';
protocol.modelContractVersion = ...
    'rolling-safe-expanded-proposal-head-model-m24-v1';
protocol.modelHeadActionCodes = [0, 90, 91, 92];
protocol.modelSelectionMaximumBannedEdges = 0;
protocol.modelEvaluationMaximumBannedEdges = 1;
protocol.modelEvaluationTopK = 16;
protocol.modelRidgeFeatureModes = {'raw', 'graph-context'};
protocol.modelRidgeLambdaGrid = [1e-4, 1e-2, 1, 100];
protocol.modelMlpFeatureModes = {'graph-context'};
protocol.modelMlpHiddenWidthGrid = [16, 32, 64];
protocol.modelMlpLambdaGrid = [1e-5, 1e-3];
protocol.modelMlpEpochCount = 200;
protocol.modelMlpLearningRate = 0.01;
protocol.modelInitializationSeed = 9042;
protocol.minimumExpandedTargetStateCaptureFraction = 0.80;
protocol.minimumExpandedSeedTargetStateCaptureFraction = 2 / 3;
protocol.minimumLegacyValueStateCaptureFraction = 0.80;
protocol.expandedProposalModelPath = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'models', ...
    'rolling_safe_expanded_proposal_heads_m24_t75_t83_v1.mat');
protocol.expandedProposalModelSha256 = ...
    ['3633d1eeaee7dbd4b00a538f553521b9', ...
     '40ac19afbd8d56daedec2f8eca941a0b'];
protocol.expandedProposalModelGenerationCommit = ...
    'a7c83d188c49916195a6b6ed6da4708ee3c569f0';
protocol.expandedProposalSelectedFamily = 'mlp';
protocol.expandedProposalSelectedFeatureMode = 'graph-context';
protocol.expandedProposalSelectedHiddenWidth = 64;
protocol.expandedProposalSelectedLambda = 1e-5;
protocol.expandedProposalSingleShotStateCaptureFraction = 2 / 54;
protocol.expandedProposalTopKStateCaptureFraction = 2 / 54;
protocol.expandedProposalTopKMinimumSeedCaptureFraction = 0;
protocol.expandedProposalTopKTargetGraphRecallFraction = 2 / 210;
protocol.expandedProposalTopKMeanBestTargetEdgeF1 = 0.197530864197531;
protocol.expandedProposalTopKMeanDistinctProposalCount = 3.92592592592593;
protocol.expandedProposalLegacyValueStateCaptureFraction = 1 / 10;
protocol.expandedProposalLegacyValueGraphRecallFraction = 1 / 34;
protocol.expandedProposalLegacyValueMeanBestTargetEdgeF1 = 0.30;
protocol.expandedProposalReturnGenerationAuthorized = false;
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
