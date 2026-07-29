function protocol = getPredictiveRewardActionAlignmentProtocol()
% GETPREDICTIVEREWARDACTIONALIGNMENTPROTOCOL Frozen training-state gate.

preflight = getPredictiveRewardAlignmentProtocol();
graph = getCvarResidualGraphPolicyDatasetProtocol();
if ~preflight.alignmentPreflightRejected || ...
        ~preflight.actionLevelPredictiveRewardAuditAuthorized
    error('Action-level reward alignment is not authorized.');
end

protocol = struct();
protocol.id = ...
    'predictive-reward-action-alignment-m24-training-states-v1';
protocol.preflightProtocolId = preflight.id;
protocol.graphDatasetProtocolId = graph.id;
protocol.presetName = graph.presetName;
protocol.trainingSeeds = graph.trainingSeeds;
protocol.snapshotTimes = graph.learningLabelTimes;
protocol.continuationStartTime = ...
    graph.continuationStartTime;
protocol.actionCodeSequences = graph.actionCodeSequences( ...
    ismember(graph.datasetSeeds, protocol.trainingSeeds), :);
protocol.filterSeedOffset = graph.filterSeedOffset;
protocol.sourceWeight = graph.sourceWeight;
protocol.dominantWeight = graph.dominantWeight;
protocol.referenceResidualWeight = ...
    graph.localResidualWeight;
protocol.residualWeightGrid = ...
    graph.residualWeightGrid;
protocol.payloadToleranceFraction = ...
    graph.payloadToleranceFraction;
protocol.featureContextMode = ...
    graph.featureContextMode;
protocol.scoreType = preflight.scoreType;
protocol.scoreTruthUsed = false;
protocol.targetTruthUsed = true;
protocol.futureTruthUsed = false;
protocol.feedbackDelay = 1;
protocol.contrastDefinition = ...
    'teacher-action-versus-best-admissible-true-task-runner-up';
protocol.minimumTeacherPositiveScoreFraction = 0.55;
protocol.minimumGlobalScoreTaskSpearman = 0.25;
protocol.minimumPairwisePreferenceAccuracy = 0.60;
protocol.minimumPositivePerSeedSpearmanFraction = 4 / 6;
protocol.expectedReceivingFormationCount = 4;
protocol.minimumFiniteActionCount = ...
    2 * numel(protocol.trainingSeeds) * ...
    numel(protocol.snapshotTimes) * ...
    protocol.expectedReceivingFormationCount;
protocol.trainingOnlyDiagnostic = true;
protocol.developmentSeedsOpened = [];
protocol.heldoutSeedsOpened = [];
protocol.fullActionRewardDatasetAuthorized = false;
protocol.banditImplementationAuthorized = false;
protocol.developmentEvaluationAuthorized = false;
protocol.heldoutM24Authorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.defaultOutputPath = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', ...
    'predictive_reward_action_alignment_m24_training_states.mat');
protocol.defaultReportPath = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', ...
    'PREDICTIVE_REWARD_ACTION_ALIGNMENT_M24_TRAINING_STATES.md');
end
