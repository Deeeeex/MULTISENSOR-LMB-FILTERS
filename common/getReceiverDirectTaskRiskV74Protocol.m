function protocol = getReceiverDirectTaskRiskV74Protocol()
% GETRECEIVERDIRECTTASKRISKV74PROTOCOL Frozen source-only direct-risk gate.

v73 = getReceiverFusionAlignmentV73Protocol();
protocol = struct();
protocol.id = 'receiver-direct-task-risk-v74-v1';
protocol.contractVersion = ...
    'receiver-direct-task-risk-v74-protocol-v1';
protocol.baseProtocolId = v73.id;
protocol.cases = v73.cases;
protocol.cacheRoot = v73.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v74', 'receiver_direct_task_risk');
protocol.fusionConfig = v73.fusionConfig;
protocol.posteriorBayesLabelAggregationMode = 'sum';
protocol.receiverTailFraction = 0.34;
protocol.receiverTailWeight = 0.50;
protocol.disagreementAggregationMode = 'mean';
protocol.disagreementTailFraction = 0.25;
protocol.maximumIncomingCount = 2;
protocol.requireNonnegativeAffectedFormationBayesObjective = true;
protocol.requireNonincreasingMeanNetworkDisagreement = true;
protocol.requireNonincreasingTailNetworkDisagreement = true;
protocol.sourceRiskEvaluationAuthorized = true;
protocol.routeExecutionAuthorized = false;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V74 evaluates the reference plus every feasible V73 formation ', ...
    'subset at the two opened source anchors. It uses exact one-round ', ...
    'mixture-aware receiver outcome distributions under current ', ...
    'independent link uncertainty, then computes truth-free posterior ', ...
    'Bayes-risk and mean/tail network disagreement. No route is ', ...
    'executed and no tracking outcome, future measurement, truth, or ', ...
    'temporal propagation model is read.'];
end
