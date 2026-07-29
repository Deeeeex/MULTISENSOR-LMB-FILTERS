function protocol = getPredictiveRewardAlignmentProtocol()
% GETPREDICTIVEREWARDALIGNMENTPROTOCOL Frozen training-only M24 score gate.

headroom = getBackboneResidualCvarTailGuardAdaptiveTrustProtocol();
protocol = struct();
protocol.id = ...
    'predictive-reward-alignment-m24-seed7-h3-v1';
protocol.headroomProtocolId = headroom.id;
protocol.presetName = headroom.presetName;
protocol.seed = headroom.seed;
protocol.continuationStartTime = ...
    headroom.continuationStartTime;
protocol.continuationEndTime = ...
    headroom.continuationEndTime;
protocol.actionTimes = ...
    protocol.continuationStartTime: ...
    (protocol.continuationEndTime - 1);
protocol.rewardTimes = protocol.actionTimes + 1;
protocol.feedbackDelay = 1;
protocol.scoreType = ...
    'poisson-measurement-intensity-log-score';
protocol.scoreTruthUsed = false;
protocol.scoreExactLmbMeasurementSetLikelihood = false;
protocol.scoreInterpretation = [ ...
    'proper-log-score-for-the-poisson-measurement-intensity-', ...
    'approximation'];
protocol.armModes = { ...
    'backbone-residual-static-a70-e05', ...
    'backbone-residual-analytic-a70-e05', ...
    headroom.armName};
protocol.referenceArmModes = protocol.armModes(1:2);
protocol.privilegedHeadroomArmMode = protocol.armModes{3};
protocol.filterSeedOffset = 100000;
protocol.rollingSafeBackboneMode = 'fixed-balanced-cycle';
protocol.maximumAttemptedByteDeviationFraction = ...
    headroom.maximumAttemptedByteDeviationFraction;
protocol.minimumArmRankSpearman = 0.50;
protocol.minimumPairedSensorTimeSpearman = 0.20;
protocol.minimumTeacherScoreAdvantage = 0;
protocol.trainingOnlyDiagnostic = true;
protocol.developmentSeedsOpened = [];
protocol.heldoutSeedsOpened = [];
protocol.developmentEvaluationAuthorized = false;
protocol.heldoutM24Authorized = false;
protocol.x36PolicyRunAuthorized = false;
protocol.banditImplementationAuthorized = false;
end
