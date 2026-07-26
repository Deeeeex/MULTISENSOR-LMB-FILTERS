function protocol = getStructuredFormationTreeCtxv2Protocol(cacheDirectory)
% GETSTRUCTUREDFORMATIONTREECTXV2PROTOCOL Freeze the M24 proxy audit design.
%
% The corrected ctxv2 datasets are split by seed, never by adjacent time
% snapshots. Hyperparameters are selected only through leave-one-seed-out
% cross-validation on the declared training seeds. The validation seeds
% can pass or reject the frozen proxy candidate, but cannot select it.

if nargin < 1 || isempty(cacheDirectory)
    cacheDirectory = fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'cache');
end

protocol = struct();
protocol.name = 'm24-hard-structured-tree-ctxv2-proxy-v1';
protocol.scenarioName = 'm24-hard';
protocol.datasetContractVersion = ...
    'directed-teacher-series-v2-receiver-row-previous-adjacency';
protocol.previousAdjacencyConvention = ...
    'receiver-row-sender-column-directed';
protocol.trainingSeeds = [19, 23, 29];
protocol.validationSeeds = [31, 37];
protocol.futureClosedLoopAuditSeeds = 43;
protocol.snapshotTimes = 75:80;
protocol.sourceWeight = 0.70;
protocol.baselineMode = 'fixed-index-star';
protocol.baselinePhase = 1;
protocol.anchorTime = 75;
protocol.featureContextMode = 'raw';
protocol.modelFamily = 'structured-ranking';
protocol.structuredLambdaGrid = [0, 0.001, 0.01, 0.1, 1];
protocol.structuredEpochCount = 80;
protocol.structuredLearningRate = 0.25;
protocol.structuredLossScale = 1;
protocol.structuredMaximumCoefficientNorm = 100;
protocol.minimumAggregateRiskImprovement = 0.05;
protocol.maximumHarmfulEdgeFraction = 1 / 3;
protocol.maximumHarmfulEdgeCount = 1;
protocol.minimumOracleCaptureFraction = 0.50;
protocol.maximumAttemptedByteDeviation = 0.02;
protocol.enforcePerEdgeSafety = false;
protocol.requirePreviousUnionStrongConnectivity = true;
protocol.excludedFeatureNames = { ...
    'same_formation', 'source_weight', ...
    'source_weight_squared', 'backbone_action'};

allSeeds = [protocol.trainingSeeds, protocol.validationSeeds];
datasetPaths = cell(1, numel(allSeeds) * ...
    numel(protocol.snapshotTimes));
datasetSeeds = zeros(size(datasetPaths));
datasetTimes = zeros(size(datasetPaths));
cursor = 0;
for seed = allSeeds
    for snapshotTime = protocol.snapshotTimes
        cursor = cursor + 1;
        datasetSeeds(cursor) = seed;
        datasetTimes(cursor) = snapshotTime;
        datasetPaths{cursor} = fullfile(cacheDirectory, sprintf( ...
            ['directed_teacher_oracle_v4_ctxv2_', ...
             'm24_hard_seed%d_t%d.mat'], seed, snapshotTime));
    end
end
trainingBlockCount = numel(protocol.trainingSeeds) * ...
    numel(protocol.snapshotTimes);
protocol.datasetPaths = datasetPaths;
protocol.datasetSeeds = datasetSeeds;
protocol.datasetTimes = datasetTimes;
protocol.trainDatasetIndices = 1:trainingBlockCount;
protocol.validationDatasetIndices = ...
    (trainingBlockCount + 1):numel(datasetPaths);
end
