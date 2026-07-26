function model = fitStructuredFormationTreeRanker(blocks, options)
% FITSTRUCTUREDFORMATIONTREERANKER Learn scores for a whole-tree decision.
%
% The learner uses a loss-augmented structured update. Privileged edge
% residuals are consumed only while fitting. At deployment, the returned
% model scores truth-free edge features and the existing exact projector
% selects a feasible rooted formation tree.

if nargin < 2 || isempty(options)
    options = struct();
end
if isempty(blocks)
    error('Structured formation-tree fitting needs at least one block.');
end
requiredFields = { ...
    'X', 'groupIds', 'receiverIndices', 'senderIndices', ...
    'rawResidual', 'baselineRisk'};
for blockIdx = 1:numel(blocks)
    if ~all(isfield(blocks(blockIdx), requiredFields))
        error('Structured formation-tree block %d is incomplete.', ...
            blockIdx);
    end
end

lambda = max(0, getField(options, 'lambda', 0.01));
epochCount = max(1, round(getField(options, 'epochCount', 80)));
learningRate = max(eps, getField(options, 'learningRate', 0.25));
lossScale = max(eps, getField(options, 'lossScale', 1));
maximumCoefficientNorm = max(eps, getField( ...
    options, 'maximumCoefficientNorm', 100));

allX = vertcat(blocks.X);
featureMean = mean(allX, 1);
featureScale = std(allX, 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
standardized = cell(1, numel(blocks));
oracleSelections = cell(1, numel(blocks));
normalizedTargets = cell(1, numel(blocks));
for blockIdx = 1:numel(blocks)
    standardized{blockIdx} = bsxfun(@rdivide, ...
        bsxfun(@minus, blocks(blockIdx).X, featureMean), ...
        featureScale);
    projectionOptions = blockProjectionOptions(blocks(blockIdx));
    oracleSelections{blockIdx} = selectRootedFormationTreeEdges( ...
        blocks(blockIdx).groupIds, ...
        blocks(blockIdx).receiverIndices, ...
        blocks(blockIdx).senderIndices, ...
        blocks(blockIdx).rawResidual, projectionOptions);
    riskScale = max(sum(blocks(blockIdx).baselineRisk), eps);
    normalizedTargets{blockIdx} = ...
        blocks(blockIdx).rawResidual / riskScale;
end

coefficient = zeros(size(allX, 2), 1);
averagedCoefficient = zeros(size(coefficient));
averageCount = 0;
updateCount = 0;
stepCount = 0;
epochMeanRegret = zeros(1, epochCount);
epochViolationRate = zeros(1, epochCount);
for epochIdx = 1:epochCount
    regretSum = 0;
    violationCount = 0;
    blockOrder = circshift( ...
        1:numel(blocks), [0, -mod(epochIdx - 1, numel(blocks))]);
    for blockIdx = blockOrder
        stepCount = stepCount + 1;
        Z = standardized{blockIdx};
        predictedScores = Z * coefficient;
        lossAugmentedScores = predictedScores - ...
            lossScale * normalizedTargets{blockIdx};
        projectionOptions = blockProjectionOptions(blocks(blockIdx));
        competitor = selectRootedFormationTreeEdges( ...
            blocks(blockIdx).groupIds, ...
            blocks(blockIdx).receiverIndices, ...
            blocks(blockIdx).senderIndices, ...
            lossAugmentedScores, projectionOptions);
        oracle = oracleSelections{blockIdx};
        oracleActual = sum( ...
            normalizedTargets{blockIdx}(oracle.exampleIndices));
        competitorActual = sum( ...
            normalizedTargets{blockIdx}(competitor.exampleIndices));
        regret = max(0, oracleActual - competitorActual);
        oracleScore = sum(predictedScores(oracle.exampleIndices));
        competitorScore = ...
            sum(predictedScores(competitor.exampleIndices));
        marginViolation = competitorScore - oracleScore + ...
            lossScale * regret;

        eta = learningRate / sqrt(stepCount);
        coefficient = max(0, 1 - eta * lambda) * coefficient;
        if marginViolation > 1e-12
            edgeCount = max(1, numel(oracle.exampleIndices));
            oracleFeatures = ...
                sum(Z(oracle.exampleIndices, :), 1) / edgeCount;
            competitorFeatures = ...
                sum(Z(competitor.exampleIndices, :), 1) / edgeCount;
            coefficient = coefficient + eta * ...
                reshape(oracleFeatures - competitorFeatures, [], 1);
            updateCount = updateCount + 1;
            violationCount = violationCount + 1;
        end
        coefficientNorm = norm(coefficient);
        if coefficientNorm > maximumCoefficientNorm
            coefficient = coefficient * ...
                (maximumCoefficientNorm / coefficientNorm);
        end
        averagedCoefficient = averagedCoefficient + coefficient;
        averageCount = averageCount + 1;
        regretSum = regretSum + regret;
    end
    epochMeanRegret(epochIdx) = regretSum / numel(blocks);
    epochViolationRate(epochIdx) = ...
        violationCount / numel(blocks);
end
if averageCount > 0
    coefficient = averagedCoefficient / averageCount;
end

model = struct();
model.kind = 'loss-augmented-formation-tree-ranker-v1';
model.family = 'structured-ranking';
model.featureMean = featureMean;
model.featureScale = featureScale;
model.coefficient = coefficient;
model.intercept = 0;
model.lambda = lambda;
model.epochCount = epochCount;
model.learningRate = learningRate;
model.lossScale = lossScale;
model.maximumCoefficientNorm = maximumCoefficientNorm;
model.stepCount = stepCount;
model.updateCount = updateCount;
model.epochMeanRegret = epochMeanRegret;
model.epochViolationRate = epochViolationRate;
end

function options = blockProjectionOptions(block)
options = struct();
if isfield(block, 'projectionOptions') && ...
        isstruct(block.projectionOptions)
    options = block.projectionOptions;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
