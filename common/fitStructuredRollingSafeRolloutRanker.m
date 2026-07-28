function model = ...
    fitStructuredRollingSafeRolloutRanker(blocks, options)
% FITSTRUCTUREDROLLINGSAFEROLLOUTRANKER Learn whole-graph edge utilities.
%
% Each optimization step compares the recorded target graph with a
% loss-augmented graph selected by the exact rolling-B3 projector at the
% target cardinality. Blocks, rather than their hundreds of candidate
% edges, receive equal training weight.

if nargin < 2 || isempty(options)
    options = struct();
end
if isempty(blocks)
    error('Structured rolling-safe fitting needs at least one block.');
end
requiredFields = { ...
    'features', 'selectedCrossMask', 'groupIds', ...
    'receiverIndices', 'senderIndices', 'physicalAdjacency', ...
    'currentTime', 'previousAdjacencyHistory', ...
    'senderPayloadBytes'};
for blockIdx = 1:numel(blocks)
    if ~all(isfield(blocks(blockIdx), requiredFields))
        error('Structured rolling-safe block %d is incomplete.', ...
            blockIdx);
    end
end

featureMask = logical(reshape(getField( ...
    options, 'featureMask', ...
    true(1, size(blocks(1).features, 2))), 1, []));
if numel(featureMask) ~= size(blocks(1).features, 2) || ...
        ~any(featureMask)
    error('Structured rolling-safe feature mask is invalid.');
end
for blockIdx = 1:numel(blocks)
    if size(blocks(blockIdx).features, 2) ~= ...
            numel(featureMask)
        error('Structured rolling-safe feature dimensions differ.');
    end
end
sourceWeight = getField(options, 'sourceWeight', 0.70);
payloadToleranceFraction = getField( ...
    options, 'payloadToleranceFraction', inf);
lambda = max(0, getField(options, 'lambda', 0.01));
epochCount = max(1, round(getField( ...
    options, 'epochCount', 40)));
learningRate = max(eps, getField( ...
    options, 'learningRate', 0.25));
lossScale = max(eps, getField(options, 'lossScale', 1));
maximumCoefficientNorm = max(eps, getField( ...
    options, 'maximumCoefficientNorm', 100));

allX = vertcat(blocks.features);
allX = allX(:, featureMask);
if isempty(allX) || any(~isfinite(allX(:)))
    error('Structured rolling-safe features must be finite.');
end
featureMean = mean(allX, 1);
featureScale = std(allX, 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
standardized = cell(1, numel(blocks));
for blockIdx = 1:numel(blocks)
    retained = blocks(blockIdx).features(:, featureMask);
    standardized{blockIdx} = bsxfun(@rdivide, ...
        bsxfun(@minus, retained, featureMean), featureScale);
end

coefficient = zeros(size(allX, 2), 1);
averagedCoefficient = zeros(size(coefficient));
averageCount = 0;
stepCount = 0;
updateCount = 0;
epochMeanHammingLoss = zeros(1, epochCount);
epochViolationRate = zeros(1, epochCount);
for epochIdx = 1:epochCount
    blockOrder = circshift( ...
        1:numel(blocks), ...
        [0, -mod(epochIdx - 1, numel(blocks))]);
    epochLoss = 0;
    epochViolations = 0;
    for blockIdx = blockOrder
        stepCount = stepCount + 1;
        block = blocks(blockIdx);
        truth = logical(reshape( ...
            block.selectedCrossMask, [], 1));
        cardinality = nnz(truth);
        Z = standardized{blockIdx};
        if numel(truth) ~= size(Z, 1)
            error('Structured rolling-safe labels are misaligned.');
        end

        eta = learningRate / sqrt(stepCount);
        coefficient = max(0, 1 - eta * lambda) * coefficient;
        if cardinality > 0
            predictedScores = Z * coefficient;
            lossAugmentedScores = predictedScores + ...
                lossScale * double(~truth) / cardinality;
            competitor = projectExactCardinality( ...
                block, lossAugmentedScores, cardinality, ...
                sourceWeight, payloadToleranceFraction);
            overlap = nnz(competitor & truth);
            normalizedHamming = ...
                (cardinality - overlap) / cardinality;
            targetScore = sum(predictedScores(truth));
            competitorScore = sum( ...
                predictedScores(competitor));
            marginViolation = competitorScore - targetScore + ...
                lossScale * normalizedHamming;
            if marginViolation > 1e-12
                targetFeatures = mean(Z(truth, :), 1);
                competitorFeatures = mean( ...
                    Z(competitor, :), 1);
                coefficient = coefficient + eta * reshape( ...
                    targetFeatures - competitorFeatures, [], 1);
                updateCount = updateCount + 1;
                epochViolations = epochViolations + 1;
            end
            epochLoss = epochLoss + normalizedHamming;
        end
        coefficientNorm = norm(coefficient);
        if coefficientNorm > maximumCoefficientNorm
            coefficient = coefficient * ...
                (maximumCoefficientNorm / coefficientNorm);
        end
        averagedCoefficient = averagedCoefficient + coefficient;
        averageCount = averageCount + 1;
    end
    epochMeanHammingLoss(epochIdx) = ...
        epochLoss / numel(blocks);
    epochViolationRate(epochIdx) = ...
        epochViolations / numel(blocks);
end
if averageCount > 0
    coefficient = averagedCoefficient / averageCount;
end
if any(~isfinite(coefficient))
    error('Structured rolling-safe fitting produced non-finite values.');
end

model = struct();
model.kind = ...
    'linear-structured-rolling-safe-rollout-ranker-v1';
model.featureMask = featureMask;
model.featureMean = featureMean;
model.featureScale = featureScale;
model.coefficient = coefficient;
model.intercept = 0;
model.sourceWeight = sourceWeight;
model.payloadToleranceFraction = payloadToleranceFraction;
model.lambda = lambda;
model.epochCount = epochCount;
model.learningRate = learningRate;
model.lossScale = lossScale;
model.maximumCoefficientNorm = maximumCoefficientNorm;
model.stepCount = stepCount;
model.updateCount = updateCount;
model.epochMeanHammingLoss = epochMeanHammingLoss;
model.epochViolationRate = epochViolationRate;
model.blockWeightedTraining = true;
model.cardinalityUsedDuringTraining = 'oracle';
end

function mask = projectExactCardinality( ...
        block, scores, cardinality, sourceWeight, ...
        payloadToleranceFraction)
options = struct( ...
    'minimumCurrentCrossEdges', cardinality, ...
    'maximumCurrentCrossEdges', cardinality);
scores = normalizeFixedCardinalityScores(scores);
[adjacency, details] = ...
    replayRollingSafeRolloutProjection( ...
        block, scores, sourceWeight, ...
        payloadToleranceFraction, options);
if details.repairTriggered || details.topologyInfeasible
    error(['Exact-cardinality structured projection cannot use ', ...
        'repair or infeasibility fallback.']);
end
nodeCount = numel(block.groupIds);
indices = sub2ind( ...
    [nodeCount, nodeCount], block.receiverIndices, ...
    block.senderIndices);
mask = reshape(logical(adjacency(indices)), [], 1);
if nnz(mask) ~= cardinality
    error('Exact-cardinality structured projection changed k.');
end
end

function normalized = normalizeFixedCardinalityScores(scores)
scores = reshape(scores, [], 1);
location = mean(scores);
scale = max(abs(scores - location));
if ~isfinite(scale) || scale <= eps
    scale = 1;
end
normalized = (scores - location) / scale;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
