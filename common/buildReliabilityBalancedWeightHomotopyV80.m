function homotopy = buildReliabilityBalancedWeightHomotopyV80( ...
        context, adjacency, referenceWeights, alphas, options)
% BUILDRELIABILITYBALANCEDWEIGHTHOMOTOPYV80 Same-support balancing path.

if nargin < 5 || isempty(options)
    options = struct();
end
tolerance = getField(options, 'sinkhornTolerance', 1e-12);
maximumIterations = getField( ...
    options, 'sinkhornMaximumIterations', 20000);
effectiveTolerance = getField( ...
    options, 'effectiveWeightTolerance', 2e-11);
nodeCount = numel(context.localPosteriorBySensor);
adjacency = logical(adjacency);
alphas = reshape(alphas, 1, []);
if ~isequal(size(adjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(referenceWeights), [nodeCount, nodeCount]) || ...
        any(~isfinite(alphas)) || any(alphas < 0) || ...
        any(alphas > 1) || numel(unique(alphas)) ~= numel(alphas) || ...
        any(diff(alphas) <= 0) || alphas(1) ~= 0 || alphas(end) ~= 1
    error('BalancedHomotopyV80:InvalidInput', ...
        'The reference route or alpha grid is invalid.');
end
support = adjacency | logical(eye(nodeCount));
if any(referenceWeights(:) < 0) || ...
        any(abs(sum(referenceWeights, 2) - 1) > 1e-12) || ...
        any(referenceWeights(:) > 0 & ~support(:)) || ...
        any(referenceWeights(support) <= 0)
    error('BalancedHomotopyV80:InvalidReference', ...
        'The reference weights do not have exact positive support.');
end

reliability = reliabilityMatrix( ...
    context.commConfig, context.currentTime, nodeCount);
if any(reliability(support) <= 0)
    error('BalancedHomotopyV80:ZeroReliability', ...
        'Positive reference support requires positive reliability.');
end
referenceEffective = expectedWeights( ...
    adjacency, referenceWeights, reliability);
[balancedEffective, iterationCount] = sinkhornProject( ...
    referenceEffective, tolerance, maximumIterations);
centering = eye(nodeCount) - ones(nodeCount) / nodeCount;
points = repmat(emptyPoint(), 1, numel(alphas));
for alphaIdx = 1:numel(alphas)
    alpha = alphas(alphaIdx);
    targetEffective = (1 - alpha) * referenceEffective + ...
        alpha * balancedEffective;
    nominal = invertExpectedReliability( ...
        targetEffective, reliability, support);
    realizedEffective = expectedWeights( ...
        adjacency, nominal, reliability);
    effectiveError = max(abs( ...
        realizedEffective(:) - targetEffective(:)));
    if effectiveError > effectiveTolerance || ...
            any(nominal(support) <= 0) || ...
            any(nominal(:) > 0 & ~support(:)) || ...
            any(abs(sum(nominal, 2) - 1) > 1e-12)
        error('BalancedHomotopyV80:InvalidPoint', ...
            'A homotopy point cannot be realized on the reference support.');
    end
    point = emptyPoint();
    point.alpha = alpha;
    point.fusionWeights = nominal;
    point.effectiveWeights = realizedEffective;
    point.centeredSpectralNorm = norm( ...
        centering * realizedEffective * centering, 2);
    point.effectiveColumnDeviation = ...
        max(abs(sum(realizedEffective, 1) - 1));
    point.centeredOperatorDistanceFromReference = norm( ...
        centering * (realizedEffective - referenceEffective) * ...
            centering, 2);
    point.nominalFrobeniusDistanceFromReference = ...
        norm(nominal - referenceWeights, 'fro');
    point.minimumPositiveWeight = min(nominal(support));
    point.maximumWeight = max(nominal(support));
    point.effectiveRealizationError = effectiveError;
    points(alphaIdx) = point;
end

homotopy = struct();
homotopy.contractVersion = ...
    'reliability-balanced-weight-homotopy-v80-v1';
homotopy.alphas = alphas;
homotopy.adjacency = adjacency;
homotopy.referenceFusionWeights = referenceWeights;
homotopy.referenceEffectiveWeights = referenceEffective;
homotopy.balancedEffectiveWeights = balancedEffective;
homotopy.points = points;
homotopy.sinkhornIterationCount = iterationCount;
homotopy.sinkhornTolerance = tolerance;
homotopy.referenceCenteredSpectralNorm = norm( ...
    centering * referenceEffective * centering, 2);
homotopy.balancedCenteredSpectralNorm = norm( ...
    centering * balancedEffective * centering, 2);
homotopy.balancedColumnDeviation = ...
    max(abs(sum(balancedEffective, 1) - 1));
homotopy.exactSupportParity = all(arrayfun( ...
    @(point) isequal(point.fusionWeights > 0, support), points));
homotopy.exactMessageCountParity = true;
homotopy.currentPhysicalLinksUsed = true;
homotopy.currentLinkReliabilityUsed = true;
homotopy.posteriorUsed = false;
homotopy.measurementUsed = false;
homotopy.truthUsed = false;
homotopy.futureLinkUsed = false;
homotopy.futureOutcomeUsed = false;
end

function [balanced, iterationCount] = ...
        sinkhornProject(matrix, tolerance, maximumIterations)
balanced = matrix;
iterationCount = 0;
for iterationCount = 1:maximumIterations
    rowSums = sum(balanced, 2);
    columnSums = sum(balanced, 1);
    if any(rowSums <= 0) || any(columnSums <= 0)
        error('BalancedHomotopyV80:InfeasibleSupport', ...
            'The effective support cannot be balanced.');
    end
    balanced = bsxfun(@rdivide, balanced, rowSums);
    balanced = bsxfun(@rdivide, balanced, sum(balanced, 1));
    if max(abs(sum(balanced, 1) - 1)) <= tolerance && ...
            max(abs(sum(balanced, 2) - 1)) <= tolerance
        return;
    end
end
error('BalancedHomotopyV80:NoConvergence', ...
    'Sinkhorn balancing did not converge within the frozen budget.');
end

function nominal = invertExpectedReliability( ...
        targetEffective, reliability, support)
raw = zeros(size(targetEffective));
raw(support) = targetEffective(support) ./ reliability(support);
nominal = bsxfun(@rdivide, raw, sum(raw, 2));
end

function effective = expectedWeights(adjacency, weights, reliability)
nodeCount = size(weights, 1);
effective = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    sources = [receiverIdx, reshape(find( ...
        adjacency(receiverIdx, :)), 1, [])];
    localWeights = weights(receiverIdx, sources) .* ...
        reliability(receiverIdx, sources);
    localWeights = localWeights / sum(localWeights);
    effective(receiverIdx, sources) = localWeights;
end
end

function reliability = reliabilityMatrix(config, currentTime, nodeCount)
drop = zeros(nodeCount);
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(:, :, timeIdx)';
    else
        drop = config.pDropByEdge';
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= nodeCount
    drop = repmat(reshape(config.pDropBySensor(1:nodeCount), ...
        1, []), nodeCount, 1);
end
reliability = 1 - min(max(drop, 0), 1);
reliability(1:nodeCount+1:end) = 1;
end

function point = emptyPoint()
point = struct( ...
    'alpha', NaN, ...
    'fusionWeights', zeros(0), ...
    'effectiveWeights', zeros(0), ...
    'centeredSpectralNorm', NaN, ...
    'effectiveColumnDeviation', NaN, ...
    'centeredOperatorDistanceFromReference', NaN, ...
    'nominalFrobeniusDistanceFromReference', NaN, ...
    'minimumPositiveWeight', NaN, ...
    'maximumWeight', NaN, ...
    'effectiveRealizationError', NaN);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
