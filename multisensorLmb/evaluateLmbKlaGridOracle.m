function oracle = evaluateLmbKlaGridOracle( ...
        localObjects, fusionWeights, options)
% EVALUATELMBKLAGRIDORACLE Deterministic 1-D/2-D density-power reference.
%
% The oracle evaluates prod_s p_s(x)^omega_s directly on a tensor grid.
% It is intended for controlled single-label calibration, not for running a
% full tracker. Candidate LMB objects can be supplied to quantify the error
% of powered-GM or projected-Gaussian approximations on the same grid.

if nargin < 3 || isempty(options)
    options = struct();
end
[localObjects, fusionWeights, stateDimension] = ...
    validateInputs(localObjects, fusionWeights);
if stateDimension > 2
    error('LmbKlaGridOracle:DimensionNotSupported', ...
        'The deterministic grid oracle supports only one or two dimensions.');
end

gridPointCount = round(getField(options, 'gridPointCount', ...
    161 + 840 * (stateDimension == 1)));
if gridPointCount < 21
    error('LmbKlaGridOracle:GridTooSmall', ...
        'At least 21 grid points per dimension are required.');
end
sigmaExtent = getField(options, 'sigmaExtent', 7);
if ~isscalar(sigmaExtent) || ~isfinite(sigmaExtent) || sigmaExtent <= 0
    error('LmbKlaGridOracle:InvalidSigmaExtent', ...
        'sigmaExtent must be positive and finite.');
end

axisValues = buildAxes( ...
    localObjects, stateDimension, gridPointCount, sigmaExtent);
[points, quadratureWeights, boundaryMask] = ...
    buildTensorGrid(axisValues);
sourceLogDensity = zeros(numel(localObjects), size(points, 2));
for sourceIdx = 1:numel(localObjects)
    sourceLogDensity(sourceIdx, :) = ...
        logGaussianMixtureDensity(localObjects{sourceIdx}, points);
end
logUnnormalizedDensity = fusionWeights * sourceLogDensity;
shift = max(logUnnormalizedDensity);
scaledMass = exp(logUnnormalizedDensity - shift) .* quadratureWeights;
scaledIntegral = sum(scaledMass);
if ~isfinite(scaledIntegral) || scaledIntegral <= 0
    error('LmbKlaGridOracle:ZeroIntegral', ...
        'The density-power integral is not positive and finite.');
end
logEta = shift + log(scaledIntegral);
pointMass = scaledMass / scaledIntegral;
[meanState, covariance] = gridMoments(points, pointMass);
fusedExistence = fuseExistence(localObjects, fusionWeights, logEta);

oracle = struct();
oracle.contractVersion = 'lmb-kla-grid-oracle-v1';
oracle.stateDimension = stateDimension;
oracle.sourceCount = numel(localObjects);
oracle.fusionWeights = fusionWeights;
oracle.gridPointCountPerDimension = gridPointCount;
oracle.sigmaExtent = sigmaExtent;
oracle.axisValues = axisValues;
oracle.logEta = logEta;
oracle.existenceProbability = fusedExistence;
oracle.mean = meanState;
oracle.covariance = covariance;
oracle.boundaryMassFraction = sum(pointMass(boundaryMask));
oracle.comparisons = compareCandidates( ...
    getField(options, 'candidateObjects', cell(1, 0)), ...
    getField(options, 'candidateNames', cell(1, 0)), ...
    localObjects, fusionWeights, points, quadratureWeights, ...
    logUnnormalizedDensity - logEta, meanState, covariance, ...
    fusedExistence, logEta);
if getField(options, 'returnGrid', false)
    oracle.points = points;
    oracle.quadratureWeights = quadratureWeights;
    oracle.normalizedDensity = exp(logUnnormalizedDensity - logEta);
end
end

function [objects, weights, stateDimension] = ...
    validateInputs(objects, weights)
if ~iscell(objects) || numel(objects) < 2
    error('LmbKlaGridOracle:InvalidObjects', ...
        'At least two single-label LMB objects are required.');
end
weights = reshape(weights, 1, []);
if numel(weights) ~= numel(objects) || any(~isfinite(weights)) || ...
        any(weights < 0) || sum(weights) <= 0
    error('LmbKlaGridOracle:InvalidWeights', ...
        'Fusion weights must be finite, nonnegative and match the sources.');
end
weights = weights / sum(weights);
stateDimension = numel(objects{1}.mu{1});
for sourceIdx = 1:numel(objects)
    object = objects{sourceIdx};
    required = {'r', 'numberOfGmComponents', 'w', 'mu', 'Sigma'};
    if ~isstruct(object) || ~isscalar(object) || ...
            ~all(isfield(object, required)) || ...
            object.numberOfGmComponents < 1 || ...
            numel(object.mu) ~= object.numberOfGmComponents || ...
            numel(object.Sigma) ~= object.numberOfGmComponents || ...
            numel(object.w) ~= object.numberOfGmComponents
        error('LmbKlaGridOracle:InvalidObject', ...
            'Each source must contain one valid Gaussian-mixture Bernoulli.');
    end
    for componentIdx = 1:object.numberOfGmComponents
        if numel(object.mu{componentIdx}) ~= stateDimension || ...
                ~isequal(size(object.Sigma{componentIdx}), ...
                    [stateDimension, stateDimension])
            error('LmbKlaGridOracle:DimensionMismatch', ...
                'All Gaussian components must share one state dimension.');
        end
    end
end
end

function axes = buildAxes(objects, dimension, pointCount, sigmaExtent)
lower = inf(dimension, 1);
upper = -inf(dimension, 1);
for sourceIdx = 1:numel(objects)
    object = objects{sourceIdx};
    for componentIdx = 1:object.numberOfGmComponents
        mu = reshape(object.mu{componentIdx}, [], 1);
        covariance = regularizeCovariance(object.Sigma{componentIdx});
        marginalSigma = sqrt(max(diag(covariance), eps));
        lower = min(lower, mu - sigmaExtent * marginalSigma);
        upper = max(upper, mu + sigmaExtent * marginalSigma);
    end
end
axes = cell(1, dimension);
for dimIdx = 1:dimension
    axes{dimIdx} = linspace(lower(dimIdx), upper(dimIdx), pointCount);
end
end

function [points, weights, boundaryMask] = buildTensorGrid(axes)
dimension = numel(axes);
coordinateGrids = cell(1, dimension);
axisQuadratureWeights = cell(1, dimension);
for dimIdx = 1:dimension
    step = axes{dimIdx}(2) - axes{dimIdx}(1);
    axisQuadratureWeights{dimIdx} = ...
        step * [0.5, ones(1, numel(axes{dimIdx}) - 2), 0.5];
end
[coordinateGrids{:}] = ndgrid(axes{:});
quadratureGrids = cell(1, dimension);
[quadratureGrids{:}] = ndgrid(axisQuadratureWeights{:});
pointCount = numel(coordinateGrids{1});
points = zeros(dimension, pointCount);
weightGrid = ones(size(coordinateGrids{1}));
boundaryGrid = false(size(coordinateGrids{1}));
for dimIdx = 1:dimension
    points(dimIdx, :) = reshape(coordinateGrids{dimIdx}, 1, []);
    weightGrid = weightGrid .* quadratureGrids{dimIdx};
    values = coordinateGrids{dimIdx};
    boundaryGrid = boundaryGrid | values == axes{dimIdx}(1) | ...
        values == axes{dimIdx}(end);
end
weights = reshape(weightGrid, 1, []);
boundaryMask = reshape(boundaryGrid, 1, []);
end

function logDensity = logGaussianMixtureDensity(object, points)
componentWeights = reshape(object.w, 1, []);
componentWeights(~isfinite(componentWeights)) = 0;
componentWeights = max(componentWeights, 0);
componentWeights = componentWeights / max(sum(componentWeights), eps);
dimension = size(points, 1);
componentLogDensity = -inf(object.numberOfGmComponents, size(points, 2));
for componentIdx = 1:object.numberOfGmComponents
    covariance = regularizeCovariance(object.Sigma{componentIdx});
    [factor, flag] = chol(covariance, 'lower');
    if flag ~= 0
        error('LmbKlaGridOracle:NonPositiveCovariance', ...
            'A Gaussian covariance is not positive definite.');
    end
    delta = points - reshape(object.mu{componentIdx}, [], 1);
    whitened = factor \ delta;
    logGaussian = -0.5 * (dimension * log(2 * pi) + ...
        2 * sum(log(diag(factor))) + sum(whitened.^2, 1));
    componentLogDensity(componentIdx, :) = ...
        log(max(componentWeights(componentIdx), realmin)) + logGaussian;
end
maximum = max(componentLogDensity, [], 1);
logDensity = maximum + log(sum(exp( ...
    componentLogDensity - maximum), 1));
end

function [meanState, covariance] = gridMoments(points, pointMass)
meanState = points * pointMass';
centered = points - meanState;
weighted = centered .* sqrt(pointMass);
covariance = weighted * weighted';
covariance = (covariance + covariance') / 2;
end

function probability = fuseExistence(objects, weights, logEta)
logPresent = logEta;
logAbsent = 0;
for sourceIdx = 1:numel(objects)
    r = min(max(objects{sourceIdx}.r, 1e-12), 1 - 1e-12);
    logPresent = logPresent + weights(sourceIdx) * log(r);
    logAbsent = logAbsent + weights(sourceIdx) * log(1 - r);
end
probability = logistic(logPresent - logAbsent);
end

function comparisons = compareCandidates( ...
    candidates, names, sourceObjects, fusionWeights, points, ...
    quadratureWeights, oracleLogDensity, oracleMean, oracleCovariance, ...
    oracleExistence, oracleLogEta)
if isempty(candidates)
    comparisons = struct([]);
    return;
end
if ~iscell(candidates)
    candidates = num2cell(candidates);
end
if isempty(names)
    names = arrayfun(@(idx) sprintf('candidate-%d', idx), ...
        1:numel(candidates), 'UniformOutput', false);
end
if numel(names) ~= numel(candidates)
    error('LmbKlaGridOracle:CandidateNameMismatch', ...
        'candidateNames must match candidateObjects.');
end
template = struct('name', '', 'gridMass', NaN, ...
    'totalVariation', NaN, 'klOracleToCandidate', NaN, ...
    'meanError', NaN, 'covarianceError', NaN, ...
    'existenceAbsoluteError', NaN, 'logEtaError', NaN);
comparisons = repmat(template, 1, numel(candidates));
oracleDensity = exp(oracleLogDensity);
sourceLogOdds = 0;
for sourceIdx = 1:numel(sourceObjects)
    r = min(max(sourceObjects{sourceIdx}.r, 1e-12), 1 - 1e-12);
    sourceLogOdds = sourceLogOdds + ...
        fusionWeights(sourceIdx) * log(r / (1 - r));
end
for candidateIdx = 1:numel(candidates)
    candidate = candidates{candidateIdx};
    logDensity = logGaussianMixtureDensity(candidate, points);
    rawDensity = exp(logDensity);
    gridMass = sum(rawDensity .* quadratureWeights);
    density = rawDensity / max(gridMass, realmin);
    pointMass = density .* quadratureWeights;
    [candidateMean, candidateCovariance] = gridMoments(points, pointMass);
    tv = 0.5 * sum(abs(oracleDensity - density) .* quadratureWeights);
    kl = sum(oracleDensity .* (oracleLogDensity - ...
        log(max(density, realmin))) .* quadratureWeights);
    candidateR = min(max(candidate.r, 1e-12), 1 - 1e-12);
    candidateLogEta = log(candidateR / (1 - candidateR)) - ...
        sourceLogOdds;
    comparisons(candidateIdx) = struct( ...
        'name', names{candidateIdx}, ...
        'gridMass', gridMass, ...
        'totalVariation', tv, ...
        'klOracleToCandidate', max(kl, 0), ...
        'meanError', norm(candidateMean - oracleMean), ...
        'covarianceError', norm( ...
            candidateCovariance - oracleCovariance, 'fro'), ...
        'existenceAbsoluteError', abs(candidate.r - oracleExistence), ...
        'logEtaError', candidateLogEta - oracleLogEta);
end
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:8
    [~, flag] = chol(covariance + jitter * eye(size(covariance)));
    if flag == 0
        covariance = covariance + jitter * eye(size(covariance));
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('LmbKlaGridOracle:NonPositiveCovariance', ...
    'A covariance cannot be regularized.');
end

function value = logistic(logOdds)
if logOdds >= 0
    value = 1 / (1 + exp(-logOdds));
else
    expValue = exp(logOdds);
    value = expValue / (1 + expValue);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
