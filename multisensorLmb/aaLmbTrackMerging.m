function objects = aaLmbTrackMerging(measurementUpdatedDistributions, model)
%  AALMBTRACKMERGING -- Merge the objects' measurement-updated distributions together using the AA-fusion rule.
%   objects = aaLmbTrackMerging(measurementUpdatedDistributions, model)
%
%   Merge the objects' measurement-updated distributions together using the AA-fusion rule.
%   文件导读：
%       AA 融合的权重消费端。existence weights 用于线性平均 Bernoulli r；
%       spatial mixture 则按 spatial weight 和 local existence probability
%       共同加权后拼接 Gaussian component。
%       当前主线偏 GA/KLA，但 AA 路径保留了同一套动态权重接口。
%
%   See also generateMultisensorModel, loopyBeliefPropagation, lmbGibbsSampling, lmbMurtysAlgorithm
%
%   Inputs
%       measurementUpdatedDistributions - (1, numberOfSensors) cell array. 
%           Each is an object struct produced by computePosteriorLmbSpatialDistributions.   
%       model - struct. A struct with the fields declared in generateMultisensorModel.
%
%   Output
%       objects - struct. A struct containing the posterior LMB's Bernoulli
%           components.

objects = measurementUpdatedDistributions{1};
strictAaWeights = resolveStrictAaWeights(model);
useKlaSpatialFusion = resolveKlaSpatialFusion(model);
for i = 1:numel(objects)
    %% 1. 读取当前目标的 AA 权重
    % Strict AA uses one alpha vector for both Bernoulli existence and spatial
    % density, matching the Bernoulli-AA formula.  The legacy branch-decoupled
    % path is a heuristic extension: existence and spatial mixture may consume
    % different AA weights.
    if strictAaWeights
        spatialWeights = resolveObjectWeightVector( ...
            model, 'aaTargetWiseWeights', 'aaSensorWeights', model.aaSensorWeights, i);
        existenceWeights = spatialWeights;
    else
        spatialWeights = resolveObjectWeightVector( ...
            model, 'aaTargetWiseWeights', 'aaSpatialWeights', model.aaSensorWeights, i);
        existenceWeights = resolveObjectWeightVector( ...
            model, 'aaTargetWiseWeights', 'aaExistenceWeights', model.aaSensorWeights, i);
    end
    %% 2. Bernoulli-AA：existence 线性平均，spatial mixture 用 r_s 加权
    % For a Bernoulli density, arithmetic averaging gives
    %   r = sum_s alpha_s r_s
    %   p(x) proportional to sum_s alpha_s r_s p_s(x).
    % The previous implementation used only alpha_s for p_s(x), which let
    % low-existence but sharp local posteriors dominate the output Gaussian.
    fusedExistence = 0;
    for s = 1:model.numberOfSensors
        localObject = measurementUpdatedDistributions{s}(i);
        localExistence = clampProbability(localObject.r);

        fusedExistence = fusedExistence + existenceWeights(s) * localExistence;
    end
    objects(i).r = clampProbability(fusedExistence);

    if useKlaSpatialFusion
        [muGa, SigmaGa] = fuseSpatialWithKla(measurementUpdatedDistributions, model, spatialWeights, i);
        objects(i).numberOfGmComponents = 1;
        objects(i).w = 1;
        objects(i).mu = {muGa};
        objects(i).Sigma = {SigmaGa};
        continue;
    end

    fusedWeights = [];
    fallbackWeights = [];
    fusedMeans = {};
    fusedCovariances = {};
    for s = 1:model.numberOfSensors
        localObject = measurementUpdatedDistributions{s}(i);
        localExistence = clampProbability(localObject.r);
        localWeights = reshape(localObject.w, 1, []);

        fusedWeights = horzcat(fusedWeights, spatialWeights(s) * localExistence * localWeights);
        fallbackWeights = horzcat(fallbackWeights, spatialWeights(s) * localWeights);
        fusedMeans = horzcat(fusedMeans, localObject.mu);
        fusedCovariances = horzcat(fusedCovariances, localObject.Sigma);
    end
    if sum(fusedWeights) <= eps
        fusedWeights = fallbackWeights;
    end
    %% 4. 按 mixture 权重排序并截断，避免 component 数量无限增长
    [~, sortedIndices] = sort(fusedWeights, 'descend');
    numberOfGmComponents = numel(fusedWeights);
    sortedIndices = sortedIndices(1:min(model.maximumNumberOfGmComponents, numberOfGmComponents));
    objects(i).numberOfGmComponents = numel(sortedIndices);
    retainedWeights = fusedWeights(sortedIndices);
    weightSum = sum(retainedWeights);
    if weightSum <= eps
        retainedWeights = ones(1, numel(sortedIndices)) / max(numel(sortedIndices), 1);
    else
        retainedWeights = retainedWeights ./ weightSum;
    end
    objects(i).w = retainedWeights;
    objects(i).mu = fusedMeans(sortedIndices);
    objects(i).Sigma = fusedCovariances(sortedIndices);
end


end

function useKlaSpatialFusion = resolveKlaSpatialFusion(model)
mode = '';
if isfield(model, 'aaSpatialFusionMode')
    mode = model.aaSpatialFusionMode;
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'aaSpatialFusionMode')
        mode = cfg.aaSpatialFusionMode;
    end
end
useKlaSpatialFusion = isKlaSpatialFusionMode(mode);
end

function tf = isKlaSpatialFusionMode(value)
tf = false;
if ischar(value) || isstring(value)
    tf = any(strcmpi(char(value), {'kla', 'ga', 'geometric', ...
        'spatial-kla', 'spatial_kla', 'hybrid-kla', 'hybrid_kla'}));
end
end

function strictAaWeights = resolveStrictAaWeights(model)
strictAaWeights = false;
if isfield(model, 'aaFusionWeightMode')
    strictAaWeights = isStrictAaMode(model.aaFusionWeightMode);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'aaFusionWeightMode')
        strictAaWeights = isStrictAaMode(cfg.aaFusionWeightMode);
    end
    if isfield(cfg, 'aaStrictWeights')
        strictAaWeights = logical(cfg.aaStrictWeights);
    end
end
end

function tf = isStrictAaMode(value)
tf = false;
if ischar(value) || isstring(value)
    tf = any(strcmpi(char(value), {'strict', 'strict-aa', 'strict_aa'}));
end
end

function weights = resolveObjectWeightVector(model, targetWiseFieldName, fieldName, fallback, objectIdx)
weights = [];
if isfield(model, targetWiseFieldName)
    targetWiseWeights = model.(targetWiseFieldName);
    if size(targetWiseWeights, 1) >= objectIdx && size(targetWiseWeights, 2) == numel(fallback)
        weights = targetWiseWeights(objectIdx, :);
    end
end
if isempty(weights)
    weights = resolveWeightVector(model, fieldName, fallback);
end
weights = normalizeWeightVector(weights, fallback);
end

function [muGa, SigmaGa] = fuseSpatialWithKla(measurementUpdatedDistributions, model, spatialWeights, objectIdx)
K = zeros(model.xDimension, model.xDimension);
h = zeros(model.xDimension, 1);
for s = 1:model.numberOfSensors
    [nu, T] = mprojectObject(model.xDimension, measurementUpdatedDistributions{s}(objectIdx));
    T = regularizeCovariance(T);
    precision = inv(T);
    weightedPrecision = spatialWeights(s) * precision;
    K = K + weightedPrecision;
    h = h + weightedPrecision * nu;
end
K = regularizeCovariance(K);
SigmaGa = regularizeCovariance(inv(K));
muGa = SigmaGa * h;
end

function [nu, T] = mprojectObject(n, object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(1, max(numel(weights), 1)) / max(numel(weights), 1);
else
    weights = weights / sum(weights);
end

nu = zeros(n, 1);
for j = 1:object.numberOfGmComponents
    nu = nu + weights(j) * object.mu{j};
end

T = zeros(n, n);
for j = 1:object.numberOfGmComponents
    delta = object.mu{j} - nu;
    T = T + weights(j) * (object.Sigma{j} + delta * delta');
end
T = regularizeCovariance(T);
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
if isempty(covariance)
    return;
end
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end

function weights = resolveWeightVector(model, fieldName, fallback)
if isfield(model, fieldName)
    weights = model.(fieldName);
else
    weights = fallback;
end
end

function weights = normalizeWeightVector(weights, fallback)
weights = reshape(weights, 1, []);
fallback = reshape(fallback, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= numel(fallback) || sum(weights) <= 0
    weights = fallback;
    weights(~isfinite(weights)) = 0;
    weights = max(weights, 0);
end
if sum(weights) <= 0
    weights = ones(1, numel(fallback)) / numel(fallback);
else
    weights = weights / sum(weights);
end
end

function value = clampProbability(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
