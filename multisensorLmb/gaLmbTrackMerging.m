function objects = gaLmbTrackMerging(measurementUpdatedDistributions, model)
% GALMBTRACKMERGING -- Merge the objects' measurement-updated distributions together using the GA-fusion rule.
%   objects = gaLmbTrackMerging(measurementUpdatedDistributions, model)
%
%   Merge the objects' measurement-updated distributions together using the GA-fusion rule.
%   This makes use of a very crude merging algorithm that is actually
%   reasonably accurate. It might be possible to extend this to Gaussian
%   mixtures, or, using the well-space mixture assumptions, apply
%   expectation propagation.
%   文件导读：
%       GA/KLA 融合的权重消费端。动态权重不是在这里计算，而是在这里
%       真正进入融合公式：spatial weights 用于 Gaussian canonical 参数
%       加权，existence weights 用于 Bernoulli existence probability 加权。
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
for i = 1:numel(objects)
    %% 1. 读取当前目标的空间分支权重和存在分支权重
    % 如果 direct baseline 返回了 target-wise weights，则优先使用目标级权重；
    % 否则使用全局 gaSpatialWeights / gaExistenceWeights。
    spatialWeights = resolveObjectWeightVector( ...
        model, 'gaTargetWiseWeights', 'gaSpatialWeights', model.gaSensorWeights, i);
    existenceWeights = resolveObjectWeightVector( ...
        model, 'gaTargetWiseWeights', 'gaExistenceWeights', model.gaSensorWeights, i);
    %% 2. 空间融合：先 m-projection，再在 canonical form 中做几何平均
    K = zeros(model.xDimension, model.xDimension);
    h = zeros(model.xDimension, 1);
    g = 0;
    for s = 1:model.numberOfSensors
        [nu, T] = mprojection(model.xDimension, measurementUpdatedDistributions{s}(i));
        % 将每个传感器的 posterior 近似成单 Gaussian，再乘上 spatial weight。
        KMatched = spatialWeights(s) * inv(T);
        hMatched = KMatched * nu;
        gMatched = -0.5 * nu' * KMatched * nu - 0.5 * spatialWeights(s) * log(det(2*pi*T));
        % Throw it on the pile
        K = K + KMatched;
        h = h + hMatched;
        g = g + gMatched;
    end
    %% 3. 将 canonical form 转回均值/协方差，并计算归一化项 eta
    SigmaGa = inv(K);
    muGa = SigmaGa * h;
    eta = exp(g + 0.5 * muGa' * K * muGa + 0.5 * log(det(2*pi*SigmaGa)));
    %% 4. 存在概率融合：用 existence weights 合并各传感器 Bernoulli r
    numerator = eta;
    partialDenominator = 1;
    for s = 1:model.numberOfSensors
        rS = measurementUpdatedDistributions{s}(i).r;
        numerator = numerator * (rS^(existenceWeights(s)));
        partialDenominator = partialDenominator *  ((1-rS)^(existenceWeights(s)));
    end
    %% 5. 写回融合后的 Bernoulli component
    objects(i).r = numerator / (numerator + partialDenominator);
    objects(i).numberOfGmComponents = 1;
    objects(i).w = 1;
    objects(i).mu = {muGa};
    objects(i).Sigma = {SigmaGa};
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

function weights = resolveWeightVector(model, fieldName, fallback)
if isfield(model, fieldName)
    weights = model.(fieldName);
else
    weights = fallback;
end
end

function weights = normalizeWeightVector(weights, fallback)
weights = reshape(weights, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= numel(fallback) || sum(weights) <= 0
    weights = reshape(fallback, 1, []);
    weights(~isfinite(weights)) = 0;
    weights = max(weights, 0);
end
if sum(weights) <= 0
    weights = ones(1, numel(fallback)) / numel(fallback);
else
    weights = weights / sum(weights);
end
end

%% M-projection：把 Gaussian mixture moment-match 成单 Gaussian，供 GA 融合使用
function [nu, T] = mprojection(n, measurementUpdatedDistribution)
% 计算 m-projection 后的均值。
nu = zeros(n, 1);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    nu = nu + measurementUpdatedDistribution.w(j) * measurementUpdatedDistribution.mu{j};
end
% 计算 m-projection 后的协方差。
T = zeros(n, n);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    w = measurementUpdatedDistribution.w(j);
    mu = measurementUpdatedDistribution.mu{j} - nu;
    Sigma = measurementUpdatedDistribution.Sigma{j};
    T = T + w * (Sigma + mu * mu');
end
end
