function objects = computePosteriorLmbSpatialDistributions(objects, r, W, posteriorParameters, model)
% COMPUTEPOSTERIORLMBSPATIALDISTRIUBUTIONS -- Complete the LMB filter's measurement update
%    objects = computePosteriorLmbSpatialDistributions(objects, r, W, posteriorParameters, model)
%
%   This function computes each object's posterior spatial distrubtion. 
%   文件导读：
%       posterior Gaussian mixture 组装步骤。关联求解器只给出存在概率 r
%       和边缘关联概率 W；本函数把这些概率乘到预先计算好的漏检/检测
%       Gaussian component 上，并完成 mixture pruning 和 component 数量上限控制。
%
%   See also generateModel, runLmbFilter, lmbPredictionStep, 
%            loopyBeliefPropagation, generateLmbAssociationMatrices
%
%   Inputs
%       objects - struct. A struct containing the prior LMB's Bernoulli
%           components. This struct is produced by lmbPredictionStep.
%       r - array. Each object's posterior existence probability.
%       W - array. An array of marginal association probabilities, where
%           each row is an object's marginal association probabilities.
%       posteriorParameters - struct. A struct whose fields are an object's
%           posterior spatial distribution parameters.
%       model - struct. A struct with the fields declared in generateModel.
%
%   Output
%       objects - struct. A struct containing the posterior LMB's Bernoulli
%           components.

groupingMethod='pairwise';
if isfield(model,'gmCanonicalizationGroupingMethod')
    groupingMethod=model.gmCanonicalizationGroupingMethod;
end
for i = 1:numel(objects)
    %% 1. 写回该 Bernoulli 的 posterior existence probability
    objects(i).r = r(i);
    %% 2. 用边缘关联概率重加权所有漏检/检测 Gaussian components
    numberOfPosteriorComponents = numel(posteriorParameters(i).w);
    posteriorWeights = reshape(W(i, :)' .* posteriorParameters(i).w, 1, numberOfPosteriorComponents);
    posteriorWeights = posteriorWeights ./ sum(posteriorWeights);
    [associationEntropy, detectionAssociationEntropy, ...
        detectionAssociationMass, associationAmbiguity] = ...
        computeLabelAssociationDiagnostics(W(i, :));
    objects(i).associationEntropy = associationEntropy;
    objects(i).detectionAssociationEntropy = detectionAssociationEntropy;
    objects(i).detectionAssociationMass = detectionAssociationMass;
    objects(i).associationAmbiguity = associationAmbiguity;
    objects(i).associationConfidence = 1 - associationAmbiguity;
    %% 3. 先形成完整 GM，再合并完全相同的 component
    % 相同 Gaussian 的权重求和不会改变密度。必须在 pruning 和数量上限
    % 之前合并，否则重复副本会任意占用 component budget，并挤掉真正
    % 不同但权重略低的模态。
    objects(i).numberOfGmComponents = numberOfPosteriorComponents;
    objects(i).w = posteriorWeights;
    objects(i).mu = reshape( ...
        posteriorParameters(i).mu, 1, numberOfPosteriorComponents);
    objects(i).Sigma = reshape( ...
        posteriorParameters(i).Sigma, 1, numberOfPosteriorComponents);
    objects(i) = canonicalizeLmbGaussianMixtureRepresentation( ...
        objects(i), struct( ...
            'weightThreshold', model.gmWeightThreshold, ...
            'groupingMethod', groupingMethod, ...
            'maximumComponentCount', ...
                model.maximumNumberOfGmComponents));
end

end

function [associationEntropy, detectionAssociationEntropy, ...
    detectionAssociationMass, associationAmbiguity] = ...
    computeLabelAssociationDiagnostics(associationWeights)
weights = reshape(associationWeights, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    associationEntropy = 0;
    detectionAssociationEntropy = 0;
    detectionAssociationMass = 0;
    associationAmbiguity = 0;
    return;
end
weights = weights / sum(weights);
associationEntropy = normalizedEntropy(weights);
if numel(weights) <= 1
    detectionAssociationEntropy = 0;
    detectionAssociationMass = 0;
    associationAmbiguity = 0;
    return;
end
detectionWeights = weights(2:end);
detectionAssociationMass = min(max(sum(detectionWeights), 0), 1);
if detectionAssociationMass <= eps
    detectionAssociationEntropy = 0;
else
    detectionAssociationEntropy = normalizedEntropy( ...
        detectionWeights / detectionAssociationMass);
end
associationAmbiguity = min(max( ...
    detectionAssociationMass * detectionAssociationEntropy, 0), 1);
end

function value = normalizedEntropy(weights)
weights = reshape(weights, 1, []);
weights = weights(weights > 0);
if numel(weights) <= 1
    value = 0;
    return;
end
entropyValue = -sum(weights .* log(weights));
value = min(max(entropyValue / log(numel(weights)), 0), 1);
end
