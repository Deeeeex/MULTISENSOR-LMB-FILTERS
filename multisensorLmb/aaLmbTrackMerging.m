function objects = aaLmbTrackMerging(measurementUpdatedDistributions, model)
%  AALMBTRACKMERGING -- Merge the objects' measurement-updated distributions together using the AA-fusion rule.
%   objects = aaLmbTrackMerging(measurementUpdatedDistributions, model)
%
%   Merge the objects' measurement-updated distributions together using the AA-fusion rule.
%   文件导读：
%       AA 融合的权重消费端。spatial weights 用于加权并拼接 Gaussian
%       mixture component，existence weights 用于线性平均 Bernoulli r。
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
%% 1. 读取 AA 的空间/存在分支权重；没有 branch 权重时回退到 aaSensorWeights
spatialWeights = resolveWeightVector(model, 'aaSpatialWeights', model.aaSensorWeights);
existenceWeights = resolveWeightVector(model, 'aaExistenceWeights', model.aaSensorWeights);
for i = 1:numel(objects)
    %% 2. 用第一个传感器初始化融合 mixture 和存在概率
    objects(i).r = existenceWeights(1) * objects(i).r;
    objects(i).w = spatialWeights(1) * objects(i).w;
    %% 3. 追加其余传感器的 Gaussian mixture component，并线性累加存在概率
    for s = 2:model.numberOfSensors
        objects(i).r = objects(i).r + existenceWeights(s) * measurementUpdatedDistributions{s}(i).r;
        objects(i).w = horzcat(objects(i).w, spatialWeights(s) * measurementUpdatedDistributions{s}(i).w);
        objects(i).mu = horzcat(objects(i).mu, measurementUpdatedDistributions{s}(i).mu);
        objects(i).Sigma = horzcat(objects(i).Sigma, measurementUpdatedDistributions{s}(i).Sigma);
    end
    %% 4. 按 mixture 权重排序并截断，避免 component 数量无限增长
    [~, sortedIndices] = sort(objects(i).w, 'descend');
    numberOfGmComponents = numel(objects(i).w);
    sortedIndices = sortedIndices(1:min(model.maximumNumberOfGmComponents, numberOfGmComponents));
    objects(i).numberOfGmComponents = numel(sortedIndices);
    objects(i).w = objects(i).w(sortedIndices) ./ sum(objects(i).w(sortedIndices));
    objects(i).mu = objects(i).mu(sortedIndices);
    objects(i).Sigma = objects(i).Sigma(sortedIndices);
end


end

function weights = resolveWeightVector(model, fieldName, fallback)
if isfield(model, fieldName)
    weights = model.(fieldName);
else
    weights = fallback;
end
end
