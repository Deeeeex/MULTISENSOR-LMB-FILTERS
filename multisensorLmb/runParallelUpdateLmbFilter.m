function stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats, sensorTrajectories)
% RUNPARALLELUPDATELMBFILTER -- Run a multi-sensor LMB filter that uses a parallel measurment update.
%   stateEstimates = runParallelUpdateLmbFilter(model, measurements)
%   stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats)
%
%   Run a multi-sensor LMB filter that uses a parallel measurment update.
%   Measurement update variants include arithmetic average (AA), geometric
%   average (GA), and parallel update (PU) update.
%   文件导读：
%       集中式多传感器 LMB 主循环，也是动态 GA/AA 权重接入滤波流程的
%       位置。每个时刻先做一次全局 prediction，再对每个传感器分别形成
%       measurement-updated posterior，随后可选调用 computeAdaptiveFusionWeights
%       更新 spatial/existence 权重，最后由 PU/GA/AA merging 完成融合。
%       在分布式实验里，本函数通常不是拿全局 8 个传感器运行，而是被
%       runDistributedLmbFilter 用“邻域子模型 + 邻域测量”调用。此时
%       model.numberOfSensors 表示局部邻域大小，传感器索引也都是局部索引。
%
%   本函数的关键调用链：
%       lmbPredictionStep
%       -> generateLmbSensorAssociationMatrices
%       -> loopyBeliefPropagation / lmbGibbsSampling / lmbMurtysAlgorithm
%       -> computePosteriorLmbSpatialDistributions
%       -> computeAdaptiveFusionWeights
%       -> aaLmbTrackMerging / gaLmbTrackMerging / puLmbTrackMerging
%
%   See also generateMultisensorModel, generateMultisensorGroundTruth, lmbPredictionStep,
%   generateLmbSensorAssociationMatrices, loopyBeliefPropagation, lmbGibbsSampling, 
%   lmbMurtysAlgorithm, computePosteriorLmbSpatialDistributions,
%   lmbMapCardinalityEstimate, aaLmbTrackMerging, gaLmbTrackMerging,
%   puLmbTrackMerging.
%
%   Inputs
%       model - struct. A struct with the fields declared in generateModel.
%       measurements - cell array. An array containing the measurements for
%           each time-step of the simulation. See also generateModel.
%       commStats - struct. Communication statistics for link quality (optional).
%
%   Output
%       stateEstimates - struct. A struct containing the LMB filter's
%           approximate MAP estimate for each time-step of the simulation, as
%           well as the objects' trajectories.

%% 1. 初始化状态：objects 是预测/融合后持续递推的 LMB component 集合
% measurements 的维度是 [sensor x time] cell。对于 distributed local run，
% sensor 维度已经由 runDistributedLmbFilter 切成当前邻域。
simulationLength = length(measurements);
% Struct containing objects' Bernoulli parameters and metadata
objects = model.object;

% 移动传感器轨迹是 association matrix 计算几何量时需要的上下文。
if nargin >= 4 && ~isempty(sensorTrajectories)
    model.sensorTrajectories = sensorTrajectories;
end
% Output struct
stateEstimates.labels = cell(simulationLength, 1);
stateEstimates.mu = cell(simulationLength, 1);
stateEstimates.Sigma = cell(simulationLength, 1);
stateEstimates.objects = objects;
%% 2. 读取动态融合配置，并初始化上一时刻权重/诊断缓存
useAdaptiveFusion = false;
if isfield(model, 'adaptiveFusion') && isfield(model.adaptiveFusion, 'enabled')
    useAdaptiveFusion = model.adaptiveFusion.enabled;
end
adaptiveCfg = struct();
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    adaptiveCfg = model.adaptiveFusion;
end
useNIS = getConfigField(adaptiveCfg, 'useNIS', true);
progressEverySteps = max(round(getConfigField(adaptiveCfg, 'progressEverySteps', 0)), 0);
progressLabel = getConfigField(adaptiveCfg, 'progressLabel', '');
captureWeightDiagnostics = logical(getConfigField(adaptiveCfg, 'captureWeightDiagnostics', false));
if captureWeightDiagnostics
    stateEstimates.adaptiveFusionDiagnostics = repmat( ...
        buildEmptyAdaptiveFusionDiagnostic(), simulationLength, 1);
end
useNisEma = getConfigField(adaptiveCfg, 'nisEmaEnabled', true);
nisEmaAlpha = getConfigField(adaptiveCfg, 'nisEmaAlpha', 0.7);
% prevWeights 是动态权重跨时刻平滑的状态入口。当前主线核心不再使用
% historyScore，但 spatial/existence branch 仍需要上一时刻权重做 EMA。
% 这里同时保留 ga/aa scalar 权重作为没有 branch-specific debug 输出时的兜底。
prevWeights = struct();
prevWeights.ga = model.gaSensorWeights;
prevWeights.aa = model.aaSensorWeights;
prevWeights.gaSpatial = getConfigField(model, 'gaSpatialWeights', model.gaSensorWeights);
prevWeights.aaSpatial = getConfigField(model, 'aaSpatialWeights', model.aaSensorWeights);
prevWeights.gaExistence = getConfigField(model, 'gaExistenceWeights', model.gaSensorWeights);
prevWeights.aaExistence = getConfigField(model, 'aaExistenceWeights', model.aaSensorWeights);
% historyState 是可选跨时刻诊断/平滑状态；默认主线为空。实验性
% history-smoothed existence confidence 会在这里保存按 label 匹配的平滑 r。
prevWeights.historyState = struct();
% 下面两个诊断矩阵来自 association 阶段。当前动态权重核心不再消费
% NIS/ambiguity 分数，但保留 commStatsLocal 字段有助于旧报告或调试。
innovationConsistency = ones(model.numberOfSensors, simulationLength);
associationAmbiguityScore = ones(model.numberOfSensors, simulationLength);
%% 3. 时间递推：每个时刻先局部更新，再融合
for t = 1:simulationLength
    %% 3.1 预测：所有传感器共享同一个 predicted LMB prior
    objects = lmbPredictionStep(objects, model, t);
    %% 3.2 每个传感器独立做 measurement update，得到 local posterior
    measurementUpdatedDistributions = cell(1, model.numberOfSensors);
    for s = 1:model.numberOfSensors
        % 每个传感器先独立处理自己的 delivered measurements。此时所有
        % sensor 都基于同一个 predicted prior objects，因此后面才能把
        % measurement-updated posteriors 做 GA/AA/PU 融合。
        [measurementUpdatedDistributions{s}, updateDiagnostics] = ...
            updateLmbWithSensorMeasurement( ...
                objects, measurements{s, t}, model, s, t, ...
                isScheduledSample(commStats, s, t));
        innovationConsistency(s, t) = updateDiagnostics.innovationScore;
        associationAmbiguityScore(s, t) = ...
            updateDiagnostics.associationConfidence;
        if useNIS && useNisEma && t > 1
            innovationConsistency(s, t) = ...
                nisEmaAlpha * innovationConsistency(s, t-1) + ...
                (1 - nisEmaAlpha) * innovationConsistency(s, t);
        end
    end
    %% 3.3 动态融合权重：只作用于 GA/AA，PU 不走该权重路径
    if useAdaptiveFusion && (strcmp(model.lmbParallelUpdateMode, 'AA') || strcmp(model.lmbParallelUpdateMode, 'GA'))
        if nargin < 3
            commStatsLocal = [];
        else
            commStatsLocal = commStats;
        end
        if isempty(commStatsLocal) || ~isstruct(commStatsLocal)
            commStatsLocal = struct();
        end
        commStatsLocal.innovationConsistency = innovationConsistency;
        commStatsLocal.associationAmbiguityScore = associationAmbiguityScore;
        % computeAdaptiveFusionWeights 是动态权重唯一入口。它读取：
        %   measurementUpdatedDistributions : 当前时刻各传感器 local posterior；
        %   measurements / commStatsLocal   : 通信可用性和 delivered/drop 统计；
        %   prevWeights                     : 上一时刻 scalar/spatial/existence 权重。
        % 返回的 debug.gaSpatialWeights / debug.gaExistenceWeights 会决定
        % GA merging 时 spatial density 和 Bernoulli existence 的不同融合权重。
        [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights( ...
            measurementUpdatedDistributions, measurements, model, t, commStatsLocal, prevWeights);
        % scalar 权重是兜底路径；如果 debug 中提供 spatial/existence 权重，
        % 后续 GA/AA merging 会优先读取 branch-specific 字段。
        model.gaSensorWeights = gaWeights;
        model.aaSensorWeights = aaWeights;
        model.gaSpatialWeights = getConfigField(debug, 'gaSpatialWeights', gaWeights);
        model.aaSpatialWeights = getConfigField(debug, 'aaSpatialWeights', aaWeights);
        model.gaExistenceWeights = getConfigField(debug, 'gaExistenceWeights', gaWeights);
        model.aaExistenceWeights = getConfigField(debug, 'aaExistenceWeights', aaWeights);
        % PD/FI direct baseline 可返回 target-wise weights；没有时要清掉旧字段，
        % 防止上一时刻或上一 arm 的 target-wise 权重残留。
        if isfield(debug, 'gaTargetWiseWeights')
            model.gaTargetWiseWeights = debug.gaTargetWiseWeights;
        elseif isfield(model, 'gaTargetWiseWeights')
            model = rmfield(model, 'gaTargetWiseWeights');
        end
        if isfield(debug, 'aaTargetWiseWeights')
            model.aaTargetWiseWeights = debug.aaTargetWiseWeights;
        elseif isfield(model, 'aaTargetWiseWeights')
            model = rmfield(model, 'aaTargetWiseWeights');
        end
        if captureWeightDiagnostics
            stateEstimates.adaptiveFusionDiagnostics(t) = ...
                buildAdaptiveFusionDiagnosticRecord( ...
                    debug, model, t, measurementUpdatedDistributions);
        end
        % 更新上一时刻状态，供下一时刻 EMA 使用。scalar、spatial、existence
        % 分开记录，是因为 decoupled KLA 两条分支可以有不同平滑系数和下界。
        prevWeights.ga = gaWeights;
        prevWeights.aa = aaWeights;
        prevWeights.gaSpatial = model.gaSpatialWeights;
        prevWeights.aaSpatial = model.aaSpatialWeights;
        prevWeights.gaExistence = model.gaExistenceWeights;
        prevWeights.aaExistence = model.aaExistenceWeights;
        if isfield(debug, 'historyState')
            prevWeights.historyState = debug.historyState;
        end
    end
    %% 3.4 跨传感器融合：此处才真正把 local posteriors 合成一个 posterior LMB
    % 到这里为止，每个 cell 里仍是一份 sensor-specific posterior。
    % GA/AA/PU merging 才是跨传感器融合点；动态权重只影响 GA/AA。
    if (strcmp(model.lmbParallelUpdateMode, 'AA'))
        objects = aaLmbTrackMerging(measurementUpdatedDistributions, model);
    elseif (strcmp(model.lmbParallelUpdateMode, 'GA'))
        objects = gaLmbTrackMerging(measurementUpdatedDistributions, model);
    else
        objects = puLmbTrackMerging(measurementUpdatedDistributions, objects, model);
    end
    %% 3.5 融合后剪枝：低存在概率 component 不再递推
    objectsLikelyToExist = [objects.r] > model.existenceThreshold;
    % 已经形成足够长轨迹的低概率 component 仍写入输出对象列表。
    discardedObjects = objects(~objectsLikelyToExist & ([objects.trajectoryLength] > model.minimumTrajectoryLength));
    stateEstimates.objects(end+1:end+numel(discardedObjects)) =  discardedObjects;
    % Keep objects with high existence probabilities
    objects = objects(objectsLikelyToExist);
    %% 3.6 MAP 基数/状态抽取：生成当前时刻对外输出的 RFS estimate
    % LMB posterior 是一组 Bernoulli components；MAP cardinality 先决定
    % 本时刻输出几个目标，再取对应 component 的最高权重 Gaussian。
    [nMap, mapIndices] = lmbMapCardinalityEstimate([objects.r]);
    % 按 MAP 选中的 component 输出 label、均值和协方差。
    stateEstimates.labels{t} = zeros(2, nMap);
    stateEstimates.mu{t} = cell(1, nMap);
    stateEstimates.Sigma{t} = cell(1, nMap);
    for i = 1:nMap
        j = mapIndices(i);
        % Gaussians in the posterior GM are sorted according to weight
        stateEstimates.labels{t}(:, i) = [objects(j).birthTime; objects(j).birthLocation];
        stateEstimates.mu{t}{i} = objects(j).mu{1};
        stateEstimates.Sigma{t}{i} = objects(j).Sigma{1};
    end
    %% 3.7 更新轨迹缓存，供后续可视化和输出使用
    for i = 1:numel(objects)
        j = objects(i).trajectoryLength;
        objects(i).trajectoryLength = j + 1;
        objects(i).trajectory(:, j+1) = objects(i).mu{1};
        objects(i).timestamps(j+1) = t;
    end
    % if progressEverySteps > 0 && (mod(t, progressEverySteps) == 0 || t == simulationLength)
    %     if isempty(progressLabel)
    %         fprintf('Filter progress %d/%d\n', t, simulationLength);
    %     else
    %         fprintf('[%s] progress %d/%d\n', progressLabel, t, simulationLength);
    %     end
    % end
end
%% 4. 收尾：保存仍存活且足够长的轨迹
discardedObjects = objects(([objects.trajectoryLength] > model.minimumTrajectoryLength));
numberOfDiscardedObjects = numel(discardedObjects);
stateEstimates.objects(end+1:end+numberOfDiscardedObjects) =  discardedObjects;
end

function record = buildEmptyAdaptiveFusionDiagnostic()
record = struct( ...
    't', NaN, ...
    'numberOfSensors', NaN, ...
    'availabilityMask', [], ...
    'aaSpatialWeights', [], ...
    'aaExistenceWeights', [], ...
    'spatialEntropy', NaN, ...
    'existenceEntropy', NaN, ...
    'spatialEffectiveSensorCount', NaN, ...
    'existenceEffectiveSensorCount', NaN, ...
    'branchL1Divergence', NaN, ...
    'maxSpatialWeight', NaN, ...
    'maxExistenceWeight', NaN, ...
    'expectedCardinality', [], ...
    'targetLocalExistence', [], ...
    'targetSpatialWeights', [], ...
    'targetExistenceWeights', [], ...
    'targetSpatialEntropy', [], ...
    'targetExistenceEntropy', [], ...
    'targetSpatialEffectiveSensorCount', [], ...
    'targetExistenceEffectiveSensorCount', [], ...
    'targetBranchL1Divergence', [], ...
    'targetSpatialActiveCount', [], ...
    'targetExistenceActiveCount', []);
end

function record = buildAdaptiveFusionDiagnosticRecord(debug, model, t, measurementUpdatedDistributions)
record = buildEmptyAdaptiveFusionDiagnostic();
numSensors = model.numberOfSensors;
record.t = t;
record.numberOfSensors = numSensors;
availabilityMask = normalizeDiagnosticMask(getConfigField(debug, 'availabilityMask', ones(1, numSensors)), numSensors);
spatialWeights = normalizeDiagnosticWeights( ...
    getConfigField(debug, 'aaSpatialWeights', getConfigField(debug, 'weights', availabilityMask)), ...
    availabilityMask);
existenceWeights = normalizeDiagnosticWeights( ...
    getConfigField(debug, 'aaExistenceWeights', getConfigField(debug, 'weights', availabilityMask)), ...
    availabilityMask);

record.availabilityMask = availabilityMask;
record.aaSpatialWeights = spatialWeights;
record.aaExistenceWeights = existenceWeights;
record.spatialEntropy = normalizedWeightEntropy(spatialWeights);
record.existenceEntropy = normalizedWeightEntropy(existenceWeights);
record.spatialEffectiveSensorCount = effectiveSensorCount(spatialWeights);
record.existenceEffectiveSensorCount = effectiveSensorCount(existenceWeights);
record.branchL1Divergence = 0.5 * sum(abs(spatialWeights - existenceWeights));
record.maxSpatialWeight = max(spatialWeights);
record.maxExistenceWeight = max(existenceWeights);
record.expectedCardinality = getConfigField(debug, 'expectedCardinality', []);

numTargets = resolveDiagnosticTargetCount(measurementUpdatedDistributions);
if numTargets <= 0
    return;
end

targetWiseWeights = getConfigField(debug, 'aaTargetWiseWeights', []);
activeThreshold = max(getAdaptiveConfigField(model, 'weightDiagnosticActiveThreshold', 1e-3), 0);
record.targetLocalExistence = zeros(numTargets, numSensors);
record.targetSpatialWeights = zeros(numTargets, numSensors);
record.targetExistenceWeights = zeros(numTargets, numSensors);
record.targetSpatialEntropy = zeros(numTargets, 1);
record.targetExistenceEntropy = zeros(numTargets, 1);
record.targetSpatialEffectiveSensorCount = zeros(numTargets, 1);
record.targetExistenceEffectiveSensorCount = zeros(numTargets, 1);
record.targetBranchL1Divergence = zeros(numTargets, 1);
record.targetSpatialActiveCount = zeros(numTargets, 1);
record.targetExistenceActiveCount = zeros(numTargets, 1);

for objectIdx = 1:numTargets
    targetSpatial = spatialWeights;
    targetExistence = existenceWeights;
    if size(targetWiseWeights, 1) >= objectIdx && size(targetWiseWeights, 2) == numSensors
        targetSpatial = normalizeDiagnosticWeights(targetWiseWeights(objectIdx, :), availabilityMask);
        targetExistence = targetSpatial;
    end
    localExistence = extractLocalExistenceForTarget(measurementUpdatedDistributions, objectIdx, numSensors);
    record.targetLocalExistence(objectIdx, :) = localExistence;

    if isAaKlaSpatialFusion(model)
        targetSpatial = applyDiagnosticKlaSpatialExistenceGate(model, targetSpatial, localExistence);
    end
    targetExistence = normalizeDiagnosticWeights(targetExistence, availabilityMask);

    record.targetSpatialWeights(objectIdx, :) = targetSpatial;
    record.targetExistenceWeights(objectIdx, :) = targetExistence;
    record.targetSpatialEntropy(objectIdx) = normalizedWeightEntropy(targetSpatial);
    record.targetExistenceEntropy(objectIdx) = normalizedWeightEntropy(targetExistence);
    record.targetSpatialEffectiveSensorCount(objectIdx) = effectiveSensorCount(targetSpatial);
    record.targetExistenceEffectiveSensorCount(objectIdx) = effectiveSensorCount(targetExistence);
    record.targetBranchL1Divergence(objectIdx) = 0.5 * sum(abs(targetSpatial - targetExistence));
    record.targetSpatialActiveCount(objectIdx) = sum(targetSpatial > activeThreshold);
    record.targetExistenceActiveCount(objectIdx) = sum(targetExistence > activeThreshold);
end
end

function mask = normalizeDiagnosticMask(mask, numSensors)
mask = reshape(mask, 1, []);
if numel(mask) ~= numSensors
    mask = ones(1, numSensors);
end
mask(~isfinite(mask)) = 0;
mask = double(mask > 0);
if sum(mask) <= 0
    mask = ones(1, numSensors);
end
end

function weights = normalizeDiagnosticWeights(weights, availabilityMask)
availabilityMask = reshape(availabilityMask, 1, []);
weights = reshape(weights, 1, []);
if numel(weights) ~= numel(availabilityMask)
    weights = availabilityMask;
end
weights(~isfinite(weights)) = 0;
weights = max(weights, 0) .* availabilityMask;
if sum(weights) <= eps
    weights = availabilityMask;
end
if sum(weights) <= eps
    weights = ones(1, numel(availabilityMask)) / max(numel(availabilityMask), 1);
else
    weights = weights / sum(weights);
end
end

function entropyValue = normalizedWeightEntropy(weights)
weights = reshape(weights, 1, []);
weights = weights(weights > 0 & isfinite(weights));
if numel(weights) <= 1
    entropyValue = 0;
    return;
end
entropyValue = -sum(weights .* log(weights)) / log(numel(weights));
end

function count = effectiveSensorCount(weights)
weights = reshape(weights, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= eps
    count = 0;
    return;
end
weights = weights / sum(weights);
count = 1 / sum(weights .^ 2);
end

function numTargets = resolveDiagnosticTargetCount(measurementUpdatedDistributions)
numTargets = 0;
for s = 1:numel(measurementUpdatedDistributions)
    numTargets = max(numTargets, numel(measurementUpdatedDistributions{s}));
end
end

function localExistence = extractLocalExistenceForTarget(measurementUpdatedDistributions, objectIdx, numSensors)
localExistence = zeros(1, numSensors);
for s = 1:numSensors
    if s <= numel(measurementUpdatedDistributions) && ...
            objectIdx <= numel(measurementUpdatedDistributions{s})
        r = measurementUpdatedDistributions{s}(objectIdx).r;
        if ~isfinite(r)
            r = 0;
        end
        localExistence(s) = min(max(r, 0), 1);
    end
end
end

function weights = applyDiagnosticKlaSpatialExistenceGate(model, weights, localExistence)
power = resolveDiagnosticKlaSpatialExistencePower(model);
if power <= 0
    return;
end
minScore = resolveDiagnosticKlaSpatialExistenceMinScore(model);
localExistence = reshape(localExistence, 1, []);
gate = minScore + (1 - minScore) * (localExistence .^ power);
candidate = reshape(weights, 1, []) .* gate;
if sum(candidate) <= eps
    return;
end
weights = candidate / sum(candidate);
end

function tf = isAaKlaSpatialFusion(model)
mode = '';
if isfield(model, 'aaSpatialFusionMode')
    mode = model.aaSpatialFusionMode;
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    mode = getConfigField(model.adaptiveFusion, 'aaSpatialFusionMode', mode);
end
tf = false;
if ischar(mode) || isstring(mode)
    tf = any(strcmpi(char(mode), {'kla', 'ga', 'geometric', ...
        'spatial-kla', 'spatial_kla', 'hybrid-kla', 'hybrid_kla'}));
end
end

function power = resolveDiagnosticKlaSpatialExistencePower(model)
power = getAdaptiveConfigField(model, 'aaKlaSpatialExistencePower', 0);
if power <= 0 && logical(getAdaptiveConfigField(model, 'aaKlaSpatialExistenceGate', false))
    power = 1.0;
end
if ~isfinite(power)
    power = 0;
end
power = max(power, 0);
end

function minScore = resolveDiagnosticKlaSpatialExistenceMinScore(model)
minScore = getAdaptiveConfigField(model, 'aaKlaSpatialExistenceMinScore', 0);
if ~isfinite(minScore)
    minScore = 0;
end
minScore = min(max(minScore, 0), 1);
end

function value = getAdaptiveConfigField(model, fieldName, defaultValue)
value = defaultValue;
if isfield(model, fieldName)
    value = model.(fieldName);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    value = getConfigField(model.adaptiveFusion, fieldName, value);
end
end

function value = getConfigField(cfg, fieldName, defaultValue)
if isfield(cfg, fieldName)
    value = cfg.(fieldName);
else
    value = defaultValue;
end
end

function tf = isScheduledSample(commStats, sensorIdx, currentTime)
% ISSCHEDULEDSAMPLE - 区分“本时刻该传感器采样但无检测”和“本时刻根本未采样”。
% 未采样时直接沿用 prior；采样但无检测时要做 missed-detection update。
tf = true;
if nargin < 1 || ~isstruct(commStats) || ~isfield(commStats, 'sensorSampleMask')
    return;
end
if size(commStats.sensorSampleMask, 1) >= sensorIdx && ...
        size(commStats.sensorSampleMask, 2) >= currentTime
    tf = commStats.sensorSampleMask(sensorIdx, currentTime) > 0;
end
end
