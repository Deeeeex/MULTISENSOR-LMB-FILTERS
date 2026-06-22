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
useLabelUncertaintyFusion = resolveLabelUncertaintyFusion(model);
useLabelExistenceTempering = resolveLabelExistenceTempering(model);
useLabelUncertaintyInflation = resolveLabelUncertaintyInflation(model);
useLabelSpatialOverlapWeights = resolveLabelSpatialOverlapWeights(model);
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
    objects(i).labelSupportMass = objects(i).r;
    objects(i).labelSupportEffectiveCount = computeLabelSupportEffectiveCount( ...
        measurementUpdatedDistributions, model, existenceWeights, i);

    if useKlaSpatialFusion
        spatialWeights = applyKlaSpatialExistenceGate( ...
            measurementUpdatedDistributions, model, spatialWeights, i);
        if useLabelUncertaintyFusion
            [muGa, SigmaGa, quality] = fuseSpatialWithLabelUncertaintyKla( ...
                measurementUpdatedDistributions, model, spatialWeights, existenceWeights, i, ...
                useLabelUncertaintyInflation, useLabelSpatialOverlapWeights);
            if useLabelExistenceTempering
                objects(i).r = temperExistenceWithLabelQuality(objects(i).r, quality);
            end
        else
            [muGa, SigmaGa] = fuseSpatialWithKla(measurementUpdatedDistributions, model, spatialWeights, i);
        end
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

function useLabelUncertaintyFusion = resolveLabelUncertaintyFusion(model)
useLabelUncertaintyFusion = false;
if isfield(model, 'useAaLabelUncertaintyFusion')
    useLabelUncertaintyFusion = logical(model.useAaLabelUncertaintyFusion);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'useAaLabelUncertaintyFusion')
        useLabelUncertaintyFusion = logical(cfg.useAaLabelUncertaintyFusion);
    end
end
end

function useLabelExistenceTempering = resolveLabelExistenceTempering(model)
useLabelExistenceTempering = false;
if isfield(model, 'useAaLabelExistenceTempering')
    useLabelExistenceTempering = logical(model.useAaLabelExistenceTempering);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'useAaLabelExistenceTempering')
        useLabelExistenceTempering = logical(cfg.useAaLabelExistenceTempering);
    end
end
end

function useLabelUncertaintyInflation = resolveLabelUncertaintyInflation(model)
useLabelUncertaintyInflation = true;
if isfield(model, 'useAaLabelUncertaintyInflation')
    useLabelUncertaintyInflation = logical(model.useAaLabelUncertaintyInflation);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'useAaLabelUncertaintyInflation')
        useLabelUncertaintyInflation = logical(cfg.useAaLabelUncertaintyInflation);
    end
end
end

function useLabelSpatialOverlapWeights = resolveLabelSpatialOverlapWeights(model)
useLabelSpatialOverlapWeights = true;
if isfield(model, 'useAaLabelSpatialOverlapWeights')
    useLabelSpatialOverlapWeights = logical(model.useAaLabelSpatialOverlapWeights);
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'useAaLabelSpatialOverlapWeights')
        useLabelSpatialOverlapWeights = logical(cfg.useAaLabelSpatialOverlapWeights);
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

function [muGa, SigmaGa, quality] = fuseSpatialWithLabelUncertaintyKla( ...
    measurementUpdatedDistributions, model, spatialWeights, existenceWeights, objectIdx, ...
    useInflation, useOverlapWeights)
[means, covariances, localExistence, validMask] = projectLocalObjects( ...
    measurementUpdatedDistributions, model, objectIdx);
baseSpatialWeights = normalizeMaskedWeights(spatialWeights, validMask);
agreement = computeLabelSpatialAgreement(means, covariances, baseSpatialWeights, localExistence, validMask);
if useOverlapWeights
    betaRaw = baseSpatialWeights .* localExistence .* agreement .* validMask;
else
    betaRaw = baseSpatialWeights .* validMask;
end
if sum(betaRaw) <= eps
    betaRaw = baseSpatialWeights .* validMask;
end
if sum(betaRaw) <= eps
    beta = normalizeMaskedWeights(spatialWeights, validMask);
else
    beta = betaRaw / sum(betaRaw);
end

[muGa, SigmaKla] = fuseProjectedGaussiansWithKla(means, covariances, beta, validMask, model.xDimension);
SigmaGa = SigmaKla;
if useInflation
    SigmaGa = regularizeCovariance(SigmaGa + computeBetweenPosteriorCovariance(means, beta, validMask, model.xDimension));
end

existenceMass = normalizeMaskedWeights(existenceWeights, validMask) .* localExistence .* validMask;
supportMass = sum(existenceMass);
if supportMass > eps
    weightedAgreement = sum(existenceMass .* agreement) / supportMass;
else
    weightedAgreement = 0;
end
quality = struct( ...
    'supportMass', min(max(supportMass, 0), 1), ...
    'weightedAgreement', min(max(weightedAgreement, 0), 1), ...
    'effectiveSupport', computeEffectiveSupport(existenceMass), ...
    'beta', beta, ...
    'agreement', agreement);
end

function [means, covariances, localExistence, validMask] = projectLocalObjects( ...
    measurementUpdatedDistributions, model, objectIdx)
numSensors = model.numberOfSensors;
means = zeros(model.xDimension, numSensors);
covariances = repmat(eye(model.xDimension), 1, 1, numSensors);
localExistence = zeros(1, numSensors);
validMask = zeros(1, numSensors);
for s = 1:numSensors
    if s > numel(measurementUpdatedDistributions) || ...
            objectIdx > numel(measurementUpdatedDistributions{s})
        continue;
    end
    localObject = measurementUpdatedDistributions{s}(objectIdx);
    if localObject.numberOfGmComponents < 1
        continue;
    end
    [nu, T] = mprojectObject(model.xDimension, localObject);
    means(:, s) = nu;
    covariances(:, :, s) = regularizeCovariance(T);
    localExistence(s) = clampProbability(localObject.r);
    validMask(s) = 1;
end
end

function agreement = computeLabelSpatialAgreement(means, covariances, weights, localExistence, validMask)
numSensors = numel(weights);
agreement = ones(1, numSensors);
for s = 1:numSensors
    if validMask(s) <= 0
        agreement(s) = 0;
        continue;
    end
    numerator = 0;
    denominator = 0;
    for j = 1:numSensors
        if j == s || validMask(j) <= 0
            continue;
        end
        pairWeight = weights(j) * localExistence(j);
        if pairWeight <= 0
            continue;
        end
        numerator = numerator + pairWeight * gaussianBhattacharyyaCoefficient( ...
            means(:, s), covariances(:, :, s), means(:, j), covariances(:, :, j));
        denominator = denominator + pairWeight;
    end
    if denominator > eps
        agreement(s) = numerator / denominator;
    else
        agreement(s) = 1;
    end
    agreement(s) = min(max(agreement(s), 0), 1);
end
end

function value = gaussianBhattacharyyaCoefficient(muA, covA, muB, covB)
covA = regularizeCovariance(covA);
covB = regularizeCovariance(covB);
covMean = regularizeCovariance((covA + covB) / 2);
delta = muA - muB;
mahalanobis = delta' * (covMean \ delta);
logValue = -0.125 * mahalanobis + 0.25 * safeLogDet(covA) + ...
    0.25 * safeLogDet(covB) - 0.5 * safeLogDet(covMean);
if ~isfinite(logValue)
    value = 0;
else
    value = exp(min(logValue, 0));
end
value = min(max(real(value), 0), 1);
end

function logDet = safeLogDet(covariance)
covariance = regularizeCovariance(covariance);
[R, p] = chol(covariance);
if p ~= 0
    covariance = regularizeCovariance(covariance + 1e-8 * eye(size(covariance)));
    [R, p] = chol(covariance);
end
if p ~= 0
    detValue = abs(det(covariance));
    logDet = log(max(detValue, realmin));
else
    logDet = 2 * sum(log(max(abs(diag(R)), realmin)));
end
end

function [muGa, SigmaGa] = fuseProjectedGaussiansWithKla(means, covariances, weights, validMask, xDimension)
K = zeros(xDimension, xDimension);
h = zeros(xDimension, 1);
for s = 1:numel(weights)
    if validMask(s) <= 0 || weights(s) <= 0
        continue;
    end
    T = regularizeCovariance(covariances(:, :, s));
    precision = inv(T);
    weightedPrecision = weights(s) * precision;
    K = K + weightedPrecision;
    h = h + weightedPrecision * means(:, s);
end
K = regularizeCovariance(K);
SigmaGa = regularizeCovariance(inv(K));
muGa = SigmaGa * h;
end

function covariance = computeBetweenPosteriorCovariance(means, weights, validMask, xDimension)
covariance = zeros(xDimension, xDimension);
if sum(weights .* validMask) <= eps
    return;
end
center = means * reshape(weights, [], 1);
for s = 1:numel(weights)
    if validMask(s) <= 0 || weights(s) <= 0
        continue;
    end
    delta = means(:, s) - center;
    covariance = covariance + weights(s) * (delta * delta');
end
covariance = regularizeCovariance(covariance);
end

function weights = normalizeMaskedWeights(weights, validMask)
weights = reshape(weights, 1, []);
validMask = reshape(validMask, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0) .* validMask;
if sum(weights) <= eps
    weights = validMask;
end
if sum(weights) <= eps
    weights = ones(size(validMask)) / max(numel(validMask), 1);
else
    weights = weights / sum(weights);
end
end

function value = computeEffectiveSupport(mass)
mass = reshape(mass, 1, []);
mass(~isfinite(mass)) = 0;
mass = max(mass, 0);
if sum(mass) <= eps
    value = 0;
    return;
end
value = (sum(mass) ^ 2) / max(sum(mass .^ 2), eps);
end

function value = computeLabelSupportEffectiveCount( ...
    measurementUpdatedDistributions, model, existenceWeights, objectIdx)
support = zeros(1, model.numberOfSensors);
for s = 1:model.numberOfSensors
    if s > numel(measurementUpdatedDistributions) || ...
            objectIdx > numel(measurementUpdatedDistributions{s})
        continue;
    end
    localObject = measurementUpdatedDistributions{s}(objectIdx);
    if localObject.numberOfGmComponents < 1
        continue;
    end
    support(s) = existenceWeights(s) * clampProbability(localObject.r);
end
value = computeEffectiveSupport(support);
end

function existence = temperExistenceWithLabelQuality(existence, quality)
existence = clampProbability(existence);
if existence <= 0 || existence >= 1
    return;
end
qualityScore = min(max(quality.supportMass * quality.weightedAgreement, eps), 1);
odds = existence / max(1 - existence, eps);
temperedOdds = odds * qualityScore;
existence = clampProbability(temperedOdds / (1 + temperedOdds));
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

function weights = applyKlaSpatialExistenceGate(measurementUpdatedDistributions, model, weights, objectIdx)
% APPLYKLASPATIALEXISTENCEGATE - Optional target-wise guard for the AA/KLA hybrid.
% The pure AA spatial mixture weights each local spatial density by alpha_s r_s.
% The KLA-spatial hybrid normally uses alpha_s only; enabling this guard restores
% a target-wise existence cue before the Gaussian KLA projection.
power = resolveKlaSpatialExistencePower(model);
if power <= 0
    return;
end
minScore = resolveKlaSpatialExistenceMinScore(model);
existenceScore = zeros(1, numel(weights));
for s = 1:numel(weights)
    if s <= numel(measurementUpdatedDistributions) && ...
            objectIdx <= numel(measurementUpdatedDistributions{s})
        r = clampProbability(measurementUpdatedDistributions{s}(objectIdx).r);
    else
        r = 0;
    end
    existenceScore(s) = minScore + (1 - minScore) * (r ^ power);
end
candidate = reshape(weights, 1, []) .* existenceScore;
if sum(candidate) <= eps
    return;
end
weights = candidate / sum(candidate);
end

function power = resolveKlaSpatialExistencePower(model)
power = 0;
if isfield(model, 'aaKlaSpatialExistencePower')
    power = model.aaKlaSpatialExistencePower;
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'aaKlaSpatialExistencePower')
        power = cfg.aaKlaSpatialExistencePower;
    elseif isfield(cfg, 'aaKlaSpatialExistenceGate') && logical(cfg.aaKlaSpatialExistenceGate)
        power = 1.0;
    end
end
if ~isfinite(power)
    power = 0;
end
power = max(power, 0);
end

function minScore = resolveKlaSpatialExistenceMinScore(model)
minScore = 0;
if isfield(model, 'aaKlaSpatialExistenceMinScore')
    minScore = model.aaKlaSpatialExistenceMinScore;
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
    if isfield(cfg, 'aaKlaSpatialExistenceMinScore')
        minScore = cfg.aaKlaSpatialExistenceMinScore;
    end
end
if ~isfinite(minScore)
    minScore = 0;
end
minScore = min(max(minScore, 0), 1);
end

function value = clampProbability(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
