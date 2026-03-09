function [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
% COMPUTEADAPTIVEFUSIONWEIGHTS - Compute adaptive GA/AA fusion weights.
%   [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
%
%   The weight model is factorized as
%       mask * covariance * innovation * history * link
%   and then normalized with temporal EMA smoothing.

numSensors = model.numberOfSensors;

cfg = struct();
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
end

emaAlpha = getField(cfg, 'emaAlpha', 0.7);
minWeight = getField(cfg, 'minWeight', 0.0);
useNIS = getField(cfg, 'useNIS', true);
useHistory = getField(cfg, 'useHistory', true);

availabilityMask = resolveAvailabilityMask(model, commStats, t, numSensors);
covScore = computeCovarianceScore(measurementUpdatedDistributions, model);
innovationScore = resolveInnovationScore(commStats, t, numSensors, useNIS);
linkQuality = computeLinkQuality(measurements, commStats, t, numSensors);
[historyScore, historyState, historyDebug] = computeHistoryScore( ...
    measurementUpdatedDistributions, covScore, innovationScore, cfg, prevWeights, useNIS, useHistory);

rawScore = availabilityMask .* covScore .* innovationScore .* historyScore .* linkQuality;
rawWeights = normalizeScores(rawScore, availabilityMask);

weights = rawWeights;
if nargin >= 6 && isstruct(prevWeights) && isfield(prevWeights, 'ga')
    if numel(prevWeights.ga) == numSensors
        weights = emaAlpha * prevWeights.ga + (1 - emaAlpha) * rawWeights;
        weights = normalizeScores(weights, availabilityMask);
    end
end

if minWeight > 0
    weights = enforceMinimumWeight(weights, availabilityMask, minWeight);
end

gaWeights = weights;
aaWeights = weights;

debug = struct();
debug.availabilityMask = availabilityMask;
debug.covScore = covScore;
debug.innovationScore = innovationScore;
debug.historyScore = historyScore;
debug.linkQuality = linkQuality;
debug.rawScore = rawScore;
debug.rawWeights = rawWeights;
debug.weights = weights;
debug.historyState = historyState;
debug.historyInstantInstability = historyDebug.instantInstability;
debug.historyInstabilityEma = historyDebug.instabilityEma;
debug.expectedCardinality = historyDebug.expectedCardinality;
end

function covScore = computeCovarianceScore(measurementUpdatedDistributions, model)
numSensors = numel(measurementUpdatedDistributions);
covScore = zeros(1, numSensors);
for s = 1:numSensors
    objects = measurementUpdatedDistributions{s};
    if isempty(objects)
        covScore(s) = 0;
        continue;
    end
    traceValues = zeros(1, numel(objects));
    traceCount = 0;
    for i = 1:numel(objects)
        if objects(i).numberOfGmComponents < 1
            continue;
        end
        [~, T] = mprojection(model.xDimension, objects(i));
        traceCount = traceCount + 1;
        traceValues(traceCount) = trace(T);
    end
    if traceCount == 0
        covScore(s) = 0;
    else
        meanTrace = mean(traceValues(1:traceCount));
        covScore(s) = 1 / (eps + meanTrace);
    end
end
end

function innovationScore = resolveInnovationScore(commStats, t, numSensors, useNIS)
innovationScore = ones(1, numSensors);
if ~useNIS
    return;
end
if nargin < 2 || ~isstruct(commStats) || ~isfield(commStats, 'innovationConsistency')
    return;
end
if size(commStats.innovationConsistency, 1) == numSensors && size(commStats.innovationConsistency, 2) >= t
    innovationScore = commStats.innovationConsistency(:, t)';
end
end

function linkQuality = computeLinkQuality(measurements, commStats, t, numSensors)
linkQuality = ones(1, numSensors);
if nargin < 2 || ~isstruct(commStats)
    return;
end
hasLinkFields = isfield(commStats, 'droppedByBandwidth') && ...
    isfield(commStats, 'droppedByLink') && isfield(commStats, 'droppedByOutage');
if ~hasLinkFields || t > size(commStats.droppedByBandwidth, 2)
    return;
end
for s = 1:numSensors
    deliveredCount = numel(measurements{s, t});
    droppedCount = commStats.droppedByBandwidth(s, t) + ...
        commStats.droppedByLink(s, t) + ...
        commStats.droppedByOutage(s, t);
    total = deliveredCount + droppedCount;
    if total > 0
        linkQuality(s) = deliveredCount / total;
    end
end
end

function mask = resolveAvailabilityMask(model, commStats, t, numSensors)
mask = ones(1, numSensors);
if nargin >= 2 && isstruct(commStats)
    if isfield(commStats, 'fusionMask') && size(commStats.fusionMask, 1) == numSensors && size(commStats.fusionMask, 2) >= t
        mask = double(commStats.fusionMask(:, t)' > 0);
        return;
    end
    if isfield(commStats, 'activeMask') && size(commStats.activeMask, 1) == numSensors && size(commStats.activeMask, 2) >= t
        mask = double(commStats.activeMask(:, t)' > 0);
        return;
    end
end
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    if isfield(model.adaptiveFusion, 'staticMask') && numel(model.adaptiveFusion.staticMask) == numSensors
        mask = double(reshape(model.adaptiveFusion.staticMask, 1, []) > 0);
    end
end
end

function [historyScore, historyState, debug] = computeHistoryScore( ...
    measurementUpdatedDistributions, covScore, innovationScore, cfg, prevWeights, useNIS, useHistory)

numSensors = numel(covScore);
expectedCardinality = computeExpectedCardinality(measurementUpdatedDistributions);

historyState = struct();
historyState.covScore = covScore;
historyState.innovationScore = innovationScore;
historyState.expectedCardinality = expectedCardinality;
historyState.instabilityEma = zeros(1, numSensors);
historyState.instantInstability = zeros(1, numSensors);

debug = struct();
debug.instantInstability = zeros(1, numSensors);
debug.instabilityEma = zeros(1, numSensors);
debug.expectedCardinality = expectedCardinality;

historyScore = ones(1, numSensors);
if ~useHistory
    return;
end

historyEmaAlpha = getField(cfg, 'historyEmaAlpha', 0.8);
historyScale = getField(cfg, 'historyScale', 2.0);
historyMinScore = getField(cfg, 'historyMinScore', 0.4);
historyMaxScore = getField(cfg, 'historyMaxScore', 1.0);
covWeight = getField(cfg, 'historyCovWeight', 0.4);
innovationWeight = getField(cfg, 'historyInnovationWeight', 0.4);
cardinalityWeight = getField(cfg, 'historyCardinalityWeight', 0.2);

if ~useNIS
    innovationWeight = 0.0;
end

if nargin < 5 || ~isstruct(prevWeights) || ~isfield(prevWeights, 'historyState')
    return;
end
prevState = prevWeights.historyState;
if ~isValidHistoryState(prevState, numSensors)
    return;
end

covDiff = abs(log(covScore + eps) - log(prevState.covScore + eps));
innovationDiff = abs(innovationScore - prevState.innovationScore);
cardinalityDiff = abs(expectedCardinality - prevState.expectedCardinality) ./ ...
    (1 + prevState.expectedCardinality);

totalWeight = covWeight + innovationWeight + cardinalityWeight;
if totalWeight <= 0
    return;
end

instantInstability = (covWeight * covDiff + innovationWeight * innovationDiff + ...
    cardinalityWeight * cardinalityDiff) / totalWeight;
instabilityEma = historyEmaAlpha * prevState.instabilityEma + ...
    (1 - historyEmaAlpha) * instantInstability;

historyScore = exp(-historyScale * instabilityEma);
historyScore = min(max(historyScore, historyMinScore), historyMaxScore);

historyState.instabilityEma = instabilityEma;
historyState.instantInstability = instantInstability;

debug.instantInstability = instantInstability;
debug.instabilityEma = instabilityEma;
end

function expectedCardinality = computeExpectedCardinality(measurementUpdatedDistributions)
numSensors = numel(measurementUpdatedDistributions);
expectedCardinality = zeros(1, numSensors);
for s = 1:numSensors
    objects = measurementUpdatedDistributions{s};
    if isempty(objects)
        continue;
    end
    expectedCardinality(s) = sum([objects.r]);
end
end

function isValid = isValidHistoryState(historyState, numSensors)
isValid = isstruct(historyState) && ...
    isfield(historyState, 'covScore') && numel(historyState.covScore) == numSensors && ...
    isfield(historyState, 'innovationScore') && numel(historyState.innovationScore) == numSensors && ...
    isfield(historyState, 'expectedCardinality') && numel(historyState.expectedCardinality) == numSensors && ...
    isfield(historyState, 'instabilityEma') && numel(historyState.instabilityEma) == numSensors;
end

function weights = normalizeScores(score, mask)
maskedScore = score .* mask;
if any(mask > 0) && sum(maskedScore) > 0
    weights = maskedScore / sum(maskedScore);
elseif any(mask > 0)
    weights = mask / sum(mask);
else
    weights = ones(size(score)) / numel(score);
end
end

function weights = enforceMinimumWeight(weights, mask, minWeight)
active = mask > 0;
if ~any(active)
    return;
end
weights(~active) = 0;
weights(active) = max(weights(active), minWeight);
weights = weights / sum(weights);
end

function value = getField(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function [nu, T] = mprojection(n, measurementUpdatedDistribution)
% Determine m-projected mean
nu = zeros(n, 1);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    nu = nu + measurementUpdatedDistribution.w(j) * measurementUpdatedDistribution.mu{j};
end
% Determine m-projected covariance
T = zeros(n, n);
for j = 1:measurementUpdatedDistribution.numberOfGmComponents
    w = measurementUpdatedDistribution.w(j);
    mu = measurementUpdatedDistribution.mu{j} - nu;
    Sigma = measurementUpdatedDistribution.Sigma{j};
    T = T + w * (Sigma + mu * mu');
end
end
