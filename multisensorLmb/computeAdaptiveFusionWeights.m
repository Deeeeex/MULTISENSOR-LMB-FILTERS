function [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
% COMPUTEADAPTIVEFUSIONWEIGHTS - Compute adaptive GA/AA fusion weights.
%   [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
%
%   This uses covariance level, link quality, and an innovation consistency
%   placeholder to build reliability scores. An EMA is applied for temporal
%   stability. Innovation consistency is left as a placeholder for now.
%
%   Inputs
%       measurementUpdatedDistributions - (1, numberOfSensors) cell array.
%       measurements - cell array. sensors x time, delivered measurements.
%       model - struct. Multisensor model.
%       t - integer. Time index.
%       commStats - struct. Communication stats (optional).
%       prevWeights - struct with fields ga, aa (optional).
%
%   Outputs
%       gaWeights - (1, numberOfSensors) adaptive GA weights.
%       aaWeights - (1, numberOfSensors) adaptive AA weights.
%       debug - struct with intermediate factors.

numSensors = model.numberOfSensors;

cfg = struct();
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
end
emaAlpha = getField(cfg, 'emaAlpha', 0.7);
minWeight = getField(cfg, 'minWeight', 0.0);

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

% Innovation consistency placeholder.
% TODO: Replace with NIS-based consistency when available.
innovationScore = ones(1, numSensors);
if nargin >= 5 && isstruct(commStats) && isfield(commStats, 'innovationConsistency')
    if size(commStats.innovationConsistency, 1) == numSensors && size(commStats.innovationConsistency, 2) >= t
        innovationScore = commStats.innovationConsistency(:, t)';
    end
end

linkQuality = ones(1, numSensors);
if nargin >= 5 && isstruct(commStats)
    hasLinkFields = isfield(commStats, 'droppedByBandwidth') && ...
        isfield(commStats, 'droppedByLink') && isfield(commStats, 'droppedByOutage');
    if hasLinkFields && t <= size(commStats.droppedByBandwidth, 2)
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
end

rawScore = covScore .* innovationScore .* linkQuality;
if all(rawScore == 0)
    rawScore = ones(1, numSensors);
end
rawWeights = rawScore / sum(rawScore);

weights = rawWeights;
if nargin >= 6 && isstruct(prevWeights) && isfield(prevWeights, 'ga')
    if numel(prevWeights.ga) == numSensors
        weights = emaAlpha * prevWeights.ga + (1 - emaAlpha) * rawWeights;
    end
end

if minWeight > 0
    weights = max(weights, minWeight);
    weights = weights / sum(weights);
end

gaWeights = weights;
aaWeights = weights;

debug = struct();
debug.covScore = covScore;
debug.innovationScore = innovationScore;
debug.linkQuality = linkQuality;
debug.rawScore = rawScore;
debug.rawWeights = rawWeights;
debug.weights = weights;
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
