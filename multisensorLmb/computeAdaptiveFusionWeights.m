function [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
% COMPUTEADAPTIVEFUSIONWEIGHTS - Compute adaptive GA/AA fusion weights.
%   [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
%
%   The weight model is factorized as
%       mask * covariance * link * cardinalityConsensus * existenceConfidence * associationAmbiguity * freshness * nisPenalty * history
%   where NIS acts as a consistency penalty instead of a quality reward.
%   The final weights are then normalized with temporal EMA smoothing.

numSensors = model.numberOfSensors;

cfg = struct();
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    cfg = model.adaptiveFusion;
end

method = lower(getField(cfg, 'method', 'factorized'));
emaAlpha = getField(cfg, 'emaAlpha', 0.7);
minWeight = getField(cfg, 'minWeight', 0.0);
useDecoupledKla = getField(cfg, 'useDecoupledKla', false);
useNIS = getField(cfg, 'useNIS', true);
useHistory = getField(cfg, 'useHistory', true);
useCovariance = getField(cfg, 'useCovariance', true);
useLinkQuality = getField(cfg, 'useLinkQuality', true);
useCardinalityConsensus = getField(cfg, 'useCardinalityConsensus', false);
useExistenceConfidence = getField(cfg, 'useExistenceConfidence', false);
useAssociationAmbiguity = getField(cfg, 'useAssociationAmbiguity', false);
spatialEmaAlpha = getField(cfg, 'spatialEmaAlpha', emaAlpha);
existenceEmaAlpha = getField(cfg, 'existenceEmaAlpha', emaAlpha);
spatialMinWeight = getField(cfg, 'spatialMinWeight', minWeight);
existenceMinWeight = getField(cfg, 'existenceMinWeight', minWeight);
spatialCovariancePower = max(getField(cfg, 'spatialCovariancePower', 1.0), 0);
spatialLinkQualityPower = max(getField(cfg, 'spatialLinkQualityPower', 1.0), 0);
existenceLinkQualityPower = max(getField(cfg, 'existenceLinkQualityPower', 1.0), 0);
existenceConfidenceWeightPower = max(getField(cfg, 'existenceConfidenceWeightPower', 1.0), 0);
spatialDecouplingStrength = min(max(getField(cfg, 'spatialDecouplingStrength', 1.0), 0), 1);
existenceDecouplingStrength = min(max(getField(cfg, 'existenceDecouplingStrength', 1.0), 0), 1);
spatialStructureStrength = max(getField(cfg, 'spatialStructureStrength', 0.0), 0);
existenceStructureStrength = max(getField(cfg, 'existenceStructureStrength', 0.0), 0);
structureReliabilityPower = max(getField(cfg, 'structureReliabilityPower', 0.0), 0);
structureReliabilityMinScore = min(max(getField(cfg, 'structureReliabilityMinScore', 0.25), 0), 1);
useStructureAwareKla = getField(cfg, 'useStructureAwareKla', false) || ...
    spatialStructureStrength > 0 || existenceStructureStrength > 0;
usePosteriorStructureConsistency = getField(cfg, 'usePosteriorStructureConsistency', true);

availabilityMask = resolveAvailabilityMask(model, commStats, t, numSensors);
if isFidFiaMethod(method)
    [gaWeights, aaWeights, debug] = computeFidFiaFusionWeights( ...
        measurementUpdatedDistributions, model, t, cfg, availabilityMask, prevWeights);
    return;
end

covScore = computeCovarianceScore(measurementUpdatedDistributions, model);
innovationPenalty = resolveInnovationPenalty(commStats, t, numSensors, useNIS);
cardinalityConsensusScore = resolveCardinalityConsensusScore( ...
    measurementUpdatedDistributions, useCardinalityConsensus, cfg);
existenceConfidenceScore = resolveExistenceConfidenceScore( ...
    measurementUpdatedDistributions, useExistenceConfidence, cfg);
associationAmbiguityScore = resolveAssociationAmbiguityScore( ...
    commStats, t, numSensors, useAssociationAmbiguity, cfg);
freshnessScore = resolveFreshnessScore(measurements, t, numSensors, cfg);
linkQuality = computeLinkQuality(measurements, commStats, t, numSensors);
[covScore, linkQuality] = applyFactorMasks(covScore, linkQuality, useCovariance, useLinkQuality);
[historyScore, historyState, historyDebug] = computeHistoryScore( ...
    measurementUpdatedDistributions, covScore, innovationPenalty, cfg, prevWeights, useNIS, useHistory);

baseScore = availabilityMask .* covScore .* linkQuality;
rawScore = baseScore .* cardinalityConsensusScore .* existenceConfidenceScore .* ...
    associationAmbiguityScore .* ...
    freshnessScore .* innovationPenalty .* historyScore;

if useDecoupledKla
    spatialDedicatedScore = availabilityMask .* (covScore .^ spatialCovariancePower) .* ...
        (linkQuality .^ spatialLinkQualityPower) .* associationAmbiguityScore .* ...
        innovationPenalty .* historyScore;
    existenceDedicatedScore = availabilityMask .* (linkQuality .^ existenceLinkQualityPower) .* ...
        (existenceConfidenceScore .^ existenceConfidenceWeightPower) .* ...
        cardinalityConsensusScore .* freshnessScore .* innovationPenalty .* historyScore;
    spatialScore = blendDecoupledScore(rawScore, spatialDedicatedScore, spatialDecouplingStrength);
    existenceScore = blendDecoupledScore(rawScore, existenceDedicatedScore, existenceDecouplingStrength);
    spatialStructurePrior = resolveStructurePrior(model, 'gaSpatialStructurePrior', 'gaTopologyWeights', numSensors);
    existenceStructurePrior = resolveStructurePrior(model, 'gaExistenceStructurePrior', 'gaTopologyWeights', numSensors);
    communicationReliabilityPrior = ones(1, numSensors);
    if structureReliabilityPower > 0
        communicationReliabilityPrior = resolveCommunicationReliabilityPrior( ...
            commStats, numSensors, structureReliabilityMinScore);
        spatialStructurePrior = applyStructurePrior( ...
            spatialStructurePrior, communicationReliabilityPrior, structureReliabilityPower);
        existenceStructurePrior = applyStructurePrior( ...
            existenceStructurePrior, communicationReliabilityPrior, structureReliabilityPower);
    end
    spatialStructureScore = ones(1, numSensors);
    existenceStructureScore = ones(1, numSensors);
    if useStructureAwareKla
        if usePosteriorStructureConsistency
            [spatialStructureScore, existenceStructureScore] = resolveStructureConsistencyScores( ...
                measurementUpdatedDistributions, model, spatialStructurePrior, existenceStructurePrior, cfg);
            spatialScore = spatialScore .* (spatialStructureScore .^ spatialStructureStrength);
            existenceScore = existenceScore .* (existenceStructureScore .^ existenceStructureStrength);
        else
            spatialScore = applyStructurePrior(spatialScore, spatialStructurePrior, spatialStructureStrength);
            existenceScore = applyStructurePrior(existenceScore, existenceStructurePrior, existenceStructureStrength);
            spatialStructureScore = spatialStructurePrior;
            existenceStructureScore = existenceStructurePrior;
        end
    end

    spatialPrev = resolvePreviousWeights(prevWeights, 'gaSpatial', 'ga', numSensors);
    existencePrev = resolvePreviousWeights(prevWeights, 'gaExistence', 'ga', numSensors);

    spatialWeights = finalizeAdaptiveWeights(spatialScore, availabilityMask, spatialPrev, ...
        spatialEmaAlpha, spatialMinWeight);
    existenceWeights = finalizeAdaptiveWeights(existenceScore, availabilityMask, existencePrev, ...
        existenceEmaAlpha, existenceMinWeight);

    gaWeights = spatialWeights;
    aaWeights = spatialWeights;
    rawWeights = spatialWeights;
else
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
    spatialWeights = weights;
    existenceWeights = weights;
end

debug = struct();
debug.availabilityMask = availabilityMask;
debug.covScore = covScore;
debug.baseScore = baseScore;
debug.innovationPenalty = innovationPenalty;
debug.innovationScore = innovationPenalty;
debug.cardinalityConsensusScore = cardinalityConsensusScore;
debug.existenceConfidenceScore = existenceConfidenceScore;
debug.associationAmbiguityScore = associationAmbiguityScore;
debug.freshnessScore = freshnessScore;
debug.historyScore = historyScore;
debug.linkQuality = linkQuality;
debug.rawScore = rawScore;
debug.rawWeights = rawWeights;
debug.weights = gaWeights;
debug.useDecoupledKla = useDecoupledKla;
debug.useStructureAwareKla = useStructureAwareKla;
debug.usePosteriorStructureConsistency = usePosteriorStructureConsistency;
debug.spatialRawScore = rawScore;
debug.existenceRawScore = rawScore;
debug.spatialStructurePrior = ones(1, numSensors);
debug.existenceStructurePrior = ones(1, numSensors);
debug.communicationReliabilityPrior = ones(1, numSensors);
debug.spatialStructureScore = ones(1, numSensors);
debug.existenceStructureScore = ones(1, numSensors);
if useDecoupledKla
    debug.spatialRawScore = spatialScore;
    debug.existenceRawScore = existenceScore;
    debug.spatialStructurePrior = spatialStructurePrior;
    debug.existenceStructurePrior = existenceStructurePrior;
    debug.communicationReliabilityPrior = communicationReliabilityPrior;
    debug.spatialStructureScore = spatialStructureScore;
    debug.existenceStructureScore = existenceStructureScore;
end
debug.gaSpatialWeights = spatialWeights;
debug.aaSpatialWeights = spatialWeights;
debug.gaExistenceWeights = existenceWeights;
debug.aaExistenceWeights = existenceWeights;
debug.historyState = historyState;
debug.historyInstantInstability = historyDebug.instantInstability;
debug.historyInstabilityEma = historyDebug.instabilityEma;
debug.expectedCardinality = historyDebug.expectedCardinality;
end

function tf = isFidFiaMethod(method)
tf = any(strcmpi(method, {'fidfia', 'fid_fia', 'fisherfia', ...
    'fisher_fia', 'informationgeometry', 'information_geometry', ...
    'caozhao', 'cao_zhao'}));
end

function [gaWeights, aaWeights, debug] = computeFidFiaFusionWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask, prevWeights)

numSensors = numel(measurementUpdatedDistributions);
[fiaScore, fidPairCounts] = computeFidFiaScore(measurementUpdatedDistributions, model, t, cfg);
rawWeights = normalizeScores(fiaScore, availabilityMask);

weights = rawWeights;
useEma = getField(cfg, 'fidFiaUseEma', false);
if useEma && nargin >= 6 && isstruct(prevWeights) && isfield(prevWeights, 'ga') && ...
        numel(prevWeights.ga) == numSensors
    emaAlpha = getField(cfg, 'fidFiaEmaAlpha', getField(cfg, 'emaAlpha', 0.7));
    weights = emaAlpha * prevWeights.ga + (1 - emaAlpha) * rawWeights;
    weights = normalizeScores(weights, availabilityMask);
end

minWeight = getField(cfg, 'fidFiaMinWeight', 0.0);
if minWeight > 0
    weights = enforceMinimumWeight(weights, availabilityMask, minWeight);
end

gaWeights = weights;
aaWeights = weights;

debug = struct();
debug.availabilityMask = availabilityMask;
debug.covScore = ones(1, numSensors);
debug.baseScore = fiaScore;
debug.innovationPenalty = ones(1, numSensors);
debug.innovationScore = debug.innovationPenalty;
debug.cardinalityConsensusScore = ones(1, numSensors);
debug.existenceConfidenceScore = ones(1, numSensors);
debug.associationAmbiguityScore = ones(1, numSensors);
debug.freshnessScore = ones(1, numSensors);
debug.historyScore = ones(1, numSensors);
debug.linkQuality = ones(1, numSensors);
debug.rawScore = fiaScore;
debug.rawWeights = rawWeights;
debug.weights = weights;
debug.method = 'fidFia';
debug.fiaScore = fiaScore;
debug.fidPairCounts = fidPairCounts;
debug.useDecoupledKla = false;
debug.useStructureAwareKla = false;
debug.usePosteriorStructureConsistency = false;
debug.spatialRawScore = fiaScore;
debug.existenceRawScore = fiaScore;
debug.spatialStructurePrior = ones(1, numSensors);
debug.existenceStructurePrior = ones(1, numSensors);
debug.communicationReliabilityPrior = ones(1, numSensors);
debug.spatialStructureScore = ones(1, numSensors);
debug.existenceStructureScore = ones(1, numSensors);
debug.gaSpatialWeights = weights;
debug.aaSpatialWeights = weights;
debug.gaExistenceWeights = weights;
debug.aaExistenceWeights = weights;
debug.historyState = struct();
debug.historyInstantInstability = zeros(1, numSensors);
debug.historyInstabilityEma = zeros(1, numSensors);
debug.expectedCardinality = computeExpectedCardinality(measurementUpdatedDistributions);
end

function [fiaScore, fidPairCounts] = computeFidFiaScore(measurementUpdatedDistributions, model, t, cfg)
numSensors = numel(measurementUpdatedDistributions);
fiaScore = zeros(1, numSensors);
fidPairCounts = zeros(1, numSensors);

existenceThreshold = getField(cfg, 'fidFiaExistenceThreshold', getField(model, 'existenceThreshold', 0));
useExistenceWeight = getField(cfg, 'fidFiaUseExistenceWeight', true);
existencePower = max(getField(cfg, 'fidFiaExistencePower', 1.0), 0);

for s = 1:numSensors
    objects = measurementUpdatedDistributions{s};
    if isempty(objects)
        continue;
    end

    [states, existenceProb] = extractFidFiaTargetStates(objects, model, existenceThreshold);
    numTargets = size(states, 2);
    if numTargets < 2
        continue;
    end

    for i = 1:(numTargets - 1)
        for j = (i + 1):numTargets
            pairWeight = 1;
            if useExistenceWeight
                pairWeight = (max(existenceProb(i), 0) * max(existenceProb(j), 0)) ^ existencePower;
            end
            if pairWeight <= 0
                continue;
            end

            fidValue = approximateLinearGaussianFid(model, s, states(:, i), states(:, j), t, cfg);
            if isfinite(fidValue) && fidValue > 0
                fiaScore(s) = fiaScore(s) + pairWeight * fidValue;
                fidPairCounts(s) = fidPairCounts(s) + 1;
            end
        end
    end
end
end

function [states, existenceProb] = extractFidFiaTargetStates(objects, model, existenceThreshold)
states = zeros(model.xDimension, numel(objects));
existenceProb = zeros(1, numel(objects));
count = 0;
for i = 1:numel(objects)
    if objects(i).numberOfGmComponents < 1 || objects(i).r <= existenceThreshold
        continue;
    end
    [mu, ~] = mprojection(model.xDimension, objects(i));
    count = count + 1;
    states(:, count) = mu;
    existenceProb(count) = objects(i).r;
end
states = states(:, 1:count);
existenceProb = existenceProb(1:count);
end

function fidValue = approximateLinearGaussianFid(model, sensorIdx, stateA, stateB, t, cfg)
fidValue = 0;
if isempty(stateA) || isempty(stateB) || numel(stateA) < 2 || numel(stateB) < 2
    return;
end

deltaState = stateB(:) - stateA(:);
deltaPos = deltaState(1:2);
if norm(deltaPos) <= eps
    return;
end

numPoints = max(1, round(getField(cfg, 'fidFiaQuadraturePoints', 3)));
useDetectionProbability = getField(cfg, 'fidFiaUseDetectionProbability', true);
Hpos = resolvePositionMeasurementJacobian(model, sensorIdx);
if isempty(Hpos)
    return;
end

if numPoints == 1
    lambdas = 0.5;
else
    lambdas = linspace(0, 1, numPoints);
end
integrand = zeros(size(lambdas));

for idx = 1:numel(lambdas)
    state = stateA(:) + lambdas(idx) * deltaState;
    [pdSensor, measurementCovariance] = evaluateSensorQuality(model, sensorIdx, state, t);
    if useDetectionProbability
        if pdSensor <= 0
            integrand(idx) = 0;
            continue;
        end
        detectionScale = sqrt(max(pdSensor, 0));
    else
        detectionScale = 1;
    end

    measurementCovariance = regularizeCovariance(measurementCovariance);
    fisherMetric = Hpos' * (measurementCovariance \ Hpos);
    metricDistanceSquared = deltaPos' * fisherMetric * deltaPos;
    integrand(idx) = detectionScale * sqrt(max(real(metricDistanceSquared), 0));
end

if numPoints == 1
    fidValue = integrand(1);
else
    fidValue = trapz(lambdas, integrand);
end
end

function Hpos = resolvePositionMeasurementJacobian(model, sensorIdx)
Hpos = [];
if ~isfield(model, 'C') || numel(model.C) < sensorIdx || isempty(model.C{sensorIdx})
    return;
end
C = model.C{sensorIdx};
if size(C, 2) < 2
    return;
end
Hpos = C(:, 1:2);
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

function prior = resolveStructurePrior(model, preferredField, fallbackField, numSensors)
prior = ones(1, numSensors);
if nargin < 1 || ~isstruct(model)
    return;
end
if isfield(model, preferredField) && numel(model.(preferredField)) == numSensors
    prior = reshape(model.(preferredField), 1, []);
elseif isfield(model, fallbackField) && numel(model.(fallbackField)) == numSensors
    prior = reshape(model.(fallbackField), 1, []);
end
prior = max(prior, eps);
prior = prior / mean(prior);
end

function adjustedScore = applyStructurePrior(score, prior, strength)
adjustedScore = score;
if nargin < 3 || strength <= 0 || isempty(prior)
    return;
end
adjustedScore = score .* (prior .^ strength);
end

function [spatialScore, existenceScore] = resolveStructureConsistencyScores( ...
    measurementUpdatedDistributions, model, spatialPrior, existencePrior, cfg)

numSensors = numel(measurementUpdatedDistributions);
spatialScore = ones(1, numSensors);
existenceScore = ones(1, numSensors);
if numSensors <= 1
    return;
end

spatialScale = max(getField(cfg, 'spatialConsistencyScale', 0.6), 0);
existenceScale = max(getField(cfg, 'existenceConsistencyScale', 2.0), 0);
spatialMinScore = min(max(getField(cfg, 'spatialConsistencyMinScore', 0.4), 0), 1);
existenceMinScore = min(max(getField(cfg, 'existenceConsistencyMinScore', 0.4), 0), 1);
summaries = repmat(struct( ...
    'r', [], ...
    'position', zeros(2, 0), ...
    'trace', [], ...
    'center', zeros(2, 1), ...
    'dispersion', 0), 1, numSensors);
for s = 1:numSensors
    summaries(s) = buildStructureSummary(measurementUpdatedDistributions{s}, model);
end

for s = 1:numSensors
    spatialWeights = reshape(spatialPrior, 1, []);
    existenceWeights = reshape(existencePrior, 1, []);
    spatialWeights(s) = 0;
    existenceWeights(s) = 0;

    spatialWeightSum = sum(spatialWeights);
    existenceWeightSum = sum(existenceWeights);
    spatialDisagreement = 0;
    existenceDisagreement = 0;

    for j = 1:numSensors
        if j == s
            continue;
        end
        [pairSpatial, pairExistence] = computePairwiseStructureDisagreement( ...
            summaries(s), summaries(j));
        spatialDisagreement = spatialDisagreement + spatialWeights(j) * pairSpatial;
        existenceDisagreement = existenceDisagreement + existenceWeights(j) * pairExistence;
    end

    if spatialWeightSum > 0
        spatialDisagreement = spatialDisagreement / spatialWeightSum;
        spatialScore(s) = spatialMinScore + (1 - spatialMinScore) * ...
            exp(-spatialScale * spatialDisagreement);
    end
    if existenceWeightSum > 0
        existenceDisagreement = existenceDisagreement / existenceWeightSum;
        existenceScore(s) = existenceMinScore + (1 - existenceMinScore) * ...
            exp(-existenceScale * existenceDisagreement);
    end
end
end

function summary = buildStructureSummary(objects, model)
summary = struct('r', [], 'position', zeros(2, 0), 'trace', [], 'center', zeros(2, 1), 'dispersion', 0);
if isempty(objects) || nargin < 2 || ~isstruct(model) || ~isfield(model, 'xDimension')
    return;
end

numObjects = numel(objects);
summary.r = extractExistenceVector(objects);
summary.position = zeros(2, numObjects);
summary.trace = zeros(1, numObjects);
for idx = 1:numObjects
    if objects(idx).numberOfGmComponents < 1
        continue;
    end
    [mu, cov] = mprojection(model.xDimension, objects(idx));
    posDim = min(2, numel(mu));
    if posDim > 0
        summary.position(1:posDim, idx) = mu(1:posDim);
    end
    summary.trace(idx) = trace(cov);
end

weights = max(summary.r, 0);
activeMask = (weights > 0) & (summary.trace > 0);
if any(activeMask)
    activeWeights = weights(activeMask);
    activePositions = summary.position(:, activeMask);
    activeTrace = summary.trace(activeMask);
    totalWeight = sum(activeWeights);
    summary.center = activePositions * (activeWeights(:) / max(totalWeight, eps));
    centeredPositions = activePositions - summary.center;
    radialSpread = sum(centeredPositions .^ 2, 1);
    summary.dispersion = sum(activeWeights .* (radialSpread + activeTrace)) / max(totalWeight, eps);
end
end

function [spatialDisagreement, existenceDisagreement] = computePairwiseStructureDisagreement(summaryA, summaryB)
spatialDisagreement = 0;
existenceDisagreement = 0;
rA = summaryA.r;
rB = summaryB.r;
maxObjects = max(numel(rA), numel(rB));
if maxObjects == 0
    return;
end

rA = padVector(rA, maxObjects);
rB = padVector(rB, maxObjects);
traceA = padVector(summaryA.trace, maxObjects);
traceB = padVector(summaryB.trace, maxObjects);

if summaryA.dispersion > 0 && summaryB.dispersion > 0
    centerDelta = summaryA.center - summaryB.center;
    spatialScale = 1 + summaryA.dispersion + summaryB.dispersion;
    centerMismatch = log(1 + (centerDelta' * centerDelta) / max(spatialScale, eps));
    spreadMismatch = abs(log((summaryA.dispersion + eps) / (summaryB.dispersion + eps)));
    spatialDisagreement = centerMismatch + 0.35 * spreadMismatch;
end

profileDiff = mean(abs(rA - rB));
expectedCardA = sum(rA);
expectedCardB = sum(rB);
expectedCardNorm = 1 + 0.5 * (expectedCardA + expectedCardB);
expectedCardDiff = abs(expectedCardA - expectedCardB) / max(expectedCardNorm, eps);
confidenceDiff = abs(computeExistenceConfidence(rA) - computeExistenceConfidence(rB));
existenceDisagreement = 0.6 * profileDiff + 0.3 * expectedCardDiff + 0.1 * confidenceDiff;
end

function confidence = computeExistenceConfidence(existenceProb)
if isempty(existenceProb)
    confidence = 0;
    return;
end
certainty = abs(2 * existenceProb - 1);
confidence = sum(existenceProb .* certainty) / (eps + sum(existenceProb));
confidence = min(max(confidence, 0), 1);
end

function values = extractExistenceVector(objects)
if isempty(objects)
    values = [];
    return;
end
values = reshape([objects.r], 1, []);
end

function padded = padVector(values, targetLength)
padded = zeros(1, targetLength);
if isempty(values)
    return;
end
count = min(numel(values), targetLength);
padded(1:count) = reshape(values(1:count), 1, []);
end

function padded = padMatrix(values, rowCount, targetColumns)
padded = zeros(rowCount, targetColumns);
if isempty(values)
    return;
end
copyRows = min(size(values, 1), rowCount);
copyCols = min(size(values, 2), targetColumns);
padded(1:copyRows, 1:copyCols) = values(1:copyRows, 1:copyCols);
end

function prior = resolveCommunicationReliabilityPrior(commStats, numSensors, minScore)
prior = ones(1, numSensors);
if nargin < 1 || ~isstruct(commStats) || ~isfield(commStats, 'pDropBySensor')
    return;
end
if numel(commStats.pDropBySensor) ~= numSensors
    return;
end
reliability = 1 - reshape(commStats.pDropBySensor, 1, []);
reliability = min(max(reliability, 0), 1);
prior = minScore + (1 - minScore) * reliability;
prior = prior / mean(prior);
end

function prev = resolvePreviousWeights(prevWeights, preferredField, fallbackField, numSensors)
prev = [];
if nargin < 1 || ~isstruct(prevWeights)
    return;
end
if isfield(prevWeights, preferredField) && numel(prevWeights.(preferredField)) == numSensors
    prev = prevWeights.(preferredField);
    return;
end
if isfield(prevWeights, fallbackField) && numel(prevWeights.(fallbackField)) == numSensors
    prev = prevWeights.(fallbackField);
end
end

function weights = finalizeAdaptiveWeights(score, mask, prev, emaAlpha, minWeight)
weights = normalizeScores(score, mask);
if ~isempty(prev)
    weights = emaAlpha * prev + (1 - emaAlpha) * weights;
    weights = normalizeScores(weights, mask);
end
if minWeight > 0
    weights = enforceMinimumWeight(weights, mask, minWeight);
end
end

function blendedScore = blendDecoupledScore(anchorScore, dedicatedScore, strength)
if strength <= 0
    blendedScore = anchorScore;
    return;
end
if strength >= 1
    blendedScore = dedicatedScore;
    return;
end
blendedScore = (anchorScore .^ (1 - strength)) .* (dedicatedScore .^ strength);
end

function cardinalityConsensusScore = resolveCardinalityConsensusScore(measurementUpdatedDistributions, useCardinalityConsensus, cfg)
numSensors = numel(measurementUpdatedDistributions);
cardinalityConsensusScore = ones(1, numSensors);
if ~useCardinalityConsensus
    return;
end

expectedCardinality = computeExpectedCardinality(measurementUpdatedDistributions);
activeMask = expectedCardinality > 0;
if ~any(activeMask)
    return;
end

referenceCardinality = median(expectedCardinality(activeMask));
scoreScale = max(getField(cfg, 'cardinalityConsensusScale', 4.0), 0);
minScore = min(max(getField(cfg, 'cardinalityConsensusMinScore', 0.4), 0), 1);
normalizer = 1 + referenceCardinality;

for s = 1:numSensors
    diffRatio = abs(expectedCardinality(s) - referenceCardinality) / normalizer;
    cardinalityConsensusScore(s) = minScore + (1 - minScore) * exp(-scoreScale * diffRatio);
end
end

function existenceConfidenceScore = resolveExistenceConfidenceScore(measurementUpdatedDistributions, useExistenceConfidence, cfg)
numSensors = numel(measurementUpdatedDistributions);
existenceConfidenceScore = ones(1, numSensors);
if ~useExistenceConfidence
    return;
end

minScore = min(max(getField(cfg, 'existenceConfidenceMinScore', 0.6), 0), 1);
power = max(getField(cfg, 'existenceConfidencePower', 1.0), 0);

for s = 1:numSensors
    objects = measurementUpdatedDistributions{s};
    if isempty(objects)
        existenceConfidenceScore(s) = 1;
        continue;
    end

    existenceProb = [objects.r];
    if isempty(existenceProb)
        existenceConfidenceScore(s) = 1;
        continue;
    end

    certainty = abs(2 * existenceProb - 1);
    weightedConfidence = sum(existenceProb .* certainty) / (eps + sum(existenceProb));
    weightedConfidence = min(max(weightedConfidence, 0), 1);
    existenceConfidenceScore(s) = minScore + (1 - minScore) * (weightedConfidence ^ power);
end
end

function associationAmbiguityScore = resolveAssociationAmbiguityScore(commStats, t, numSensors, useAssociationAmbiguity, cfg)
associationAmbiguityScore = ones(1, numSensors);
if ~useAssociationAmbiguity
    return;
end
if nargin < 1 || ~isstruct(commStats)
    return;
end

rawScore = [];
if isfield(commStats, 'associationAmbiguityScore')
    values = commStats.associationAmbiguityScore;
    if ismatrix(values) && size(values, 1) == numSensors && size(values, 2) >= t
        rawScore = values(:, t)';
    elseif isvector(values) && numel(values) == numSensors
        rawScore = reshape(values, 1, []);
    end
elseif isfield(commStats, 'associationConfidence')
    values = commStats.associationConfidence;
    if ismatrix(values) && size(values, 1) == numSensors && size(values, 2) >= t
        rawScore = values(:, t)';
    elseif isvector(values) && numel(values) == numSensors
        rawScore = reshape(values, 1, []);
    end
end

if isempty(rawScore)
    return;
end

rawScore = min(max(rawScore, 0), 1);
minScore = min(max(getField(cfg, 'associationAmbiguityMinScore', 0.85), 0), 1);
power = max(getField(cfg, 'associationAmbiguityPower', 1.0), 0);
associationAmbiguityScore = minScore + (1 - minScore) * (rawScore .^ power);
end

function [covScore, linkQuality] = applyFactorMasks(covScore, linkQuality, useCovariance, useLinkQuality)
if ~useCovariance
    covScore = ones(size(covScore));
end
if ~useLinkQuality
    linkQuality = ones(size(linkQuality));
end
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

function innovationPenalty = resolveInnovationPenalty(commStats, t, numSensors, useNIS)
innovationPenalty = ones(1, numSensors);
if ~useNIS
    return;
end
if nargin < 2 || ~isstruct(commStats) || ~isfield(commStats, 'innovationConsistency')
    return;
end
if size(commStats.innovationConsistency, 1) == numSensors && size(commStats.innovationConsistency, 2) >= t
    innovationPenalty = commStats.innovationConsistency(:, t)';
end
end

function freshnessScore = resolveFreshnessScore(measurements, t, numSensors, cfg)
freshnessScore = ones(1, numSensors);
useFreshness = getField(cfg, 'useFreshness', false);
if ~useFreshness
    return;
end
if nargin < 1 || ~iscell(measurements) || isempty(measurements)
    return;
end

freshnessDecay = max(getField(cfg, 'freshnessDecay', 0.5), 0);
freshnessMinScore = min(max(getField(cfg, 'freshnessMinScore', 0.4), 0), 1);

numSteps = size(measurements, 2);
currentStep = min(max(round(t), 1), numSteps);
numSensorsLocal = min(numSensors, size(measurements, 1));

for s = 1:numSensorsLocal
    lastObservedStep = 0;
    for tau = currentStep:-1:1
        if numel(measurements{s, tau}) > 0
            lastObservedStep = tau;
            break;
        end
    end

    if lastObservedStep <= 0
        age = currentStep;
    else
        age = currentStep - lastObservedStep;
    end

    freshnessScore(s) = freshnessMinScore + (1 - freshnessMinScore) * ...
        exp(-freshnessDecay * age);
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
