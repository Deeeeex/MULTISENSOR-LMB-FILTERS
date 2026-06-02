function [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
% COMPUTEADAPTIVEFUSIONWEIGHTS - Compute adaptive GA/AA fusion weights.
%   [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights(measurementUpdatedDistributions, measurements, model, t, commStats, prevWeights)
%
%   The weight model is factorized as
%       mask * covariance * link * cardinalityConsensus * existenceConfidence * associationAmbiguity * freshness/informationDecay * nisPenalty * history
%   where NIS acts as a consistency penalty instead of a quality reward.
%   The final weights are then normalized with temporal EMA smoothing.
%   File guide:
%       Adaptive weighting engine for GA/AA fusion. It supports the general
%       factorized mode plus direct PD-weighted, FI-weighted, and FID-FIA
%       baselines, and returns both fusion weights and debug factors for
%       experiment reports.

numSensors = model.numberOfSensors;

% Read feature flags and hyperparameters once at the top. The helper
% functions below should stay side-effect free, except for debug state.
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
useCtFiDecay = getField(cfg, 'useCtFiDecay', false) || ...
    getField(cfg, 'useInformationDecay', false);
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
useFidFiaExistence = getField(cfg, 'useFidFiaExistence', false);

availabilityMask = resolveAvailabilityMask(model, commStats, t, numSensors);
% Direct baseline modes bypass the general factor product so their reported
% weights remain faithful to the named PD/FI/FID-FIA scoring rule.
if isFidFiaMethod(method)
    [gaWeights, aaWeights, debug] = computeFidFiaFusionWeights( ...
        measurementUpdatedDistributions, model, t, cfg, availabilityMask, prevWeights);
    return;
end
if isFiTraceGaMethod(method)
    [gaWeights, aaWeights, debug] = computeFiTraceGaFusionWeights( ...
        measurementUpdatedDistributions, model, t, cfg, availabilityMask);
    return;
end
if isPdWeightedGaMethod(method)
    [gaWeights, aaWeights, debug] = computePdWeightedGaFusionWeights( ...
        measurementUpdatedDistributions, model, t, cfg, availabilityMask);
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
ctFiDecayScore = resolveCtFiDecayScore( ...
    measurementUpdatedDistributions, model, t, commStats, cfg, useCtFiDecay);
linkQuality = computeLinkQuality(measurements, commStats, t, numSensors);
[covScore, linkQuality] = applyFactorMasks(covScore, linkQuality, useCovariance, useLinkQuality);
[historyScore, historyState, historyDebug] = computeHistoryScore( ...
    measurementUpdatedDistributions, covScore, innovationPenalty, cfg, prevWeights, useNIS, useHistory);
[fidFiaExistenceScore, fidFiaScore, fidFiaPairCounts] = resolveFidFiaExistenceScore( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask, useFidFiaExistence);

% General adaptive mode: multiply comparable scores, then normalize only at
% the end. This keeps the debug fields interpretable as individual factors.
baseScore = availabilityMask .* covScore .* linkQuality;
rawScore = baseScore .* cardinalityConsensusScore .* existenceConfidenceScore .* ...
    associationAmbiguityScore .* ...
    freshnessScore .* ctFiDecayScore .* innovationPenalty .* historyScore;

if useDecoupledKla
    % Decoupled mode lets spatial KLA weights and existence/cardinality
    % weights react to different factor subsets while still sharing the
    % same availability mask and temporal smoothing machinery.
    spatialDedicatedScore = availabilityMask .* (covScore .^ spatialCovariancePower) .* ...
        (linkQuality .^ spatialLinkQualityPower) .* associationAmbiguityScore .* ...
        ctFiDecayScore .* innovationPenalty .* historyScore;
    existenceDedicatedScore = availabilityMask .* (linkQuality .^ existenceLinkQualityPower) .* ...
        (existenceConfidenceScore .^ existenceConfidenceWeightPower) .* ...
        cardinalityConsensusScore .* freshnessScore .* ctFiDecayScore .* innovationPenalty .* historyScore;
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
    if useFidFiaExistence
        fidFiaExistenceStrength = max(getField(cfg, 'fidFiaExistenceStrength', 0.5), 0);
        existenceScore = existenceScore .* (fidFiaExistenceScore .^ fidFiaExistenceStrength);
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

% The debug struct is intentionally verbose: report scripts use it to audit
% which factors drove the selected weights at every time step.
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
debug.ctFiDecayScore = ctFiDecayScore;
debug.useCtFiDecay = useCtFiDecay;
debug.historyScore = historyScore;
debug.linkQuality = linkQuality;
debug.rawScore = rawScore;
debug.rawWeights = rawWeights;
debug.weights = gaWeights;
debug.useDecoupledKla = useDecoupledKla;
debug.useStructureAwareKla = useStructureAwareKla;
debug.usePosteriorStructureConsistency = usePosteriorStructureConsistency;
debug.useFidFiaExistence = useFidFiaExistence;
debug.fidFiaExistenceScore = fidFiaExistenceScore;
debug.fidFiaScore = fidFiaScore;
debug.fidFiaPairCounts = fidFiaPairCounts;
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

function tf = isFiTraceGaMethod(method)
tf = any(strcmpi(method, {'fitracega', 'fi_trace_ga', 'fi-weighted-ga', ...
    'fiweightedga', 'fi_weighted_ga', 'fishertracega', ...
    'fisher_trace_ga'}));
end

function tf = isPdWeightedGaMethod(method)
tf = any(strcmpi(method, {'pdweightedga', 'pd_weighted_ga', ...
    'pd-weighted-ga', 'detectionweightedga', 'detection_weighted_ga'}));
end

function [gaWeights, aaWeights, debug] = computeFiTraceGaFusionWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask)

numSensors = numel(measurementUpdatedDistributions);
[targetWeights, sensorScores, targetScores] = computeTargetWiseFiTraceWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask);
weights = normalizeScores(sensorScores, availabilityMask);

gaWeights = weights;
aaWeights = weights;
debug = buildDirectWeightDebug( ...
    measurementUpdatedDistributions, availabilityMask, sensorScores, weights, 'fiTraceGa');
debug.fiTraceScore = sensorScores;
debug.fiTraceTargetScores = targetScores;
debug.gaTargetWiseWeights = targetWeights;
debug.aaTargetWiseWeights = targetWeights;
debug.gaSpatialWeights = weights;
debug.aaSpatialWeights = weights;
debug.gaExistenceWeights = weights;
debug.aaExistenceWeights = weights;
debug.useTargetWiseFiWeights = true;
debug.numberOfSensors = numSensors;
end

function [gaWeights, aaWeights, debug] = computePdWeightedGaFusionWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask)

[targetWeights, sensorScores, targetScores] = computeTargetWisePdWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask);
weights = normalizeScores(sensorScores, availabilityMask);

gaWeights = weights;
aaWeights = weights;
debug = buildDirectWeightDebug( ...
    measurementUpdatedDistributions, availabilityMask, sensorScores, weights, 'pdWeightedGa');
debug.pdScore = sensorScores;
debug.pdTargetScores = targetScores;
debug.gaTargetWiseWeights = targetWeights;
debug.aaTargetWiseWeights = targetWeights;
debug.gaSpatialWeights = weights;
debug.aaSpatialWeights = weights;
debug.gaExistenceWeights = weights;
debug.aaExistenceWeights = weights;
debug.useTargetWiseFiWeights = false;
end

function debug = buildDirectWeightDebug(measurementUpdatedDistributions, availabilityMask, rawScore, weights, methodName)
numSensors = numel(rawScore);
debug = struct();
debug.availabilityMask = availabilityMask;
debug.covScore = ones(1, numSensors);
debug.baseScore = rawScore;
debug.innovationPenalty = ones(1, numSensors);
debug.innovationScore = debug.innovationPenalty;
debug.cardinalityConsensusScore = ones(1, numSensors);
debug.existenceConfidenceScore = ones(1, numSensors);
debug.associationAmbiguityScore = ones(1, numSensors);
debug.freshnessScore = ones(1, numSensors);
debug.ctFiDecayScore = ones(1, numSensors);
debug.useCtFiDecay = false;
debug.historyScore = ones(1, numSensors);
debug.linkQuality = ones(1, numSensors);
debug.rawScore = rawScore;
debug.rawWeights = weights;
debug.weights = weights;
debug.method = methodName;
debug.useFidFiaExistence = false;
debug.fidFiaScore = ones(1, numSensors);
debug.fidFiaExistenceScore = ones(1, numSensors);
debug.fidFiaPairCounts = zeros(1, numSensors);
debug.useDecoupledKla = false;
debug.useStructureAwareKla = false;
debug.usePosteriorStructureConsistency = false;
debug.spatialRawScore = rawScore;
debug.existenceRawScore = rawScore;
debug.spatialStructurePrior = ones(1, numSensors);
debug.existenceStructurePrior = ones(1, numSensors);
debug.communicationReliabilityPrior = ones(1, numSensors);
debug.spatialStructureScore = ones(1, numSensors);
debug.existenceStructureScore = ones(1, numSensors);
debug.historyState = struct();
debug.historyInstantInstability = zeros(1, numSensors);
debug.historyInstabilityEma = zeros(1, numSensors);
debug.expectedCardinality = computeExpectedCardinality(measurementUpdatedDistributions);
end

function [targetWeights, sensorScores, targetScores] = computeTargetWiseFiTraceWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask)

numSensors = numel(measurementUpdatedDistributions);
numTargets = resolveNumberOfBernoulliTracks(measurementUpdatedDistributions);
targetScores = zeros(numTargets, numSensors);
for i = 1:numTargets
    for s = 1:numSensors
        if availabilityMask(s) <= 0
            continue;
        end
        targetScores(i, s) = estimateBernoulliFiTrace( ...
            measurementUpdatedDistributions{s}, i, model, s, t, cfg);
    end
end

targetWeights = normalizeTargetScores(targetScores, availabilityMask);
sensorScores = summarizeTargetScores(targetScores, availabilityMask);
end

function [targetWeights, sensorScores, targetScores] = computeTargetWisePdWeights( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask)

numSensors = numel(measurementUpdatedDistributions);
numTargets = resolveNumberOfBernoulliTracks(measurementUpdatedDistributions);
targetScores = zeros(numTargets, numSensors);
for i = 1:numTargets
    for s = 1:numSensors
        if availabilityMask(s) <= 0
            continue;
        end
        targetScores(i, s) = estimateBernoulliDetectionScore( ...
            measurementUpdatedDistributions{s}, i, model, s, t, cfg);
    end
end

targetWeights = normalizeTargetScores(targetScores, availabilityMask);
sensorScores = summarizeTargetScores(targetScores, availabilityMask);
end

function numTargets = resolveNumberOfBernoulliTracks(measurementUpdatedDistributions)
numTargets = 0;
for s = 1:numel(measurementUpdatedDistributions)
    numTargets = max(numTargets, numel(measurementUpdatedDistributions{s}));
end
end

function score = estimateBernoulliFiTrace(objects, objectIdx, model, sensorIdx, t, cfg)
score = 0;
if objectIdx > numel(objects) || objects(objectIdx).numberOfGmComponents < 1
    return;
end

[mu, covariance] = mprojection(model.xDimension, objects(objectIdx));
covariance = regularizeCovariance(covariance);
infoTrace = trace(pinv(covariance));
if ~isfinite(infoTrace) || infoTrace <= 0
    return;
end

scale = 1;
if getField(cfg, 'fiTraceUseExistenceProbability', false)
    existencePower = max(getField(cfg, 'fiTraceExistencePower', 1.0), 0);
    scale = scale * (max(objects(objectIdx).r, 0) ^ existencePower);
end
if getField(cfg, 'fiTraceUseDetectionProbability', false)
    [pdSensor, ~] = evaluateSensorQuality(model, sensorIdx, mu, t);
    scale = scale * max(pdSensor, 0);
end
if getField(cfg, 'fiTraceUseClutterPenalty', false)
    clutterRate = resolveSensorValue(model.clutterRate, sensorIdx, 0);
    scale = scale / (1 + max(clutterRate, 0));
end

score = max(scale, 0) * infoTrace;
end

function score = estimateBernoulliDetectionScore(objects, objectIdx, model, sensorIdx, t, cfg)
score = 0;
if objectIdx > numel(objects) || objects(objectIdx).numberOfGmComponents < 1
    return;
end
[mu, ~] = mprojection(model.xDimension, objects(objectIdx));
[pdSensor, ~] = evaluateSensorQuality(model, sensorIdx, mu, t);
score = max(pdSensor, 0) ^ max(getField(cfg, 'pdWeightPower', 1.0), 0);
end

function targetWeights = normalizeTargetScores(targetScores, availabilityMask)
targetWeights = zeros(size(targetScores));
for i = 1:size(targetScores, 1)
    targetWeights(i, :) = normalizeScores(targetScores(i, :), availabilityMask);
end
end

function sensorScores = summarizeTargetScores(targetScores, availabilityMask)
if isempty(targetScores)
    sensorScores = availabilityMask;
    return;
end
sensorScores = mean(targetScores, 1);
sensorScores(~isfinite(sensorScores)) = 0;
if ~any(sensorScores .* availabilityMask > 0)
    sensorScores = availabilityMask;
end
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
debug.ctFiDecayScore = ones(1, numSensors);
debug.useCtFiDecay = false;
debug.historyScore = ones(1, numSensors);
debug.linkQuality = ones(1, numSensors);
debug.rawScore = fiaScore;
debug.rawWeights = rawWeights;
debug.weights = weights;
debug.method = 'fidFia';
debug.fiaScore = fiaScore;
debug.fidPairCounts = fidPairCounts;
debug.useFidFiaExistence = false;
debug.fidFiaScore = rawWeights;
debug.fidFiaExistenceScore = ones(1, numSensors);
debug.fidFiaPairCounts = fidPairCounts;
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

function [fidFiaExistenceScore, normalizedFiaScore, fidPairCounts] = resolveFidFiaExistenceScore( ...
    measurementUpdatedDistributions, model, t, cfg, availabilityMask, useFidFiaExistence)

numSensors = numel(measurementUpdatedDistributions);
fidFiaExistenceScore = ones(1, numSensors);
normalizedFiaScore = zeros(1, numSensors);
fidPairCounts = zeros(1, numSensors);
if ~useFidFiaExistence
    return;
end

[fiaScore, fidPairCounts] = computeFidFiaScore(measurementUpdatedDistributions, model, t, cfg);
availableScores = fiaScore(:)' .* reshape(availabilityMask, 1, []);
maxScore = max(availableScores);
if isfinite(maxScore) && maxScore > eps
    normalizedFiaScore = availableScores / maxScore;
end
normalizedFiaScore(~isfinite(normalizedFiaScore)) = 0;
normalizedFiaScore = min(max(normalizedFiaScore, 0), 1);

scorePower = max(getField(cfg, 'fidFiaExistencePower', 1.0), 0);
minScore = min(max(getField(cfg, 'fidFiaExistenceMinScore', 0.4), 0), 1);
fidFiaExistenceScore = minScore + (1 - minScore) * (normalizedFiaScore .^ scorePower);
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

function ctFiDecayScore = resolveCtFiDecayScore( ...
    measurementUpdatedDistributions, model, t, commStats, cfg, useCtFiDecay)
numSensors = numel(measurementUpdatedDistributions);
ctFiDecayScore = ones(1, numSensors);
if ~useCtFiDecay
    return;
end

sampleAge = resolveSampleAgeFromCommStats(commStats, t, numSensors);
minScore = min(max(getField(cfg, 'ctFiDecayMinScore', 0.35), 0), 1);
scorePower = max(getField(cfg, 'ctFiDecayPower', 1.0), 0);
timeStep = max(getField(model, 'T', 1), eps);
rawInformation = zeros(1, numSensors);

for s = 1:numSensors
    dt = max(sampleAge(s), 0) * timeStep;
    rawInformation(s) = estimateCtFiInformation( ...
        measurementUpdatedDistributions{s}, model, s, t, dt, cfg);
end

validInformation = rawInformation(isfinite(rawInformation) & rawInformation > 0);
if isempty(validInformation)
    return;
end

maxInformation = max(validInformation);
normalizedInformation = rawInformation / max(maxInformation, eps);
normalizedInformation(~isfinite(normalizedInformation)) = 0;
normalizedInformation = min(max(normalizedInformation, 0), 1);
ctFiDecayScore = minScore + (1 - minScore) * (normalizedInformation .^ scorePower);
end

function sampleAge = resolveSampleAgeFromCommStats(commStats, t, numSensors)
sampleAge = zeros(1, numSensors);
if nargin < 1 || ~isstruct(commStats) || ~isfield(commStats, 'sensorSampleAge')
    return;
end
if size(commStats.sensorSampleAge, 1) == numSensors && size(commStats.sensorSampleAge, 2) >= t
    sampleAge = reshape(commStats.sensorSampleAge(:, t), 1, []);
end
sampleAge(~isfinite(sampleAge)) = 0;
sampleAge = max(sampleAge, 0);
end

function information = estimateCtFiInformation(objects, model, sensorIdx, t, dt, cfg)
information = NaN;
activeInformation = [];
existenceThreshold = getField(model, 'existenceThreshold', 0);
if ~isempty(objects)
    for i = 1:numel(objects)
        if objects(i).numberOfGmComponents < 1 || objects(i).r <= existenceThreshold
            continue;
        end
        [mu, ~] = mprojection(model.xDimension, objects(i));
        activeInformation(end+1) = estimateStateCtFiInformation(model, sensorIdx, mu, t, dt, cfg); %#ok<AGROW>
    end
end

if ~isempty(activeInformation)
    information = mean(activeInformation(isfinite(activeInformation)));
end
if ~isfinite(information)
    information = estimateStateCtFiInformation(model, sensorIdx, [], t, dt, cfg);
end
if ~isfinite(information)
    information = 0;
end
end

function information = estimateStateCtFiInformation(model, sensorIdx, state, t, dt, cfg)
useDetectionProbability = getField(cfg, 'ctFiUseDetectionProbability', true);
if ~isempty(state)
    [pdSensor, measurementCovariance] = evaluateSensorQuality(model, sensorIdx, state, t);
else
    pdSensor = resolveSensorValue(model.detectionProbability, sensorIdx, 1.0);
    measurementCovariance = model.Q{sensorIdx};
end
measurementCovariance = regularizeCovariance(measurementCovariance);
positionProcessCovariance = computeCtPositionProcessCovariance(model, dt, size(measurementCovariance, 1), cfg);
effectiveCovariance = regularizeCovariance(measurementCovariance + positionProcessCovariance);

pdScale = 1;
if useDetectionProbability
    pdScale = max(pdSensor, 0);
end
information = pdScale * trace(pinv(effectiveCovariance));
end

function positionProcessCovariance = computeCtPositionProcessCovariance(model, dt, zDimension, cfg)
if dt <= 0
    positionProcessCovariance = zeros(zDimension);
    return;
end
processNoiseScale = getField(cfg, 'ctFiProcessNoiseScale', NaN);
if ~isfinite(processNoiseScale)
    processNoiseScale = inferProcessNoiseScale(model);
end
positionVarianceGrowth = max(processNoiseScale, 0) * (dt ^ 3) / 3;
positionProcessCovariance = positionVarianceGrowth * eye(zDimension);
end

function processNoiseScale = inferProcessNoiseScale(model)
processNoiseScale = 1;
if ~isfield(model, 'R') || isempty(model.R) || ~isfield(model, 'xDimension')
    return;
end
halfDim = floor(model.xDimension / 2);
if size(model.R, 1) < 2 * halfDim
    return;
end
timeStep = max(getField(model, 'T', 1), eps);
velocityBlock = model.R((halfDim + 1):(2 * halfDim), (halfDim + 1):(2 * halfDim));
diagValues = diag(velocityBlock);
diagValues = diagValues(isfinite(diagValues) & diagValues > 0);
if ~isempty(diagValues)
    processNoiseScale = mean(diagValues) / timeStep;
end
end

function value = resolveSensorValue(values, sensorIdx, defaultValue)
if isempty(values)
    value = defaultValue;
elseif isscalar(values)
    value = values;
elseif numel(values) >= sensorIdx
    value = values(sensorIdx);
else
    value = values(1);
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
