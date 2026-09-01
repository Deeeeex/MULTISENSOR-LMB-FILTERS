function result = rankCausalSourceOffersV230( ...
        sourcePosterior, beneficiaryCacheBySensor, ...
        beneficiarySensorIds, cacheAgeBySensor, ...
        cacheEventTypeBySensor, model, options)
% RANKCAUSALSOURCEOFFERSV230 Rank at most two source-side label offers.
%
% The source may inspect its current posterior and full ordinary posteriors
% previously delivered to it by beneficiary sensors.  Missing labels count
% as negative evidence only in a full-posterior cache entry.  Truth, future
% outcomes, remote inventories outside the cache, and numeric label values
% are not score inputs.

if nargin < 7 || isempty(options)
    options = struct();
end
protocol = getCausalSourceOfferRankerV230Protocol();
rankConfig = protocol.ranking;
cacheConfig = protocol.cache;
maximumOffers = getField(options, ...
    'maximumOffersPerSource', rankConfig.maximumOffersPerSource);
maximumCacheAge = getField(options, ...
    'maximumCacheAgePages', cacheConfig.maximumAgePages);
requiredEventType = getField(options, ...
    'requiredCacheEventType', cacheConfig.requiredEventType);

beneficiarySensorIds = reshape(beneficiarySensorIds, 1, []);
cacheAgeBySensor = reshape(cacheAgeBySensor, 1, []);
cacheEventTypeBySensor = reshape(cacheEventTypeBySensor, 1, []);
entryCount = numel(beneficiarySensorIds);
if ~isstruct(sourcePosterior) || ~iscell(beneficiaryCacheBySensor) || ...
        numel(beneficiaryCacheBySensor) ~= entryCount || ...
        numel(cacheAgeBySensor) ~= entryCount || ...
        numel(cacheEventTypeBySensor) ~= entryCount || ...
        isempty(beneficiarySensorIds) || ...
        any(~isfinite(beneficiarySensorIds)) || ...
        any(beneficiarySensorIds ~= round(beneficiarySensorIds)) || ...
        any(beneficiarySensorIds < 1) || ...
        numel(unique(beneficiarySensorIds)) ~= entryCount || ...
        any(isnan(cacheAgeBySensor)) || any(cacheAgeBySensor < 0) || ...
        any(~isfinite(cacheEventTypeBySensor)) || ...
        any(cacheEventTypeBySensor ~= round(cacheEventTypeBySensor)) || ...
        ~isstruct(model) || ~isscalar(model) || ...
        ~isfield(model, 'xDimension') || model.xDimension < 2 || ...
        ~isscalar(maximumOffers) || maximumOffers < 1 || ...
        maximumOffers ~= round(maximumOffers) || ...
        maximumOffers > rankConfig.maximumOffersPerSource || ...
        ~isscalar(maximumCacheAge) || ~isfinite(maximumCacheAge) || ...
        maximumCacheAge < 0 || ...
        ~isscalar(requiredEventType) || ...
        ~isfinite(requiredEventType)
    error('CausalSourceOfferRankerV230:InvalidInput', ...
        'The source-offer ranking request is malformed.');
end

validCacheMask = ~cellfun(@isempty, beneficiaryCacheBySensor) & ...
    cacheAgeBySensor <= maximumCacheAge & ...
    cacheEventTypeBySensor >= requiredEventType;
validIndices = find(validCacheMask);
sourceSummary = summarizeLmbPosteriorForDisagreement( ...
    sourcePosterior, model);

result = emptyResult(protocol, beneficiarySensorIds, ...
    cacheAgeBySensor, cacheEventTypeBySensor, validCacheMask);
if numel(validIndices) < ...
        cacheConfig.minimumCachedBeneficiarySensors || ...
        isempty(sourceSummary.labels)
    result.status = 'insufficient-causal-cache';
    return;
end

cacheSummaries = cell(1, entryCount);
for entryIndex = validIndices
    cacheSummaries{entryIndex} = ...
        summarizeLmbPosteriorForDisagreement( ...
            beneficiaryCacheBySensor{entryIndex}, model);
end
rawFreshness = cacheConfig.ageDecay .^ ...
    cacheAgeBySensor(validIndices);
freshnessWeights = rawFreshness / sum(rawFreshness);

candidateCount = size(sourceSummary.labels, 2);
candidates = repmat(emptyCandidate(), 1, candidateCount);
keep = false(1, candidateCount);
for candidateIndex = 1:candidateCount
    label = sourceSummary.labels(:, candidateIndex);
    sourceExistence = sourceSummary.existence(candidateIndex);
    referenceExistenceByEntry = zeros(1, numel(validIndices));
    referenceMeans = nan(2, numel(validIndices));
    referenceCovariances = nan(2, 2, numel(validIndices));
    spatialEntryMask = false(1, numel(validIndices));
    for localIndex = 1:numel(validIndices)
        summary = cacheSummaries{validIndices(localIndex)};
        match = find(all(summary.labels == label, 1), 1);
        if isempty(match)
            continue;
        end
        referenceExistenceByEntry(localIndex) = ...
            summary.existence(match);
        if summary.existence(match) >= ...
                rankConfig.minimumSpatialExistence
            referenceMeans(:, localIndex) = ...
                summary.positionMean(:, match);
            referenceCovariances(:, :, localIndex) = ...
                summary.positionCovariance(:, :, match);
            spatialEntryMask(localIndex) = true;
        end
    end
    referenceExistence = sum( ...
        freshnessWeights .* referenceExistenceByEntry);
    if max(sourceExistence, referenceExistence) < ...
            rankConfig.minimumActionableExistence
        continue;
    end

    existenceScore = abs(safeLogit(sourceExistence, ...
        rankConfig.existenceLogitFloor) - ...
        safeLogit(referenceExistence, ...
            rankConfig.existenceLogitFloor)) * ...
        max(sourceExistence, referenceExistence);
    [sourceMean, sourceCovariance] = sourceMoments( ...
        sourceSummary, candidateIndex, rankConfig.covarianceFloor);
    [referenceMean, referenceCovariance, ...
        referenceSpatialExistence, spatialSupportCount] = ...
        aggregateReferenceSpatialMoments( ...
            referenceMeans, referenceCovariances, ...
            referenceExistenceByEntry, freshnessWeights, ...
            spatialEntryMask, rankConfig.covarianceFloor);
    spatialMahalanobis = 0;
    precisionGain = 0;
    spatialScore = 0;
    if spatialSupportCount > 0 && ...
            sourceExistence >= rankConfig.minimumSpatialExistence
        totalCovariance = regularizeCovariance( ...
            sourceCovariance + referenceCovariance, ...
            rankConfig.covarianceFloor);
        delta = sourceMean - referenceMean;
        spatialMahalanobis = sqrt(max(real( ...
            delta' * (totalCovariance \ delta)), 0));
        precisionGain = max(0, log( ...
            max(trace(referenceCovariance), ...
                rankConfig.covarianceFloor) / ...
            max(trace(sourceCovariance), ...
                rankConfig.covarianceFloor)));
        spatialScore = sqrt(max(sourceExistence * ...
            referenceSpatialExistence, 0)) * ...
            (spatialMahalanobis + ...
             rankConfig.precisionGainWeight * precisionGain);
    end

    sourceObject = findLabelObject(sourcePosterior, label);
    payloadStats = estimateLmbPayloadSize( ...
        sourceObject, model, 2, struct());
    candidates(candidateIndex) = struct( ...
        'label', reshape(label, 2, 1), ...
        'sourceExistence', sourceExistence, ...
        'referenceExistence', referenceExistence, ...
        'existenceScore', existenceScore, ...
        'spatialScore', spatialScore, ...
        'spatialMahalanobis', spatialMahalanobis, ...
        'precisionGain', precisionGain, ...
        'spatialSupportCount', spatialSupportCount, ...
        'payloadBytesPerReceiver', payloadStats.estimatedBytes, ...
        'mode', '', 'selected', false);
    keep(candidateIndex) = true;
end
candidates = candidates(keep);
if isempty(candidates)
    result.status = 'no-actionable-source-label';
    return;
end

existenceOrder = rankCandidates(candidates, 'existenceScore');
spatialOrder = rankCandidates(candidates, 'spatialScore');
selectedIndices = zeros(1, 0);
selectedModes = cell(1, 0);
[selectedIndices, selectedModes] = selectFromMode( ...
    selectedIndices, selectedModes, existenceOrder, candidates, ...
    'existence-surprise', rankConfig.minimumPositiveScore, ...
    maximumOffers);
[selectedIndices, selectedModes] = selectFromMode( ...
    selectedIndices, selectedModes, spatialOrder, candidates, ...
    'spatial-utility', rankConfig.minimumPositiveScore, ...
    maximumOffers);
if numel(selectedIndices) < maximumOffers
    jointOrder = rankJointAlternatives(candidates);
    [selectedIndices, selectedModes] = selectFromMode( ...
        selectedIndices, selectedModes, jointOrder, candidates, ...
        'fallback-next-best', rankConfig.minimumPositiveScore, ...
        maximumOffers);
end
for selectedPosition = 1:numel(selectedIndices)
    candidateIndex = selectedIndices(selectedPosition);
    candidates(candidateIndex).selected = true;
    candidates(candidateIndex).mode = selectedModes{selectedPosition};
end

result.status = 'ranked';
result.candidates = candidates;
result.existenceRanking = labelsAt(candidates, existenceOrder);
result.spatialRanking = labelsAt(candidates, spatialOrder);
result.selectedCandidateIndices = selectedIndices;
result.selectedLabels = labelsAt(candidates, selectedIndices);
result.selectedModes = selectedModes;
result.selectedOfferCount = numel(selectedIndices);
result.cacheFreshnessMass = sum(rawFreshness);
result.cacheCoverageFraction = ...
    numel(validIndices) / entryCount;
end

function result = emptyResult(protocol, sensorIds, ages, eventTypes, mask)
result = struct();
result.contractVersion = 'causal-source-offer-ranking-v230-v1';
result.protocolId = protocol.id;
result.status = 'uninitialized';
result.beneficiarySensorIds = sensorIds;
result.cacheAgeBySensor = ages;
result.cacheEventTypeBySensor = eventTypes;
result.validCacheMask = mask;
result.validCacheSensorIds = sensorIds(mask);
result.validCacheCount = nnz(mask);
result.cacheCoverageFraction = nnz(mask) / max(numel(mask), 1);
result.cacheFreshnessMass = 0;
result.candidates = repmat(emptyCandidate(), 1, 0);
result.existenceRanking = zeros(2, 0);
result.spatialRanking = zeros(2, 0);
result.selectedCandidateIndices = zeros(1, 0);
result.selectedLabels = zeros(2, 0);
result.selectedModes = cell(1, 0);
result.selectedOfferCount = 0;
result.truthUsed = false;
result.futureInformationUsed = false;
result.numericLabelIdentifiersUsedAsScoreFeatures = false;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function value = emptyCandidate()
value = struct('label', zeros(2, 1), ...
    'sourceExistence', 0, 'referenceExistence', 0, ...
    'existenceScore', 0, 'spatialScore', 0, ...
    'spatialMahalanobis', 0, 'precisionGain', 0, ...
    'spatialSupportCount', 0, 'payloadBytesPerReceiver', 0, ...
    'mode', '', 'selected', false);
end

function order = rankCandidates(candidates, scoreField)
score = [candidates.(scoreField)]';
payload = [candidates.payloadBytesPerReceiver]';
labels = reshape([candidates.label], 2, [])';
keys = [-score, payload, labels];
[~, order] = sortrows(keys, 1:size(keys, 2));
order = reshape(order, 1, []);
end

function order = rankJointAlternatives(candidates)
existenceScore = [candidates.existenceScore]';
spatialScore = [candidates.spatialScore]';
existenceRank = tiedOrdinalRank(-existenceScore);
spatialRank = tiedOrdinalRank(-spatialScore);
bestRank = min(existenceRank, spatialRank);
bestScore = max(existenceScore, spatialScore);
payload = [candidates.payloadBytesPerReceiver]';
labels = reshape([candidates.label], 2, [])';
keys = [bestRank, -bestScore, payload, labels];
[~, order] = sortrows(keys, 1:size(keys, 2));
order = reshape(order, 1, []);
end

function ranks = tiedOrdinalRank(values)
[sorted, order] = sort(values, 'ascend');
ranks = zeros(size(values));
rank = 1;
for idx = 1:numel(sorted)
    if idx > 1 && abs(sorted(idx) - sorted(idx - 1)) > 1e-12
        rank = idx;
    end
    ranks(order(idx)) = rank;
end
end

function [selected, modes] = selectFromMode( ...
        selected, modes, order, candidates, mode, minimumScore, maximum)
for candidateIndex = order
    if numel(selected) >= maximum
        return;
    end
    if any(selected == candidateIndex)
        continue;
    end
    if strcmp(mode, 'spatial-utility')
        score = candidates(candidateIndex).spatialScore;
    elseif strcmp(mode, 'existence-surprise')
        score = candidates(candidateIndex).existenceScore;
    else
        score = max(candidates(candidateIndex).existenceScore, ...
            candidates(candidateIndex).spatialScore);
    end
    if score <= minimumScore
        continue;
    end
    selected(end + 1) = candidateIndex; %#ok<AGROW>
    modes{end + 1} = mode; %#ok<AGROW>
    if ~strcmp(mode, 'fallback-next-best')
        return;
    end
end
end

function labels = labelsAt(candidates, indices)
labels = zeros(2, 0);
for index = reshape(indices, 1, [])
    labels(:, end + 1) = candidates(index).label; %#ok<AGROW>
end
end

function [meanVector, covariance] = sourceMoments( ...
        summary, index, covarianceFloor)
meanVector = summary.positionMean(:, index);
covariance = regularizeCovariance( ...
    summary.positionCovariance(:, :, index), covarianceFloor);
end

function [meanVector, covariance, meanExistence, supportCount] = ...
        aggregateReferenceSpatialMoments( ...
            means, covariances, existence, freshness, mask, floorValue)
indices = find(mask);
supportCount = numel(indices);
meanVector = zeros(2, 1);
covariance = eye(2) * floorValue;
meanExistence = 0;
if isempty(indices)
    return;
end
weights = freshness(indices) .* existence(indices);
if sum(weights) <= 0
    weights = freshness(indices);
end
weights = weights / sum(weights);
for localIndex = 1:numel(indices)
    entryIndex = indices(localIndex);
    meanVector = meanVector + ...
        weights(localIndex) * means(:, entryIndex);
end
covariance = zeros(2);
for localIndex = 1:numel(indices)
    entryIndex = indices(localIndex);
    delta = means(:, entryIndex) - meanVector;
    covariance = covariance + weights(localIndex) * ...
        (covariances(:, :, entryIndex) + delta * delta');
end
covariance = regularizeCovariance(covariance, floorValue);
meanExistence = sum(weights .* existence(indices));
end

function covariance = regularizeCovariance(covariance, floorValue)
covariance = real((covariance + covariance') / 2);
if any(~isfinite(covariance(:)))
    covariance = eye(size(covariance, 1)) * floorValue;
    return;
end
[vectors, values] = eig(covariance);
eigenvalues = max(real(diag(values)), floorValue);
covariance = real(vectors * diag(eigenvalues) * vectors');
covariance = (covariance + covariance') / 2;
end

function value = safeLogit(probability, floorValue)
probability = min(max(probability, floorValue), 1 - floorValue);
value = log(probability / (1 - probability));
end

function object = findLabelObject(objects, label)
object = [];
for objectIndex = 1:numel(objects)
    if objects(objectIndex).birthTime == label(1) && ...
            objects(objectIndex).birthLocation == label(2)
        object = objects(objectIndex);
        return;
    end
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
