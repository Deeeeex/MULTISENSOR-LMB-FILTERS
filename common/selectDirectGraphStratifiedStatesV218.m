function selection = selectDirectGraphStratifiedStatesV218( ...
        times, formationIds, featureNames, featuresByTime, options)
% SELECTDIRECTGRAPHSTRATIFIEDSTATESV218 Select four causal state strata.
%
% One formation-time pair is selected for each registered stratum.  Every
% input is current-trajectory observable.  Feature values are converted to
% within-trajectory midrank percentiles so M24 and X36 share one rule.

if nargin < 5 || isempty(options)
    options = struct();
end
protocol = getField(options, 'protocol', ...
    getDirectGraphStratifiedSamplingV218Protocol());
times = reshape(times, 1, []);
formationIds = reshape(formationIds, 1, []);
featureNames = reshape(featureNames, 1, []);
timeCount = numel(times);
formationCount = numel(formationIds);
featureCount = numel(featureNames);
if timeCount < 1 || formationCount < 1 || featureCount < 1 || ...
        any(~isfinite(times)) || any(times ~= round(times)) || ...
        any(diff(times) <= 0) || ...
        any(~isfinite(formationIds)) || ...
        any(formationIds ~= round(formationIds)) || ...
        numel(unique(formationIds)) ~= formationCount || ...
        numel(unique(featureNames)) ~= featureCount || ...
        ~iscell(featuresByTime) || ...
        numel(featuresByTime) ~= timeCount || ...
        ~isstruct(protocol) || ~isscalar(protocol) || ...
        ~all(isfield(protocol, {'strata', 'collection'}))
    error('DirectGraphStratifiedSamplingV218:InvalidInput', ...
        'V218 requires one finite complete observable trajectory.');
end

pairCount = timeCount * formationCount;
rawFeatures = zeros(pairCount, featureCount);
pairTimes = zeros(pairCount, 1);
pairFormationIds = zeros(pairCount, 1);
pairCursor = 0;
for timeIdx = 1:timeCount
    page = featuresByTime{timeIdx};
    if ~isnumeric(page) || ...
            ~isequal(size(page), [formationCount, featureCount]) || ...
            any(~isfinite(page(:)))
        error('DirectGraphStratifiedSamplingV218:FeaturePage', ...
            'A V218 feature page is missing, non-finite or mis-sized.');
    end
    rows = pairCursor + (1:formationCount);
    rawFeatures(rows, :) = page;
    pairTimes(rows) = times(timeIdx);
    pairFormationIds(rows) = formationIds;
    pairCursor = pairCursor + formationCount;
end

allFeatureRanks = zeros(size(rawFeatures));
for featureIdx = 1:featureCount
    allFeatureRanks(:, featureIdx) = ...
        midrankPercentile(rawFeatures(:, featureIdx));
end

stratumNames = reshape(protocol.strata.names, 1, []);
stratumCount = numel(stratumNames);
stratumScores = zeros(pairCount, stratumCount);
componentFeatureIndices = cell(1, stratumCount);
componentRanks = cell(1, stratumCount);
for stratumIdx = 1:stratumCount
    requested = protocol.strata.featureNames{stratumIdx};
    directions = reshape( ...
        protocol.strata.featureDirections{stratumIdx}, 1, []);
    indices = featureIndices(featureNames, requested);
    directedRanks = allFeatureRanks(:, indices);
    for componentIdx = 1:numel(indices)
        if directions(componentIdx) < 0
            directedRanks(:, componentIdx) = ...
                1 - directedRanks(:, componentIdx);
        end
    end
    componentFeatureIndices{stratumIdx} = indices;
    componentRanks{stratumIdx} = directedRanks;
    stratumScores(:, stratumIdx) = mean(directedRanks, 2);
end

[selectedPairIndices, selectedOrder, aggregateScore] = ...
    selectAcrossAllStratumOrders( ...
        pairTimes, stratumScores, allFeatureRanks, protocol.collection);
selectedTimes = reshape(pairTimes(selectedPairIndices), 1, []);
selectedFormationIds = reshape( ...
    pairFormationIds(selectedPairIndices), 1, []);
selectedScores = zeros(1, stratumCount);
selectedComponentRanks = cell(1, stratumCount);
for stratumIdx = 1:stratumCount
    selectedScores(stratumIdx) = ...
        stratumScores(selectedPairIndices(stratumIdx), stratumIdx);
    selectedComponentRanks{stratumIdx} = ...
        componentRanks{stratumIdx}( ...
            selectedPairIndices(stratumIdx), :);
end

auditSelection(selectedPairIndices, selectedTimes, protocol.collection);
selection = struct();
selection.contractVersion = ...
    'direct-graph-stratified-state-selection-v218-v1';
selection.protocolId = protocol.id;
selection.stratumNames = stratumNames;
selection.stratumFeatureNames = protocol.strata.featureNames;
selection.stratumFeatureDirections = ...
    protocol.strata.featureDirections;
selection.featureNames = featureNames;
selection.pairTimes = pairTimes;
selection.pairFormationIds = pairFormationIds;
selection.rawFeatures = rawFeatures;
selection.featureMidrankPercentiles = allFeatureRanks;
selection.stratumScores = stratumScores;
selection.selectedPairIndicesByStratum = selectedPairIndices;
selection.selectedTimesByStratum = selectedTimes;
selection.selectedFormationIdsByStratum = selectedFormationIds;
selection.selectedScoresByStratum = selectedScores;
selection.selectedRawFeaturesByStratum = ...
    rawFeatures(selectedPairIndices, :);
selection.selectedComponentRanksByStratum = ...
    selectedComponentRanks;
selection.selectedStratumOrder = selectedOrder;
selection.aggregateSelectionScore = aggregateScore;
selection.uniqueSelectedTimes = unique(selectedTimes, 'sorted');
selection.minimumDistinctTimeSeparation = ...
    protocol.collection.minimumDistinctTimeSeparation;
selection.allowSameTimeDifferentFormation = ...
    protocol.collection.allowSameTimeDifferentFormation;
selection.rankNormalization = ...
    protocol.collection.rankNormalization;
selection.selectionRule = protocol.collection.selectionRule;
selection.numericFormationIdentifiersUsedAsScores = false;
selection.storageOrderUsedOnlyForExactObservableTies = true;
selection.truthUsed = false;
selection.futureMeasurementsUsed = false;
selection.futureLinkOutcomesUsed = false;
selection.futureTrackingOutcomesUsed = false;
end

function [bestPairs, bestOrder, bestScore] = ...
        selectAcrossAllStratumOrders( ...
            pairTimes, scores, observableRanks, collection)
stratumCount = size(scores, 2);
orders = perms(1:stratumCount);
bestPairs = zeros(1, stratumCount);
bestOrder = zeros(1, stratumCount);
bestScore = -inf;
bestSignature = [];
for orderIdx = 1:size(orders, 1)
    order = orders(orderIdx, :);
    selected = zeros(1, stratumCount);
    success = true;
    for cursor = 1:stratumCount
        stratumIdx = order(cursor);
        ranking = [-scores(:, stratumIdx), pairTimes, ...
            -observableRanks, (1:numel(pairTimes))'];
        [~, ranked] = sortrows( ...
            ranking, 1:size(ranking, 2));
        chosen = 0;
        for candidate = reshape(ranked, 1, [])
            previous = selected(selected > 0);
            if any(previous == candidate) || ...
                    ~timeCompatible(pairTimes(candidate), ...
                        pairTimes(previous), collection)
                continue;
            end
            chosen = candidate;
            break;
        end
        if chosen == 0
            success = false;
            break;
        end
        selected(stratumIdx) = chosen;
    end
    if ~success
        continue;
    end
    linear = sub2ind(size(scores), ...
        selected, 1:stratumCount);
    total = sum(scores(linear));
    signature = [reshape(pairTimes(selected), 1, []), ...
        reshape(selected, 1, []), order];
    if total > bestScore + 1e-12 || ...
            (abs(total - bestScore) <= 1e-12 && ...
             (isempty(bestSignature) || ...
              lexicographicallyLess(signature, bestSignature)))
        bestPairs = selected;
        bestOrder = order;
        bestScore = total;
        bestSignature = signature;
    end
end
if ~isfinite(bestScore) || any(bestPairs == 0)
    error('DirectGraphStratifiedSamplingV218:NoSeparatedSelection', ...
        'The complete trajectory cannot cover all V218 strata.');
end
end

function passed = timeCompatible(candidateTime, selectedTimes, collection)
if isempty(selectedTimes)
    passed = true;
    return;
end
same = selectedTimes == candidateTime;
if any(same) && ~collection.allowSameTimeDifferentFormation
    passed = false;
    return;
end
different = ~same;
passed = all(abs(selectedTimes(different) - candidateTime) >= ...
    collection.minimumDistinctTimeSeparation);
end

function auditSelection(indices, times, collection)
if numel(unique(indices)) ~= numel(indices)
    error('DirectGraphStratifiedSamplingV218:DuplicatePair', ...
        'One formation-time pair was assigned to multiple V218 strata.');
end
uniqueTimes = unique(times, 'sorted');
if numel(uniqueTimes) > 1
    separations = abs(uniqueTimes(:) - uniqueTimes(:)');
    separations(1:size(separations, 1)+1:end) = inf;
    if min(separations(:)) < ...
            collection.minimumDistinctTimeSeparation
        error('DirectGraphStratifiedSamplingV218:TimeSeparation', ...
            'Distinct selected V218 states violate temporal separation.');
    end
end
end

function percentiles = midrankPercentile(values)
values = reshape(values, [], 1);
count = numel(values);
[sortedValues, order] = sort(values, 'ascend');
ranks = zeros(count, 1);
startIdx = 1;
while startIdx <= count
    endIdx = startIdx;
    while endIdx < count && ...
            sortedValues(endIdx + 1) == sortedValues(startIdx)
        endIdx = endIdx + 1;
    end
    ranks(order(startIdx:endIdx)) = (startIdx + endIdx) / 2;
    startIdx = endIdx + 1;
end
if count == 1
    percentiles = 0.5;
else
    percentiles = (ranks - 1) / (count - 1);
end
end

function indices = featureIndices(names, requested)
indices = zeros(1, numel(requested));
for idx = 1:numel(requested)
    match = find(strcmp(names, requested{idx}));
    if numel(match) ~= 1
        error('DirectGraphStratifiedSamplingV218:FeatureContract', ...
            'Missing or ambiguous V218 feature: %s', requested{idx});
    end
    indices(idx) = match;
end
end

function passed = lexicographicallyLess(left, right)
firstDifference = find(left ~= right, 1);
passed = ~isempty(firstDifference) && ...
    left(firstDifference) < right(firstDifference);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
