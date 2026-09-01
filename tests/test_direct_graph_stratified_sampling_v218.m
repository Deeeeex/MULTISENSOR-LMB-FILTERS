function test_direct_graph_stratified_sampling_v218()
% TEST_DIRECT_GRAPH_STRATIFIED_SAMPLING_V218 Rank and separation contract.

protocol = getDirectGraphStratifiedSamplingV218Protocol();
featureNames = unique([protocol.strata.featureNames{:}], 'stable');
times = [1, 13, 25, 37];
formationIds = [1, 2];
features = cell(1, numel(times));
cursor = 0;
for timeIdx = 1:numel(times)
    page = zeros(numel(formationIds), numel(featureNames));
    for formationIdx = 1:numel(formationIds)
        cursor = cursor + 1;
        page(formationIdx, :) = ...
            cursor + (1:numel(featureNames)) / 100;
    end
    features{timeIdx} = page;
end

selection = selectDirectGraphStratifiedStatesV218( ...
    times, formationIds, featureNames, features, ...
    struct('protocol', protocol));
assert(numel(selection.selectedPairIndicesByStratum) == 4);
assert(numel(unique(selection.selectedPairIndicesByStratum)) == 4);
assert(all(selection.selectedScoresByStratum >= 0));
assert(all(selection.selectedScoresByStratum <= 1));
assert(~selection.truthUsed && ...
    ~selection.futureMeasurementsUsed && ...
    ~selection.futureLinkOutcomesUsed && ...
    ~selection.futureTrackingOutcomesUsed);
assert(~selection.numericFormationIdentifiersUsedAsScores);

uniqueTimes = unique(selection.selectedTimesByStratum);
if numel(uniqueTimes) > 1
    distances = abs(uniqueTimes(:) - uniqueTimes(:)');
    distances(1:size(distances, 1)+1:end) = inf;
    assert(min(distances(:)) >= ...
        protocol.collection.minimumDistinctTimeSeparation);
end

affineFeatures = cellfun( ...
    @(page) 3 * page + 7, features, 'UniformOutput', false);
affineSelection = selectDirectGraphStratifiedStatesV218( ...
    times, formationIds, featureNames, affineFeatures, ...
    struct('protocol', protocol));
assert(isequal(selection.selectedPairIndicesByStratum, ...
    affineSelection.selectedPairIndicesByStratum));
assert(max(abs(selection.stratumScores(:) - ...
    affineSelection.stratumScores(:))) < 1e-12);
fprintf('test_direct_graph_stratified_sampling_v218 passed\n');
end
