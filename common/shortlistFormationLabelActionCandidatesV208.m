function [shortlisted, details] = ...
        shortlistFormationLabelActionCandidatesV208(candidates, options)
% SHORTLISTFORMATIONLABELACTIONCANDIDATESV208 Truth-free diverse union.
%
% The V190 bank may contain more than one thousand executable source-label
% keys on one X36 page.  V208 retains the union of several bounded,
% interpretable rankings within each formation.  Semantic label and source
% identifiers are used only to return executable routing keys; they never
% enter the numeric feature matrix or the ranking scores.

if nargin < 2 || isempty(options)
    options = struct();
end
maximumPerCriterion = getField(options, 'maximumPerCriterion', 2);
if ~isscalar(maximumPerCriterion) || ...
        ~isfinite(maximumPerCriterion) || ...
        maximumPerCriterion < 1 || ...
        maximumPerCriterion ~= round(maximumPerCriterion)
    error('FormationActionShortlistV208:InvalidOptions', ...
        'maximumPerCriterion must be a positive integer.');
end
if isempty(candidates)
    shortlisted = candidates;
    details = emptyDetails(maximumPerCriterion);
    return;
end
if ~isstruct(candidates) || ...
        ~all(isfield(candidates, { ...
            'candidateIndex', 'formationId', 'sourceId', 'label'}))
    error('FormationActionShortlistV208:InvalidCandidates', ...
        'A V190 candidate array is required.');
end

candidateCount = numel(candidates);
featureRows = zeros(candidateCount, 0);
featureNames = cell(1, 0);
for candidateIdx = 1:candidateCount
    [values, names] = ...
        buildFormationLabelActionModeFeaturesV202( ...
            candidates(candidateIdx));
    if candidateIdx == 1
        featureRows = zeros(candidateCount, numel(values));
        featureNames = names;
    elseif ~isequal(featureNames, names)
        error('FormationActionShortlistV208:FeatureDrift', ...
            'The V202 mode feature contract changed between candidates.');
    end
    featureRows(candidateIdx, :) = values;
end

criterionNames = { ...
    'bounded_risk_reduction', ...
    'observation_handover_evidence', ...
    'precision_refresh_evidence', ...
    'source_quality', ...
    'source_observation_opportunity', ...
    'peer_consensus_median', ...
    'handover_opportunity', ...
    'bounded_log_precision_gain', ...
    'receiver_existence_deficit', ...
    'negative_bounded_position_distance_maximum'};
criterionFeatureNames = criterionNames;
criterionFeatureNames{end} = ...
    'bounded_position_distance_maximum';
criterionDirection = ones(1, numel(criterionNames));
criterionDirection(end) = -1;
criterionIndices = resolveIndices( ...
    featureNames, criterionFeatureNames);

formationIds = unique([candidates.formationId], 'stable');
selectedMask = false(1, candidateCount);
selectedByCriterion = false(candidateCount, numel(criterionNames));
for formationId = reshape(formationIds, 1, [])
    formationRows = find([candidates.formationId] == formationId);
    for criterionIdx = 1:numel(criterionNames)
        score = criterionDirection(criterionIdx) * ...
            featureRows(formationRows, criterionIndices(criterionIdx));
        % Stable secondary keys keep the output deterministic without using
        % semantic identifier magnitudes as learned information.
        orderTable = [ ...
            -score(:), ...
            [candidates(formationRows).candidateIndex]'];
        [~, order] = sortrows(orderTable, [1, 2]);
        keepCount = min(maximumPerCriterion, numel(order));
        keepRows = formationRows(order(1:keepCount));
        selectedMask(keepRows) = true;
        selectedByCriterion(keepRows, criterionIdx) = true;
    end
end

selectedIndices = find(selectedMask);
shortlisted = candidates(selectedIndices);
details = struct();
details.contractVersion = ...
    'formation-label-action-diverse-shortlist-v208-v1';
details.maximumPerCriterion = maximumPerCriterion;
details.criterionNames = criterionNames;
details.featureNames = featureNames;
details.fullCandidateCount = candidateCount;
details.shortlistedCandidateCount = numel(shortlisted);
details.formationIds = formationIds;
details.fullCountByFormation = zeros(1, numel(formationIds));
details.shortlistedCountByFormation = zeros(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    formationId = formationIds(formationIdx);
    details.fullCountByFormation(formationIdx) = nnz( ...
        [candidates.formationId] == formationId);
    details.shortlistedCountByFormation(formationIdx) = nnz( ...
        [shortlisted.formationId] == formationId);
end
details.selectedCandidateIndices = ...
    [shortlisted.candidateIndex];
details.selectedByCriterion = ...
    selectedByCriterion(selectedIndices, :);
details.shortlistedFeatureMatrix = featureRows(selectedIndices, :);
details.truthUsed = false;
details.futureInformationUsed = false;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.numericSourceIdentifiersUsedAsFeatures = false;
end

function indices = resolveIndices(names, requested)
indices = zeros(1, numel(requested));
for requestedIdx = 1:numel(requested)
    match = find(strcmp(names, requested{requestedIdx}));
    if numel(match) ~= 1
        error('FormationActionShortlistV208:MissingFeature', ...
            'Required V202 feature is missing: %s.', ...
            requested{requestedIdx});
    end
    indices(requestedIdx) = match;
end
end

function details = emptyDetails(maximumPerCriterion)
details = struct();
details.contractVersion = ...
    'formation-label-action-diverse-shortlist-v208-v1';
details.maximumPerCriterion = maximumPerCriterion;
details.criterionNames = { ...
    'bounded_risk_reduction', ...
    'observation_handover_evidence', ...
    'precision_refresh_evidence', ...
    'source_quality', ...
    'source_observation_opportunity', ...
    'peer_consensus_median', ...
    'handover_opportunity', ...
    'bounded_log_precision_gain', ...
    'receiver_existence_deficit', ...
    'negative_bounded_position_distance_maximum'};
details.featureNames = cell(1, 0);
details.fullCandidateCount = 0;
details.shortlistedCandidateCount = 0;
details.formationIds = zeros(1, 0);
details.fullCountByFormation = zeros(1, 0);
details.shortlistedCountByFormation = zeros(1, 0);
details.selectedCandidateIndices = zeros(1, 0);
details.selectedByCriterion = ...
    false(0, numel(details.criterionNames));
details.shortlistedFeatureMatrix = zeros(0, 0);
details.truthUsed = false;
details.futureInformationUsed = false;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.numericSourceIdentifiersUsedAsFeatures = false;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
