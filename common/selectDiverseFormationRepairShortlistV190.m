function [shortlist, details] = ...
        selectDiverseFormationRepairShortlistV190(candidates, options)
% SELECTDIVERSEFORMATIONREPAIRSHORTLISTV190 Union causal rankings.
%
% The coarse shortlist preserves different notions of value instead of
% trusting one hand-crafted scalar.  Label keys break deterministic ties
% only; they are not learned features.

if nargin < 2 || isempty(options)
    options = struct();
end
criteria = getField(options, 'criteria', defaultCriteria());
perCriterion = getField(options, 'perCriterion', 2);
maximumPerFormation = getField(options, 'maximumPerFormation', 12);
validateInputs(candidates, criteria, perCriterion, maximumPerFormation);
if isempty(candidates)
    shortlist = candidates;
    details = emptyDetails(criteria);
    return;
end

formationIds = unique([candidates.formationId], 'stable');
criterionCount = numel(criteria);
candidateCount = numel(candidates);
criterionHits = false(candidateCount, criterionCount);
rankMatrix = inf(candidateCount, criterionCount);
selectedIndices = zeros(1, 0);
selectedCountByFormation = zeros(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    formationCandidateIndices = find( ...
        [candidates.formationId] == formationIds(formationIdx));
    for criterionIdx = 1:criterionCount
        order = rankFormationCandidates( ...
            candidates, formationCandidateIndices, ...
            criteria(criterionIdx));
        rankMatrix(order, criterionIdx) = 1:numel(order);
        chosen = order(1:min(perCriterion, numel(order)));
        criterionHits(chosen, criterionIdx) = true;
    end
    unionIndices = formationCandidateIndices( ...
        any(criterionHits(formationCandidateIndices, :), 2));
    hitCount = sum(criterionHits(unionIndices, :), 2);
    rankSum = sum(rankMatrix(unionIndices, :), 2);
    tie = [[candidates(unionIndices).sourceId]', ...
        reshape([candidates(unionIndices).label], 2, [])'];
    ranking = [-hitCount, rankSum, tie, unionIndices'];
    [~, order] = sortrows(ranking, 1:size(ranking, 2));
    unionIndices = unionIndices(order);
    unionIndices = unionIndices( ...
        1:min(maximumPerFormation, numel(unionIndices)));
    selectedIndices = [selectedIndices, unionIndices]; %#ok<AGROW>
    selectedCountByFormation(formationIdx) = numel(unionIndices);
end

shortlist = candidates(selectedIndices);
details = struct();
details.contractVersion = ...
    'diverse-formation-repair-shortlist-v190-v1';
details.formationIds = formationIds;
details.inputCandidateCount = candidateCount;
details.selectedCandidateIndices = selectedIndices;
details.selectedCandidateCount = numel(selectedIndices);
details.selectedCountByFormation = selectedCountByFormation;
details.criterionNames = {criteria.name};
details.criterionDirections = [criteria.direction];
details.criterionHits = criterionHits(selectedIndices, :);
details.rankMatrix = rankMatrix(selectedIndices, :);
details.perCriterion = perCriterion;
details.maximumPerFormation = maximumPerFormation;
details.semanticLabelKeysUsedForTieBreakOnly = true;
details.truthUsed = false;
details.futureInformationUsed = false;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.trackingBenefitClaimed = false;
end

function order = rankFormationCandidates( ...
        candidates, indices, criterion)
values = arrayfun(@(candidate) ...
    candidate.(criterion.field), candidates(indices));
primary = -criterion.direction * reshape(values, [], 1);
source = reshape([candidates(indices).sourceId], [], 1);
labels = reshape([candidates(indices).label], 2, [])';
rows = [primary, source, labels, reshape(indices, [], 1)];
[~, localOrder] = sortrows(rows, 1:size(rows, 2));
order = indices(localOrder);
end

function criteria = defaultCriteria()
criteria = repmat(struct('name', '', 'field', '', 'direction', 1), 1, 7);
criteria(1) = makeCriterion( ...
    'opportunity', 'opportunityMedian', 1);
criteria(2) = makeCriterion( ...
    'minimum-risk-reduction', 'minimumRiskReduction', 1);
criteria(3) = makeCriterion( ...
    'source-quality', 'sourceQuality', 1);
criteria(4) = makeCriterion( ...
    'peer-consensus', 'peerConsensusMedian', 1);
criteria(5) = makeCriterion( ...
    'receiver-disagreement', 'receiverDisagreementMedian', 1);
criteria(6) = makeCriterion( ...
    'precision-gain', 'precisionGainNormalized', 1);
criteria(7) = makeCriterion( ...
    'motion-compatibility', 'positionDistanceMaximumNormalized', -1);
end

function value = makeCriterion(name, field, direction)
value = struct('name', name, 'field', field, 'direction', direction);
end

function validateInputs( ...
        candidates, criteria, perCriterion, maximumPerFormation)
if ~isstruct(candidates) || ~isstruct(criteria) || isempty(criteria) || ...
        ~isscalar(perCriterion) || ~isfinite(perCriterion) || ...
        perCriterion < 1 || perCriterion ~= round(perCriterion) || ...
        ~isscalar(maximumPerFormation) || ...
        ~isfinite(maximumPerFormation) || ...
        maximumPerFormation < 1 || ...
        maximumPerFormation ~= round(maximumPerFormation)
    error('FormationRepairShortlistV190:InvalidInput', ...
        'The candidate set or shortlist limit is invalid.');
end
required = {'formationId', 'sourceId', 'label'};
if ~isempty(candidates) && any(~isfield(candidates, required))
    error('FormationRepairShortlistV190:CandidateContract', ...
        'Candidate routing fields are missing.');
end
for criterionIdx = 1:numel(criteria)
    criterion = criteria(criterionIdx);
    if ~isfield(criterion, 'name') || ~ischar(criterion.name) || ...
            isempty(criterion.name) || ...
            ~isfield(criterion, 'field') || ~ischar(criterion.field) || ...
            isempty(criterion.field) || ...
            ~isfield(criterion, 'direction') || ...
            ~isscalar(criterion.direction) || ...
            ~ismember(criterion.direction, [-1, 1]) || ...
            (~isempty(candidates) && ...
                ~isfield(candidates, criterion.field))
        error('FormationRepairShortlistV190:CriterionContract', ...
            'A shortlist criterion is invalid.');
    end
    if ~isempty(candidates)
        values = arrayfun(@(candidate) ...
            candidate.(criterion.field), candidates);
        if any(~isfinite(values))
            error('FormationRepairShortlistV190:CriterionValue', ...
                'A shortlist criterion contains a nonfinite value.');
        end
    end
end
end

function details = emptyDetails(criteria)
details = struct( ...
    'contractVersion', 'diverse-formation-repair-shortlist-v190-v1', ...
    'formationIds', zeros(1, 0), ...
    'inputCandidateCount', 0, ...
    'selectedCandidateIndices', zeros(1, 0), ...
    'selectedCandidateCount', 0, ...
    'selectedCountByFormation', zeros(1, 0), ...
    'criterionNames', {{criteria.name}}, ...
    'criterionDirections', [criteria.direction], ...
    'criterionHits', false(0, numel(criteria)), ...
    'rankMatrix', zeros(0, numel(criteria)), ...
    'perCriterion', 0, ...
    'maximumPerFormation', 0, ...
    'semanticLabelKeysUsedForTieBreakOnly', true, ...
    'truthUsed', false, ...
    'futureInformationUsed', false, ...
    'numericLabelIdentifiersUsedAsFeatures', false, ...
    'trackingBenefitClaimed', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
