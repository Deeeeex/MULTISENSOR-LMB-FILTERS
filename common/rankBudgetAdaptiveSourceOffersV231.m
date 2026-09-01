function result = rankBudgetAdaptiveSourceOffersV231( ...
        sourcePosterior, beneficiaryCacheBySensor, ...
        beneficiarySensorIds, cacheAgeBySensor, ...
        cacheEventTypeBySensor, model, bytePlan)
% RANKBUDGETADAPTIVESOURCEOFFERSV231 Fill a byte-derived proposal set.

protocol = getBudgetAdaptiveSourceOfferProtocolV231();
if ~isstruct(bytePlan) || ~isscalar(bytePlan) || ...
        ~isfield(bytePlan, 'protocolId') || ...
        ~strcmp(bytePlan.protocolId, protocol.id) || ...
        ~isfield(bytePlan, 'offersPerSource') || ...
        ~isfield(bytePlan, 'existenceBreadthPerSource') || ...
        ~isfield(bytePlan, 'spatialBreadthPerSource')
    error('BudgetAdaptiveSourceOfferV231:PlanContract', ...
        'A V231 byte-derived breadth plan is required.');
end

base = rankCausalSourceOffersV230( ...
    sourcePosterior, beneficiaryCacheBySensor, ...
    beneficiarySensorIds, cacheAgeBySensor, ...
    cacheEventTypeBySensor, model, struct());
result = base;
result.contractVersion = ...
    'budget-adaptive-source-offer-ranking-v231-v1';
result.protocolId = protocol.id;
result.scoreProtocolId = base.protocolId;
result.bytePlan = bytePlan;
result.maximumOffersPerSource = bytePlan.offersPerSource;
result.existenceBreadthPerSource = ...
    bytePlan.existenceBreadthPerSource;
result.spatialBreadthPerSource = ...
    bytePlan.spatialBreadthPerSource;
result.selectedCandidateIndices = zeros(1, 0);
result.selectedLabels = zeros(2, 0);
result.selectedModes = cell(1, 0);
result.selectedOfferCount = 0;
if ~strcmp(base.status, 'ranked') || ...
        ~bytePlan.controlExecutable || ...
        bytePlan.offersPerSource < 1
    result.status = 'no-budget-adaptive-proposals';
    result.evidenceBoundary = protocol.evidenceBoundary;
    return;
end

selectedLabels = zeros(2, 0);
selectedModes = cell(1, 0);
[selectedLabels, selectedModes] = addLabels( ...
    selectedLabels, selectedModes, base.existenceRanking, ...
    bytePlan.existenceBreadthPerSource, 'existence-surprise');
[selectedLabels, selectedModes] = addLabels( ...
    selectedLabels, selectedModes, base.spatialRanking, ...
    bytePlan.spatialBreadthPerSource, 'spatial-utility');
if size(selectedLabels, 2) < bytePlan.offersPerSource
    interleaved = interleaveRankings( ...
        base.existenceRanking, base.spatialRanking);
    [selectedLabels, selectedModes] = addLabels( ...
        selectedLabels, selectedModes, interleaved, ...
        bytePlan.offersPerSource - size(selectedLabels, 2), ...
        'breadth-fill');
end
if size(selectedLabels, 2) > bytePlan.offersPerSource
    selectedLabels = selectedLabels(:, 1:bytePlan.offersPerSource);
    selectedModes = selectedModes(1:bytePlan.offersPerSource);
end

selectedIndices = zeros(1, size(selectedLabels, 2));
for selectedPosition = 1:size(selectedLabels, 2)
    selectedIndices(selectedPosition) = findCandidateIndex( ...
        base.candidates, selectedLabels(:, selectedPosition));
end
candidates = base.candidates;
for candidateIndex = 1:numel(candidates)
    candidates(candidateIndex).selected = false;
    candidates(candidateIndex).mode = '';
end
for selectedPosition = 1:numel(selectedIndices)
    candidateIndex = selectedIndices(selectedPosition);
    candidates(candidateIndex).selected = true;
    candidates(candidateIndex).mode = selectedModes{selectedPosition};
end

result.status = 'ranked-budget-adaptive';
result.candidates = candidates;
result.selectedCandidateIndices = selectedIndices;
result.selectedLabels = selectedLabels;
result.selectedModes = selectedModes;
result.selectedOfferCount = size(selectedLabels, 2);
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function [selected, modes] = addLabels( ...
        selected, modes, ranking, count, mode)
added = 0;
for rankingPosition = 1:size(ranking, 2)
    if added >= count
        return;
    end
    label = ranking(:, rankingPosition);
    if ~isempty(selected) && any(all(selected == label, 1))
        continue;
    end
    selected(:, end + 1) = label; %#ok<AGROW>
    modes{end + 1} = mode; %#ok<AGROW>
    added = added + 1;
end
end

function labels = interleaveRankings(left, right)
labels = zeros(2, 0);
maximumDepth = max(size(left, 2), size(right, 2));
for depth = 1:maximumDepth
    if depth <= size(left, 2)
        labels(:, end + 1) = left(:, depth); %#ok<AGROW>
    end
    if depth <= size(right, 2)
        labels(:, end + 1) = right(:, depth); %#ok<AGROW>
    end
end
end

function index = findCandidateIndex(candidates, label)
index = 0;
for candidateIndex = 1:numel(candidates)
    if all(candidates(candidateIndex).label == label)
        index = candidateIndex;
        return;
    end
end
if index == 0
    error('BudgetAdaptiveSourceOfferV231:CandidateMapping', ...
        'A selected label is absent from the scored candidate set.');
end
end
