function [shortlist, details] = ...
        selectModeDiversePositionSafeRepairCandidatesV217( ...
            candidates, options)
% SELECTMODEDIVERSEPOSITIONSAFEREPAIRCANDIDATESV217 Bound H3 queries.
%
% After the deterministic position-support projection, keep the union of
% the top candidates for three distinct causal hypotheses: precision
% refresh, receiver existence deficit, and disagreement rescue.  These are
% proposal modes, not predicted action values.

if nargin < 2 || isempty(options)
    options = struct();
end
topPerMode = getField(options, 'topPerMode', 3);
if ~isscalar(topPerMode) || ~isnumeric(topPerMode) || ...
        ~isfinite(topPerMode) || topPerMode < 1 || ...
        topPerMode ~= round(topPerMode) || ~isstruct(candidates)
    error('ModeDiversePositionSafeRepairV217:InvalidInput', ...
        'A V190 candidate array and positive Top-K are required.');
end
candidates = reshape(candidates, 1, []);
if isempty(candidates)
    shortlist = candidates;
    details = emptyDetails(topPerMode);
    return;
end
required = {'candidateIndex', 'formationId', 'sourceId', 'label', ...
    'opportunityMedian', 'positionSupport99Passed'};
if any(~isfield(candidates, required)) || ...
        any(~[candidates.positionSupport99Passed])
    error('ModeDiversePositionSafeRepairV217:CandidateContract', ...
        'Only position-safe V190 candidates may enter the mode shortlist.');
end

candidateCount = numel(candidates);
modeNames = { ...
    'precision-refresh', ...
    'receiver-existence-deficit', ...
    'disagreement-rescue'};
modeScores = zeros(candidateCount, numel(modeNames));
featureNames = cell(1, 0);
featureMatrix = zeros(candidateCount, 0);
for candidateIdx = 1:candidateCount
    [values, names] = buildFormationLabelActionModeFeaturesV202( ...
        candidates(candidateIdx));
    if candidateIdx == 1
        featureNames = names;
        featureMatrix = zeros(candidateCount, numel(values));
    elseif ~isequal(featureNames, names)
        error('ModeDiversePositionSafeRepairV217:FeatureDrift', ...
            'V202 action-mode features changed between candidates.');
    end
    featureMatrix(candidateIdx, :) = values;
end
modeScores(:, 1) = featureMatrix(:, uniqueFeatureIndex( ...
    featureNames, 'precision_refresh_evidence'));
modeScores(:, 2) = featureMatrix(:, uniqueFeatureIndex( ...
    featureNames, 'receiver_existence_deficit'));
modeScores(:, 3) = reshape([candidates.opportunityMedian], [], 1);
if any(~isfinite(modeScores(:))) || any(modeScores(:) < 0)
    error('ModeDiversePositionSafeRepairV217:InvalidModeScore', ...
        'Every causal proposal score must be finite and nonnegative.');
end

formationIds = unique([candidates.formationId], 'stable');
selectedByMode = false(candidateCount, numel(modeNames));
for formationId = reshape(formationIds, 1, [])
    formationRows = find([candidates.formationId] == formationId);
    for modeIdx = 1:numel(modeNames)
        orderTable = [ ...
            -modeScores(formationRows, modeIdx), ...
            reshape([candidates(formationRows).candidateIndex], [], 1)];
        [~, order] = sortrows(orderTable, [1, 2]);
        keep = formationRows(order( ...
            1:min(topPerMode, numel(order))));
        selectedByMode(keep, modeIdx) = true;
    end
end
selectedRows = find(any(selectedByMode, 2));
shortlist = candidates(selectedRows);

details = struct();
details.contractVersion = ...
    'mode-diverse-position-safe-repair-shortlist-v217-v1';
details.modeNames = modeNames;
details.topPerMode = topPerMode;
details.maximumPerFormation = topPerMode * numel(modeNames);
details.formationIds = formationIds;
details.inputCandidateCount = candidateCount;
details.shortlistedCandidateCount = numel(shortlist);
details.selectedCandidateIndices = reshape( ...
    [shortlist.candidateIndex], 1, []);
details.selectedByMode = selectedByMode(selectedRows, :);
details.modeScores = modeScores(selectedRows, :);
details.featureNames = featureNames;
details.featureMatrix = featureMatrix(selectedRows, :);
details.semanticIdentifiersUsedAsFeatures = false;
details.modeScoresAreActionValues = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function idx = uniqueFeatureIndex(names, requested)
idx = find(strcmp(names, requested));
if numel(idx) ~= 1
    error('ModeDiversePositionSafeRepairV217:MissingFeature', ...
        'Required action-mode feature is missing: %s.', requested);
end
end

function details = emptyDetails(topPerMode)
details = struct( ...
    'contractVersion', ...
        'mode-diverse-position-safe-repair-shortlist-v217-v1', ...
    'modeNames', {{ ...
        'precision-refresh', ...
        'receiver-existence-deficit', ...
        'disagreement-rescue'}}, ...
    'topPerMode', topPerMode, ...
    'maximumPerFormation', 3 * topPerMode, ...
    'formationIds', zeros(1, 0), ...
    'inputCandidateCount', 0, ...
    'shortlistedCandidateCount', 0, ...
    'selectedCandidateIndices', zeros(1, 0), ...
    'selectedByMode', false(0, 3), ...
    'modeScores', zeros(0, 3), ...
    'featureNames', {cell(1, 0)}, ...
    'featureMatrix', zeros(0, 0), ...
    'semanticIdentifiersUsedAsFeatures', false, ...
    'modeScoresAreActionValues', false, ...
    'truthUsed', false, ...
    'futureInformationUsed', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
