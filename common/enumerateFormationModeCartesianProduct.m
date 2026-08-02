function [modeVectors, actionIndices] = ...
    enumerateFormationModeCartesianProduct(modeChoices, modeCount)
% ENUMERATEFORMATIONMODECARTESIANPRODUCT Structured per-formation grid.

if ~iscell(modeChoices) || isempty(modeChoices) || ...
        ~isscalar(modeCount) || ~isfinite(modeCount) || ...
        modeCount ~= round(modeCount) || modeCount < 1
    error('Formation mode Cartesian-product inputs are invalid.');
end

formationCount = numel(modeChoices);
normalizedChoices = cell(1, formationCount);
candidateCount = 1;
for formationIdx = 1:formationCount
    choices = reshape(modeChoices{formationIdx}, 1, []);
    if isempty(choices) || any(~isfinite(choices)) || ...
            any(choices ~= round(choices)) || any(choices < 1) || ...
            any(choices > modeCount) || numel(unique(choices)) ~= ...
                numel(choices)
        error('Formation mode choices are invalid.');
    end
    normalizedChoices{formationIdx} = choices;
    candidateCount = candidateCount * numel(choices);
end

modeVectors = zeros(candidateCount, formationCount);
repeatBlock = candidateCount;
for formationIdx = 1:formationCount
    choices = normalizedChoices{formationIdx};
    repeatBlock = repeatBlock / numel(choices);
    pattern = reshape(repmat(choices, repeatBlock, 1), 1, []);
    pattern = repmat(pattern, 1, ...
        candidateCount / numel(pattern));
    modeVectors(:, formationIdx) = pattern(:);
end
actionIndices = formationModeVectorToActionIndex( ...
    modeVectors, modeCount);

if size(unique(modeVectors, 'rows'), 1) ~= candidateCount || ...
        numel(unique(actionIndices)) ~= candidateCount
    error('Formation mode Cartesian product is not unique.');
end
end
