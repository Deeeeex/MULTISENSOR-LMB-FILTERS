function [modeVectors, actionIndices, distances] = ...
    enumerateFormationModeVectorHammingBall( ...
        centerModes, modeCount, maximumDistance)
% ENUMERATEFORMATIONMODEVECTORHAMMINGBALL Deterministic local vector set.

centerModes = reshape(centerModes, 1, []);
formationCount = numel(centerModes);
if formationCount < 1 || ~isscalar(modeCount) || ...
        ~isfinite(modeCount) || modeCount ~= round(modeCount) || ...
        modeCount < 2 || any(~isfinite(centerModes)) || ...
        any(centerModes ~= round(centerModes)) || ...
        any(centerModes < 1) || any(centerModes > modeCount) || ...
        ~isscalar(maximumDistance) || ...
        ~isfinite(maximumDistance) || ...
        maximumDistance ~= round(maximumDistance) || ...
        maximumDistance < 0 || maximumDistance > formationCount
    error('Formation mode-vector Hamming-ball request is invalid.');
end
actionCount = modeCount ^ formationCount;
allModes = zeros(actionCount, formationCount);
for actionIdx = 1:actionCount
    code = actionIdx - 1;
    for formationIdx = formationCount:-1:1
        allModes(actionIdx, formationIdx) = ...
            mod(code, modeCount) + 1;
        code = floor(code / modeCount);
    end
end
allDistances = sum(allModes ~= centerModes, 2);
included = allDistances <= maximumDistance;
modeVectors = allModes(included, :);
actionIndices = find(included);
distances = allDistances(included);
[~, order] = sortrows([distances, actionIndices], [1, 2]);
modeVectors = modeVectors(order, :);
actionIndices = reshape(actionIndices(order), 1, []);
distances = reshape(distances(order), 1, []);
end
