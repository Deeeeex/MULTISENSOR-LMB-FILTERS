function rmseSeries = computeSetRmseOverTime(stateEstimates, groundTruthRfs)
% COMPUTESETRMSEOVERTIME Hungarian-matched position RMSE over time.

simulationLength = numel(groundTruthRfs.x);
rmseSeries = NaN(1, simulationLength);
for currentTime = 1:simulationLength
    rmseSeries(currentTime) = computeSetRmseAtTime( ...
        stateEstimates, groundTruthRfs, currentTime);
end
end

function rmse = computeSetRmseAtTime(stateEstimates, groundTruthRfs, currentTime)
truthCells = groundTruthRfs.x{currentTime};
estimateCells = stateEstimates.mu{currentTime};
if isempty(truthCells) && isempty(estimateCells)
    rmse = 0;
    return;
end
if isempty(truthCells) || isempty(estimateCells)
    rmse = NaN;
    return;
end

truth = cell2mat(cellfun( ...
    @(state) state(1:2), truthCells, 'UniformOutput', false));
estimate = cell2mat(cellfun( ...
    @(state) state(1:2), estimateCells, 'UniformOutput', false));
distances = pairwiseDistances(truth, estimate);
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
if isempty(matched)
    rmse = NaN;
else
    rmse = sqrt(mean(matched .^ 2));
end
end

function distances = pairwiseDistances(left, right)
distances = zeros(size(left, 2), size(right, 2));
for leftIdx = 1:size(left, 2)
    for rightIdx = 1:size(right, 2)
        delta = left(:, leftIdx) - right(:, rightIdx);
        distances(leftIdx, rightIdx) = sqrt(delta' * delta);
    end
end
end
