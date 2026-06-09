function [positionDisagreement, cardinalityDispersion, ospaDisagreement] = ...
    computeDistributedConsensusMetrics(stateEstimatesBySensor, model)
% COMPUTEDISTRIBUTEDCONSENSUSMETRICS Network disagreement over time.

numberOfSensors = numel(stateEstimatesBySensor);
simulationLength = numel(stateEstimatesBySensor{1}.mu);
positionDisagreement = NaN(1, simulationLength);
cardinalityDispersion = zeros(1, simulationLength);
ospaDisagreement = zeros(1, simulationLength);

for currentTime = 1:simulationLength
    counts = zeros(1, numberOfSensors);
    for sensorIdx = 1:numberOfSensors
        counts(sensorIdx) = numel( ...
            stateEstimatesBySensor{sensorIdx}.mu{currentTime});
    end
    cardinalityDispersion(currentTime) = ...
        mean(abs(counts - median(counts)));

    positionValues = [];
    ospaValues = [];
    for leftIdx = 1:numberOfSensors-1
        for rightIdx = leftIdx+1:numberOfSensors
            positionValue = estimateSetRmse( ...
                stateEstimatesBySensor{leftIdx}, ...
                stateEstimatesBySensor{rightIdx}, currentTime);
            if isfinite(positionValue)
                positionValues(end+1) = positionValue; %#ok<AGROW>
            end
            ospaValues(end+1) = estimateSetOspa( ...
                stateEstimatesBySensor{leftIdx}, ...
                stateEstimatesBySensor{rightIdx}, currentTime, model); %#ok<AGROW>
        end
    end
    if ~isempty(positionValues)
        positionDisagreement(currentTime) = mean(positionValues);
    end
    if ~isempty(ospaValues)
        ospaDisagreement(currentTime) = mean(ospaValues);
    end
end
end

function distance = estimateSetRmse(leftEstimate, rightEstimate, currentTime)
leftCells = leftEstimate.mu{currentTime};
rightCells = rightEstimate.mu{currentTime};
if isempty(leftCells) && isempty(rightCells)
    distance = 0;
    return;
end
if isempty(leftCells) || isempty(rightCells)
    distance = NaN;
    return;
end

left = cell2mat(cellfun( ...
    @(state) state(1:2), leftCells, 'UniformOutput', false));
right = cell2mat(cellfun( ...
    @(state) state(1:2), rightCells, 'UniformOutput', false));
distances = pairwiseDistances(left, right);
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
if isempty(matched)
    distance = NaN;
else
    distance = sqrt(mean(matched .^ 2));
end
end

function distance = estimateSetOspa(leftEstimate, rightEstimate, currentTime, model)
leftMu = leftEstimate.mu{currentTime};
leftSigma = leftEstimate.Sigma{currentTime};
rightMu = rightEstimate.mu{currentTime};
rightSigma = rightEstimate.Sigma{currentTime};
if isempty(leftMu) && isempty(rightMu)
    distance = 0;
    return;
end
if isempty(leftMu) || isempty(rightMu)
    distance = model.ospaParameters.eC;
    return;
end
[leftToRight, ~] = ospa( ...
    leftMu, leftMu, leftSigma, rightMu, rightSigma, model.ospaParameters);
[rightToLeft, ~] = ospa( ...
    rightMu, rightMu, rightSigma, leftMu, leftSigma, model.ospaParameters);
distance = 0.5 * (leftToRight(1) + rightToLeft(1));
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
