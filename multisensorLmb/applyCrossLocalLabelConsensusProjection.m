function projectedEstimates = applyCrossLocalLabelConsensusProjection(stateEstimatesBySensor, model)
% APPLYCROSSLOCALLABELCONSENSUSPROJECTION Project local outputs onto a shared label set.
%
% This is an estimate-level consensus pass: at each time step it selects a
% cardinality-median medoid local estimate as the label reference, matches
% other local outputs to that reference, and moment-averages matched states.

projectedEstimates = stateEstimatesBySensor;
if isempty(stateEstimatesBySensor)
    return;
end

numberOfSensors = numel(stateEstimatesBySensor);
simulationLength = numel(stateEstimatesBySensor{1}.mu);
stateDimension = resolveStateDimension(stateEstimatesBySensor);
cutoff = resolveConsensusCutoff(model);

for currentTime = 1:simulationLength
    referenceIdx = selectReferenceSensor(stateEstimatesBySensor, currentTime, cutoff);
    if referenceIdx <= 0
        for sensorIdx = 1:numberOfSensors
            projectedEstimates{sensorIdx}.labels{currentTime} = zeros(2, 0);
            projectedEstimates{sensorIdx}.mu{currentTime} = {};
            projectedEstimates{sensorIdx}.Sigma{currentTime} = {};
        end
        continue;
    end

    referenceLabels = stateEstimatesBySensor{referenceIdx}.labels{currentTime};
    referenceMu = stateEstimatesBySensor{referenceIdx}.mu{currentTime};
    referenceSigma = stateEstimatesBySensor{referenceIdx}.Sigma{currentTime};
    referenceCount = numel(referenceMu);
    if referenceCount <= 0
        consensusLabels = zeros(2, 0);
        consensusMu = {};
        consensusSigma = {};
    else
        [consensusMu, consensusSigma] = buildConsensusStates( ...
            stateEstimatesBySensor, currentTime, referenceMu, referenceSigma, ...
            stateDimension);
        consensusLabels = referenceLabels(:, 1:referenceCount);
    end

    for sensorIdx = 1:numberOfSensors
        projectedEstimates{sensorIdx}.labels{currentTime} = consensusLabels;
        projectedEstimates{sensorIdx}.mu{currentTime} = consensusMu;
        projectedEstimates{sensorIdx}.Sigma{currentTime} = consensusSigma;
    end
end
end

function referenceIdx = selectReferenceSensor(stateEstimatesBySensor, currentTime, cutoff)
numberOfSensors = numel(stateEstimatesBySensor);
counts = zeros(1, numberOfSensors);
for sensorIdx = 1:numberOfSensors
    counts(sensorIdx) = numel(stateEstimatesBySensor{sensorIdx}.mu{currentTime});
end
if all(counts == 0)
    referenceIdx = 0;
    return;
end

medianCount = median(counts);
candidateMask = abs(counts - medianCount) == min(abs(counts - medianCount));
candidates = find(candidateMask);
meanDistance = inf(1, numel(candidates));
for candidateIdx = 1:numel(candidates)
    sensorIdx = candidates(candidateIdx);
    distances = zeros(1, numberOfSensors);
    for otherIdx = 1:numberOfSensors
        distances(otherIdx) = estimateSetDistance( ...
            stateEstimatesBySensor{sensorIdx}.mu{currentTime}, ...
            stateEstimatesBySensor{otherIdx}.mu{currentTime}, cutoff);
    end
    meanDistance(candidateIdx) = mean(distances);
end
[~, bestLocalIdx] = min(meanDistance);
referenceIdx = candidates(bestLocalIdx);
end

function [consensusMu, consensusSigma] = buildConsensusStates( ...
    stateEstimatesBySensor, currentTime, referenceMu, referenceSigma, stateDimension)
referenceCount = numel(referenceMu);
matchedMeans = cell(1, referenceCount);
matchedCovariances = cell(1, referenceCount);
for referenceIdx = 1:referenceCount
    matchedMeans{referenceIdx} = {};
    matchedCovariances{referenceIdx} = {};
end
referencePositions = extractPositions(referenceMu);

for sensorIdx = 1:numel(stateEstimatesBySensor)
    localMu = stateEstimatesBySensor{sensorIdx}.mu{currentTime};
    localSigma = stateEstimatesBySensor{sensorIdx}.Sigma{currentTime};
    if isempty(localMu)
        continue;
    end
    localPositions = extractPositions(localMu);
    distances = pairwiseDistances(localPositions, referencePositions);
    [matching, ~] = Hungarian(distances);
    for localIdx = 1:size(matching, 1)
        referenceIdx = find(matching(localIdx, :) == 1, 1);
        if isempty(referenceIdx)
            continue;
        end
        matchedMeans{referenceIdx}{end+1} = localMu{localIdx};
        matchedCovariances{referenceIdx}{end+1} = localSigma{localIdx};
    end
end

consensusMu = cell(1, referenceCount);
consensusSigma = cell(1, referenceCount);
for referenceIdx = 1:referenceCount
    if isempty(matchedMeans{referenceIdx})
        matchedMeans{referenceIdx} = {referenceMu{referenceIdx}};
        matchedCovariances{referenceIdx} = {referenceSigma{referenceIdx}};
    end
    [consensusMu{referenceIdx}, consensusSigma{referenceIdx}] = ...
        momentMatchStates(matchedMeans{referenceIdx}, ...
            matchedCovariances{referenceIdx}, stateDimension);
end
end

function [mu, Sigma] = momentMatchStates(means, covariances, stateDimension)
count = numel(means);
mu = zeros(stateDimension, 1);
for idx = 1:count
    mu = mu + means{idx};
end
mu = mu / max(count, 1);

Sigma = zeros(stateDimension, stateDimension);
for idx = 1:count
    delta = means{idx} - mu;
    Sigma = Sigma + covariances{idx} + delta * delta';
end
Sigma = regularizeCovariance(Sigma / max(count, 1));
end

function distance = estimateSetDistance(leftMu, rightMu, cutoff)
if isempty(leftMu) && isempty(rightMu)
    distance = 0;
    return;
end
if isempty(leftMu) || isempty(rightMu)
    distance = cutoff;
    return;
end
leftPositions = extractPositions(leftMu);
rightPositions = extractPositions(rightMu);
distances = pairwiseDistances(leftPositions, rightPositions);
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
unmatchedCount = abs(numel(leftMu) - numel(rightMu));
cost = sum(matched .^ 2) + unmatchedCount * cutoff ^ 2;
distance = sqrt(cost / max(numel(leftMu), numel(rightMu)));
end

function positions = extractPositions(muCells)
if isempty(muCells)
    positions = zeros(2, 0);
else
    positions = cell2mat(cellfun(@(state) state(1:2), ...
        muCells, 'UniformOutput', false));
end
end

function distances = pairwiseDistances(leftPositions, rightPositions)
distances = zeros(size(leftPositions, 2), size(rightPositions, 2));
for leftIdx = 1:size(leftPositions, 2)
    for rightIdx = 1:size(rightPositions, 2)
        delta = leftPositions(:, leftIdx) - rightPositions(:, rightIdx);
        distances(leftIdx, rightIdx) = sqrt(delta' * delta);
    end
end
end

function stateDimension = resolveStateDimension(stateEstimatesBySensor)
stateDimension = 4;
for sensorIdx = 1:numel(stateEstimatesBySensor)
    for timeIdx = 1:numel(stateEstimatesBySensor{sensorIdx}.mu)
        localMu = stateEstimatesBySensor{sensorIdx}.mu{timeIdx};
        if ~isempty(localMu)
            stateDimension = numel(localMu{1});
            return;
        end
    end
end
end

function cutoff = resolveConsensusCutoff(model)
cutoff = [];
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion) && ...
        isfield(model.adaptiveFusion, 'crossLocalConsensusCutoff')
    candidate = model.adaptiveFusion.crossLocalConsensusCutoff;
    if isnumeric(candidate) && isfinite(candidate) && candidate > 0
        cutoff = candidate;
    end
end
if isempty(cutoff) && isfield(model, 'ospaParameters') && ...
        isfield(model.ospaParameters, 'eC')
    cutoff = model.ospaParameters.eC;
end
if isempty(cutoff) || ~isnumeric(cutoff) || ~isfinite(cutoff) || cutoff <= 0
    cutoff = 5;
end
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
if isempty(covariance)
    return;
end
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end
