function [measurementsDelivered, commStats] = applyCommunicationModel(measurements, model, commConfig)
% APPLYCOMMUNICATIONMODEL - Apply multi-level communication constraints to measurements.
%   [measurementsDelivered, commStats] = applyCommunicationModel(measurements, model, commConfig)
%
%   Applies global measurement budget, link loss, and node outage models
%   before measurements are passed to a filter. This function is designed
%   to be called by entry scripts (e.g., runMultisensorFilters).
%
%   Inputs
%       measurements - cell array. For multi-sensor: sensors x time cell matrix.
%       model - struct. Used for numberOfSensors if provided.
%       commConfig - struct. Communication configuration (see docs).
%
%   Outputs
%       measurementsDelivered - cell array. Same shape as input measurements.
%       commStats - struct. Basic statistics about drops and outages.

if nargin < 3
    commConfig = struct();
end
if nargin < 2
    model = struct();
end

originalSize = size(measurements);
isVector = isvector(measurements);
if isVector
    measurements = reshape(measurements, 1, []);
end

if ~iscell(measurements)
    error('measurements must be a cell array.');
end

numSensors = size(measurements, 1);
numSteps = size(measurements, 2);
if isfield(model, 'numberOfSensors')
    numSensors = model.numberOfSensors;
end

level = getField(commConfig, 'level', 0);

% Defaults
globalMaxMeasurements = getField(commConfig, 'globalMaxMeasurementsPerStep', []);
if isempty(globalMaxMeasurements)
    globalMaxMeasurements = getField(commConfig, 'globalMaxSensorPacketsPerStep', []);
end
if isempty(globalMaxMeasurements)
    globalMaxMeasurements = inf;
end
sensorWeights = getField(commConfig, 'sensorWeights', ones(1, numSensors));
priorityPolicy = getField(commConfig, 'priorityPolicy', 'weightedPriority');
measurementSelectionPolicy = getField(commConfig, 'measurementSelectionPolicy', 'firstK');

linkModel = getField(commConfig, 'linkModel', 'fixed');
pDrop = getField(commConfig, 'pDrop', 0);
pGoodToBad = getField(commConfig, 'pGoodToBad', 0.1);
pBadToGood = getField(commConfig, 'pBadToGood', 0.3);
pDropGood = getField(commConfig, 'pDropGood', 0.05);
pDropBad = getField(commConfig, 'pDropBad', 0.4);

maxOutageNodes = getField(commConfig, 'maxOutageNodes', 1);
outageSchedule = getField(commConfig, 'outageSchedule', []);
outageMinDuration = getField(commConfig, 'outageMinDuration', 10);
outageMaxDuration = getField(commConfig, 'outageMaxDuration', 30);

if numel(sensorWeights) ~= numSensors
    sensorWeights = ones(1, numSensors);
end

measurementsDelivered = measurements;

commStats = struct();
commStats.allowedPacketsPerStep = zeros(1, numSteps);
commStats.allowedMeasurementsPerStep = zeros(1, numSteps);
commStats.droppedByBandwidth = zeros(numSensors, numSteps);
commStats.droppedByLink = zeros(numSensors, numSteps);
commStats.droppedByOutage = zeros(numSensors, numSteps);
commStats.linkState = zeros(numSensors, numSteps); % 0=good, 1=bad
commStats.outageSchedule = outageSchedule;

if level <= 0
    if isVector
        measurementsDelivered = reshape(measurementsDelivered, originalSize);
    end
    return;
end

if level >= 3 && isempty(outageSchedule)
    outageSchedule = generateRandomOutage(numSensors, numSteps, maxOutageNodes, outageMinDuration, outageMaxDuration);
    commStats.outageSchedule = outageSchedule;
end

if strcmpi(linkModel, 'markov')
    isBad = false(numSensors, 1);
end

for t = 1:numSteps
    % Clear delivered packets for this time step
    for s = 1:numSensors
        measurementsDelivered{s, t} = {};
    end

    % Identify sensors with non-empty packets
    hasPacket = false(numSensors, 1);
    for s = 1:numSensors
        hasPacket(s) = numel(measurements{s, t}) > 0;
    end

    % Level 1: global measurement budget
    remainingBudget = globalMaxMeasurements;
    allowedSensors = false(numSensors, 1);
    if level >= 1
        candidates = find(hasPacket);
        ordered = prioritizeSensors(candidates, sensorWeights, priorityPolicy);
        for idx = 1:numel(ordered)
            s = ordered(idx);
            if remainingBudget <= 0
                break;
            end
            count = numel(measurements{s, t});
            if count <= remainingBudget
                measurementsDelivered{s, t} = measurements{s, t};
                remainingBudget = remainingBudget - count;
                allowedSensors(s) = true;
            else
                measurementsDelivered{s, t} = selectMeasurements(measurements{s, t}, remainingBudget, measurementSelectionPolicy);
                commStats.droppedByBandwidth(s, t) = count - remainingBudget;
                remainingBudget = 0;
                allowedSensors(s) = true;
            end
        end
        if remainingBudget <= 0
            droppedSensors = hasPacket & ~allowedSensors;
            for s = find(droppedSensors)'
                commStats.droppedByBandwidth(s, t) = numel(measurements{s, t});
            end
        end
    else
        for s = find(hasPacket)'
            measurementsDelivered{s, t} = measurements{s, t};
        end
        allowedSensors = hasPacket;
    end

    commStats.allowedPacketsPerStep(t) = sum(allowedSensors);
    commStats.allowedMeasurementsPerStep(t) = sum(cellfun(@numel, measurementsDelivered(:, t)));

    % Level 2: link loss
    if level >= 2
        for s = find(allowedSensors)'
            deliveredCount = numel(measurementsDelivered{s, t});
            if deliveredCount == 0
                continue;
            end
            dropPacket = false;
            if strcmpi(linkModel, 'markov')
                if isBad(s)
                    if rand < pBadToGood
                        isBad(s) = false;
                    end
                else
                    if rand < pGoodToBad
                        isBad(s) = true;
                    end
                end
                commStats.linkState(s, t) = isBad(s);
                if isBad(s)
                    dropPacket = rand < pDropBad;
                else
                    dropPacket = rand < pDropGood;
                end
            else
                dropPacket = rand < pDrop;
            end
            if dropPacket
                commStats.droppedByLink(s, t) = deliveredCount;
                measurementsDelivered{s, t} = {};
            end
        end
    end

    % Level 3: node outage (max nodes)
    if level >= 3 && ~isempty(outageSchedule)
        outagedSensors = sensorsOutagedAt(outageSchedule, t);
        for s = outagedSensors(:)'
            deliveredCount = numel(measurementsDelivered{s, t});
            commStats.droppedByOutage(s, t) = deliveredCount;
            measurementsDelivered{s, t} = {};
        end
    end
end

if isVector
    measurementsDelivered = reshape(measurementsDelivered, originalSize);
end
end

function value = getField(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function ordered = prioritizeSensors(sensors, weights, policy)
sensorWeights = weights(sensors);
switch lower(policy)
    case 'weightedshuffle'
        % Sort by weight, shuffle within equal-weight groups
        [sortedWeights, sortIdx] = sort(sensorWeights, 'descend');
        ordered = sensors(sortIdx);
        uniqueWeights = unique(sortedWeights);
        for i = 1:numel(uniqueWeights)
            w = uniqueWeights(i);
            idx = find(sortedWeights == w);
            if numel(idx) > 1
                shuffled = idx(randperm(numel(idx)));
                ordered(idx) = ordered(shuffled);
            end
        end
    otherwise
        % weightedPriority
        [~, sortIdx] = sort(sensorWeights, 'descend');
        ordered = sensors(sortIdx);
end
end

function selected = selectMeasurements(measurementsCell, k, policy)
if k <= 0
    selected = {};
    return;
end
count = numel(measurementsCell);
if k >= count
    selected = measurementsCell;
    return;
end
switch lower(policy)
    case 'random'
        idx = randperm(count, k);
        selected = measurementsCell(idx);
    otherwise
        % firstK
        selected = measurementsCell(1:k);
end
end

function schedule = generateRandomOutage(numSensors, numSteps, maxOutageNodes, minDuration, maxDuration)
schedule = struct('sensor', {}, 'start', {}, 'end', {});
if maxOutageNodes <= 0 || numSensors <= 0
    return;
end

numOutageNodes = min(maxOutageNodes, numSensors);
sensorIds = randperm(numSensors, numOutageNodes);
for i = 1:numOutageNodes
    duration = randi([minDuration, maxDuration]);
    if duration >= numSteps
        startTime = 1;
        endTime = numSteps;
    else
        startTime = randi([1, numSteps - duration + 1]);
        endTime = startTime + duration - 1;
    end
    schedule(end+1).sensor = sensorIds(i);
    schedule(end).start = startTime;
    schedule(end).end = endTime;
end
end

function sensors = sensorsOutagedAt(schedule, t)
sensors = [];
for i = 1:numel(schedule)
    if t >= schedule(i).start && t <= schedule(i).end
        sensors(end+1) = schedule(i).sensor; %#ok<AGROW>
    end
end
end
