function [stateEstimatesBySensor, localModels, neighborMap] = runDistributedLmbFilter(model, measurements, sensorTrajectories, neighborMap, commStats)
% RUNDISTRIBUTEDLMBFILTER - Run local fusion at each sensor using neighbor data
%   [stateEstimatesBySensor, localModels, neighborMap] =
%   runDistributedLmbFilter(model, measurements, sensorTrajectories, neighborMap)
%
%   Each sensor performs local fusion using its own measurements plus those
%   received from neighboring sensors. There is no centralized fusion.
%
%   Inputs
%       model - full multi-sensor model
%       measurements - cell array [S x T] of measurements
%       sensorTrajectories - cell array of sensor trajectories (optional)
%       neighborMap - cell array of neighbor indices per sensor (optional)
%
%   Outputs
%       stateEstimatesBySensor - cell array of state estimates per sensor
%       localModels - cell array of per-sensor sub-models
%       neighborMap - resolved neighbor map

numberOfSensors = model.numberOfSensors;
if nargin < 3
    sensorTrajectories = [];
end
if nargin < 4 || isempty(neighborMap)
    neighborMap = computeNeighborMap(model, sensorTrajectories);
end

stateEstimatesBySensor = cell(1, numberOfSensors);
localModels = cell(1, numberOfSensors);

if isfield(model, 'fusionWeighting')
    fusionWeighting = model.fusionWeighting;
else
    fusionWeighting = 'Uniform';
end
if strcmpi(fusionWeighting, 'Metropolis')
    weightsBySensor = computeMetropolisWeights(neighborMap);
else
    weightsBySensor = computeUniformWeights(neighborMap);
end

for s = 1:numberOfSensors
    neighborIdx = neighborMap{s};
    localModels{s} = buildSubModel(model, neighborIdx);
    localModels{s}.gaSensorWeights = weightsBySensor{s};
    localModels{s}.aaSensorWeights = weightsBySensor{s};
    localMeasurements = measurements(neighborIdx, :);
    localSensorTraj = [];
    if ~isempty(sensorTrajectories)
        localSensorTraj = sensorTrajectories(neighborIdx);
        localModels{s}.sensorTrajectories = localSensorTraj;
    end
    localCommStats = [];
    if nargin >= 5 && isstruct(commStats)
        localCommStats = sliceCommStats(commStats, neighborIdx);
    end
    stateEstimatesBySensor{s} = runParallelUpdateLmbFilter(localModels{s}, localMeasurements, localCommStats, localSensorTraj);
end
end

function neighborMap = computeNeighborMap(model, sensorTrajectories)
    numberOfSensors = model.numberOfSensors;
    neighborMap = cell(1, numberOfSensors);
    if isfield(model, 'sensorCommRange')
        commRange = model.sensorCommRange;
    else
        commRange = 50;
    end

    positions = zeros(2, numberOfSensors);
    if ~isempty(sensorTrajectories)
        for s = 1:numberOfSensors
            positions(:, s) = sensorTrajectories{s}(1:2, 1);
        end
    elseif isfield(model, 'sensorInitialStates') && ~isempty(model.sensorInitialStates)
        for s = 1:numberOfSensors
            positions(:, s) = model.sensorInitialStates{s}(1:2);
        end
    else
        positions = zeros(2, numberOfSensors);
    end

    for s = 1:numberOfSensors
        deltas = positions - positions(:, s);
        dists = sqrt(sum(deltas.^2, 1));
        neighborIdx = find(dists <= commRange);
        if isempty(neighborIdx)
            neighborIdx = s;
        end
        neighborMap{s} = neighborIdx;
    end
end

function subModel = buildSubModel(model, sensorIdx)
    subModel = model;
    subModel.numberOfSensors = numel(sensorIdx);
    subModel.C = model.C(sensorIdx);
    subModel.Q = model.Q(sensorIdx);
    subModel.clutterRate = model.clutterRate(sensorIdx);
    subModel.clutterPerUnitVolume = model.clutterPerUnitVolume(sensorIdx);
    subModel.detectionProbability = model.detectionProbability(sensorIdx);
end

function localCommStats = sliceCommStats(commStats, sensorIdx)
    localCommStats = commStats;
    if isfield(commStats, 'droppedByBandwidth')
        localCommStats.droppedByBandwidth = commStats.droppedByBandwidth(sensorIdx, :);
    end
    if isfield(commStats, 'droppedByLink')
        localCommStats.droppedByLink = commStats.droppedByLink(sensorIdx, :);
    end
    if isfield(commStats, 'droppedByOutage')
        localCommStats.droppedByOutage = commStats.droppedByOutage(sensorIdx, :);
    end
    if isfield(commStats, 'innovationConsistency')
        localCommStats.innovationConsistency = commStats.innovationConsistency(sensorIdx, :);
    end
end

function weightsBySensor = computeUniformWeights(neighborMap)
    numberOfSensors = numel(neighborMap);
    weightsBySensor = cell(1, numberOfSensors);
    for s = 1:numberOfSensors
        n = numel(neighborMap{s});
        weightsBySensor{s} = ones(1, n) / n;
    end
end

function weightsBySensor = computeMetropolisWeights(neighborMap)
    numberOfSensors = numel(neighborMap);
    degrees = zeros(1, numberOfSensors);
    for s = 1:numberOfSensors
        degrees(s) = numel(neighborMap{s});
    end
    weightsBySensor = cell(1, numberOfSensors);
    for s = 1:numberOfSensors
        neighborIdx = neighborMap{s};
        nLocal = numel(neighborIdx);
        w = zeros(1, nLocal);
        sumOff = 0;
        for k = 1:nLocal
            j = neighborIdx(k);
            if j ~= s
                w(k) = 1 / (1 + max(degrees(s), degrees(j)));
                sumOff = sumOff + w(k);
            end
        end
        selfIdx = find(neighborIdx == s, 1, 'first');
        if isempty(selfIdx)
            neighborIdx(end+1) = s;
            w(end+1) = 0;
            selfIdx = numel(w);
        end
        w(selfIdx) = max(0, 1 - sumOff);
        if abs(sum(w) - 1) > 1e-6
            w = w / sum(w);
        end
        weightsBySensor{s} = w;
    end
end
