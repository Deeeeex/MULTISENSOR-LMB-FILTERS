function stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats, sensorTrajectories)
% RUNPARALLELUPDATELMBFILTER -- Run a multi-sensor LMB filter that uses a parallel measurment update.
%   stateEstimates = runParallelUpdateLmbFilter(model, measurements)
%   stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats)
%
%   Run a multi-sensor LMB filter that uses a parallel measurment update.
%   Measurement update variants include arithmetic average (AA), geometric
%   average (GA), and parallel update (PU) update.
%   File guide:
%       Main centralized multi-sensor LMB driver. It performs one prediction
%       per time step, runs a local association/update for each sensor, then
%       fuses the local posteriors using PU, GA, or AA. Adaptive weighting
%       and communication diagnostics are wired here.
%
%   See also generateMultisensorModel, generateMultisensorGroundTruth, lmbPredictionStep,
%   generateLmbSensorAssociationMatrices, loopyBeliefPropagation, lmbGibbsSampling, 
%   lmbMurtysAlgorithm, computePosteriorLmbSpatialDistributions,
%   lmbMapCardinalityEstimate, aaLmbTrackMerging, gaLmbTrackMerging,
%   puLmbTrackMerging.
%
%   Inputs
%       model - struct. A struct with the fields declared in generateModel.
%       measurements - cell array. An array containing the measurements for
%           each time-step of the simulation. See also generateModel.
%       commStats - struct. Communication statistics for link quality (optional).
%
%   Output
%       stateEstimates - struct. A struct containing the LMB filter's
%           approximate MAP estimate for each time-step of the simulation, as
%           well as the objects' trajectories.

%% Initialise variables
simulationLength = length(measurements);
% Struct containing objects' Bernoulli parameters and metadata
objects = model.object;

% Phase 1: Mobile Sensor Support - Pass sensor trajectories to model
if nargin >= 3 && ~isempty(sensorTrajectories)
    model.sensorTrajectories = sensorTrajectories;
end
% Output struct
stateEstimates.labels = cell(simulationLength, 1);
stateEstimates.mu = cell(simulationLength, 1);
stateEstimates.Sigma = cell(simulationLength, 1);
stateEstimates.objects = objects;
% Adaptive fusion config
useAdaptiveFusion = false;
if isfield(model, 'adaptiveFusion') && isfield(model.adaptiveFusion, 'enabled')
    useAdaptiveFusion = model.adaptiveFusion.enabled;
end
adaptiveCfg = struct();
if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
    adaptiveCfg = model.adaptiveFusion;
end
useNIS = getConfigField(adaptiveCfg, 'useNIS', true);
progressEverySteps = max(round(getConfigField(adaptiveCfg, 'progressEverySteps', 0)), 0);
progressLabel = getConfigField(adaptiveCfg, 'progressLabel', '');
    useNisEma = getConfigField(adaptiveCfg, 'nisEmaEnabled', true);
    nisEmaAlpha = getConfigField(adaptiveCfg, 'nisEmaAlpha', 0.7);
    prevWeights = struct();
    prevWeights.ga = model.gaSensorWeights;
    prevWeights.aa = model.aaSensorWeights;
    prevWeights.gaSpatial = getConfigField(model, 'gaSpatialWeights', model.gaSensorWeights);
    prevWeights.aaSpatial = getConfigField(model, 'aaSpatialWeights', model.aaSensorWeights);
    prevWeights.gaExistence = getConfigField(model, 'gaExistenceWeights', model.gaSensorWeights);
    prevWeights.aaExistence = getConfigField(model, 'aaExistenceWeights', model.aaSensorWeights);
    prevWeights.historyState = struct();
    innovationConsistency = ones(model.numberOfSensors, simulationLength);
    associationAmbiguityScore = ones(model.numberOfSensors, simulationLength);
%% Run the LMB filter
for t = 1:simulationLength
    %% Prediction
    objects = lmbPredictionStep(objects, model, t);
    %% Measurement update
    measurementUpdatedDistributions = cell(1, model.numberOfSensors);
    for s = 1:model.numberOfSensors
        if (numel(measurements{s, t}))
            % Phase 1: Support for mobile sensors - pass current time
            if model.sensorMotionEnabled
                [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, measurements{s, t}, model, s, t);
            else
                [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, measurements{s, t}, model, s);
            end
            if isfield(associationMatrices, 'innovationScore') && isfinite(associationMatrices.innovationScore)
                innovationConsistency(s, t) = associationMatrices.innovationScore;
            end
            if isfield(associationMatrices, 'associationAmbiguityScore') && ...
                    isfinite(associationMatrices.associationAmbiguityScore)
                associationAmbiguityScore(s, t) = associationMatrices.associationAmbiguityScore;
            end
            if useNIS && useNisEma && t > 1
                innovationConsistency(s, t) = nisEmaAlpha * innovationConsistency(s, t-1) + ...
                    (1 - nisEmaAlpha) * innovationConsistency(s, t);
            end
            if (strcmp(model.dataAssociationMethod, 'LBP'))
                % Data association by way of loopy belief propagation
                [r, W] = loopyBeliefPropagation(associationMatrices, model.lbpConvergenceTolerance, model.maximumNumberOfLbpIterations);
            elseif(strcmp(model.dataAssociationMethod, 'Gibbs'))
                % Data association by way of Gibbs sampling
                [r, W] = lmbGibbsSampling(associationMatrices, model.numberOfSamples);
            else
                % Data association by way of Murty's algorithm
                [r, W] = lmbMurtysAlgorithm(associationMatrices, model.numberOfAssignments);
            end
            % Compute measurement-updated spatial distributions
            measurementUpdatedDistributions{s} = computePosteriorLmbSpatialDistributions(objects, r, W, posteriorParameters, model);
        else
            % Non-sampling instants in multi-rate runs are prediction-only;
            % scheduled scans with no detections still receive a missed update.
            if isScheduledSample(commStats, s, t)
                measurementUpdatedDistributions{s} = applyMissedDetectionUpdate(objects, model, s, t);
            else
                measurementUpdatedDistributions{s} = objects;
            end
            innovationConsistency(s, t) = 1;
            associationAmbiguityScore(s, t) = 1;
            if useNIS && useNisEma && t > 1
                innovationConsistency(s, t) = nisEmaAlpha * innovationConsistency(s, t-1) + ...
                    (1 - nisEmaAlpha) * innovationConsistency(s, t);
            end
        end
    end
    %% Adaptive fusion weights (GA/AA only)
    if useAdaptiveFusion && (strcmp(model.lmbParallelUpdateMode, 'AA') || strcmp(model.lmbParallelUpdateMode, 'GA'))
        if nargin < 3
            commStatsLocal = [];
        else
            commStatsLocal = commStats;
        end
        if isempty(commStatsLocal) || ~isstruct(commStatsLocal)
            commStatsLocal = struct();
        end
        commStatsLocal.innovationConsistency = innovationConsistency;
        commStatsLocal.associationAmbiguityScore = associationAmbiguityScore;
        [gaWeights, aaWeights, debug] = computeAdaptiveFusionWeights( ...
            measurementUpdatedDistributions, measurements, model, t, commStatsLocal, prevWeights);
        model.gaSensorWeights = gaWeights;
        model.aaSensorWeights = aaWeights;
        model.gaSpatialWeights = getConfigField(debug, 'gaSpatialWeights', gaWeights);
        model.aaSpatialWeights = getConfigField(debug, 'aaSpatialWeights', aaWeights);
        model.gaExistenceWeights = getConfigField(debug, 'gaExistenceWeights', gaWeights);
        model.aaExistenceWeights = getConfigField(debug, 'aaExistenceWeights', aaWeights);
        if isfield(debug, 'gaTargetWiseWeights')
            model.gaTargetWiseWeights = debug.gaTargetWiseWeights;
        elseif isfield(model, 'gaTargetWiseWeights')
            model = rmfield(model, 'gaTargetWiseWeights');
        end
        if isfield(debug, 'aaTargetWiseWeights')
            model.aaTargetWiseWeights = debug.aaTargetWiseWeights;
        elseif isfield(model, 'aaTargetWiseWeights')
            model = rmfield(model, 'aaTargetWiseWeights');
        end
        prevWeights.ga = gaWeights;
        prevWeights.aa = aaWeights;
        prevWeights.gaSpatial = model.gaSpatialWeights;
        prevWeights.aaSpatial = model.aaSpatialWeights;
        prevWeights.gaExistence = model.gaExistenceWeights;
        prevWeights.aaExistence = model.aaExistenceWeights;
        if isfield(debug, 'historyState')
            prevWeights.historyState = debug.historyState;
        end
    end
    %% Track merging
    if (strcmp(model.lmbParallelUpdateMode, 'AA'))
        objects = aaLmbTrackMerging(measurementUpdatedDistributions, model);
    elseif (strcmp(model.lmbParallelUpdateMode, 'GA'))
        objects = gaLmbTrackMerging(measurementUpdatedDistributions, model);
    else
        objects = puLmbTrackMerging(measurementUpdatedDistributions, objects, model);
    end
    %% Gate tracks
    % Determine which objects have high existence probabilities
    objectsLikelyToExist = [objects.r] > model.existenceThreshold;
    % Objects with low existence probabilities and long trajectories are worth exporting
    discardedObjects = objects(~objectsLikelyToExist & ([objects.trajectoryLength] > model.minimumTrajectoryLength));
    stateEstimates.objects(end+1:end+numel(discardedObjects)) =  discardedObjects;
    % Keep objects with high existence probabilities
    objects = objects(objectsLikelyToExist);
    %% MAP cardinality extraction
    % Determine approximate MAP estimate of the posterior LMB
    [nMap, mapIndices] = lmbMapCardinalityEstimate([objects.r]);
    % Extract RFS state estimate
    stateEstimates.labels{t} = zeros(2, nMap);
    stateEstimates.mu{t} = cell(1, nMap);
    stateEstimates.Sigma{t} = cell(1, nMap);
    for i = 1:nMap
        j = mapIndices(i);
        % Gaussians in the posterior GM are sorted according to weight
        stateEstimates.labels{t}(:, i) = [objects(j).birthTime; objects(j).birthLocation];
        stateEstimates.mu{t}{i} = objects(j).mu{1};
        stateEstimates.Sigma{t}{i} = objects(j).Sigma{1};
    end
    %% Update each object's trajectory
    for i = 1:numel(objects)
        j = objects(i).trajectoryLength;
        objects(i).trajectoryLength = j + 1;
        objects(i).trajectory(:, j+1) = objects(i).mu{1};
        objects(i).timestamps(j+1) = t;
    end
    if progressEverySteps > 0 && (mod(t, progressEverySteps) == 0 || t == simulationLength)
        if isempty(progressLabel)
            fprintf('Filter progress %d/%d\n', t, simulationLength);
        else
            fprintf('[%s] progress %d/%d\n', progressLabel, t, simulationLength);
        end
    end
end
%% Get any long trajectories that weren't extracted
discardedObjects = objects(([objects.trajectoryLength] > model.minimumTrajectoryLength));
numberOfDiscardedObjects = numel(discardedObjects);
stateEstimates.objects(end+1:end+numberOfDiscardedObjects) =  discardedObjects;
end

function value = getConfigField(cfg, fieldName, defaultValue)
if isfield(cfg, fieldName)
    value = cfg.(fieldName);
else
    value = defaultValue;
end
end

function tf = isScheduledSample(commStats, sensorIdx, currentTime)
tf = true;
if nargin < 1 || ~isstruct(commStats) || ~isfield(commStats, 'sensorSampleMask')
    return;
end
if size(commStats.sensorSampleMask, 1) >= sensorIdx && ...
        size(commStats.sensorSampleMask, 2) >= currentTime
    tf = commStats.sensorSampleMask(sensorIdx, currentTime) > 0;
end
end

function updatedObjects = applyMissedDetectionUpdate(objects, model, sensorIdx, currentTime)
updatedObjects = objects;
for i = 1:numel(objects)
    missedLikelihood = zeros(1, objects(i).numberOfGmComponents);
    for j = 1:objects(i).numberOfGmComponents
        [pdSensor, ~] = evaluateSensorQuality(model, sensorIdx, objects(i).mu{j}, currentTime);
        missedLikelihood(j) = max(1 - pdSensor, realmin);
    end

    missedAverage = sum(objects(i).w .* missedLikelihood);
    denominator = 1 - objects(i).r + objects(i).r * missedAverage;
    if denominator > 0
        updatedObjects(i).r = (objects(i).r * missedAverage) / denominator;
    end

    updatedWeights = objects(i).w .* missedLikelihood;
    weightSum = sum(updatedWeights);
    if weightSum > 0
        updatedObjects(i).w = updatedWeights / weightSum;
    end
end
end
