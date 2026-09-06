function [updatedObjects, diagnostics] = updateLmbWithSensorMeasurement( ...
    predictedObjects, measurements, model, sensorIdx, currentTime, isScheduledSample)
% UPDATELMBWITHSENSORMEASUREMENT Run one sensor-specific LMB update.
%
% This primitive is shared by the existing parallel-update filter and the
% event-triggered posterior-exchange prototype. Besides the posterior, it
% returns compact innovation and association diagnostics for triggering.

if nargin < 6 || isempty(isScheduledSample)
    isScheduledSample = true;
end

diagnostics = defaultDiagnostics();
splitStats = struct();
if isScheduledSample && isfield(model,'fovGaussianSplitting') && ...
        isfield(model.fovGaussianSplitting,'enabled') && ...
        model.fovGaussianSplitting.enabled
    [predictedObjects,splitStats] = splitLmbPredictionAtFovBoundary( ...
        predictedObjects,model,sensorIdx,currentTime,model.fovGaussianSplitting);
end
diagnostics.fovSplit = splitStats;
[predictiveLogScore, predictiveScoreDetails] = ...
    computeLmbPoissonMeasurementLogScore( ...
        predictedObjects, measurements, model, sensorIdx, currentTime);
diagnostics.predictiveMeasurementLogScore = predictiveLogScore;
diagnostics.predictiveMeasurementExpectedCount = ...
    predictiveScoreDetails.totalExpectedCount;
diagnostics.predictiveMeasurementObservedCount = ...
    predictiveScoreDetails.measurementCount;
if isempty(measurements)
    if isScheduledSample
        updatedObjects = applyMissedDetectionUpdate( ...
            predictedObjects, model, sensorIdx, currentTime);
        diagnostics.didMissedDetectionUpdate = true;
    else
        updatedObjects = predictedObjects;
        diagnostics.wasScheduled = false;
    end
    return;
end

if model.sensorMotionEnabled
    [associationMatrices, posteriorParameters] = ...
        generateLmbSensorAssociationMatrices( ...
            predictedObjects, measurements, model, sensorIdx, currentTime);
else
    [associationMatrices, posteriorParameters] = ...
        generateLmbSensorAssociationMatrices( ...
            predictedObjects, measurements, model, sensorIdx);
end

diagnostics = extractDiagnostics(associationMatrices);
diagnostics.fovSplit = splitStats;
diagnostics.predictiveMeasurementLogScore = predictiveLogScore;
diagnostics.predictiveMeasurementExpectedCount = ...
    predictiveScoreDetails.totalExpectedCount;
diagnostics.predictiveMeasurementObservedCount = ...
    predictiveScoreDetails.measurementCount;
if strcmp(model.dataAssociationMethod, 'LBP')
    [r, W] = loopyBeliefPropagation( ...
        associationMatrices, model.lbpConvergenceTolerance, ...
        model.maximumNumberOfLbpIterations);
elseif strcmp(model.dataAssociationMethod, 'Gibbs')
    [r, W] = lmbGibbsSampling(associationMatrices, model.numberOfSamples);
else
    [r, W] = lmbMurtysAlgorithm( ...
        associationMatrices, model.numberOfAssignments);
end

updatedObjects = computePosteriorLmbSpatialDistributions( ...
    predictedObjects, r, W, posteriorParameters, model);
end

function diagnostics = defaultDiagnostics()
diagnostics = struct( ...
    'associationConfidence', 1.0, ...
    'innovationScore', 1.0, ...
    'innovationNovelty', 0.0, ...
    'nisAgg', NaN, ...
    'nisNorm', NaN, ...
    'nisDeviation', 0.0, ...
    'predictiveMeasurementLogScore', NaN, ...
    'predictiveMeasurementExpectedCount', NaN, ...
    'predictiveMeasurementObservedCount', NaN, ...
    'wasScheduled', true, ...
    'didMissedDetectionUpdate', false);
end

function diagnostics = extractDiagnostics(associationMatrices)
diagnostics = defaultDiagnostics();
diagnostics.associationConfidence = getField( ...
    associationMatrices, 'associationConfidence', 1.0);
diagnostics.innovationScore = getField( ...
    associationMatrices, 'innovationScore', 1.0);
diagnostics.nisAgg = getField(associationMatrices, 'nisAgg', NaN);
diagnostics.nisNorm = getField(associationMatrices, 'nisNorm', NaN);
diagnostics.nisDeviation = getField( ...
    associationMatrices, 'nisDeviation', 0.0);

if isfinite(diagnostics.nisNorm)
    novelty = 1 - exp(-max(diagnostics.nisNorm, 0));
else
    novelty = 0;
end
diagnostics.innovationNovelty = clamp01( ...
    novelty * diagnostics.innovationScore);
end

function updatedObjects = applyMissedDetectionUpdate( ...
    objects, model, sensorIdx, currentTime)
updatedObjects = objects;
for i = 1:numel(objects)
    missedLikelihood = zeros(1, objects(i).numberOfGmComponents);
    for j = 1:objects(i).numberOfGmComponents
        [pdSensor, ~] = evaluateSensorQuality( ...
            model, sensorIdx, objects(i).mu{j}, currentTime);
        missedLikelihood(j) = max(1 - pdSensor, realmin);
    end

    missedAverage = sum(objects(i).w .* missedLikelihood);
    denominator = 1 - objects(i).r + objects(i).r * missedAverage;
    if denominator > 0
        updatedObjects(i).r = ...
            (objects(i).r * missedAverage) / denominator;
    end

    updatedWeights = objects(i).w .* missedLikelihood;
    weightSum = sum(updatedWeights);
    if weightSum > 0
        updatedObjects(i).w = updatedWeights / weightSum;
    end
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
