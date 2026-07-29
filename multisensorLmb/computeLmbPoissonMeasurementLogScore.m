function [logScore, details] = ...
    computeLmbPoissonMeasurementLogScore( ...
        predictedObjects, measurements, model, sensorIdx, currentTime)
% COMPUTELMBPOISSONMEASUREMENTLOGSCORE Observable predictive set score.
%
% The predicted LMB induces the first-moment measurement intensity
%
%   lambda(z) = kappa(z) + sum_l r_l p_D,l p_l(z).
%
% Treating this intensity as a Poisson point-process forecast gives the
% logarithmic score
%
%   sum_{z in Z} log(lambda(z)) - integral lambda(z) dz.
%
% This score is computed before the current measurement update. It uses no
% target truth and is therefore suitable as delayed feedback for a topology
% action chosen at the preceding time step. The Poisson approximation is a
% scoring model for the measurement intensity; it is not the exact LMB
% multi-object measurement-set likelihood.

if nargin < 5 || isempty(currentTime)
    currentTime = 1;
end
if nargin < 4 || isempty(sensorIdx)
    sensorIdx = 1;
end
if isempty(measurements)
    measurements = cell(0, 1);
end

measurementCount = numel(measurements);
clutterIntensity = max( ...
    resolveSensorValue(model.clutterPerUnitVolume, sensorIdx, 0), ...
    realmin);
clutterExpectedCount = max( ...
    resolveSensorValue(model.clutterRate, sensorIdx, 0), 0);
measurementIntensity = ...
    clutterIntensity * ones(1, measurementCount);
targetExpectedCount = 0;

for objectIdx = 1:numel(predictedObjects)
    object = predictedObjects(objectIdx);
    existenceProbability = min(max(object.r, 0), 1);
    for componentIdx = 1:object.numberOfGmComponents
        componentWeight = max(object.w(componentIdx), 0);
        [detectionProbability, measurementCovariance] = ...
            evaluateSensorQuality( ...
                model, sensorIdx, ...
                object.mu{componentIdx}, currentTime);
        detectionProbability = ...
            min(max(detectionProbability, 0), 1);
        componentDetectionMass = existenceProbability * ...
            componentWeight * detectionProbability;
        targetExpectedCount = targetExpectedCount + ...
            componentDetectionMass;
        if componentDetectionMass <= 0 || measurementCount == 0
            continue;
        end

        if model.sensorMotionEnabled
            sensorPosition = ...
                model.sensorTrajectories{sensorIdx}(1:2, currentTime);
            targetPosition = object.mu{componentIdx}(1:2);
            predictedMeasurementMean = sensorPosition + ...
                model.C{sensorIdx} * ...
                [targetPosition - sensorPosition; 0; 0];
        else
            predictedMeasurementMean = ...
                model.C{sensorIdx} * object.mu{componentIdx};
        end
        innovationCovariance = ...
            model.C{sensorIdx} * object.Sigma{componentIdx} * ...
            model.C{sensorIdx}' + measurementCovariance;
        [logNormalizer, validCovariance] = ...
            gaussianLogNormalizer(innovationCovariance);
        if ~validCovariance
            continue;
        end
        for measurementIdx = 1:measurementCount
            innovation = measurements{measurementIdx} - ...
                predictedMeasurementMean;
            logDensity = logNormalizer - 0.5 * ...
                (innovation' * (innovationCovariance \ innovation));
            measurementIntensity(measurementIdx) = ...
                measurementIntensity(measurementIdx) + ...
                componentDetectionMass * exp(logDensity);
        end
    end
end

totalExpectedCount = clutterExpectedCount + targetExpectedCount;
logIntensities = log(max(measurementIntensity, realmin));
logScore = sum(logIntensities) - totalExpectedCount;

details = struct( ...
    'scoreType', 'poisson-measurement-intensity-log-score', ...
    'truthUsed', false, ...
    'exactLmbMeasurementSetLikelihood', false, ...
    'measurementCount', measurementCount, ...
    'clutterExpectedCount', clutterExpectedCount, ...
    'targetExpectedCount', targetExpectedCount, ...
    'totalExpectedCount', totalExpectedCount, ...
    'meanLogIntensity', finiteMean(logIntensities), ...
    'minimumIntensity', finiteMinimum(measurementIntensity));
end

function [logNormalizer, valid] = gaussianLogNormalizer(covariance)
dimension = size(covariance, 1);
covariance = 0.5 * (covariance + covariance');
[cholFactor, cholFlag] = chol(covariance);
valid = cholFlag == 0 && all(isfinite(cholFactor(:)));
if ~valid
    logNormalizer = -Inf;
    return;
end
logDeterminant = 2 * sum(log(diag(cholFactor)));
logNormalizer = ...
    -0.5 * dimension * log(2 * pi) - 0.5 * logDeterminant;
end

function value = resolveSensorValue(values, sensorIdx, defaultValue)
if isempty(values)
    value = defaultValue;
elseif isscalar(values)
    value = values;
elseif sensorIdx <= numel(values)
    value = values(sensorIdx);
else
    value = values(end);
end
if ~isfinite(value)
    value = defaultValue;
end
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = finiteMinimum(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end
