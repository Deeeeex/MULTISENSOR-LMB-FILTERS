function [risk, details] = evaluateLmbTopologyTaskRisk( ...
    posteriors, model, currentTime, options)
% EVALUATELMBTOPOLOGYTASKRISK Truth-labelled current/future tracking risk.
%
% This offline teacher target deliberately contains no topology-switching
% or communication penalty. It scores the downstream labelled LMB state:
% Bernoulli existence calibration plus normalized expected position and
% velocity error (including posterior covariance). Future horizons predict
% the candidate-fused posterior without seeing future measurements. They
% test whether today's fused information remains useful under motion, but
% cannot encode which sensor will acquire new information during a future
% sensing handover.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~iscell(posteriors)
    posteriors = {posteriors};
end
horizons = getField(options, 'horizonSteps', 0);
horizons = unique(max(0, round(reshape(horizons, 1, []))));
discountFactor = min(max(getField( ...
    options, 'discountFactor', 0.9), 0), 1);

scenario = model.dynamicTopologyScenario;
truthTrajectories = scenario.targetTrajectories;
timeCount = size(truthTrajectories{1}, 2);
horizons = horizons(currentTime + horizons <= timeCount);
if isempty(horizons)
    horizons = 0;
end
weights = discountFactor .^ horizons;
weights = weights / max(sum(weights), eps);

positionScale = getField(options, 'positionScale', ...
    resolvePositionScale(model, scenario.config));
velocityScale = getField(options, 'velocityScale', ...
    getField(scenario.config, 'targetSpeedLimit', 15));
positionScale = max(positionScale, eps);
velocityScale = max(velocityScale, eps);
existenceWeight = max(getField( ...
    options, 'existenceWeight', 1.0), 0);
positionWeight = max(getField( ...
    options, 'positionWeight', 1.0), 0);
velocityWeight = max(getField( ...
    options, 'velocityWeight', 0.15), 0);
missedStateRisk = max(getField( ...
    options, 'missedStateRisk', 1.0), 0);
maxStateRisk = max(getField( ...
    options, 'maxStateRisk', 4.0), missedStateRisk);
sensorAggregationMode = lower(strrep(getField( ...
    options, 'sensorAggregationMode', 'mean'), '_', '-'));
cvarFraction = min(max(getField( ...
    options, 'sensorCvarFraction', 0.25), eps), 1);
cvarWeight = min(max(getField( ...
    options, 'sensorCvarWeight', 0.5), 0), 1);

predicted = posteriors;
previousHorizon = 0;
horizonRisk = zeros(1, numel(horizons));
horizonExistence = zeros(1, numel(horizons));
horizonPosition = zeros(1, numel(horizons));
horizonVelocity = zeros(1, numel(horizons));
horizonMiss = zeros(1, numel(horizons));
for horizonIdx = 1:numel(horizons)
    horizon = horizons(horizonIdx);
    for predictionStep = (previousHorizon + 1):horizon
        futureTime = currentTime + predictionStep;
        for sensorIdx = 1:numel(predicted)
            predicted{sensorIdx} = lmbPredictionStep( ...
                predicted{sensorIdx}, model, futureTime);
        end
    end
    previousHorizon = horizon;
    [horizonRisk(horizonIdx), components] = scorePosteriorSet( ...
        predicted, truthTrajectories, model, currentTime + horizon, ...
        positionScale, velocityScale, existenceWeight, ...
        positionWeight, velocityWeight, missedStateRisk, ...
        maxStateRisk, sensorAggregationMode, ...
        cvarFraction, cvarWeight);
    horizonExistence(horizonIdx) = components.existence;
    horizonPosition(horizonIdx) = components.position;
    horizonVelocity(horizonIdx) = components.velocity;
    horizonMiss(horizonIdx) = components.missedState;
end

risk = sum(weights .* horizonRisk);
details = struct();
details.horizonSteps = horizons;
details.horizonWeights = weights;
details.horizonRisk = horizonRisk;
details.risk = risk;
details.existenceRisk = sum(weights .* horizonExistence);
details.positionRisk = sum(weights .* horizonPosition);
details.velocityRisk = sum(weights .* horizonVelocity);
details.missedStateRisk = sum(weights .* horizonMiss);
details.positionScale = positionScale;
details.velocityScale = velocityScale;
details.sensorAggregationMode = sensorAggregationMode;
details.sensorCvarFraction = cvarFraction;
details.sensorCvarWeight = cvarWeight;
end

function [risk, components] = scorePosteriorSet( ...
    posteriors, truthTrajectories, model, currentTime, ...
    positionScale, velocityScale, existenceWeight, ...
    positionWeight, velocityWeight, missedStateRisk, maxStateRisk, ...
    sensorAggregationMode, cvarFraction, cvarWeight)
targetCount = numel(truthTrajectories);
sensorRisk = zeros(1, numel(posteriors));
sensorExistence = zeros(1, numel(posteriors));
sensorPosition = zeros(1, numel(posteriors));
sensorVelocity = zeros(1, numel(posteriors));
sensorMiss = zeros(1, numel(posteriors));
birthTimes = resolveBirthTimes(model, targetCount);

for sensorIdx = 1:numel(posteriors)
    objects = posteriors{sensorIdx};
    existenceTerms = zeros(1, targetCount);
    positionTerms = zeros(1, targetCount);
    velocityTerms = zeros(1, targetCount);
    missTerms = zeros(1, targetCount);
    stateTerms = zeros(1, targetCount);
    for targetIdx = 1:targetCount
        truth = truthTrajectories{targetIdx}(:, currentTime);
        active = all(isfinite(truth));
        object = findLabel(objects, ...
            [birthTimes(targetIdx); targetIdx]);
        r = objectExistence(object);
        if active
            existenceTerms(targetIdx) = (1 - r)^2;
            if isempty(object)
                missTerms(targetIdx) = missedStateRisk;
                stateTerms(targetIdx) = missedStateRisk;
                continue;
            end
            [positionRisk, velocityRisk] = ...
                expectedStateError(object, truth, ...
                    positionScale, velocityScale);
            positionTerms(targetIdx) = positionRisk;
            velocityTerms(targetIdx) = velocityRisk;
            conditionalStateRisk = min( ...
                positionWeight * positionRisk + ...
                velocityWeight * velocityRisk, maxStateRisk);
            missTerms(targetIdx) = (1 - r) * missedStateRisk;
            stateTerms(targetIdx) = ...
                r * conditionalStateRisk + missTerms(targetIdx);
        else
            existenceTerms(targetIdx) = r^2;
        end
    end
    extraFalseRisk = unregisteredExistenceRisk( ...
        objects, birthTimes, targetCount);
    normalization = max(targetCount, 1);
    sensorExistence(sensorIdx) = ...
        (sum(existenceTerms) + extraFalseRisk) / normalization;
    sensorPosition(sensorIdx) = ...
        mean(positionTerms);
    sensorVelocity(sensorIdx) = ...
        mean(velocityTerms);
    sensorMiss(sensorIdx) = mean(missTerms);
    sensorRisk(sensorIdx) = existenceWeight * ...
        sensorExistence(sensorIdx) + mean(stateTerms);
end

meanSensorRisk = mean(sensorRisk);
sortedSensorRisk = sort(sensorRisk, 'descend');
tailCount = max(1, ceil(cvarFraction * numel(sensorRisk)));
cvarSensorRisk = mean(sortedSensorRisk(1:tailCount));
switch sensorAggregationMode
    case 'mean'
        risk = meanSensorRisk;
    case {'cvar', 'tail'}
        risk = cvarSensorRisk;
    case {'mean-cvar', 'risk-sensitive'}
        risk = (1 - cvarWeight) * meanSensorRisk + ...
            cvarWeight * cvarSensorRisk;
    otherwise
        error('Unknown sensorAggregationMode: %s', ...
            sensorAggregationMode);
end
components = struct( ...
    'existence', mean(sensorExistence), ...
    'position', mean(sensorPosition), ...
    'velocity', mean(sensorVelocity), ...
    'missedState', mean(sensorMiss), ...
    'meanSensorRisk', meanSensorRisk, ...
    'cvarSensorRisk', cvarSensorRisk, ...
    'sensorRisk', sensorRisk);
end

function [positionRisk, velocityRisk] = expectedStateError( ...
    object, truth, positionScale, velocityScale)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
componentCount = object.numberOfGmComponents;
if numel(weights) ~= componentCount || sum(weights) <= 0
    weights = ones(1, componentCount);
end
weights = weights / sum(weights);
positionRisk = 0;
velocityRisk = 0;
for componentIdx = 1:componentCount
    meanVector = object.mu{componentIdx};
    covariance = object.Sigma{componentIdx};
    positionDelta = meanVector(1:2) - truth(1:2);
    positionRisk = positionRisk + weights(componentIdx) * ...
        (positionDelta' * positionDelta + ...
         trace(covariance(1:2, 1:2))) / positionScale^2;
    if numel(meanVector) >= 4 && numel(truth) >= 4
        velocityDelta = meanVector(3:4) - truth(3:4);
        velocityRisk = velocityRisk + weights(componentIdx) * ...
            (velocityDelta' * velocityDelta + ...
             trace(covariance(3:4, 3:4))) / velocityScale^2;
    end
end
end

function value = unregisteredExistenceRisk( ...
    objects, birthTimes, targetCount)
value = 0;
for objectIdx = 1:numel(objects)
    targetIdx = objects(objectIdx).birthLocation;
    registered = targetIdx >= 1 && targetIdx <= targetCount && ...
        objects(objectIdx).birthTime == birthTimes(targetIdx);
    if ~registered
        value = value + objectExistence(objects(objectIdx))^2;
    end
end
end

function birthTimes = resolveBirthTimes(model, targetCount)
if isfield(model, 'birthTimeByLocation') && ...
        numel(model.birthTimeByLocation) >= targetCount
    birthTimes = reshape( ...
        model.birthTimeByLocation(1:targetCount), 1, []);
elseif isfield(model.dynamicTopologyScenario, 'target') && ...
        isfield(model.dynamicTopologyScenario.target, 'birthTimes')
    birthTimes = reshape( ...
        model.dynamicTopologyScenario.target.birthTimes, 1, []);
else
    birthTimes = ones(1, targetCount);
end
end

function object = findLabel(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function value = objectExistence(object)
if isempty(object)
    value = 0;
else
    value = min(max(object.r, 0), 1);
end
end

function value = resolvePositionScale(model, config)
if isfield(model, 'ospaParameters') && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
else
    value = getField(config, 'ospaPositionCutoff', 100);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
