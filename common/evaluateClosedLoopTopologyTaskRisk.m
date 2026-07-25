function [risk, details] = evaluateClosedLoopTopologyTaskRisk( ...
    currentLocalPosteriors, currentAdjacency, model, measurements, ...
    commConfig, currentTime, continuationAdjacency, ...
    triggerConfig, options)
% EVALUATECLOSEDLOOPTOPOLOGYTASKRISK Short teacher-forced rollout risk.
%
% A candidate graph is used for the current fusion round. The rollout then
% consumes the registered future measurements, performs LMB prediction and
% local update at every node, and fuses over a common continuation graph.
% Thus only the first topology action differs between candidates, while the
% future exogenous data and continuation policy are paired.

if nargin < 9 || isempty(options)
    options = struct();
end
horizon = max(0, round(getField(options, 'horizonSteps', 3)));
discountFactor = min(max(getField( ...
    options, 'discountFactor', 0.9), 0), 1);
expectedDeliveryWeighting = getField( ...
    options, 'expectedDeliveryWeighting', true);
candidatePersistenceSteps = max(1, round(getField( ...
    options, 'candidatePersistenceSteps', 1)));
riskOptions = getField(options, 'riskOptions', struct());
riskOptions.horizonSteps = 0;
sampleMask = getField(options, 'sampleMask', []);

truthTimeCount = size( ...
    model.dynamicTopologyScenario.targetTrajectories{1}, 2);
measurementTimeCount = size(measurements, 2);
horizon = min(horizon, ...
    min(truthTimeCount, measurementTimeCount) - currentTime);
horizon = max(horizon, 0);
rolloutTimes = currentTime:(currentTime + horizon);
weights = discountFactor .^ (0:horizon);
weights = weights / max(sum(weights), eps);

fusionOptions = struct( ...
    'expectedDeliveryWeighting', expectedDeliveryWeighting);
[objectsBySensor, ~] = fuseLmbNetworkOnTopology( ...
    currentLocalPosteriors, currentAdjacency, model, ...
    commConfig, currentTime, triggerConfig, fusionOptions);
objectsBySensor = pruneRolloutObjects( ...
    objectsBySensor, model.existenceThreshold);

stepRisks = zeros(1, horizon + 1);
stepDetails = cell(1, horizon + 1);
[stepRisks(1), stepDetails{1}] = ...
    evaluateLmbTopologyTaskRisk( ...
        objectsBySensor, model, currentTime, riskOptions);

for rolloutStep = 1:horizon
    futureTime = currentTime + rolloutStep;
    localPosteriors = cell(size(objectsBySensor));
    for sensorIdx = 1:numel(objectsBySensor)
        predicted = lmbPredictionStep( ...
            objectsBySensor{sensorIdx}, model, futureTime);
        isScheduled = resolveScheduledSample( ...
            sampleMask, sensorIdx, futureTime);
        [localPosteriors{sensorIdx}, ~] = ...
            updateLmbWithSensorMeasurement( ...
                predicted, measurements{sensorIdx, futureTime}, ...
                model, sensorIdx, futureTime, isScheduled);
    end
    if rolloutStep < candidatePersistenceSteps
        nextAdjacency = currentAdjacency;
    else
        nextAdjacency = resolveContinuationAdjacency( ...
            continuationAdjacency, rolloutStep);
    end
    [objectsBySensor, ~] = fuseLmbNetworkOnTopology( ...
        localPosteriors, nextAdjacency, model, ...
        commConfig, futureTime, triggerConfig, fusionOptions);
    objectsBySensor = pruneRolloutObjects( ...
        objectsBySensor, model.existenceThreshold);
    [stepRisks(rolloutStep + 1), ...
        stepDetails{rolloutStep + 1}] = ...
        evaluateLmbTopologyTaskRisk( ...
            objectsBySensor, model, futureTime, riskOptions);
end

risk = sum(weights .* stepRisks);
details = struct();
details.risk = risk;
details.rolloutTimes = rolloutTimes;
details.stepWeights = weights;
details.stepRisks = stepRisks;
details.stepDetails = stepDetails;
details.horizonSteps = horizon;
details.usesFutureMeasurements = horizon > 0;
details.candidatePersistenceSteps = candidatePersistenceSteps;
if candidatePersistenceSteps > 1
    details.continuationPolicy = 'candidate-then-fixed-registered-graph';
else
    details.continuationPolicy = 'fixed-registered-graph';
end
end

function objectsBySensor = pruneRolloutObjects( ...
    objectsBySensor, threshold)
for sensorIdx = 1:numel(objectsBySensor)
    objects = objectsBySensor{sensorIdx};
    if isempty(objects)
        continue;
    end
    keep = [objects.r] > threshold & ...
        [objects.numberOfGmComponents] > 0;
    objectsBySensor{sensorIdx} = objects(keep);
end
end

function adjacency = resolveContinuationAdjacency(sequence, rolloutStep)
if ndims(sequence) < 3 || size(sequence, 3) == 1
    adjacency = sequence;
else
    timeIdx = min(rolloutStep, size(sequence, 3));
    adjacency = sequence(:, :, timeIdx);
end
end

function scheduled = resolveScheduledSample( ...
    sampleMask, sensorIdx, currentTime)
if isempty(sampleMask)
    scheduled = true;
elseif size(sampleMask, 1) >= sensorIdx && ...
        size(sampleMask, 2) >= currentTime
    scheduled = logical(sampleMask(sensorIdx, currentTime));
else
    scheduled = true;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
