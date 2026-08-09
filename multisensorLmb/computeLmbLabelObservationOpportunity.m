function opportunity = computeLmbLabelObservationOpportunity( ...
        model, sensorIdx, predictedObject, currentTime)
% COMPUTELMBLABELOBSERVATIONOPPORTUNITY Expected p_D under one label GM.
%
% The calculation is truth-free. Each Gaussian component is integrated by
% a positive-weight two-dimensional cubature rule over position, while the
% actual runtime FoV, range and state-dependent sensor-quality model are
% evaluated by evaluateSensorQuality.

requiredFields = {'numberOfGmComponents', 'w', 'mu', 'Sigma'};
if ~isstruct(predictedObject) || ~isscalar(predictedObject) || ...
        ~all(isfield(predictedObject, requiredFields)) || ...
        predictedObject.numberOfGmComponents < 1 || ...
        numel(predictedObject.w) ~= ...
            predictedObject.numberOfGmComponents || ...
        numel(predictedObject.mu) ~= ...
            predictedObject.numberOfGmComponents || ...
        numel(predictedObject.Sigma) ~= ...
            predictedObject.numberOfGmComponents
    error('LmbObservationOpportunity:InvalidObject', ...
        'A valid predicted Gaussian-mixture label is required.');
end
if ~isscalar(sensorIdx) || sensorIdx < 1 || ...
        sensorIdx ~= round(sensorIdx) || ...
        ~isscalar(currentTime) || currentTime < 1 || ...
        currentTime ~= round(currentTime)
    error('LmbObservationOpportunity:InvalidIndex', ...
        'A positive sensor index and time are required.');
end

componentWeights = reshape(predictedObject.w, 1, []);
componentWeights(~isfinite(componentWeights)) = 0;
componentWeights = max(componentWeights, 0);
if sum(componentWeights) <= 0
    error('LmbObservationOpportunity:InvalidWeights', ...
        'The predicted mixture weights are not positive.');
end
componentWeights = componentWeights / sum(componentWeights);
componentCount = predictedObject.numberOfGmComponents;
componentExpectedDetection = zeros(1, componentCount);
componentInFovProbability = zeros(1, componentCount);
componentMeanRange = zeros(1, componentCount);
componentMeanOffAxisDeg = zeros(1, componentCount);
componentMeanPointDetection = zeros(1, componentCount);

for componentIdx = 1:componentCount
    meanState = reshape(predictedObject.mu{componentIdx}, [], 1);
    covariance = predictedObject.Sigma{componentIdx};
    if numel(meanState) < 2 || ...
            ~isequal(size(covariance), [numel(meanState), numel(meanState)]) || ...
            any(~isfinite(meanState)) || any(~isfinite(covariance(:)))
        error('LmbObservationOpportunity:InvalidComponent', ...
            'Every component must contain a finite state and covariance.');
    end
    positionCovariance = regularizePositionCovariance( ...
        covariance(1:2, 1:2));
    factor = chol(positionCovariance, 'lower');
    positionPoints = meanState(1:2) + ...
        sqrt(2) * [factor, -factor];
    pointDetection = zeros(1, 4);
    pointInFov = zeros(1, 4);
    pointRange = zeros(1, 4);
    pointOffAxisDeg = zeros(1, 4);
    for pointIdx = 1:4
        state = meanState;
        state(1:2) = positionPoints(:, pointIdx);
        [pointDetection(pointIdx), ~, info] = ...
            evaluateSensorQuality(model, sensorIdx, state, currentTime);
        pointInFov(pointIdx) = double(info.inFov);
        pointRange(pointIdx) = info.range;
        pointOffAxisDeg(pointIdx) = info.offAxisDeg;
    end
    [meanPointDetection, ~] = evaluateSensorQuality( ...
        model, sensorIdx, meanState, currentTime);
    componentExpectedDetection(componentIdx) = mean(pointDetection);
    componentInFovProbability(componentIdx) = mean(pointInFov);
    componentMeanRange(componentIdx) = mean(pointRange);
    componentMeanOffAxisDeg(componentIdx) = mean(pointOffAxisDeg);
    componentMeanPointDetection(componentIdx) = meanPointDetection;
end

opportunity = struct();
opportunity.contractVersion = 'lmb-label-observation-opportunity-v1';
opportunity.expectedDetectionProbability = ...
    sum(componentWeights .* componentExpectedDetection);
opportunity.inFovProbability = ...
    sum(componentWeights .* componentInFovProbability);
opportunity.outsideFovProbability = 1 - opportunity.inFovProbability;
opportunity.meanRange = sum(componentWeights .* componentMeanRange);
opportunity.meanOffAxisDeg = ...
    sum(componentWeights .* componentMeanOffAxisDeg);
opportunity.meanPointDetectionProbability = ...
    sum(componentWeights .* componentMeanPointDetection);
opportunity.componentWeights = componentWeights;
opportunity.componentExpectedDetectionProbability = ...
    componentExpectedDetection;
opportunity.componentInFovProbability = componentInFovProbability;
opportunity.usesTargetTruth = false;
opportunity.usesFutureMeasurements = false;
opportunity.integrationRule = ...
    'positive-weight-position-spherical-radial-cubature';
end

function covariance = regularizePositionCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:8
    [~, flag] = chol(covariance + jitter * eye(2));
    if flag == 0
        covariance = covariance + jitter * eye(2);
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('LmbObservationOpportunity:InvalidCovariance', ...
    'The position covariance is not positive definite.');
end
