function summary = summarizeLmbPosteriorForDisagreement(objects, model)
% SUMMARIZELMBPOSTERIORFORDISAGREEMENT Cache label-wise LMB moments.
%
% The disagreement metric only consumes each Bernoulli's existence
% probability and its first two spatial moments. Preparing those values
% once avoids repeatedly moment matching the same posterior in network
% counterfactual screens.

labels = zeros(2, 0);
existence = zeros(1, 0);
positionMean = zeros(2, 0);
positionCovariance = zeros(2, 2, 0);
for objectIdx = 1:numel(objects)
    object = objects(objectIdx);
    if object.numberOfGmComponents < 1
        continue;
    end
    [meanVector, covariance] = ...
        moments(object, model.xDimension);
    labels(:, end + 1) = [ ...
        object.birthTime; object.birthLocation]; %#ok<AGROW>
    existence(end + 1) = ...
        min(max(object.r, 0), 1); %#ok<AGROW>
    positionMean(:, end + 1) = meanVector(1:2); %#ok<AGROW>
    positionCovariance(:, :, end + 1) = ...
        covariance(1:2, 1:2); %#ok<AGROW>
end
summary = struct( ...
    'contractVersion', 'lmb-disagreement-summary-v1', ...
    'labels', labels, ...
    'existence', existence, ...
    'positionMean', positionMean, ...
    'positionCovariance', positionCovariance);
end

function [meanVector, covariance] = moments(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || ...
        sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end
