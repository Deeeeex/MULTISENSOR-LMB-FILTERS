function [risk, details] = ...
        computeObservableLmbLabelBayesRisk(object, model)
% COMPUTEOBSERVABLELMBLABELBAYESRISK Truth-free scalar label-risk proxy.
%
% Equal-cost Bernoulli decision error is averaged with existence-weighted,
% E-OSPA-cutoff-normalized posterior position MSE.  This is a source-ranking
% proxy, not an equality to realized OSPA, RMSE or tracking loss.

positionCutoff = resolvePositionCutoff(model);
if isempty(object) || ~isstruct(object) || ...
        object.numberOfGmComponents <= 0
    risk = 0.5;
    details = struct( ...
        'present', false, ...
        'existence', 0, ...
        'positionTrace', positionCutoff^2, ...
        'existenceRisk', 1, ...
        'localizationRisk', 0, ...
        'truthUsed', false, ...
        'futureInformationUsed', false);
    return;
end
componentCount = object.numberOfGmComponents;
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= componentCount || sum(weights) <= 0
    weights = ones(1, componentCount);
end
weights = weights / sum(weights);
stateDimension = numel(object.mu{1});
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:componentCount
    meanVector = meanVector + weights(componentIdx) * ...
        object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:componentCount
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
positionDimension = min(2, stateDimension);
positionTrace = max(real(trace(covariance( ...
    1:positionDimension, 1:positionDimension))), 0);
existence = clamp01(object.r);
existenceRisk = min(existence, 1 - existence);
localizationRisk = existence * min( ...
    positionTrace / max(positionCutoff^2, eps), 1);
risk = 0.5 * existenceRisk + 0.5 * localizationRisk;
details = struct( ...
    'present', true, ...
    'existence', existence, ...
    'positionTrace', positionTrace, ...
    'existenceRisk', existenceRisk, ...
    'localizationRisk', localizationRisk, ...
    'truthUsed', false, ...
    'futureInformationUsed', false);
end

function value = resolvePositionCutoff(model)
value = NaN;
if isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('ObservableLabelRisk:MissingCutoff', ...
        'A positive E-OSPA position cutoff is required.');
end
end

function value = clamp01(value)
if ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end
