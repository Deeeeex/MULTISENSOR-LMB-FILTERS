function [risk, details] = ...
    evaluateLmbTopologyCurrentEospa( ...
        posterior, model, currentTime, triggerConfig)
% EVALUATELMBTOPOLOGYCURRENTEOSPA Exact current-step Euclidean OSPA oracle.
%
% This privileged diagnostic mirrors the registered filter's current
% state-extraction path: existence pruning, LMB MAP cardinality, and the
% largest-weight Gaussian component for each selected Bernoulli.  It is
% intentionally limited to the current configuration, where prediction-
% and trajectory-consistent reweighting, joint extraction, and separation
% constraints are disabled.

if nargin < 4 || isempty(triggerConfig)
    triggerConfig = struct();
end
unsupported = logical(getField(triggerConfig, ...
        'mixtureAwarePredictionConsistencyEnabled', false)) || ...
    logical(getField(triggerConfig, ...
        'mixtureAwareTrajectoryConsistencyEnabled', false)) || ...
    logical(getField(triggerConfig, ...
        'mixtureAwareJointExtractionEnabled', false)) || ...
    getField(triggerConfig, ...
        'mixtureAwareExtractionMinSeparation', 0) > 0;
if unsupported
    error(['Current E-OSPA oracle does not support the configured ', ...
        'state-extraction refinements.']);
end
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, ...
            'targetTrajectories')
    error('Current E-OSPA oracle requires scenario truth trajectories.');
end

objects = reshape(posterior, 1, []);
if ~isempty(objects)
    objects = objects([objects.r] > model.existenceThreshold);
end
estimateMeans = {};
estimateCovariances = {};
mapCardinality = 0;
if ~isempty(objects)
    [mapCardinality, mapIndices] = ...
        lmbMapCardinalityEstimate([objects.r]);
    estimateMeans = cell(1, mapCardinality);
    estimateCovariances = cell(1, mapCardinality);
    for mapIdx = 1:mapCardinality
        object = objects(mapIndices(mapIdx));
        weights = reshape(object.w, 1, []);
        weights(~isfinite(weights)) = -inf;
        [~, componentIdx] = max(weights);
        if isempty(componentIdx) || ...
                ~isfinite(weights(componentIdx))
            componentIdx = 1;
        end
        estimateMeans{mapIdx} = ...
            object.mu{componentIdx};
        estimateCovariances{mapIdx} = ...
            object.Sigma{componentIdx};
    end
end

trajectories = ...
    model.dynamicTopologyScenario.targetTrajectories;
truthStates = {};
truthMeans = {};
truthCovariances = {};
for targetIdx = 1:numel(trajectories)
    if currentTime <= size(trajectories{targetIdx}, 2)
        state = trajectories{targetIdx}(:, currentTime);
        if all(isfinite(state))
            truthStates{end + 1} = state; %#ok<AGROW>
            truthMeans{end + 1} = state; %#ok<AGROW>
            truthCovariances{end + 1} = ...
                eye(numel(state)); %#ok<AGROW>
        end
    end
end

[euclideanOspa, ~] = ospa( ...
    truthStates, truthMeans, truthCovariances, ...
    estimateMeans, estimateCovariances, ...
    model.ospaParameters);
risk = euclideanOspa(1);
details = struct();
details.contractVersion = ...
    'lmb-current-eospa-oracle-v1';
details.currentTime = currentTime;
details.truthCardinality = numel(truthStates);
details.estimateCardinality = mapCardinality;
details.euclideanOspa = risk;
details.truthUsed = true;
details.deployable = false;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
