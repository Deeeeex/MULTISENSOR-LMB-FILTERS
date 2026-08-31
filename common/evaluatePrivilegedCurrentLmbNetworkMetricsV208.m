function metrics = evaluatePrivilegedCurrentLmbNetworkMetricsV208( ...
        posteriorBySensor, model, currentTime, groupIds, ...
        baseline, affectedReceiverIds)
% EVALUATEPRIVILEGEDCURRENTLMBNETWORKMETRICSV208 Offline action targets.
%
% Truth is used only to score a frozen current-state action.  When a
% baseline result is supplied, only affected receiver metrics and consensus
% pairs are recomputed.

if nargin < 5
    baseline = [];
end
if nargin < 6
    affectedReceiverIds = [];
end
sensorCount = numel(posteriorBySensor);
groupIds = reshape(groupIds, 1, []);
if ~iscell(posteriorBySensor) || sensorCount < 1 || ...
        numel(groupIds) ~= sensorCount || ...
        ~isstruct(model) || ...
        ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'targetTrajectories') || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime < 1 || currentTime ~= round(currentTime)
    error('PrivilegedNetworkMetricsV208:InvalidInput', ...
        'Posterior, truth-bearing model, time, and groups are required.');
end

if isempty(baseline)
    eospa = nan(1, sensorCount);
    rmse = nan(1, sensorCount);
    cardinalityError = nan(1, sensorCount);
    estimates = cell(1, sensorCount);
    consensusMatrix = nan(sensorCount);
    evaluationIds = 1:sensorCount;
else
    required = {'eospaBySensor', 'rmseBySensor', ...
        'cardinalityErrorBySensor', ...
        'internalStateEstimateBySensor', ...
        'internalConsensusOspaMatrix'};
    if ~isstruct(baseline) || ~all(isfield(baseline, required))
        error('PrivilegedNetworkMetricsV208:InvalidBaseline', ...
            'The incremental baseline is incomplete.');
    end
    eospa = baseline.eospaBySensor;
    rmse = baseline.rmseBySensor;
    cardinalityError = baseline.cardinalityErrorBySensor;
    estimates = baseline.internalStateEstimateBySensor;
    consensusMatrix = baseline.internalConsensusOspaMatrix;
    evaluationIds = reshape(unique(affectedReceiverIds), 1, []);
    if any(evaluationIds < 1 | evaluationIds > sensorCount) || ...
            any(evaluationIds ~= round(evaluationIds))
        error('PrivilegedNetworkMetricsV208:InvalidReceivers', ...
            'Affected receiver identifiers are invalid.');
    end
end

truthCount = currentTruthCount(model, currentTime);
for sensorIdx = evaluationIds
    posterior = posteriorBySensor{sensorIdx};
    eospa(sensorIdx) = evaluateLmbTopologyCurrentEospa( ...
        posterior, model, currentTime, struct());
    rmse(sensorIdx) = currentPosteriorRmse( ...
        posterior, model, currentTime);
    estimates{sensorIdx} = posteriorStateEstimate(posterior, model);
    cardinalityError(sensorIdx) = abs( ...
        numel(estimates{sensorIdx}.mu{1}) - truthCount);
end

if isempty(baseline)
    for leftIdx = 1:sensorCount-1
        for rightIdx = leftIdx+1:sensorCount
            value = estimateStateSetOspa( ...
                estimates{leftIdx}, estimates{rightIdx}, model);
            consensusMatrix(leftIdx, rightIdx) = value;
            consensusMatrix(rightIdx, leftIdx) = value;
        end
    end
else
    for receiverIdx = evaluationIds
        peers = setdiff(1:sensorCount, receiverIdx);
        for peerIdx = reshape(peers, 1, [])
            value = estimateStateSetOspa( ...
                estimates{receiverIdx}, estimates{peerIdx}, model);
            consensusMatrix(receiverIdx, peerIdx) = value;
            consensusMatrix(peerIdx, receiverIdx) = value;
        end
    end
end

upperMask = triu(true(sensorCount), 1);
formationIds = unique(groupIds, 'stable');
formationEospa = nan(1, numel(formationIds));
formationRmse = nan(1, numel(formationIds));
for formationIdx = 1:numel(formationIds)
    members = groupIds == formationIds(formationIdx);
    formationEospa(formationIdx) = finiteMean(eospa(members));
    formationRmse(formationIdx) = finiteMean(rmse(members));
end

metrics = struct();
metrics.contractVersion = ...
    'privileged-current-lmb-network-metrics-v208-v1';
metrics.meanEospa = finiteMean(eospa);
metrics.meanRmse = finiteMean(rmse);
metrics.meanCardinalityError = finiteMean(cardinalityError);
metrics.consensusOspa = finiteMean(consensusMatrix(upperMask));
metrics.worstSensorEospa = maximumFinite(eospa);
metrics.worstSensorRmse = maximumFinite(rmse);
metrics.eospaBySensor = eospa;
metrics.rmseBySensor = rmse;
metrics.cardinalityErrorBySensor = cardinalityError;
metrics.formationIds = formationIds;
metrics.formationMeanEospa = formationEospa;
metrics.formationMeanRmse = formationRmse;
metrics.internalStateEstimateBySensor = estimates;
metrics.internalConsensusOspaMatrix = consensusMatrix;
metrics.truthUsed = true;
metrics.futureInformationUsed = false;
metrics.deployable = false;
end

function estimate = posteriorStateEstimate(posterior, model)
objects = reshape(posterior, 1, []);
if ~isempty(objects)
    objects = objects([objects.r] > model.existenceThreshold);
end
estimate = struct('labels', {{zeros(2, 0)}}, ...
    'mu', {{cell(1, 0)}}, 'Sigma', {{cell(1, 0)}});
if isempty(objects)
    return;
end
[cardinality, indices] = lmbMapCardinalityEstimate([objects.r]);
estimate.labels{1} = zeros(2, cardinality);
estimate.mu{1} = cell(1, cardinality);
estimate.Sigma{1} = cell(1, cardinality);
for idx = 1:cardinality
    object = objects(indices(idx));
    componentIdx = maximumWeightComponent(object);
    estimate.labels{1}(:, idx) = ...
        [object.birthTime; object.birthLocation];
    estimate.mu{1}{idx} = object.mu{componentIdx};
    estimate.Sigma{1}{idx} = object.Sigma{componentIdx};
end
end

function idx = maximumWeightComponent(object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = -inf;
[~, idx] = max(weights);
if isempty(idx) || ~isfinite(weights(idx))
    idx = 1;
end
end

function value = currentPosteriorRmse(posterior, model, currentTime)
estimate = posteriorStateEstimate(posterior, model);
estimatePosition = zeros(2, numel(estimate.mu{1}));
for idx = 1:numel(estimate.mu{1})
    estimatePosition(:, idx) = estimate.mu{1}{idx}(1:2);
end
truth = currentTruthPositions(model, currentTime);
if isempty(truth) && isempty(estimatePosition)
    value = 0;
    return;
elseif isempty(truth) || isempty(estimatePosition)
    value = NaN;
    return;
end
distances = pairwiseDistances(truth, estimatePosition);
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
if isempty(matched)
    value = NaN;
else
    value = sqrt(mean(matched .^ 2));
end
end

function positions = currentTruthPositions(model, currentTime)
positions = zeros(2, 0);
trajectories = model.dynamicTopologyScenario.targetTrajectories;
for targetIdx = 1:numel(trajectories)
    if currentTime <= size(trajectories{targetIdx}, 2)
        state = trajectories{targetIdx}(:, currentTime);
        if all(isfinite(state))
            positions(:, end + 1) = state(1:2); %#ok<AGROW>
        end
    end
end
end

function count = currentTruthCount(model, currentTime)
count = size(currentTruthPositions(model, currentTime), 2);
end

function distance = estimateStateSetOspa(left, right, model)
leftMu = left.mu{1};
leftSigma = left.Sigma{1};
rightMu = right.mu{1};
rightSigma = right.Sigma{1};
if isempty(leftMu) && isempty(rightMu)
    distance = 0;
    return;
elseif isempty(leftMu) || isempty(rightMu)
    distance = model.ospaParameters.eC;
    return;
end
[leftToRight, ~] = ospa( ...
    leftMu, leftMu, leftSigma, rightMu, rightSigma, ...
    model.ospaParameters);
[rightToLeft, ~] = ospa( ...
    rightMu, rightMu, rightSigma, leftMu, leftSigma, ...
    model.ospaParameters);
distance = 0.5 * (leftToRight(1) + rightToLeft(1));
end

function values = pairwiseDistances(left, right)
values = zeros(size(left, 2), size(right, 2));
for leftIdx = 1:size(left, 2)
    delta = bsxfun(@minus, right, left(:, leftIdx));
    values(leftIdx, :) = sqrt(sum(delta .^ 2, 1));
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

function value = maximumFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end
