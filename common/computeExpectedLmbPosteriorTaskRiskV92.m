function [networkRisk, details] = ...
    computeExpectedLmbPosteriorTaskRiskV92( ...
        networkDetails, groupIds, model, options)
% COMPUTEEXPECTEDLMBPOSTERIORTASKRISKV92 Truth-free LMB risk proxy.
%
% The per-label existence term is the minimum equal-cost Bernoulli decision
% error.  The localization term is the posterior position MSE normalized and
% capped by the registered Euclidean OSPA cutoff.  Delivery outcomes are
% averaged exactly from the one-round routing enumerator.  The scalar network
% risk combines the receiver mean and upper receiver tail; it is a routing
% proxy, not an equality to E-OSPA or a recursive tracking guarantee.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(networkDetails) || ~isscalar(networkDetails) || ...
        ~isfield(networkDetails, 'receiverDistributions') || ...
        ~iscell(networkDetails.receiverDistributions) || ...
        ~isstruct(model) || ~isscalar(model)
    error('TaskRiskV92:InvalidInput', ...
        'One valid expected-routing distribution and model are required.');
end
receiverDistributions = networkDetails.receiverDistributions;
nodeCount = numel(receiverDistributions);
groupIds = reshape(groupIds, 1, []);
if nodeCount < 1 || numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds))
    error('TaskRiskV92:InvalidInput', ...
        'Formation identities must match the receiver distributions.');
end

existenceWeight = getField(options, 'existenceWeight', 0.50);
localizationWeight = getField(options, 'localizationWeight', 0.50);
tailFraction = getField(options, 'receiverTailFraction', 0.25);
tailWeight = getField(options, 'receiverTailWeight', 0.50);
weights = [existenceWeight, localizationWeight];
if any(~isfinite(weights)) || any(weights < 0) || sum(weights) <= 0 || ...
        ~isscalar(tailFraction) || ~isfinite(tailFraction) || ...
        tailFraction <= 0 || tailFraction > 1 || ...
        ~isscalar(tailWeight) || ~isfinite(tailWeight) || tailWeight < 0
    error('TaskRiskV92:InvalidOptions', ...
        'Task-risk weights and receiver-tail options are invalid.');
end
weights = weights / sum(weights);
positionCutoff = resolvePositionCutoff(model, options);

expectedReceiverRisk = zeros(1, nodeCount);
expectedReceiverExistenceRisk = zeros(1, nodeCount);
expectedReceiverLocalizationRisk = zeros(1, nodeCount);
expectedReceiverLabelCount = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    distribution = receiverDistributions{receiverIdx};
    validateDistribution(distribution);
    for outcomeIdx = 1:numel(distribution.probability)
        probability = distribution.probability(outcomeIdx);
        summary = distribution.summary{outcomeIdx};
        [existenceRisk, localizationRisk, labelCount] = ...
            outcomeRisk(summary, positionCutoff);
        expectedReceiverExistenceRisk(receiverIdx) = ...
            expectedReceiverExistenceRisk(receiverIdx) + ...
            probability * existenceRisk;
        expectedReceiverLocalizationRisk(receiverIdx) = ...
            expectedReceiverLocalizationRisk(receiverIdx) + ...
            probability * localizationRisk;
        expectedReceiverLabelCount(receiverIdx) = ...
            expectedReceiverLabelCount(receiverIdx) + ...
            probability * labelCount;
    end
    expectedReceiverRisk(receiverIdx) = ...
        weights(1) * expectedReceiverExistenceRisk(receiverIdx) + ...
        weights(2) * expectedReceiverLocalizationRisk(receiverIdx);
end

meanReceiverRisk = mean(expectedReceiverRisk);
tailReceiverRisk = upperTailMean(expectedReceiverRisk, tailFraction);
networkRisk = meanReceiverRisk + tailWeight * tailReceiverRisk;
groups = unique(groupIds, 'stable');
formationMeanRisk = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    formationMeanRisk(formationIdx) = mean( ...
        expectedReceiverRisk(groupIds == groups(formationIdx)));
end
if ~isfinite(networkRisk) || any(~isfinite(formationMeanRisk))
    error('TaskRiskV92:NonFiniteRisk', ...
        'The expected posterior task risk is non-finite.');
end

details = struct();
details.contractVersion = 'expected-lmb-posterior-task-risk-v92-v1';
details.networkRisk = networkRisk;
details.meanReceiverRisk = meanReceiverRisk;
details.tailReceiverRisk = tailReceiverRisk;
details.maximumReceiverRisk = max(expectedReceiverRisk);
details.expectedReceiverRisk = expectedReceiverRisk;
details.expectedReceiverExistenceRisk = ...
    expectedReceiverExistenceRisk;
details.expectedReceiverLocalizationRisk = ...
    expectedReceiverLocalizationRisk;
details.expectedReceiverLabelCount = expectedReceiverLabelCount;
details.formationIds = groups;
details.formationMeanRisk = formationMeanRisk;
details.maximumFormationMeanRisk = max(formationMeanRisk);
details.positionCutoff = positionCutoff;
details.existenceWeight = weights(1);
details.localizationWeight = weights(2);
details.receiverTailFraction = tailFraction;
details.receiverTailWeight = tailWeight;
details.existenceTerm = 'minimum-equal-cost-bernoulli-decision-error';
details.localizationTerm = ...
    'existence-weighted-capped-position-posterior-mse';
details.trackingOutcomeUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.eospaEqualityClaimed = false;
details.recursiveTrackingGuaranteeClaimed = false;
end

function [existenceRisk, localizationRisk, labelCount] = ...
        outcomeRisk(summary, positionCutoff)
required = {'labels', 'existence', 'positionCovariance'};
if ~isstruct(summary) || ~isscalar(summary) || ...
        ~all(isfield(summary, required))
    error('TaskRiskV92:InvalidSummary', ...
        'A receiver outcome lacks an LMB moment summary.');
end
existence = reshape(summary.existence, 1, []);
labelCount = numel(existence);
if size(summary.labels, 2) ~= labelCount || ...
        size(summary.positionCovariance, 3) ~= labelCount || ...
        any(~isfinite(existence)) || any(existence < 0) || ...
        any(existence > 1)
    error('TaskRiskV92:InvalidSummary', ...
        'An LMB outcome summary is malformed.');
end
existenceRisk = sum(min(existence, 1 - existence));
localizationRisk = 0;
cutoffSquared = positionCutoff^2;
for labelIdx = 1:labelCount
    covariance = summary.positionCovariance(:, :, labelIdx);
    if ~isequal(size(covariance), [2, 2]) || ...
            any(~isfinite(covariance(:)))
        error('TaskRiskV92:InvalidSummary', ...
            'A position covariance is malformed.');
    end
    covariance = (covariance + covariance') / 2;
    normalizedMse = max(trace(covariance), 0) / cutoffSquared;
    localizationRisk = localizationRisk + existence(labelIdx) * ...
        min(normalizedMse, 1);
end
end

function validateDistribution(distribution)
required = {'probability', 'summary'};
if ~isstruct(distribution) || ~isscalar(distribution) || ...
        ~all(isfield(distribution, required)) || ...
        ~isnumeric(distribution.probability) || ...
        ~iscell(distribution.summary) || ...
        numel(distribution.probability) ~= ...
            numel(distribution.summary) || ...
        any(~isfinite(distribution.probability)) || ...
        any(distribution.probability < 0) || ...
        abs(sum(distribution.probability) - 1) > 1e-10
    error('TaskRiskV92:InvalidDistribution', ...
        'A receiver delivery-outcome distribution is malformed.');
end
end

function value = resolvePositionCutoff(model, options)
value = getField(options, 'positionCutoff', NaN);
if ~isfinite(value) && isfield(model, 'ospaParameters') && ...
        isstruct(model.ospaParameters) && ...
        isfield(model.ospaParameters, 'eC')
    value = model.ospaParameters.eC;
end
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('TaskRiskV92:MissingPositionCutoff', ...
        'The model must provide a positive Euclidean OSPA cutoff.');
end
end

function value = upperTailMean(values, fraction)
values = sort(reshape(values, 1, []), 'descend');
count = max(1, ceil(fraction * numel(values)));
value = mean(values(1:count));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
