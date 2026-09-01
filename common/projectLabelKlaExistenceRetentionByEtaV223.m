function [allowed, details] = ...
        projectLabelKlaExistenceRetentionByEtaV223( ...
            inputExistence, existenceWeights, logEta, ...
            referenceExistence, options)
% PROJECTLABELKLAEXISTENCERETENTIONBYETAV223 Bernoulli-KLA eta gate.
%
% A routed label is admitted only when the spatial-overlap normalizer produced
% by the active fusion implementation preserves enough Bernoulli existence
% relative to the ordinary KLA result.
% The identity used by the projector is
%
%   logit(r_fused) = sum_i w_i logit(r_i) + log(eta).
%
% The log-odds identity is exact for the supplied eta.  In the current
% mixture-aware path, eta itself is produced by the repository's truncated
% componentwise powered-GM approximation.  This helper is deliberately
% independent of a learned value score: a model may rank candidates, but it
% cannot override the deterministic eta gate.

if nargin < 5 || isempty(options)
    options = struct();
end
minimumMapExistence = getField(options, ...
    'minimumMapExistence', 0.50);
maximumReferenceLogOddsDrop = getField(options, ...
    'maximumReferenceLogOddsDrop', 0);
identityTolerance = getField(options, 'identityTolerance', 1e-10);

inputExistence = reshape(inputExistence, 1, []);
existenceWeights = reshape(existenceWeights, 1, []);
if isempty(inputExistence) || ...
        numel(inputExistence) ~= numel(existenceWeights) || ...
        any(~isfinite(inputExistence)) || ...
        any(inputExistence <= 0) || any(inputExistence >= 1) || ...
        any(~isfinite(existenceWeights)) || ...
        any(existenceWeights < 0) || sum(existenceWeights) <= 0 || ...
        ~isscalar(logEta) || ~isfinite(logEta) || logEta > 1e-12 || ...
        ~validProbability(referenceExistence) || ...
        ~validProbability(minimumMapExistence) || ...
        ~isscalar(maximumReferenceLogOddsDrop) || ...
        ~isfinite(maximumReferenceLogOddsDrop) || ...
        maximumReferenceLogOddsDrop < 0 || ...
        ~isscalar(identityTolerance) || ~isfinite(identityTolerance) || ...
        identityTolerance < 0
    error('LabelKlaEtaProjectionV223:InvalidInput', ...
        'The eta-aware existence-retention request is malformed.');
end

existenceWeights = existenceWeights / sum(existenceWeights);
inputLogOdds = probabilityLogOdds(inputExistence);
weightedInputLogOdds = sum(existenceWeights .* inputLogOdds);
candidateLogOdds = weightedInputLogOdds + logEta;
candidateExistence = logistic(candidateLogOdds);
referenceLogOdds = probabilityLogOdds(referenceExistence);

% Protect an ordinary MAP-positive label from crossing the extraction
% threshold.  For a MAP-negative ordinary label, do not make its existence
% evidence worse.  A nonzero tolerance may later be frozen from development
% data, but it must remain an explicit protocol parameter.
if referenceExistence >= minimumMapExistence
    requiredFusedLogOdds = max( ...
        probabilityLogOdds(minimumMapExistence), ...
        referenceLogOdds - maximumReferenceLogOddsDrop);
    protectedMapPositiveReference = true;
else
    requiredFusedLogOdds = ...
        referenceLogOdds - maximumReferenceLogOddsDrop;
    protectedMapPositiveReference = false;
end
requiredLogEta = requiredFusedLogOdds - weightedInputLogOdds;
allowed = logEta >= requiredLogEta - identityTolerance;

details = struct();
details.contractVersion = 'label-kla-eta-retention-projection-v223-v1';
details.normalizedExistenceWeights = existenceWeights;
details.inputExistence = inputExistence;
details.inputLogOdds = inputLogOdds;
details.weightedInputLogOdds = weightedInputLogOdds;
details.logEta = logEta;
details.spatialNormalizer = exp(logEta);
details.candidateLogOdds = candidateLogOdds;
details.candidateExistence = candidateExistence;
details.referenceExistence = referenceExistence;
details.referenceLogOdds = referenceLogOdds;
details.minimumMapExistence = minimumMapExistence;
details.maximumReferenceLogOddsDrop = maximumReferenceLogOddsDrop;
details.requiredFusedLogOdds = requiredFusedLogOdds;
details.requiredLogEta = requiredLogEta;
details.logEtaMargin = logEta - requiredLogEta;
details.protectedMapPositiveReference = ...
    protectedMapPositiveReference;
details.allowed = allowed;
details.truthUsed = false;
details.futureInformationUsed = false;
details.learnedOverrideAllowed = false;
end

function values = probabilityLogOdds(probabilities)
probabilities = min(max(probabilities, realmin), 1 - eps);
values = log(probabilities) - log(1 - probabilities);
end

function value = logistic(logOdds)
if logOdds >= 0
    value = 1 / (1 + exp(-logOdds));
else
    expValue = exp(logOdds);
    value = expValue / (1 + expValue);
end
end

function valid = validProbability(value)
valid = isscalar(value) && isnumeric(value) && isfinite(value) && ...
    value > 0 && value < 1;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
