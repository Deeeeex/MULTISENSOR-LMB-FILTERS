function [risk, details] = ...
    computeLmbPosteriorSummaryBayesRisk(summary, options)
% COMPUTELMBPOSTERIORSUMMARYBAYESRISK Truth-free Bernoulli OSPA proxy.
%
% For each label, compare two admissible point decisions under the current
% Bernoulli posterior: report no object, or report one object at the
% posterior position mean.  With OSPA order two and cutoff c, the
% normalized conditional risks are
%
%   R_empty = r,
%   R_point = (1-r) + r * min(trace(P) / c^2, 1).
%
% The point-decision localization term is conservative: for
% Z = ||X-E[X]||^2/c^2, concavity of min(Z, 1) gives
% E[min(Z, 1)] <= min(E[Z], 1) = min(trace(P)/c^2, 1).  The minimum above
% is therefore an upper-bound proxy for the better of the two restricted
% decisions.  It is label-wise and truth-free; it is not an exact
% multi-object E-OSPA and it cannot detect a confidently biased posterior.
% A separate disagreement guard is therefore still required by the
% routing policy.

if nargin < 2 || isempty(options)
    options = struct();
end
positionCutoff = getField(options, 'positionCutoff', 100);
aggregationMode = lower(strrep(char(getField( ...
    options, 'labelAggregationMode', 'mean')), '_', '-'));
if ~isscalar(positionCutoff) || ~isfinite(positionCutoff) || ...
        positionCutoff <= 0 || ...
        ~ismember(aggregationMode, {'mean', 'sum'})
    error('Posterior Bayes-risk options are invalid.');
end
validateSummary(summary);
labelUniverse = getField(options, 'labelUniverse', summary.labels);
validateLabels(labelUniverse, 'label universe');

labelCount = size(labelUniverse, 2);
existence = zeros(1, labelCount);
normalizedPositionVariance = zeros(1, labelCount);
emptyDecisionRisk = zeros(1, labelCount);
pointDecisionRisk = zeros(1, labelCount);
perLabelRisk = zeros(1, labelCount);
pointDecisionSelected = false(1, labelCount);
for labelIdx = 1:labelCount
    summaryIdx = findLabel(summary.labels, ...
        labelUniverse(:, labelIdx));
    if summaryIdx > 0
        existence(labelIdx) = summary.existence(summaryIdx);
        covariance = summary.positionCovariance(:, :, summaryIdx);
        normalizedPositionVariance(labelIdx) = min(max( ...
            trace(covariance) / positionCutoff^2, 0), 1);
    end
    r = existence(labelIdx);
    emptyDecisionRisk(labelIdx) = r;
    pointDecisionRisk(labelIdx) = ...
        (1 - r) + r * normalizedPositionVariance(labelIdx);
    pointDecisionSelected(labelIdx) = ...
        pointDecisionRisk(labelIdx) < emptyDecisionRisk(labelIdx);
    perLabelRisk(labelIdx) = min( ...
        emptyDecisionRisk(labelIdx), pointDecisionRisk(labelIdx));
end

if isempty(perLabelRisk)
    risk = 0;
elseif strcmp(aggregationMode, 'sum')
    risk = sum(perLabelRisk);
else
    risk = mean(perLabelRisk);
end
if ~isfinite(risk) || risk < 0
    error('Posterior Bayes risk is invalid.');
end

details = struct( ...
    'contractVersion', 'lmb-posterior-bayes-risk-v1', ...
    'positionCutoff', positionCutoff, ...
    'labelAggregationMode', aggregationMode, ...
    'labelUniverse', labelUniverse, ...
    'labelCount', labelCount, ...
    'existence', existence, ...
    'normalizedPositionVariance', normalizedPositionVariance, ...
    'emptyDecisionRisk', emptyDecisionRisk, ...
    'pointDecisionRisk', pointDecisionRisk, ...
    'pointDecisionSelected', pointDecisionSelected, ...
    'perLabelRisk', perLabelRisk, ...
    'risk', risk, ...
    'pointDecisionRiskUpperBound', true, ...
    'restrictedDecisionRiskUpperBound', true, ...
    'multiObjectEospaExact', false, ...
    'confidentBiasDetectable', false, ...
    'truthUsed', false, ...
    'groundTruthUsed', false, ...
    'futureOutcomeUsed', false);
end

function validateSummary(summary)
required = {'labels', 'existence', 'positionCovariance'};
if ~isstruct(summary) || ...
        any(~isfield(summary, required))
    error('Posterior Bayes risk requires an LMB moment summary.');
end
validateLabels(summary.labels, 'summary labels');
labelCount = size(summary.labels, 2);
existence = reshape(summary.existence, 1, []);
if numel(existence) ~= labelCount || ...
        any(~isfinite(existence)) || ...
        any(existence < 0) || any(existence > 1) || ...
        size(summary.positionCovariance, 1) ~= 2 || ...
        size(summary.positionCovariance, 2) ~= 2 || ...
        size(summary.positionCovariance, 3) ~= labelCount
    error('Posterior Bayes-risk summary dimensions are invalid.');
end
for labelIdx = 1:labelCount
    covariance = summary.positionCovariance(:, :, labelIdx);
    transposed = reshape(covariance', [], 1);
    if any(~isfinite(covariance(:))) || ...
            max(abs(covariance(:) - transposed)) > 1e-9 || ...
            min(eig((covariance + covariance') / 2)) < -1e-9
        error('Posterior Bayes-risk covariance is invalid.');
    end
end
end

function validateLabels(labels, description)
if ~isnumeric(labels) || size(labels, 1) ~= 2 || ...
        any(~isfinite(labels(:))) || ...
        size(unique(labels', 'rows'), 1) ~= size(labels, 2)
    error('Posterior Bayes-risk %s are invalid.', description);
end
end

function index = findLabel(labels, label)
index = 0;
if isempty(labels)
    return;
end
match = find(all(labels == label, 1), 1);
if ~isempty(match)
    index = match;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
