function [posterior, details] = ...
        selectPositiveValueReferenceLmbLabels( ...
            currentPosterior, referencePosterior, maximumLabelEdits, ...
            existenceThreshold, model, currentTime)
% SELECTPOSITIVEVALUEREFERENCELMBLABELS Greedy truth-valued label oracle.

if ~isscalar(maximumLabelEdits) || ~isfinite(maximumLabelEdits) || ...
        maximumLabelEdits < 1 || ...
        maximumLabelEdits ~= round(maximumLabelEdits)
    error('PositiveValueReferenceLabel:InvalidCapacity', ...
        'The positive-value label capacity must be a positive integer.');
end
posterior = reshape(currentPosterior, 1, []);
referencePosterior = reshape(referencePosterior, 1, []);
labels = unique([posteriorLabels(posterior), ...
    posteriorLabels(referencePosterior)]', 'rows', 'stable')';
if size(unique(posteriorLabels(posterior)', 'rows'), 1) ~= ...
        numel(posterior) || ...
        size(unique(posteriorLabels(referencePosterior)', 'rows'), 1) ~= ...
        numel(referencePosterior)
    error('PositiveValueReferenceLabel:DuplicateLabel', ...
        'A source LMB posterior contains duplicate labels.');
end
candidateLabels = zeros(2, 0);
for idx = 1:size(labels, 2)
    label = labels(:, idx);
    currentIdx = findLabel(posteriorLabels(posterior), label);
    referenceIdx = findLabel( ...
        posteriorLabels(referencePosterior), label);
    if isempty(currentIdx) || isempty(referenceIdx) || ...
            ~isequaln(posterior(currentIdx), ...
                referencePosterior(referenceIdx))
        candidateLabels(:, end + 1) = label; %#ok<AGROW>
    end
end

[currentRisk, ~] = evaluateLmbTopologyCurrentEospa( ...
    posterior, model, currentTime, struct());
details = struct( ...
    'contractVersion', 'positive-value-reference-label-selection-v1', ...
    'candidateLabelCount', size(candidateLabels, 2), ...
    'selectedLabelCount', 0, ...
    'selectedLabels', zeros(2, 0), ...
    'selectedTombstone', false(1, 0), ...
    'marginalEospaReduction', zeros(1, 0), ...
    'initialEospa', currentRisk, ...
    'finalEospa', currentRisk, ...
    'estimatedPayloadBytes', 0, ...
    'truthUsed', true);
selected = false(1, size(candidateLabels, 2));
tolerance = 1e-9;
for editIdx = 1:maximumLabelEdits
    bestGain = 0;
    bestRisk = currentRisk;
    bestCandidateIdx = 0;
    bestPosterior = posterior;
    bestTombstone = false;
    for candidateIdx = find(~selected)
        label = candidateLabels(:, candidateIdx);
        [trial, tombstone] = applyReferenceEdit( ...
            posterior, referencePosterior, label);
        [trialRisk, ~] = evaluateLmbTopologyCurrentEospa( ...
            trial, model, currentTime, struct());
        gain = currentRisk - trialRisk;
        if gain > bestGain + tolerance || ...
                (abs(gain - bestGain) <= tolerance && gain > tolerance && ...
                 (bestCandidateIdx == 0 || lexicographicallyBefore( ...
                    label, candidateLabels(:, bestCandidateIdx))))
            bestGain = gain;
            bestRisk = trialRisk;
            bestCandidateIdx = candidateIdx;
            bestPosterior = trial;
            bestTombstone = tombstone;
        end
    end
    if bestCandidateIdx == 0 || bestGain <= tolerance
        break;
    end
    selected(bestCandidateIdx) = true;
    posterior = bestPosterior;
    currentRisk = bestRisk;
    label = candidateLabels(:, bestCandidateIdx);
    details.selectedLabels(:, end + 1) = label; %#ok<AGROW>
    details.selectedTombstone(end + 1) = bestTombstone; %#ok<AGROW>
    details.marginalEospaReduction(end + 1) = bestGain; %#ok<AGROW>
end
details.selectedLabelCount = size(details.selectedLabels, 2);
details.finalEospa = currentRisk;
if details.selectedLabelCount > 0
    details.estimatedPayloadBytes = 32;
end
for idx = 1:details.selectedLabelCount
    label = details.selectedLabels(:, idx);
    referenceIdx = findLabel( ...
        posteriorLabels(referencePosterior), label);
    if isempty(referenceIdx)
        details.estimatedPayloadBytes = ...
            details.estimatedPayloadBytes + 8 * 3;
    else
        object = referencePosterior(referenceIdx);
        stateDimension = numel(object.mu{1});
        details.estimatedPayloadBytes = ...
            details.estimatedPayloadBytes + 8 * ( ...
                3 + object.numberOfGmComponents * ...
                (1 + stateDimension + stateDimension * stateDimension));
    end
end
if details.finalEospa > details.initialEospa + tolerance
    error('PositiveValueReferenceLabel:NonmonotoneSelection', ...
        'The positive-value label oracle increased current E-OSPA.');
end
if existenceThreshold < 0 || existenceThreshold >= 1
    error('PositiveValueReferenceLabel:InvalidExistenceThreshold', ...
        'The LMB existence threshold is invalid.');
end
end

function [posterior, tombstone] = ...
        applyReferenceEdit(posterior, referencePosterior, label)
currentIdx = findLabel(posteriorLabels(posterior), label);
referenceIdx = findLabel(posteriorLabels(referencePosterior), label);
tombstone = isempty(referenceIdx);
if tombstone
    if ~isempty(currentIdx)
        posterior(currentIdx) = [];
    end
elseif isempty(currentIdx)
    posterior(end + 1) = referencePosterior(referenceIdx); %#ok<AGROW>
else
    posterior(currentIdx) = referencePosterior(referenceIdx);
end
end

function labels = posteriorLabels(objects)
labels = zeros(2, numel(objects));
for idx = 1:numel(objects)
    labels(:, idx) = [objects(idx).birthTime; objects(idx).birthLocation];
end
end

function idx = findLabel(labels, label)
idx = find(all(labels == label, 1), 1);
end

function result = lexicographicallyBefore(left, right)
result = left(1) < right(1) || ...
    (left(1) == right(1) && left(2) < right(2));
end
