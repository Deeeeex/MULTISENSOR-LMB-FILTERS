function [posterior, details] = mergeSparseReferenceLmbLabels( ...
        currentPosterior, referencePosterior, maximumLabelEdits, ...
        existenceThreshold)
% MERGESPARSEREFERENCELMBLABELS Apply bounded complete-label state edits.
%
% The ranking is deliberately reference-relative and therefore privileged.
% A selected label copies its complete GM Bernoulli state from the reference;
% a label absent from the reference becomes an explicit tombstone.

if nargin < 4 || isempty(existenceThreshold)
    existenceThreshold = 1e-2;
end
if ~isscalar(maximumLabelEdits) || ~isfinite(maximumLabelEdits) || ...
        maximumLabelEdits < 0 || ...
        maximumLabelEdits ~= round(maximumLabelEdits)
    error('SparseReferenceLabel:InvalidCapacity', ...
        'The maximum number of complete-label edits must be a nonnegative integer.');
end
currentPosterior = reshape(currentPosterior, 1, []);
referencePosterior = reshape(referencePosterior, 1, []);
current = summarizePosterior(currentPosterior, existenceThreshold);
reference = summarizePosterior(referencePosterior, existenceThreshold);
labels = unique([current.labels, reference.labels]', 'rows', 'stable')';
candidate = repmat(emptyCandidate(), 1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    currentIdx = findLabel(current.labels, label);
    referenceIdx = findLabel(reference.labels, label);
    if ~isempty(currentIdx) && ~isempty(referenceIdx) && ...
            isequaln(currentPosterior(currentIdx), ...
                referencePosterior(referenceIdx))
        continue;
    end
    currentR = 0;
    referenceR = 0;
    if ~isempty(currentIdx)
        currentR = current.existence(currentIdx);
    end
    if ~isempty(referenceIdx)
        referenceR = reference.existence(referenceIdx);
    end
    row = emptyCandidate();
    row.label = label;
    row.currentIndex = currentIdx;
    row.referenceIndex = referenceIdx;
    row.mapMembershipMismatch = ...
        ismemberLabel(current.mapLabels, label) ~= ...
        ismemberLabel(reference.mapLabels, label);
    row.supportCrossing = ...
        (currentR > existenceThreshold) ~= ...
        (referenceR > existenceThreshold);
    row.existenceGap = abs(currentR - referenceR);
    row.positionGap = positionGap( ...
        currentPosterior, currentIdx, referencePosterior, referenceIdx);
    candidate(end + 1) = row; %#ok<AGROW>
end

posterior = currentPosterior;
details = struct( ...
    'contractVersion', 'sparse-reference-label-merge-v1', ...
    'candidateLabelCount', numel(candidate), ...
    'selectedLabelCount', 0, ...
    'selectedLabels', zeros(2, 0), ...
    'selectedTombstone', false(1, 0), ...
    'estimatedPayloadBytes', 0, ...
    'privilegedReferenceUsed', maximumLabelEdits > 0);
if maximumLabelEdits == 0 || isempty(candidate)
    return;
end
ranking = zeros(numel(candidate), 6);
for idx = 1:numel(candidate)
    finitePositionGap = candidate(idx).positionGap;
    if ~isfinite(finitePositionGap)
        finitePositionGap = realmax('double');
    end
    ranking(idx, :) = [ ...
        -double(candidate(idx).mapMembershipMismatch), ...
        -double(candidate(idx).supportCrossing), ...
        -candidate(idx).existenceGap, ...
        -finitePositionGap, ...
        candidate(idx).label(1), candidate(idx).label(2)];
end
[~, order] = sortrows(ranking, 1:size(ranking, 2));
selected = candidate(order(1:min(maximumLabelEdits, numel(order))));

selectedBytes = 32; % One compact message header at this receiver-time cell.
for idx = 1:numel(selected)
    label = selected(idx).label;
    currentIdx = findLabel(posteriorLabels(posterior), label);
    referenceIdx = selected(idx).referenceIndex;
    tombstone = isempty(referenceIdx);
    if tombstone
        if ~isempty(currentIdx)
            posterior(currentIdx) = [];
        end
        selectedBytes = selectedBytes + 8 * 3;
    else
        referenceObject = referencePosterior(referenceIdx);
        if isempty(currentIdx)
            posterior(end + 1) = referenceObject; %#ok<AGROW>
        else
            posterior(currentIdx) = referenceObject;
        end
        stateDimension = numel(referenceObject.mu{1});
        selectedBytes = selectedBytes + objectBytes( ...
            referenceObject, stateDimension);
    end
    details.selectedLabels(:, end + 1) = label; %#ok<AGROW>
    details.selectedTombstone(end + 1) = tombstone; %#ok<AGROW>
end
details.selectedLabelCount = numel(selected);
details.estimatedPayloadBytes = selectedBytes;
end

function summary = summarizePosterior(objects, threshold)
labels = posteriorLabels(objects);
if size(unique(labels', 'rows'), 1) ~= size(labels, 2)
    error('SparseReferenceLabel:DuplicateLabel', ...
        'A source LMB posterior contains duplicate labels.');
end
existence = zeros(1, numel(objects));
if ~isempty(objects)
    existence = [objects.r];
end
active = existence > threshold;
activeLabels = labels(:, active);
mapLabels = zeros(2, 0);
if any(active)
    [~, mapIndices] = lmbMapCardinalityEstimate(existence(active));
    mapLabels = activeLabels(:, mapIndices);
end
summary = struct( ...
    'labels', labels, ...
    'existence', existence, ...
    'mapLabels', mapLabels);
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

function present = ismemberLabel(labels, label)
present = any(all(labels == label, 1));
end

function gap = positionGap(left, leftIdx, right, rightIdx)
if isempty(leftIdx) || isempty(rightIdx) || ...
        left(leftIdx).numberOfGmComponents < 1 || ...
        right(rightIdx).numberOfGmComponents < 1
    gap = inf;
    return;
end
gap = norm(left(leftIdx).mu{1}(1:2) - right(rightIdx).mu{1}(1:2));
end

function bytes = objectBytes(object, stateDimension)
scalarCount = 3 + object.numberOfGmComponents * ...
    (1 + stateDimension + stateDimension * stateDimension);
bytes = 8 * scalarCount;
end

function row = emptyCandidate()
row = struct( ...
    'label', zeros(2, 1), ...
    'currentIndex', [], ...
    'referenceIndex', [], ...
    'mapMembershipMismatch', false, ...
    'supportCrossing', false, ...
    'existenceGap', 0, ...
    'positionGap', 0);
end
