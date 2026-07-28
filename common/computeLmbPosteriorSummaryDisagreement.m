function metrics = computeLmbPosteriorSummaryDisagreement(left, right)
% COMPUTELMBPOSTERIORSUMMARYDISAGREEMENT Compare cached LMB moments.

labels = collectLabels(left.labels, right.labels);
if isempty(labels)
    metrics = struct( ...
        'existence', 0, 'spatial', 0, ...
        'combined', 0, 'labelCount', 0);
    return;
end
existenceTerms = zeros(1, size(labels, 2));
spatialTerms = zeros(1, size(labels, 2));
combinedTerms = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    leftIdx = findLabel(left.labels, labels(:, labelIdx));
    rightIdx = findLabel(right.labels, labels(:, labelIdx));
    leftExistence = valueOrZero(left.existence, leftIdx);
    rightExistence = valueOrZero(right.existence, rightIdx);
    existenceTerms(labelIdx) = ...
        abs(leftExistence - rightExistence);
    if leftIdx > 0 && rightIdx > 0
        positionScale = sqrt(max(trace( ...
            left.positionCovariance(:, :, leftIdx) + ...
            right.positionCovariance(:, :, rightIdx)), 1));
        spatialTerms(labelIdx) = min(norm( ...
            left.positionMean(:, leftIdx) - ...
            right.positionMean(:, rightIdx)) / ...
            positionScale, 5);
    end
    combinedTerms(labelIdx) = existenceTerms(labelIdx) + ...
        min(leftExistence, rightExistence) * ...
            spatialTerms(labelIdx);
end
metrics = struct( ...
    'existence', mean(existenceTerms), ...
    'spatial', mean(spatialTerms), ...
    'combined', mean(combinedTerms), ...
    'labelCount', size(labels, 2));
end

function labels = collectLabels(leftLabels, rightLabels)
labels = leftLabels;
for labelIdx = 1:size(rightLabels, 2)
    if isempty(labels) || ...
            ~any(all(labels == rightLabels(:, labelIdx), 1))
        labels(:, end + 1) = rightLabels(:, labelIdx); %#ok<AGROW>
    end
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

function value = valueOrZero(values, index)
if index < 1
    value = 0;
else
    value = values(index);
end
end
