function report = compareLmbPosteriorSnapshots(leftSnapshots, rightSnapshots)
% COMPARELMBPOSTERIORSNAPSHOTS Compare compact posterior sequences by label.
%
% Inputs must be same-sized cell arrays. Empty cells mean a snapshot is
% missing; an empty posterior must instead be represented by a valid compact
% snapshot whose arrays have zero labels.

if ~iscell(leftSnapshots) || ~iscell(rightSnapshots)
    error('compareLmbPosteriorSnapshots:InvalidInput', ...
        'Both inputs must be snapshot cell arrays.');
end
if ~isequal(size(leftSnapshots), size(rightSnapshots))
    error('compareLmbPosteriorSnapshots:SizeMismatch', ...
        'Snapshot cell arrays must have identical dimensions.');
end

report = struct( ...
    'snapshotCount', numel(leftSnapshots), ...
    'missingSnapshotCount', 0, ...
    'labelSetMismatchCount', 0, ...
    'missingLabelCount', 0, ...
    'missingFromLeftCount', 0, ...
    'missingFromRightCount', 0, ...
    'comparisonCount', 0, ...
    'maxAbsR', 0, ...
    'maxAbsMu', 0, ...
    'maxAbsSigma', 0, ...
    'maxExistenceResidual', 0, ...
    'maxMeanResidual', 0, ...
    'maxCovarianceResidual', 0, ...
    'exactMatch', true);

for snapshotIdx = 1:numel(leftSnapshots)
    left = leftSnapshots{snapshotIdx};
    right = rightSnapshots{snapshotIdx};
    if isempty(left) || isempty(right)
        report = recordMissingSnapshot(report, left, right);
        continue;
    end
    left = canonicalizeSnapshot(left);
    right = canonicalizeSnapshot(right);
    if size(left.means, 1) ~= size(right.means, 1)
        error('compareLmbPosteriorSnapshots:StateDimensionMismatch', ...
            'Snapshots at the same position must share a state dimension.');
    end

    [leftMatches, rightMatched] = matchLabels( ...
        left.labels, right.labels);
    missingFromRight = sum(leftMatches == 0);
    missingFromLeft = sum(~rightMatched);
    if missingFromRight > 0 || missingFromLeft > 0
        report.labelSetMismatchCount = ...
            report.labelSetMismatchCount + 1;
        report.missingFromRightCount = ...
            report.missingFromRightCount + missingFromRight;
        report.missingFromLeftCount = ...
            report.missingFromLeftCount + missingFromLeft;
        report.missingLabelCount = report.missingLabelCount + ...
            missingFromRight + missingFromLeft;
    end

    for leftIdx = find(leftMatches > 0)
        rightIdx = leftMatches(leftIdx);
        report.comparisonCount = report.comparisonCount + 1;
        report.maxAbsR = max(report.maxAbsR, ...
            abs(left.r(leftIdx) - right.r(rightIdx)));
        report.maxAbsMu = max(report.maxAbsMu, max(abs( ...
            left.means(:, leftIdx) - right.means(:, rightIdx))));
        covarianceDifference = abs( ...
            left.covariances(:, :, leftIdx) - ...
            right.covariances(:, :, rightIdx));
        report.maxAbsSigma = max(report.maxAbsSigma, ...
            max(covarianceDifference(:)));
    end
end

report.maxExistenceResidual = report.maxAbsR;
report.maxMeanResidual = report.maxAbsMu;
report.maxCovarianceResidual = report.maxAbsSigma;
report.exactMatch = report.missingSnapshotCount == 0 && ...
    report.labelSetMismatchCount == 0 && ...
    report.missingLabelCount == 0 && ...
    report.maxAbsR == 0 && report.maxAbsMu == 0 && ...
    report.maxAbsSigma == 0;
end

function report = recordMissingSnapshot(report, left, right)
leftMissing = isempty(left);
rightMissing = isempty(right);
report.missingSnapshotCount = report.missingSnapshotCount + ...
    leftMissing + rightMissing;
if leftMissing && rightMissing
    return;
end
report.labelSetMismatchCount = report.labelSetMismatchCount + 1;
if leftMissing
    right = canonicalizeSnapshot(right);
    missingCount = size(right.labels, 2);
    report.missingFromLeftCount = ...
        report.missingFromLeftCount + missingCount;
else
    left = canonicalizeSnapshot(left);
    missingCount = size(left.labels, 2);
    report.missingFromRightCount = ...
        report.missingFromRightCount + missingCount;
end
report.missingLabelCount = report.missingLabelCount + missingCount;
end

function snapshot = canonicalizeSnapshot(snapshot)
if ~isstruct(snapshot) || numel(snapshot) ~= 1
    error('compareLmbPosteriorSnapshots:InvalidSnapshot', ...
        'Each nonempty cell must contain one compact snapshot struct.');
end
required = {'labels', 'r', 'means', 'covariances'};
for fieldIdx = 1:numel(required)
    if ~isfield(snapshot, required{fieldIdx})
        error('compareLmbPosteriorSnapshots:MissingSnapshotField', ...
            'Snapshot is missing field %s.', required{fieldIdx});
    end
end
labels = snapshot.labels;
existence = snapshot.r;
means = snapshot.means;
covariances = snapshot.covariances;
if ~isnumeric(labels) || ~isreal(labels) || size(labels, 1) ~= 2 || ...
        any(~isfinite(labels(:))) || any(labels(:) ~= floor(labels(:)))
    error('compareLmbPosteriorSnapshots:InvalidLabels', ...
        'Snapshot labels must be a finite 2-by-N integer matrix.');
end
labelCount = size(labels, 2);
if ~isnumeric(existence) || ~isreal(existence) || ...
        numel(existence) ~= labelCount || any(~isfinite(existence(:)))
    error('compareLmbPosteriorSnapshots:InvalidExistence', ...
        'Snapshot existence values must contain N finite scalars.');
end
if ~isnumeric(means) || ~isreal(means) || ...
        size(means, 2) ~= labelCount || any(~isfinite(means(:)))
    error('compareLmbPosteriorSnapshots:InvalidMeans', ...
        'Snapshot means must be a finite d-by-N matrix.');
end
stateDimension = size(means, 1);
if ~isnumeric(covariances) || ~isreal(covariances) || ...
        size(covariances, 1) ~= stateDimension || ...
        size(covariances, 2) ~= stateDimension || ...
        size(covariances, 3) ~= labelCount || ...
        any(~isfinite(covariances(:)))
    error('compareLmbPosteriorSnapshots:InvalidCovariances', ...
        'Snapshot covariances must be a finite d-by-d-by-N array.');
end

[sortedLabels, order] = sortrows(double(labels'), [1, 2]);
sortedLabels = sortedLabels';
if labelCount > 1 && any(all( ...
        sortedLabels(:, 1:end-1) == sortedLabels(:, 2:end), 1))
    error('compareLmbPosteriorSnapshots:DuplicateLabel', ...
        'Snapshot labels must be unique.');
end
snapshot.labels = sortedLabels;
snapshot.r = reshape(double(existence(order)), 1, []);
snapshot.means = double(means(:, order));
snapshot.covariances = double(covariances(:, :, order));
end

function [leftMatches, rightMatched] = matchLabels(leftLabels, rightLabels)
leftCount = size(leftLabels, 2);
rightCount = size(rightLabels, 2);
leftMatches = zeros(1, leftCount);
rightMatched = false(1, rightCount);
for leftIdx = 1:leftCount
    match = find(all( ...
        rightLabels == leftLabels(:, leftIdx), 1), 1);
    if ~isempty(match)
        leftMatches(leftIdx) = match;
        rightMatched(match) = true;
    end
end
end
