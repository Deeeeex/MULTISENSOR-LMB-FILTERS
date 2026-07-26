function records = buildBehaviorContinuationHistoryRecords( ...
        diagnostics, snapshotTimes, historyDepth)
% BUILDBEHAVIORCONTINUATIONHISTORYRECORDS Bind history to predecision state.

if nargin < 3 || isempty(historyDepth)
    historyDepth = 2;
end
historyDepth = max(0, round(historyDepth));
snapshotTimes = reshape(snapshotTimes, 1, []);
if isempty(snapshotTimes)
    records = {};
    return;
end
required = {'topologyActiveEdge', 'delivered'};
if ~all(isfield(diagnostics, required))
    error('Behavior diagnostics do not contain topology histories.');
end
selected = logical(diagnostics.topologyActiveEdge);
delivered = logical(diagnostics.delivered);
if ~isequal(size(selected), size(delivered)) || ...
        size(selected, 1) ~= size(selected, 2)
    error('Behavior selected/delivered histories have incompatible shapes.');
end
sensorCount = size(selected, 1);
timeCount = size(selected, 3);
if any(snapshotTimes < 1 | snapshotTimes > timeCount | ...
        mod(snapshotTimes, 1) ~= 0)
    error('Behavior history snapshot time is outside diagnostics.');
end
records = cell(1, max(snapshotTimes));
for snapshotTime = snapshotTimes
    count = min(historyDepth, snapshotTime - 1);
    times = (snapshotTime - count):(snapshotTime - 1);
    if count == 0
        selectedHistory = false(sensorCount, sensorCount, 0);
        deliveredHistory = false(sensorCount, sensorCount, 0);
        times = [];
    else
        selectedHistory = selected(:, :, times);
        deliveredHistory = delivered(:, :, times);
    end
    validatePages( ...
        selectedHistory, deliveredHistory, sensorCount, count);
    records{snapshotTime} = struct( ...
        'snapshotTime', snapshotTime, ...
        'times', reshape(times, 1, []), ...
        'selectedDirectedEdgeHistory', selectedHistory, ...
        'deliveredDirectedEdgeHistory', deliveredHistory, ...
        'source', 'behavior-filter-diagnostics');
end
end

function validatePages(selected, delivered, sensorCount, pageCount)
if size(selected, 1) ~= sensorCount || ...
        size(selected, 2) ~= sensorCount || ...
        size(selected, 3) ~= pageCount || ...
        size(delivered, 1) ~= sensorCount || ...
        size(delivered, 2) ~= sensorCount || ...
        size(delivered, 3) ~= pageCount
    error('Behavior history page shape is invalid.');
end
for pageIdx = 1:pageCount
    selectedPage = selected(:, :, pageIdx);
    deliveredPage = delivered(:, :, pageIdx);
    if any(diag(selectedPage)) || any(diag(deliveredPage))
        error('Behavior history pages must not contain self edges.');
    end
    if any(deliveredPage(:) & ~selectedPage(:))
        error('Delivered behavior edges must be a subset of selected edges.');
    end
end
end
