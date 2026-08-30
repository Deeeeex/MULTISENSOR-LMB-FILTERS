function [posterior, details] = selectObservableOneHopRiskLabels( ...
        currentPosterior, localPosteriorBySensor, physicalAdjacency, ...
        directedLinkDelivered, config, receiverIdx, currentTime, model)
% SELECTOBSERVABLEONEHOPRISKLABELS Request bounded complete labels by risk.
%
% Every physical neighbor advertises its active label keys and scalar
% posterior Bayes risks.  The receiver chooses the minimum-risk source for
% each label, ranks labels by receiver-risk minus source-risk, and requests
% at most K positive-risk-reduction complete Bernoulli GM densities.  The
% routine reads no truth, future measurements, or alternative-arm state.

sensorCount = numel(localPosteriorBySensor);
posterior = reshape(currentPosterior, 1, []);
details = emptyDetails(sensorCount);
if ~config.observableOneHopRiskLabelEnabled
    return;
end

pageIdx = find(config.observableOneHopRiskLabelTimes == currentTime, 1);
if isempty(pageIdx) || ~ismember(receiverIdx, ...
        config.observableOneHopRiskLabelSensorIdsByTime{pageIdx})
    return;
end
if ~isequal(size(physicalAdjacency), [sensorCount, sensorCount]) || ...
        ~isequal(size(directedLinkDelivered), [sensorCount, sensorCount])
    error('ObservableOneHopRiskLabel:InvalidLinkState', ...
        'Physical adjacency and directed delivery state must be S-by-S.');
end

details.triggered = true;
details.pageIndex = pageIdx;
neighbors = reshape(find(logical(physicalAdjacency(receiverIdx, :))), 1, []);
neighbors = neighbors(neighbors ~= receiverIdx);
details.physicalNeighborCount = numel(neighbors);
activeThreshold = config.observableOneHopRiskLabelExistenceThreshold;

advertisedBySource = cell(1, sensorCount);
for sourceIdx = neighbors
    active = selectActiveLabels( ...
        localPosteriorBySensor{sourceIdx}, activeThreshold);
    if isempty(active)
        continue;
    end
    bytes = config.observableOneHopRiskLabelSynopsisHeaderBytes + ...
        config.observableOneHopRiskLabelSynopsisBytesPerLabel * ...
        numel(active);
    details.attemptedSynopsisBytesBySource(sourceIdx) = bytes;
    details.attemptedSynopsisMessageBySource(sourceIdx) = true;
    if directedLinkDelivered(sourceIdx, receiverIdx)
        advertisedBySource{sourceIdx} = active;
        details.deliveredSynopsisBytesBySource(sourceIdx) = bytes;
        details.deliveredSynopsisMessageBySource(sourceIdx) = true;
    end
end

candidates = buildCandidates( ...
    posterior, advertisedBySource, neighbors, receiverIdx, ...
    currentTime, model);
details.candidateLabelCount = numel(candidates);
if isempty(candidates)
    finalizeByteDetails();
    return;
end

score = [candidates.riskReduction]';
labelRows = reshape([candidates.label], 2, [])';
sourceIds = [candidates.source]';
ranking = [-score, labelRows, sourceIds];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
positive = order(score(order) > ...
    config.observableOneHopRiskLabelMinimumRiskReduction + 1e-12);
selectedCount = min( ...
    config.observableOneHopRiskLabelMaximumEdits, numel(positive));
selected = candidates(positive(1:selectedCount));
details.selectedLabelCount = numel(selected);
if isempty(selected)
    finalizeByteDetails();
    return;
end
details.selectedLabels = reshape([selected.label], 2, []);
details.selectedSources = reshape([selected.source], 1, []);
details.selectedRiskReduction = reshape( ...
    [selected.riskReduction], 1, []);

sources = unique(details.selectedSources, 'stable');
for sourceIdx = sources
    selectedMask = details.selectedSources == sourceIdx;
    requested = selected(selectedMask);
    requestBytes = config.observableOneHopRiskLabelRequestHeaderBytes + ...
        config.observableOneHopRiskLabelRequestBytesPerLabel * ...
        numel(requested);
    details.attemptedRequestBytesBySource(sourceIdx) = requestBytes;
    details.attemptedRequestMessageBySource(sourceIdx) = true;
    if ~directedLinkDelivered(receiverIdx, sourceIdx)
        continue;
    end
    details.deliveredRequestBytesBySource(sourceIdx) = requestBytes;
    details.deliveredRequestMessageBySource(sourceIdx) = true;

    responseObjects = reshape([requested.sourceObject], 1, []);
    responseStats = estimateLmbPayloadSize( ...
        responseObjects, model, 2, struct());
    if responseStats.objectCount ~= numel(responseObjects)
        error('ObservableOneHopRiskLabel:PayloadThresholdDrift', ...
            'A selected complete label was omitted from its response.');
    end
    details.attemptedResponseBytesBySource(sourceIdx) = ...
        responseStats.estimatedBytes;
    details.attemptedResponseMessageBySource(sourceIdx) = true;
    if ~directedLinkDelivered(sourceIdx, receiverIdx)
        continue;
    end
    details.deliveredResponseBytesBySource(sourceIdx) = ...
        responseStats.estimatedBytes;
    details.deliveredResponseMessageBySource(sourceIdx) = true;
    for objectIdx = 1:numel(responseObjects)
        posterior = replaceLabelObject(posterior, ...
            responseObjects(objectIdx));
        details.appliedLabels(:, end + 1) = [ ...
            responseObjects(objectIdx).birthTime; ...
            responseObjects(objectIdx).birthLocation]; %#ok<AGROW>
        details.appliedSources(end + 1) = sourceIdx; %#ok<AGROW>
    end
end
details.appliedLabelCount = size(details.appliedLabels, 2);
details.applied = details.appliedLabelCount > 0;
finalizeByteDetails();

    function finalizeByteDetails()
        details.attemptedSynopsisBytes = sum( ...
            details.attemptedSynopsisBytesBySource);
        details.deliveredSynopsisBytes = sum( ...
            details.deliveredSynopsisBytesBySource);
        details.attemptedRequestBytes = sum( ...
            details.attemptedRequestBytesBySource);
        details.deliveredRequestBytes = sum( ...
            details.deliveredRequestBytesBySource);
        details.attemptedResponseBytes = sum( ...
            details.attemptedResponseBytesBySource);
        details.deliveredResponseBytes = sum( ...
            details.deliveredResponseBytesBySource);
        details.attemptedTotalBytes = details.attemptedSynopsisBytes + ...
            details.attemptedRequestBytes + details.attemptedResponseBytes;
        details.deliveredTotalBytes = details.deliveredSynopsisBytes + ...
            details.deliveredRequestBytes + details.deliveredResponseBytes;
    end
end

function candidates = buildCandidates( ...
        receiverPosterior, advertisedBySource, neighbors, receiverIdx, ...
        currentTime, model)
labels = zeros(2, 0);
for sourceIdx = neighbors
    objects = advertisedBySource{sourceIdx};
    for objectIdx = 1:numel(objects)
        label = objectLabel(objects(objectIdx));
        if isempty(labels) || ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end

candidates = repmat(emptyCandidate(), 1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    best = emptyCandidate();
    for sourceIdx = neighbors
        object = findLabelObject(advertisedBySource{sourceIdx}, label);
        if isempty(object)
            continue;
        end
        sourceRisk = computeObservableLmbLabelBayesRisk(object, model);
        if isempty(best.sourceObject) || ...
                sourceRisk < best.sourceRisk - 1e-12 || ...
                (abs(sourceRisk - best.sourceRisk) <= 1e-12 && ...
                 sourceIdx < best.source)
            best.label = label;
            best.source = sourceIdx;
            best.sourceObject = object;
            best.sourceRisk = sourceRisk;
        end
    end
    if isempty(best.sourceObject)
        continue;
    end
    receiverObject = findLabelObject(receiverPosterior, label);
    receiverRisk = computeObservableLmbLabelBayesRisk( ...
        receiverObject, model);
    best.receiverRisk = receiverRisk;
    best.riskReduction = receiverRisk - best.sourceRisk;
    best.receiver = receiverIdx;
    best.currentTime = currentTime;
    candidates(end + 1) = best; %#ok<AGROW>
end
end

function objects = selectActiveLabels(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
active = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
objects = objects(active);
end

function posterior = replaceLabelObject(posterior, object)
idx = findLabelIndex(posterior, objectLabel(object));
if idx == 0
    posterior(end + 1) = object; %#ok<AGROW>
else
    posterior(idx) = object;
end
end

function object = findLabelObject(objects, label)
idx = findLabelIndex(objects, label);
if idx == 0
    object = [];
else
    object = objects(idx);
end
end

function idx = findLabelIndex(objects, label)
idx = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function label = objectLabel(object)
label = [object.birthTime; object.birthLocation];
end

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), ...
    'receiver', 0, ...
    'source', 0, ...
    'currentTime', 0, ...
    'sourceObject', [], ...
    'sourceRisk', inf, ...
    'receiverRisk', inf, ...
    'riskReduction', -inf);
end

function details = emptyDetails(sensorCount)
details = struct( ...
    'contractVersion', 'observable-one-hop-risk-label-transfer-v162-v1', ...
    'sourceRule', 'minimum-current-label-posterior-bayes-risk', ...
    'labelRule', 'positive-receiver-minus-source-risk-top-k', ...
    'triggered', false, ...
    'applied', false, ...
    'pageIndex', 0, ...
    'physicalNeighborCount', 0, ...
    'candidateLabelCount', 0, ...
    'selectedLabelCount', 0, ...
    'appliedLabelCount', 0, ...
    'selectedLabels', zeros(2, 0), ...
    'selectedSources', zeros(1, 0), ...
    'selectedRiskReduction', zeros(1, 0), ...
    'appliedLabels', zeros(2, 0), ...
    'appliedSources', zeros(1, 0), ...
    'attemptedSynopsisMessageBySource', false(1, sensorCount), ...
    'deliveredSynopsisMessageBySource', false(1, sensorCount), ...
    'attemptedRequestMessageBySource', false(1, sensorCount), ...
    'deliveredRequestMessageBySource', false(1, sensorCount), ...
    'attemptedResponseMessageBySource', false(1, sensorCount), ...
    'deliveredResponseMessageBySource', false(1, sensorCount), ...
    'attemptedSynopsisBytesBySource', zeros(1, sensorCount), ...
    'deliveredSynopsisBytesBySource', zeros(1, sensorCount), ...
    'attemptedRequestBytesBySource', zeros(1, sensorCount), ...
    'deliveredRequestBytesBySource', zeros(1, sensorCount), ...
    'attemptedResponseBytesBySource', zeros(1, sensorCount), ...
    'deliveredResponseBytesBySource', zeros(1, sensorCount), ...
    'attemptedSynopsisBytes', 0, ...
    'deliveredSynopsisBytes', 0, ...
    'attemptedRequestBytes', 0, ...
    'deliveredRequestBytes', 0, ...
    'attemptedResponseBytes', 0, ...
    'deliveredResponseBytes', 0, ...
    'attemptedTotalBytes', 0, ...
    'deliveredTotalBytes', 0, ...
    'truthUsed', false, ...
    'futureInformationUsed', false);
end
