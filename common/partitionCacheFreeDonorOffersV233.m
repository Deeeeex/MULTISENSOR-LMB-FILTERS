function partition = partitionCacheFreeDonorOffersV233( ...
        sourcePosterior, model, plan, sourceId)
% PARTITIONCACHEFREEDONOROFFERSV233 Assign active labels to coverage phases.

protocol = getCacheFreeDonorOfferCoverageV233Protocol();
if ~isstruct(sourcePosterior) || ~isstruct(model) || ...
        ~isscalar(model) || ~isstruct(plan) || ~isscalar(plan) || ...
        ~isfield(plan, 'protocolId') || ...
        ~strcmp(plan.protocolId, protocol.id) || ...
        ~isscalar(sourceId) || ~isfinite(sourceId) || ...
        sourceId ~= round(sourceId) || sourceId < 1
    error('CacheFreeDonorOfferV233:InvalidPartitionInput', ...
        'The source-offer partition request is malformed.');
end

active = [sourcePosterior.numberOfGmComponents] > 0 & ...
    [sourcePosterior.r] >= protocol.coverage.minimumActiveExistence;
objects = sourcePosterior(active);
labels = zeros(2, numel(objects));
payloadBytes = zeros(1, numel(objects));
for objectIndex = 1:numel(objects)
    labels(:, objectIndex) = [ ...
        objects(objectIndex).birthTime; ...
        objects(objectIndex).birthLocation];
    stats = estimateLmbPayloadSize( ...
        objects(objectIndex), model, 2, struct());
    payloadBytes(objectIndex) = stats.estimatedBytes;
end
if ~isempty(labels)
    [~, order] = sortrows(labels', [1, 2]);
    labels = labels(:, order);
    payloadBytes = payloadBytes(order);
end

capacity = plan.coverageCapacityPerSource;
retainedCount = min(size(labels, 2), capacity);
retainedLabels = labels(:, 1:retainedCount);
retainedPayloadBytes = payloadBytes(1:retainedCount);
phaseLabels = cell(1, plan.phaseCount);
phasePayloadBytes = cell(1, plan.phaseCount);
for labelIndex = 1:retainedCount
    phase = mod(labelIndex - 1, plan.phaseCount) + 1;
    phaseLabels{phase}(:, end + 1) = ...
        retainedLabels(:, labelIndex); %#ok<AGROW>
    phasePayloadBytes{phase}(end + 1) = ...
        retainedPayloadBytes(labelIndex); %#ok<AGROW>
end
if any(cellfun(@(x) size(x, 2), phaseLabels) > ...
        plan.offersPerSourcePerPhase)
    error('CacheFreeDonorOfferV233:PhaseOverflow', ...
        'The deterministic coverage schedule exceeded its phase cap.');
end

partition = struct();
partition.contractVersion = ...
    'cache-free-donor-offer-partition-v233-v1';
partition.protocolId = protocol.id;
partition.sourceId = sourceId;
partition.activeLabelCount = size(labels, 2);
partition.retainedLabelCount = retainedCount;
partition.droppedLabelCount = size(labels, 2) - retainedCount;
partition.coverageComplete = partition.droppedLabelCount == 0;
partition.retainedLabels = retainedLabels;
partition.phaseLabels = phaseLabels;
partition.phasePayloadBytes = phasePayloadBytes;
partition.numericLabelIdentifiersUsedOnlyForCoverageSchedule = true;
partition.numericLabelIdentifiersUsedAsValueFeatures = false;
partition.truthUsed = false;
partition.futureInformationUsed = false;
partition.validationClaimAllowed = false;
partition.evidenceBoundary = protocol.evidenceBoundary;
end
