function metrics = computeFormationHandoverBundleOpportunityV85( ...
        context, control, transport, groupIds, options)
% COMPUTEFORMATIONHANDOVERBUNDLEOPPORTUNITYV85 Aggregate safe rows.

if nargin < 5 || isempty(options)
    options = getFormationHandoverBundleV85Protocol();
end
requiredTransport = { ...
    'slotRecords', 'networkReferenceExistenceMass', ...
    'receiverFusionMode', 'receiverFirstInputEnforced', ...
    'sourceFormationChangePrefilterUsed', ...
    'currentNovelSupportPrefilterUsed', ...
    'minimumCurrentNovelSupportFractionPrefilter'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        ~isstruct(control) || ~isscalar(control) || ...
        ~isfield(control, 'referenceScore') || ...
        ~isstruct(transport) || ~isscalar(transport) || ...
        ~all(isfield(transport, requiredTransport)) || ...
        ~strcmp(transport.receiverFusionMode, 'mixture-aware-heavy') || ...
        ~transport.receiverFirstInputEnforced || ...
        ~transport.sourceFormationChangePrefilterUsed || ...
        ~transport.currentNovelSupportPrefilterUsed || ...
        abs(transport.minimumCurrentNovelSupportFractionPrefilter - ...
            options.minimumEdgeNoveltyPrefilterFraction) > 1e-12
    error('FormationHandoverBundleV85:InvalidInput', ...
        'V85 requires aligned mixture-aware V84-style edge records.');
end

posteriors = reshape(context.localPosteriorBySensor, 1, []);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
if numel(posteriors) ~= numel(groupIds)
    error('FormationHandoverBundleV85:DimensionMismatch', ...
        'The V85 posterior and formation identities do not align.');
end
referenceMass = referenceMassByFormation(control, groups, groupIds);
networkMass = transport.networkReferenceExistenceMass;
formationCount = numel(groups);
gainMass = zeros(1, formationCount);
harmMass = zeros(1, formationCount);
netMass = zeros(1, formationCount);
noveltyMass = zeros(1, formationCount);
receiverCount = zeros(1, formationCount);
slotIndices = cell(1, formationCount);
receiverIndices = cell(1, formationCount);

for slotIdx = 1:numel(transport.slotRecords)
    slot = transport.slotRecords(slotIdx);
    if ~qualifyingSlot(slot, options)
        continue;
    end
    receiverIdx = slot.receiverIdx;
    incumbentIdx = slot.incumbentSenderIdx;
    candidateIdx = slot.bestCandidateSenderIdx;
    formationIdx = find(groups == groupIds(receiverIdx), 1);
    novelty = senderNoveltyMass( ...
        posteriors{receiverIdx}, posteriors{incumbentIdx}, ...
        posteriors{candidateIdx}, slot.bestLinkReliability);
    gainMass(formationIdx) = gainMass(formationIdx) + ...
        networkMass * slot.bestTransportGainFraction;
    harmMass(formationIdx) = harmMass(formationIdx) + ...
        networkMass * slot.bestSupportedHarmFraction;
    netMass(formationIdx) = netMass(formationIdx) + ...
        networkMass * slot.bestNetHeadroomFraction;
    noveltyMass(formationIdx) = ...
        noveltyMass(formationIdx) + novelty;
    receiverCount(formationIdx) = receiverCount(formationIdx) + 1;
    slotIndices{formationIdx}(end + 1) = slotIdx;
    receiverIndices{formationIdx}(end + 1) = receiverIdx;
end

localGain = gainMass ./ referenceMass;
localHarm = harmMass ./ referenceMass;
localNet = netMass ./ referenceMass;
localNovelty = noveltyMass ./ referenceMass;
sensorsPerFormation = arrayfun(@(group) ...
    nnz(groupIds == group), groups);
receiverCoverage = receiverCount ./ sensorsPerFormation;
actionMask = receiverCount >= options.minimumChangedReceiverCount & ...
    localNet >= options.minimumBundleLocalNetFraction - 1e-12 & ...
    localNovelty >= ...
        options.minimumBundleSenderNoveltyFraction - 1e-12 & ...
    localHarm <= options.maximumProtectedHarmRatio * ...
        localGain + 1e-12;
scores = zeros(1, formationCount);
for formationIdx = find(actionMask)
    scores(formationIdx) = min([ ...
        localNet(formationIdx) / ...
            options.minimumBundleLocalNetFraction, ...
        localNovelty(formationIdx) / ...
            options.minimumBundleSenderNoveltyFraction, ...
        receiverCount(formationIdx) / ...
            options.minimumChangedReceiverCount]);
end
bestFormationIdx = [];
if any(actionMask)
    candidates = find(actionMask);
    ranking = [-scores(candidates)', -localNet(candidates)', ...
        -receiverCoverage(candidates)', candidates'];
    [~, order] = sortrows(ranking, [1, 2, 3, 4]);
    bestFormationIdx = candidates(order(1));
end

metrics = struct();
metrics.contractVersion = ...
    'formation-handover-bundle-opportunity-v85-v1';
metrics.groups = groups;
metrics.groupIds = groupIds;
metrics.referenceExistenceMassByFormation = referenceMass;
metrics.gainMassByFormation = gainMass;
metrics.harmMassByFormation = harmMass;
metrics.netMassByFormation = netMass;
metrics.senderNoveltyMassByFormation = noveltyMass;
metrics.localGainFractionByFormation = localGain;
metrics.localHarmFractionByFormation = localHarm;
metrics.localNetFractionByFormation = localNet;
metrics.localSenderNoveltyFractionByFormation = localNovelty;
metrics.changedReceiverCountByFormation = receiverCount;
metrics.changedReceiverCoverageByFormation = receiverCoverage;
metrics.slotIndicesByFormation = slotIndices;
metrics.receiverIndicesByFormation = receiverIndices;
metrics.actionMask = actionMask;
metrics.actionFormationIds = groups(actionMask);
metrics.actionScoreByFormation = scores;
metrics.bestFormationIndex = bestFormationIdx;
metrics.bestFormationId = NaN;
metrics.bestSlotIndices = zeros(1, 0);
metrics.bestReceiverIndices = zeros(1, 0);
metrics.bestLocalNetFraction = NaN;
metrics.bestLocalSenderNoveltyFraction = NaN;
metrics.bestChangedReceiverCount = 0;
metrics.bestChangedReceiverCoverage = 0;
metrics.maximumActionScore = 0;
if ~isempty(bestFormationIdx)
    metrics.bestFormationId = groups(bestFormationIdx);
    metrics.bestSlotIndices = slotIndices{bestFormationIdx};
    metrics.bestReceiverIndices = receiverIndices{bestFormationIdx};
    metrics.bestLocalNetFraction = localNet(bestFormationIdx);
    metrics.bestLocalSenderNoveltyFraction = ...
        localNovelty(bestFormationIdx);
    metrics.bestChangedReceiverCount = receiverCount(bestFormationIdx);
    metrics.bestChangedReceiverCoverage = ...
        receiverCoverage(bestFormationIdx);
    metrics.maximumActionScore = scores(bestFormationIdx);
end
metrics.minimumEdgeNoveltyPrefilterFraction = ...
    options.minimumEdgeNoveltyPrefilterFraction;
metrics.minimumBundleLocalNetFraction = ...
    options.minimumBundleLocalNetFraction;
metrics.minimumBundleSenderNoveltyFraction = ...
    options.minimumBundleSenderNoveltyFraction;
metrics.minimumChangedReceiverCount = ...
    options.minimumChangedReceiverCount;
metrics.receiverFusionMode = transport.receiverFusionMode;
metrics.receiverFirstInputEnforced = ...
    transport.receiverFirstInputEnforced;
metrics.routeConnectivityProjected = false;
metrics.messageBudgetProjected = false;
metrics.routeExecuted = false;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureOutcomesUsed = false;
end

function value = qualifyingSlot(slot, options)
required = { ...
    'bestSafe', 'bestNetHeadroomFraction', ...
    'bestTransportGainFraction', 'bestSupportedHarmFraction', ...
    'bestDownwardCrossingCount', 'bestCandidateSenderIdx', ...
    'bestLinkReliability'};
value = all(isfield(slot, required)) && slot.bestSafe && ...
    isfinite(slot.bestNetHeadroomFraction) && ...
    slot.bestNetHeadroomFraction > 0 && ...
    isfinite(slot.bestTransportGainFraction) && ...
    isfinite(slot.bestSupportedHarmFraction) && ...
    slot.bestSupportedHarmFraction <= ...
        options.maximumProtectedHarmRatio * ...
        slot.bestTransportGainFraction + 1e-12 && ...
    (~options.requireZeroSupportedDownwardCrossings || ...
        slot.bestDownwardCrossingCount == 0);
end

function mass = senderNoveltyMass(receiver, incumbent, candidate, reliability)
labels = collectLabels({receiver, incumbent, candidate});
baseline = max( ...
    supportByLabel(receiver, labels), ...
    supportByLabel(incumbent, labels));
candidateSupport = supportByLabel(candidate, labels);
candidateExistence = existenceByLabel(candidate, labels);
mass = reliability * sum(max(candidateSupport - baseline, 0) .* ...
    candidateExistence);
end

function mass = referenceMassByFormation(control, groups, groupIds)
details = control.referenceScore.retentionDetails;
values = details.expectedReferenceExistenceByReceiver;
if ~iscell(values) || numel(values) ~= numel(groupIds)
    error('FormationHandoverBundleV85:MissingReferenceMass', ...
        'Receiver reference existence is unavailable.');
end
mass = zeros(1, numel(groups));
for receiverIdx = 1:numel(groupIds)
    formationIdx = find(groups == groupIds(receiverIdx), 1);
    current = reshape(values{receiverIdx}, 1, []);
    if any(~isfinite(current)) || any(current < -1e-12)
        error('FormationHandoverBundleV85:InvalidReferenceMass', ...
            'Receiver reference existence is invalid.');
    end
    mass(formationIdx) = mass(formationIdx) + sum(max(current, 0));
end
if any(mass <= eps)
    error('FormationHandoverBundleV85:InvalidReferenceMass', ...
        'A receiver formation has no reference existence mass.');
end
end

function labels = collectLabels(posteriors)
labels = zeros(2, 0);
for posteriorIdx = 1:numel(posteriors)
    objects = posteriors{posteriorIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents <= 0
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ...
                ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
end

function values = existenceByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    objectIdx = findObject(objects, labels(:, labelIdx));
    if ~isempty(objectIdx)
        values(labelIdx) = clamp01(objects(objectIdx).r);
    end
end
end

function values = supportByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    objectIdx = findObject(objects, labels(:, labelIdx));
    if ~isempty(objectIdx)
        values(labelIdx) = clamp01(getField( ...
            objects(objectIdx), 'detectionAssociationMass', 0));
    end
end
end

function idx = findObject(objects, label)
idx = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
