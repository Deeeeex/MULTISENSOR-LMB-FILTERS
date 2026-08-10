function metrics = computeRepeatedMultiGatewayHandoverNominationsV89( ...
        context, groupIds, options)
% COMPUTEREPEATEDMULTIGATEWAYHANDOVERNOMINATIONSV89 Fast current handovers.

if nargin < 3 || isempty(options)
    options = getRepeatedMultiGatewayHandoverV89Protocol();
end
required = {'localPosteriorBySensor', 'physicalAdjacency', ...
    'model', 'commConfig', 'currentTime'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required))
    error('RepeatedMultiGatewayV89:InvalidContext', ...
        'V89 requires one current observable topology state.');
end
posteriors = reshape(context.localPosteriorBySensor, 1, []);
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(posteriors);
physical = logical(context.physicalAdjacency);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(physical), [nodeCount, nodeCount])
    error('RepeatedMultiGatewayV89:DimensionMismatch', ...
        'V89 posterior, formation and physical graph dimensions drifted.');
end

[referenceAdjacency, reference] = ...
    selectPhysicalFormationTreeResidualTourPolicy(context, struct( ...
        'dominantWeight', options.dominantWeight, ...
        'residualWeight', options.sourceWeight));
referenceWeights = reference.fusionWeightMatrix;
formationMass = localExistenceMassByFormation(posteriors, groupIds);
edgeRecords = repmat(emptyEdgeRecord(), 1, 0);
for receiverIdx = 1:nodeCount
    incumbent = find(reference.residualAdjacency(receiverIdx, :) & ...
        groupIds ~= groupIds(receiverIdx));
    if numel(incumbent) ~= 1
        continue;
    end
    incumbentIdx = incumbent(1);
    alternatives = find(physical(receiverIdx, :) & ...
        groupIds ~= groupIds(receiverIdx) & ...
        groupIds ~= groupIds(incumbentIdx) & ...
        ~referenceAdjacency(receiverIdx, :));
    for candidateIdx = reshape(alternatives, 1, [])
        reliability = linkReliability( ...
            context.commConfig, candidateIdx, receiverIdx, ...
            context.currentTime);
        [novelMass, protectedDeficitMass, maximumNovelSupport] = ...
            supportTransferMass( ...
                posteriors{receiverIdx}, ...
                posteriors{incumbentIdx}, ...
                posteriors{candidateIdx}, reliability);
        denominator = formationMass(groupIds(receiverIdx));
        novelty = novelMass / denominator;
        protectedDeficit = protectedDeficitMass / denominator;
        members = find(groupIds == groupIds(receiverIdx));
        broadcastReceivers = setdiff(members, receiverIdx, 'stable');
        broadcastPhysical = all(physical(broadcastReceivers, receiverIdx));
        enabled = novelty >= ...
                options.minimumSenderNoveltyFraction - 1e-12 && ...
            maximumNovelSupport >= ...
                options.minimumCandidateAssociationSupport - 1e-12 && ...
            (~options.requireFullFormationBroadcastPhysicality || ...
                broadcastPhysical);
        edgeRecords(end + 1) = struct( ... %#ok<AGROW>
            'receiverIdx', receiverIdx, ...
            'receiverFormationId', groupIds(receiverIdx), ...
            'incumbentSenderIdx', incumbentIdx, ...
            'incumbentFormationId', groupIds(incumbentIdx), ...
            'candidateSenderIdx', candidateIdx, ...
            'candidateFormationId', groupIds(candidateIdx), ...
            'linkReliability', reliability, ...
            'senderNoveltyFraction', novelty, ...
            'protectedSupportDeficitFraction', protectedDeficit, ...
            'maximumNovelAssociationSupport', maximumNovelSupport, ...
            'broadcastReceiverIndices', broadcastReceivers, ...
            'fullFormationBroadcastPhysical', broadcastPhysical, ...
            'actionEnabled', enabled, ...
            'rankingScore', novelty);
    end
end

selected = selectFormationDistinctGateways(edgeRecords, options);
selectedFormationIds = [selected.receiverFormationId];
selectedGatewayIndices = [selected.receiverIdx];
metrics = struct();
metrics.contractVersion = ...
    'repeated-multi-gateway-handover-nominations-v89-v1';
metrics.currentTime = context.currentTime;
metrics.referenceAdjacency = logical(referenceAdjacency);
metrics.referenceFusionWeights = referenceWeights;
metrics.referenceMessageCount = nnz(referenceAdjacency);
metrics.groupIds = groupIds;
metrics.edgeRecords = edgeRecords;
metrics.selectedGateways = selected;
metrics.selectedGatewayIndices = selectedGatewayIndices;
metrics.selectedFormationIds = selectedFormationIds;
metrics.selectedGatewayCount = numel(selected);
metrics.selectedFormationCoverageFraction = ...
    numel(selectedFormationIds) / numel(unique(groupIds, 'stable'));
metrics.aggregateSenderNoveltyFraction = sum( ...
    [selected.senderNoveltyFraction]);
metrics.aggregateProtectedSupportDeficitFraction = sum( ...
    [selected.protectedSupportDeficitFraction]);
metrics.routeExecuted = false;
metrics.klaOutcomeRead = false;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureOutcomesUsed = false;
metrics.trackingOutcomeRead = false;
end

function selected = selectFormationDistinctGateways(records, options)
selected = repmat(emptyEdgeRecord(), 1, 0);
eligible = find([records.actionEnabled]);
if isempty(eligible)
    return;
end
scores = [records(eligible).rankingScore];
novel = [records(eligible).maximumNovelAssociationSupport];
ranking = [-scores(:), -novel(:), eligible(:)];
[~, order] = sortrows(ranking, [1, 2, 3]);
for idx = reshape(eligible(order), 1, [])
    candidate = records(idx);
    if any([selected.receiverFormationId] == ...
            candidate.receiverFormationId)
        continue;
    end
    selected(end + 1) = candidate; %#ok<AGROW>
    if numel(selected) >= options.maximumGatewayCount
        break;
    end
end
end

function mass = localExistenceMassByFormation(posteriors, groupIds)
formationCount = max(groupIds);
mass = zeros(1, formationCount);
for sensorIdx = 1:numel(posteriors)
    objects = posteriors{sensorIdx};
    if isempty(objects)
        continue;
    end
    mass(groupIds(sensorIdx)) = mass(groupIds(sensorIdx)) + ...
        sum(max(reshape([objects.r], 1, []), 0));
end
if any(mass(unique(groupIds)) <= eps)
    error('RepeatedMultiGatewayV89:InvalidFormationMass', ...
        'A V89 receiver formation has no current existence mass.');
end
end

function [novelMass, deficitMass, maximumNovelSupport] = ...
        supportTransferMass(receiver, incumbent, candidate, reliability)
labels = collectLabels({receiver, incumbent, candidate});
receiverSupport = supportByLabel(receiver, labels);
incumbentSupport = supportByLabel(incumbent, labels);
candidateSupport = supportByLabel(candidate, labels);
incumbentExistence = existenceByLabel(incumbent, labels);
candidateExistence = existenceByLabel(candidate, labels);
baseline = max(receiverSupport, incumbentSupport);
candidateBaseline = max(receiverSupport, candidateSupport);
novelSupport = max(candidateSupport - baseline, 0);
protectedDeficit = max(incumbentSupport - candidateBaseline, 0);
novelMass = reliability * sum(novelSupport .* candidateExistence);
deficitMass = reliability * sum( ...
    protectedDeficit .* incumbentExistence);
maximumNovelSupport = finiteMax( ...
    candidateSupport(novelSupport > 0));
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
    object = objectByLabel(objects, labels(:, labelIdx));
    if ~isempty(object)
        values(labelIdx) = clamp01(object.r);
    end
end
end

function values = supportByLabel(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    object = objectByLabel(objects, labels(:, labelIdx));
    if ~isempty(object)
        values(labelIdx) = clamp01(getField( ...
            object, 'detectionAssociationMass', 0));
    end
end
end

function object = objectByLabel(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function reliability = linkReliability(config, sender, receiver, time)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(time, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(sender, receiver, timeIdx);
    else
        drop = config.pDropByEdge(sender, receiver);
    end
else
    drop = getField(config, 'defaultDropProbability', 0);
end
reliability = 1 - clamp01(drop);
end

function value = finiteMax(values)
values = reshape(values, 1, []);
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = max(values);
end
end

function record = emptyEdgeRecord()
record = struct( ...
    'receiverIdx', NaN, ...
    'receiverFormationId', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'incumbentFormationId', NaN, ...
    'candidateSenderIdx', NaN, ...
    'candidateFormationId', NaN, ...
    'linkReliability', NaN, ...
    'senderNoveltyFraction', NaN, ...
    'protectedSupportDeficitFraction', NaN, ...
    'maximumNovelAssociationSupport', NaN, ...
    'broadcastReceiverIndices', zeros(1, 0), ...
    'fullFormationBroadcastPhysical', false, ...
    'actionEnabled', false, ...
    'rankingScore', 0);
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
