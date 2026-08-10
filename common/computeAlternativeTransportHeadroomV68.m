function metrics = computeAlternativeTransportHeadroomV68( ...
        context, control, groupIds, options)
% COMPUTEALTERNATIVETRANSPORTHEADROOMV68 Raw fixed-budget edge headroom.
%
% For every registered cross-formation residual input, replace its sender
% with one currently physical but unused cross-formation sender. The same
% row weight and message count are retained. Reference and candidate receiver
% posteriors use the historical moment-matched KLA unless
% options.fusionConfig explicitly enables the mixture-aware heavy-fusion
% path. In that path the receiver is input first and selected neighbors are
% marked as delivered heavy messages.
% The sum of the best safe replacement per residual slot is an optimistic
% pre-projection bound, not an executable route.

if nargin < 4 || isempty(options)
    options = getAlternativeTransportHeadroomV68Protocol();
end
requiredControl = {'referenceFusionWeights', 'referenceScore'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        ~isfield(context, 'physicalAdjacency') || ...
        ~isfield(context, 'model') || ...
        ~isstruct(control) || ~isscalar(control) || ...
        ~all(isfield(control, requiredControl))
    error('AlternativeTransportV68:InvalidInput', ...
        'V68 requires one observable state and its reference control.');
end
posteriors = reshape(context.localPosteriorBySensor, 1, []);
nodeCount = numel(posteriors);
groupIds = reshape(groupIds, 1, []);
weights = control.referenceFusionWeights;
physical = logical(context.physicalAdjacency);
sourceWeight = getField(options, 'sourceWeight', 0.05);
supportThreshold = getField( ...
    options, 'positiveSupportThreshold', 0.20);
decisionThreshold = getField( ...
    options, 'decisionExistenceThreshold', 0.50);
fusionConfig = getField(options, 'fusionConfig', struct());
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(weights), [nodeCount, nodeCount]) || ...
        ~isequal(size(physical), [nodeCount, nodeCount]) || ...
        any(abs(sum(weights, 2) - 1) > 1e-10) || ...
        ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1 || ...
        ~isstruct(fusionConfig) || ~isscalar(fusionConfig)
    error('AlternativeTransportV68:DimensionMismatch', ...
        'V68 graph dimensions or weights are invalid.');
end

referenceMass = sum(reshape(control.referenceScore.retentionDetails. ...
    expectedReferenceCardinality, 1, []));
if ~isscalar(referenceMass) || ~isfinite(referenceMass) || ...
        referenceMass <= eps
    error('AlternativeTransportV68:InvalidReferenceMass', ...
        'The common network reference mass is unavailable.');
end

edgeRecords = repmat(emptyEdgeRecord(), 1, 0);
slotRecords = repmat(emptySlotRecord(), 1, 0);
for receiverIdx = 1:nodeCount
    activeSenders = find(weights(receiverIdx, :) > 1e-12);
    incumbentCrossSenders = activeSenders( ...
        groupIds(activeSenders) ~= groupIds(receiverIdx));
    if isempty(incumbentCrossSenders)
        continue;
    end
    referenceFused = fuseReceiver( ...
        posteriors, activeSenders, weights(receiverIdx, activeSenders), ...
        receiverIdx, context.model, fusionConfig);
    for incumbentIdx = 1:numel(incumbentCrossSenders)
        incumbentSender = incumbentCrossSenders(incumbentIdx);
        slotWeight = weights(receiverIdx, incumbentSender);
        if abs(slotWeight - sourceWeight) > 1e-10
            continue;
        end
        alternatives = find(physical(receiverIdx, :) & ...
            groupIds ~= groupIds(receiverIdx) & ...
            weights(receiverIdx, :) <= 1e-12);
        bestRecord = emptyEdgeRecord();
        for senderIdx = reshape(alternatives, 1, [])
            candidateSenders = activeSenders;
            candidateSenders(candidateSenders == incumbentSender) = ...
                senderIdx;
            candidateWeights = weights(receiverIdx, activeSenders);
            candidateFused = fuseReceiver( ...
                posteriors, candidateSenders, candidateWeights, ...
                receiverIdx, context.model, fusionConfig);
            record = compareCandidate( ...
                referenceFused, candidateFused, ...
                posteriors{receiverIdx}, posteriors{incumbentSender}, ...
                posteriors{senderIdx}, receiverIdx, incumbentSender, ...
                senderIdx, slotWeight, referenceMass, ...
                linkReliability(context.commConfig, senderIdx, ...
                    receiverIdx, context.currentTime), ...
                supportThreshold, decisionThreshold);
            edgeRecords(end + 1) = record; %#ok<AGROW>
            if record.safe && (...
                    isnan(bestRecord.netHeadroomFraction) || ...
                    record.netHeadroomFraction > ...
                        bestRecord.netHeadroomFraction)
                bestRecord = record;
            end
        end
        slotRecords(end + 1) = makeSlotRecord( ...
            receiverIdx, incumbentSender, slotWeight, bestRecord); %#ok<AGROW>
    end
end

safeSelected = [slotRecords.bestSafe];
positive = safeSelected & ...
    [slotRecords.bestNetHeadroomFraction] > 0;
optimisticTransport = sumFinite( ...
    [slotRecords(positive).bestTransportGainFraction]);
optimisticHarm = sumFinite( ...
    [slotRecords(positive).bestSupportedHarmFraction]);
optimisticNet = sumFinite( ...
    [slotRecords(positive).bestNetHeadroomFraction]);

metrics = struct();
metrics.contractVersion = ...
    'alternative-transport-headroom-v68-v1';
metrics.nodeCount = nodeCount;
metrics.groupIds = groupIds;
metrics.sourceWeight = sourceWeight;
metrics.networkReferenceExistenceMass = referenceMass;
metrics.edgeRecords = edgeRecords;
metrics.slotRecords = slotRecords;
metrics.residualSlotCount = numel(slotRecords);
metrics.safePositiveSlotCount = nnz(positive);
metrics.optimisticTransportGainFraction = optimisticTransport;
metrics.optimisticSupportedHarmFraction = optimisticHarm;
metrics.optimisticNetHeadroomFraction = optimisticNet;
metrics.routeConnectivityProjected = false;
metrics.messageCountChanged = false;
metrics.receiverFusionMode = receiverFusionMode(fusionConfig);
metrics.receiverFirstInputEnforced = ...
    strcmp(metrics.receiverFusionMode, 'mixture-aware-heavy');
metrics.neighborMessageEventType = ...
    2 * double(metrics.receiverFirstInputEnforced);
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureOutcomesUsed = false;
end

function fused = fuseReceiver( ...
        posteriors, senders, weights, receiverIdx, model, fusionConfig)
mixtureAware = strcmp( ...
    receiverFusionMode(fusionConfig), 'mixture-aware-heavy');
if mixtureAware
    receiverPosition = find(senders == receiverIdx, 1);
    if isempty(receiverPosition)
        error('AlternativeTransportV68:MissingReceiverInput', ...
            'Mixture-aware source scoring requires the receiver input.');
    end
    order = [receiverPosition, ...
        setdiff(1:numel(senders), receiverPosition, 'stable')];
    senders = senders(order);
    weights = weights(order);
end
sources = cell(1, numel(senders));
for sourceIdx = 1:numel(senders)
    sources{sourceIdx} = posteriors{senders(sourceIdx)};
end
if mixtureAware
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones(1, numel(senders) - 1)]);
    fused = fuseLmbPosteriorsByLabel( ...
        sources, weights, model, weights, fusionDetails, fusionConfig);
else
    fused = fuseLmbPosteriorsByLabel( ...
        sources, weights, model, weights);
end
end

function mode = receiverFusionMode(config)
if isstruct(config) && isscalar(config) && ...
        logical(getField(config, ...
            'mixtureAwareHeavyFusionEnabled', false))
    mode = 'mixture-aware-heavy';
else
    mode = 'moment-matched';
end
end

function record = compareCandidate( ...
        reference, candidate, receiverLocal, incumbentLocal, ...
        senderLocal, receiverIdx, incumbentSender, senderIdx, ...
        sourceWeight, referenceMass, reliability, supportThreshold, ...
        decisionThreshold)
labels = collectLabels(reference, candidate);
referenceExistence = existenceByLabel(reference, labels);
candidateExistence = existenceByLabel(candidate, labels);
receiverSupport = supportByLabel(receiverLocal, labels);
incumbentSupport = supportByLabel(incumbentLocal, labels);
senderSupport = supportByLabel(senderLocal, labels);
protectedSupport = max(receiverSupport, incumbentSupport);
transportGain = reliability * sum( ...
    max(candidateExistence - referenceExistence, 0) .* senderSupport);
supportedHarm = reliability * sum( ...
    max(referenceExistence - candidateExistence, 0) .* ...
        protectedSupport);
downward = protectedSupport >= supportThreshold - 1e-12 & ...
    referenceExistence >= decisionThreshold & ...
    candidateExistence < decisionThreshold;
upward = senderSupport >= supportThreshold - 1e-12 & ...
    referenceExistence < decisionThreshold & ...
    candidateExistence >= decisionThreshold;
record = emptyEdgeRecord();
record.receiverIdx = receiverIdx;
record.incumbentSenderIdx = incumbentSender;
record.candidateSenderIdx = senderIdx;
record.sourceWeight = sourceWeight;
record.linkReliability = reliability;
record.transportGainFraction = transportGain / referenceMass;
record.supportedHarmFraction = supportedHarm / referenceMass;
record.netHeadroomFraction = ...
    (transportGain - supportedHarm) / referenceMass;
record.upwardCrossingCount = nnz(upward);
record.downwardCrossingCount = nnz(downward);
record.safe = nnz(downward) == 0 && ...
    supportedHarm <= transportGain + 1e-12;
end

function record = makeSlotRecord( ...
        receiverIdx, incumbentSender, sourceWeight, best)
record = emptySlotRecord();
record.receiverIdx = receiverIdx;
record.incumbentSenderIdx = incumbentSender;
record.sourceWeight = sourceWeight;
record.bestSafe = isfinite(best.netHeadroomFraction);
if record.bestSafe
    record.bestCandidateSenderIdx = best.candidateSenderIdx;
    record.bestLinkReliability = best.linkReliability;
    record.bestTransportGainFraction = best.transportGainFraction;
    record.bestSupportedHarmFraction = best.supportedHarmFraction;
    record.bestNetHeadroomFraction = best.netHeadroomFraction;
    record.bestUpwardCrossingCount = best.upwardCrossingCount;
    record.bestDownwardCrossingCount = best.downwardCrossingCount;
end
end

function record = emptyEdgeRecord()
record = struct( ...
    'receiverIdx', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'candidateSenderIdx', NaN, ...
    'sourceWeight', NaN, ...
    'linkReliability', NaN, ...
    'transportGainFraction', NaN, ...
    'supportedHarmFraction', NaN, ...
    'netHeadroomFraction', NaN, ...
    'upwardCrossingCount', NaN, ...
    'downwardCrossingCount', NaN, ...
    'safe', false);
end

function record = emptySlotRecord()
record = struct( ...
    'receiverIdx', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'sourceWeight', NaN, ...
    'bestSafe', false, ...
    'bestCandidateSenderIdx', NaN, ...
    'bestLinkReliability', NaN, ...
    'bestTransportGainFraction', NaN, ...
    'bestSupportedHarmFraction', NaN, ...
    'bestNetHeadroomFraction', NaN, ...
    'bestUpwardCrossingCount', NaN, ...
    'bestDownwardCrossingCount', NaN);
end

function labels = collectLabels(first, second)
labels = zeros(2, 0);
for objects = {first, second}
    current = objects{1};
    for objectIdx = 1:numel(current)
        if current(objectIdx).numberOfGmComponents <= 0
            continue;
        end
        label = [current(objectIdx).birthTime; ...
            current(objectIdx).birthLocation];
        if isempty(findLabel(labels, label))
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

function idx = findLabel(labels, label)
idx = [];
if ~isempty(labels)
    idx = find(all(bsxfun(@eq, labels, label), 1), 1);
end
end

function reliability = linkReliability(config, senderIdx, receiverIdx, time)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(time, size(config.pDropByEdge, 3));
        drop = config.pDropByEdge(senderIdx, receiverIdx, timeIdx);
    else
        drop = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    drop = config.pDropBySensor(senderIdx);
else
    drop = 0;
end
reliability = 1 - clamp01(drop);
end

function value = sumFinite(values)
values = reshape(values, 1, []);
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = sum(values);
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
