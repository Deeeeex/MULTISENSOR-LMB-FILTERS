function metrics = computeBraidedHandoverInformationFlowV84( ...
        context, control, transport, groupIds, options)
% COMPUTEBRAIDEDHANDOVERINFORMATIONFLOWV84 Current-only sender novelty.
%
% A useful handover edge must do two things at once: replace the incumbent
% source formation, and contribute currently associated label support that
% neither the receiver nor the incumbent sender has. The exact one-step KLA
% counterfactual comes from V68; V84 adds the missing information-origin test
% and normalizes both terms by the affected receiver formation.

if nargin < 5 || isempty(options)
    options = getBraidedHandoverOpportunityV84Protocol();
end
requiredContext = {'localPosteriorBySensor'};
requiredControl = {'referenceScore'};
requiredTransport = { ...
    'edgeRecords', 'networkReferenceExistenceMass', ...
    'routeConnectivityProjected', 'receiverFusionMode', ...
    'receiverFirstInputEnforced', ...
    'sourceFormationChangePrefilterUsed', ...
    'currentNovelSupportPrefilterUsed', ...
    'minimumCurrentNovelSupportFractionPrefilter'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, requiredContext)) || ...
        ~isstruct(control) || ~isscalar(control) || ...
        ~all(isfield(control, requiredControl)) || ...
        ~isstruct(transport) || ~isscalar(transport) || ...
        ~all(isfield(transport, requiredTransport)) || ...
        ~strcmp(transport.receiverFusionMode, 'mixture-aware-heavy') || ...
        ~isequal(transport.receiverFirstInputEnforced, true) || ...
        ~isequal(transport.sourceFormationChangePrefilterUsed, true) || ...
        ~isequal(transport.currentNovelSupportPrefilterUsed, true) || ...
        abs(transport.minimumCurrentNovelSupportFractionPrefilter - ...
            getField(options, 'minimumSenderNoveltyFraction', 0)) > 1e-12
    error('BraidedHandoverV84:InvalidInformationFlowInput', ...
        ['V84 requires aligned current state and V68 edges scored by ', ...
         'the mixture-aware heavy-fusion receiver.']);
end

posteriors = reshape(context.localPosteriorBySensor, 1, []);
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(posteriors);
if numel(groupIds) ~= nodeCount
    error('BraidedHandoverV84:InformationFlowDimensionMismatch', ...
        'The V84 posterior and formation identities do not align.');
end
groups = unique(groupIds, 'stable');
referenceMass = referenceMassByFormation(control, groups, groupIds);
networkMass = transport.networkReferenceExistenceMass;
if ~isscalar(networkMass) || ~isfinite(networkMass) || networkMass <= eps
    error('BraidedHandoverV84:InvalidNetworkMass', ...
        'The V84 transport denominator is unavailable.');
end

minimumNet = getField(options, 'minimumLocalNetFraction', 0.01);
minimumNovelty = getField( ...
    options, 'minimumSenderNoveltyFraction', 0.005);
maximumHarmRatio = getField( ...
    options, 'maximumProtectedHarmRatio', 1.0);
validateThresholds(minimumNet, minimumNovelty, maximumHarmRatio);

edgeRecords = repmat(emptyRecord(), 1, numel(transport.edgeRecords));
for edgeIdx = 1:numel(transport.edgeRecords)
    edge = transport.edgeRecords(edgeIdx);
    receiverIdx = edge.receiverIdx;
    incumbentIdx = edge.incumbentSenderIdx;
    candidateIdx = edge.candidateSenderIdx;
    if any(~isfinite([receiverIdx, incumbentIdx, candidateIdx])) || ...
            any([receiverIdx, incumbentIdx, candidateIdx] < 1) || ...
            any([receiverIdx, incumbentIdx, candidateIdx] > nodeCount)
        error('BraidedHandoverV84:InvalidTransportEdge', ...
            'A V68 sender substitution has invalid endpoints.');
    end
    formationIdx = find(groups == groupIds(receiverIdx), 1);
    denominator = referenceMass(formationIdx);
    noveltyMass = senderNoveltyMass( ...
        posteriors{receiverIdx}, posteriors{incumbentIdx}, ...
        posteriors{candidateIdx}, edge.linkReliability);
    localGain = networkMass * edge.transportGainFraction / denominator;
    localHarm = networkMass * edge.supportedHarmFraction / denominator;
    localNet = networkMass * edge.netHeadroomFraction / denominator;
    localNovelty = noveltyMass / denominator;
    changesFormation = ...
        groupIds(candidateIdx) ~= groupIds(incumbentIdx);
    harmSafe = localHarm <= maximumHarmRatio * localGain + 1e-12;
    enabled = changesFormation && edge.safe && harmSafe && ...
        localNet >= minimumNet - 1e-12 && ...
        localNovelty >= minimumNovelty - 1e-12;
    score = 0;
    if enabled
        score = min(localNet / minimumNet, ...
            localNovelty / minimumNovelty);
    end
    edgeRecords(edgeIdx) = struct( ...
        'receiverIdx', receiverIdx, ...
        'receiverFormationId', groupIds(receiverIdx), ...
        'incumbentSenderIdx', incumbentIdx, ...
        'incumbentFormationId', groupIds(incumbentIdx), ...
        'candidateSenderIdx', candidateIdx, ...
        'candidateFormationId', groupIds(candidateIdx), ...
        'changesSourceFormation', changesFormation, ...
        'linkReliability', edge.linkReliability, ...
        'localTransportGainFraction', localGain, ...
        'localProtectedHarmFraction', localHarm, ...
        'localNetFraction', localNet, ...
        'localSenderNoveltyFraction', localNovelty, ...
        'upwardCrossingCount', edge.upwardCrossingCount, ...
        'downwardCrossingCount', edge.downwardCrossingCount, ...
        'safe', edge.safe, ...
        'actionEnabled', enabled, ...
        'rankingScore', score);
end

enabled = [edgeRecords.actionEnabled];
best = emptyRecord();
if any(enabled)
    eligible = find(enabled);
    scores = [edgeRecords(eligible).rankingScore];
    nets = [edgeRecords(eligible).localNetFraction];
    novelty = [edgeRecords(eligible).localSenderNoveltyFraction];
    ranking = [-scores(:), -nets(:), -novelty(:), eligible(:)];
    [~, order] = sortrows(ranking, [1, 2, 3, 4]);
    best = edgeRecords(eligible(order(1)));
end

metrics = struct();
metrics.contractVersion = ...
    'braided-handover-information-flow-v84-v1';
metrics.groups = groups;
metrics.groupIds = groupIds;
metrics.referenceExistenceMassByFormation = referenceMass;
metrics.edgeRecords = edgeRecords;
metrics.actionableEdgeCount = nnz(enabled);
metrics.bestAction = best;
metrics.maximumRankingScore = best.rankingScore;
metrics.maximumLocalNetFraction = best.localNetFraction;
metrics.maximumLocalSenderNoveltyFraction = ...
    best.localSenderNoveltyFraction;
metrics.minimumLocalNetFraction = minimumNet;
metrics.minimumSenderNoveltyFraction = minimumNovelty;
metrics.maximumProtectedHarmRatio = maximumHarmRatio;
metrics.sourceFormationChangeRequired = true;
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

function mass = senderNoveltyMass(receiver, incumbent, candidate, reliability)
labels = collectLabels({receiver, incumbent, candidate});
receiverSupport = supportByLabel(receiver, labels);
incumbentSupport = supportByLabel(incumbent, labels);
candidateSupport = supportByLabel(candidate, labels);
candidateExistence = existenceByLabel(candidate, labels);
baselineSupport = max(receiverSupport, incumbentSupport);
mass = reliability * sum( ...
    max(candidateSupport - baselineSupport, 0) .* candidateExistence);
end

function mass = referenceMassByFormation(control, groups, groupIds)
details = control.referenceScore.retentionDetails;
if ~isstruct(details) || ...
        ~isfield(details, 'expectedReferenceExistenceByReceiver') || ...
        numel(details.expectedReferenceExistenceByReceiver) ~= ...
            numel(groupIds)
    error('BraidedHandoverV84:MissingReferenceExistence', ...
        'Receiver-level reference existence is unavailable.');
end
mass = zeros(1, numel(groups));
for receiverIdx = 1:numel(groupIds)
    formationIdx = find(groups == groupIds(receiverIdx), 1);
    values = reshape( ...
        details.expectedReferenceExistenceByReceiver{receiverIdx}, 1, []);
    if any(~isfinite(values)) || any(values < -1e-12)
        error('BraidedHandoverV84:InvalidReferenceExistence', ...
            'Receiver-level reference existence is invalid.');
    end
    mass(formationIdx) = mass(formationIdx) + sum(max(values, 0));
end
if any(~isfinite(mass)) || any(mass <= eps)
    error('BraidedHandoverV84:InvalidFormationMass', ...
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

function validateThresholds(net, novelty, harmRatio)
values = [net, novelty, harmRatio];
if any(~isfinite(values)) || any(values < 0) || net <= 0 || novelty <= 0
    error('BraidedHandoverV84:InvalidThreshold', ...
        'V84 opportunity thresholds must be positive and finite.');
end
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = emptyRecord()
value = struct( ...
    'receiverIdx', NaN, ...
    'receiverFormationId', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'incumbentFormationId', NaN, ...
    'candidateSenderIdx', NaN, ...
    'candidateFormationId', NaN, ...
    'changesSourceFormation', false, ...
    'linkReliability', NaN, ...
    'localTransportGainFraction', NaN, ...
    'localProtectedHarmFraction', NaN, ...
    'localNetFraction', NaN, ...
    'localSenderNoveltyFraction', NaN, ...
    'upwardCrossingCount', NaN, ...
    'downwardCrossingCount', NaN, ...
    'safe', false, ...
    'actionEnabled', false, ...
    'rankingScore', 0);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
