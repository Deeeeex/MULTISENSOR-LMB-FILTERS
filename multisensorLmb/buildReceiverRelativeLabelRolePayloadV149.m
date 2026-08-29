function [payload, details] = ...
        buildReceiverRelativeLabelRolePayloadV149( ...
            workingObjects, referenceObjects, receiverObjects, ...
            model, eventType, updateDiagnostics, options)
% BUILDRECEIVERRELATIVELABELROLEPAYLOADV149 Safe receiver-state teacher.

if nargin < 7 || isempty(options)
    options = struct();
end
protocol = getReceiverRelativeLabelRoleTeacherV149Protocol();
config = protocol.receiverTeacher;
workingObjects = reshape(workingObjects, 1, []);
referenceObjects = reshape(referenceObjects, 1, []);
receiverObjects = reshape(receiverObjects, 1, []);
labels = collectLabels({referenceObjects, workingObjects});
records = repmat(emptyRecord(model.object), 1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    working = objectByLabel(workingObjects, label);
    reference = objectByLabel(referenceObjects, label);
    receiver = objectByLabel(receiverObjects, label);
    if ~isempty(reference)
        [selected, role, advantage, rescue] = chooseSharedObject( ...
            receiver, working, reference, model, eventType, ...
            config, options);
        records(labelIdx) = struct( ...
            'label', label, 'object', selected, ...
            'referenceObject', reference, 'role', role, ...
            'kind', 'reference-cover', 'advantage', advantage, ...
            'kept', true, 'initiallyKept', true, ...
            'referenceRescue', rescue, ...
            'revertible', strcmp(role, 'working'), ...
            'bytesSavingIfReverted', max( ...
                singletonBytes(selected, model, eventType, ...
                    updateDiagnostics) - ...
                singletonBytes(reference, model, eventType, ...
                    updateDiagnostics), 0));
    else
        [include, advantage] = chooseWorkingOnlyObject( ...
            receiver, working, model, eventType, config, options);
        records(labelIdx) = struct( ...
            'label', label, 'object', working, ...
            'referenceObject', [], 'role', 'working', ...
            'kind', 'working-only', 'advantage', advantage, ...
            'kept', include, 'initiallyKept', include, ...
            'referenceRescue', false, 'revertible', false, ...
            'bytesSavingIfReverted', 0);
    end
end

referenceStats = estimateLmbPayloadSize( ...
    referenceObjects, model, eventType, updateDiagnostics);
capBytes = referenceStats.estimatedBytes;
[payload, candidateStats] = materialize( ...
    records, model, eventType, updateDiagnostics);
revertedSharedCount = 0;
droppedWorkingOnlyCount = 0;
fallbackUsed = false;
while candidateStats.estimatedBytes > capBytes + 1e-9
    revertible = find([records.kept] & [records.revertible] & ...
        strcmp({records.role}, 'working') & ...
        [records.bytesSavingIfReverted] > 1e-9);
    if ~isempty(revertible)
        ratios = [records(revertible).advantage] ./ ...
            [records(revertible).bytesSavingIfReverted];
        ranking = [ratios', [records(revertible).advantage]', ...
            revertible'];
        [~, order] = sortrows(ranking, [1, 2, 3]);
        recordIdx = revertible(order(1));
        records(recordIdx).object = ...
            records(recordIdx).referenceObject;
        records(recordIdx).role = 'reference';
        records(recordIdx).revertible = false;
        records(recordIdx).bytesSavingIfReverted = 0;
        revertedSharedCount = revertedSharedCount + 1;
    else
        optional = find([records.kept] & ...
            strcmp({records.kind}, 'working-only'));
        if isempty(optional)
            payload = referenceObjects;
            candidateStats = referenceStats;
            fallbackUsed = true;
            break;
        end
        ranking = [[records(optional).advantage]', optional'];
        [~, order] = sortrows(ranking, [1, 2]);
        records(optional(order(1))).kept = false;
        droppedWorkingOnlyCount = droppedWorkingOnlyCount + 1;
    end
    [payload, candidateStats] = materialize( ...
        records, model, eventType, updateDiagnostics);
end
if candidateStats.estimatedBytes > capBytes + 1e-9
    error('ReceiverRelativeLabelRoleV149:ByteProjectionFailure', ...
        'The projected payload exceeds its same-edge full-R byte cap.');
end

coverViolationCount = countReferenceCoverViolations( ...
    payload, referenceObjects);
if coverViolationCount ~= 0
    error('ReceiverRelativeLabelRoleV149:ReferenceCoverViolation', ...
        'The V149 projection removed a label from the R payload.');
end
kept = [records.kept];
if fallbackUsed
    selectedWorkingLabelCount = 0;
    selectedReferenceLabelCount = numel(referenceObjects);
    referenceRescueLabelCount = nnz([records.referenceRescue]);
    droppedWorkingOnlyCount = nnz( ...
        [records.initiallyKept] & ...
        strcmp({records.kind}, 'working-only'));
    revertedSharedCount = nnz( ...
        strcmp({records.kind}, 'reference-cover') & ...
        [records.initiallyKept] & ...
        ~cellfun(@isempty, {records.referenceObject}) & ...
        strcmp({records.role}, 'working')) + revertedSharedCount;
else
    selectedWorkingLabelCount = nnz( ...
        kept & strcmp({records.role}, 'working'));
    selectedReferenceLabelCount = nnz( ...
        kept & strcmp({records.role}, 'reference'));
    referenceRescueLabelCount = nnz( ...
        kept & [records.referenceRescue]);
end
selectedAdvantages = [records( ...
    kept & strcmp({records.role}, 'working')).advantage];
details = struct( ...
    'contractVersion', ...
        'v149-receiver-relative-label-role-payload-details-v1', ...
    'candidateLabelCount', numel(records), ...
    'selectedLabelCount', numel(payload), ...
    'selectedWorkingLabelCount', selectedWorkingLabelCount, ...
    'selectedReferenceLabelCount', selectedReferenceLabelCount, ...
    'referenceRescueLabelCount', referenceRescueLabelCount, ...
    'droppedOptionalLabelCount', droppedWorkingOnlyCount, ...
    'revertedSharedLabelCount', revertedSharedCount, ...
    'workingOnlyAddedLabelCount', nnz(kept & ...
        strcmp({records.kind}, 'working-only')), ...
    'referenceCoverViolationCount', coverViolationCount, ...
    'referenceCoverPreserved', coverViolationCount == 0, ...
    'referencePayloadBytes', capBytes, ...
    'candidatePayloadBytes', candidateStats.estimatedBytes, ...
    'meanSelectedWorkingAdvantage', finiteMean(selectedAdvantages), ...
    'fallbackUsed', fallbackUsed, ...
    'receiverTeacherUsed', true, ...
    'receiverStateDirectlyAccessed', true, ...
    'onlineDeployable', false, ...
    'truthUsed', false, 'futureMeasurementUsed', false, ...
    'futureOutcomeUsed', false);
end

function [selected, role, advantage, rescue] = chooseSharedObject( ...
        receiver, working, reference, model, eventType, config, options)
rescue = objectExistence(reference) >= ...
        config.decisionExistenceThreshold - 1e-12 && ...
    objectExistence(receiver) < ...
        config.decisionExistenceThreshold - 1e-12 && ...
    objectExistence(working) < ...
        config.decisionExistenceThreshold - 1e-12;
if isempty(working) || rescue
    selected = reference;
    role = 'reference';
    advantage = 0;
    return;
end
fusedWorking = virtualFuse( ...
    receiver, working, model, eventType, options);
fusedReference = virtualFuse( ...
    receiver, reference, model, eventType, options);
anchors = {receiver, working, reference};
workingLoss = receiverTaskLoss( ...
    fusedWorking, anchors, model, config);
referenceLoss = receiverTaskLoss( ...
    fusedReference, anchors, model, config);
advantage = referenceLoss - workingLoss;
if advantage > config.minimumWorkingAdvantage
    selected = working;
    role = 'working';
else
    selected = reference;
    role = 'reference';
end
end

function [include, advantage] = chooseWorkingOnlyObject( ...
        receiver, working, model, eventType, config, options)
include = false;
advantage = 0;
if isempty(working) || ...
        supportStrength(working, model) < ...
            config.minimumWorkingOnlySupport - 1e-12
    return;
end
baseline = virtualFuse(receiver, [], model, eventType, options);
candidate = virtualFuse(receiver, working, model, eventType, options);
anchors = {receiver, working};
baselineLoss = receiverTaskLoss(baseline, anchors, model, config);
candidateLoss = receiverTaskLoss(candidate, anchors, model, config);
advantage = baselineLoss - candidateLoss;
include = advantage > config.minimumWorkingAdvantage;
end

function fused = virtualFuse(receiver, source, model, eventType, options)
if isempty(receiver) && isempty(source)
    fused = [];
    return;
end
if isempty(receiver)
    receiver = model.object([]);
end
if isempty(source)
    source = model.object([]);
end
receiverIdx = getField(options, 'receiverIdx', 1);
senderIdx = getField(options, 'senderIdx', 2);
currentTime = getField(options, 'currentTime', 1);
receiverWeight = getField(options, 'receiverWeight', 0.5);
senderWeight = getField(options, 'senderWeight', 0.5);
weights = [receiverWeight, senderWeight];
weights(~isfinite(weights) | weights < 0) = 0;
if sum(weights) <= 0
    weights = [0.5, 0.5];
end
weights = weights / sum(weights);
fusionDetails = struct( ...
    'eventType', [0, max(eventType, 1)], ...
    'sourceIndices', [receiverIdx, senderIdx], ...
    'isStale', [false, false], ...
    'isSelf', [true, false], ...
    'currentTime', currentTime);
fusionConfig = getField(options, 'fusionConfig', ...
    buildMixtureAwareKlaReferenceConfig(struct( ...
        'missingLabelFusionMode', 'fov-aware-censored', ...
        'useStaleNeighborCache', false)));
fusionModel = model;
if isfield(model, 'sensorObservation')
    fusionModel = ...
        materializeObservableSensorFusionModel(model, currentTime);
end
objects = fuseLmbPosteriorsByLabel( ...
    {receiver, source}, weights, fusionModel, weights, ...
    fusionDetails, fusionConfig);
if isempty(objects)
    fused = [];
elseif numel(objects) == 1
    fused = objects(1);
else
    error('ReceiverRelativeLabelRoleV149:UnexpectedVirtualFusion', ...
        'A one-label virtual fusion returned multiple labels.');
end
end

function loss = receiverTaskLoss(candidate, anchors, model, config)
loss = 0;
positionScale = resolvePositionScale(model);
for anchorIdx = 1:numel(anchors)
    anchor = anchors{anchorIdx};
    strength = supportStrength(anchor, model);
    if strength <= 0
        continue;
    end
    if anchorIdx == 1
        strength = strength * config.receiverAnchorMultiplier;
    end
    candidateExistence = objectExistence(candidate);
    anchorExistence = objectExistence(anchor);
    existenceLoss = (candidateExistence - anchorExistence) ^ 2;
    [positionLoss, uncertaintyLoss] = objectStateLoss( ...
        candidate, anchor, model, positionScale);
    crossingLoss = double( ...
        objectAssociation(anchor) >= ...
            config.positiveAssociationThreshold - 1e-12 && ...
        anchorExistence >= ...
            config.decisionExistenceThreshold - 1e-12 && ...
        candidateExistence < ...
            config.decisionExistenceThreshold - 1e-12) * ...
        config.decisionCrossingPenalty;
    loss = loss + strength * ( ...
        config.existenceLossWeight * existenceLoss + ...
        config.positionLossWeight * positionLoss + ...
        config.uncertaintyLossWeight * uncertaintyLoss + ...
        crossingLoss);
end
end

function [positionLoss, uncertaintyLoss] = ...
        objectStateLoss(candidate, anchor, model, positionScale)
if isempty(anchor) || anchor.numberOfGmComponents <= 0
    positionLoss = 0;
    uncertaintyLoss = 0;
    return;
end
if isempty(candidate) || candidate.numberOfGmComponents <= 0
    positionLoss = 1;
    uncertaintyLoss = 1;
    return;
end
[candidateMean, candidateCovariance] = ...
    momentMatch(candidate, model.xDimension);
[anchorMean, anchorCovariance] = ...
    momentMatch(anchor, model.xDimension);
positionDimension = min(2, model.xDimension);
distance = norm(candidateMean(1:positionDimension) - ...
    anchorMean(1:positionDimension)) / max(positionScale, eps);
positionLoss = min(distance ^ 2, 1);
candidateTrace = trace(candidateCovariance( ...
    1:positionDimension, 1:positionDimension));
anchorTrace = trace(anchorCovariance( ...
    1:positionDimension, 1:positionDimension));
uncertaintyLoss = min(abs(candidateTrace - anchorTrace) / ...
    max(positionScale ^ 2, eps), 1);
end

function strength = supportStrength(object, model)
strength = 0;
if isempty(object) || object.numberOfGmComponents <= 0
    return;
end
[~, covariance] = momentMatch(object, model.xDimension);
positionDimension = min(2, model.xDimension);
positionScale = resolvePositionScale(model);
uncertainty = trace(covariance(1:positionDimension, ...
    1:positionDimension)) / max(positionScale ^ 2, eps);
spatialCredibility = 0.25 / (1 + max(uncertainty, 0));
strength = objectExistence(object) * ...
    max(objectAssociation(object), spatialCredibility);
end

function [meanVector, covariance] = momentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:numel(weights)
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:numel(weights)
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ( ...
        object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function [payload, stats] = materialize( ...
        records, model, eventType, updateDiagnostics)
kept = find([records.kept]);
if isempty(kept)
    payload = model.object;
else
    payload = records(kept(1)).object([]);
    for recordIdx = kept
        payload(end + 1) = records(recordIdx).object; %#ok<AGROW>
    end
end
stats = estimateLmbPayloadSize( ...
    payload, model, eventType, updateDiagnostics);
end

function count = countReferenceCoverViolations(payload, reference)
count = 0;
for objectIdx = 1:numel(reference)
    label = [reference(objectIdx).birthTime; ...
        reference(objectIdx).birthLocation];
    count = count + double(isempty(objectByLabel(payload, label)));
end
end

function bytes = singletonBytes( ...
        object, model, eventType, updateDiagnostics)
stats = estimateLmbPayloadSize( ...
    object, model, eventType, updateDiagnostics);
bytes = stats.estimatedBytes;
end

function labels = collectLabels(collections)
labels = zeros(2, 0);
for collectionIdx = 1:numel(collections)
    objects = collections{collectionIdx};
    for objectIdx = 1:numel(objects)
        object = objects(objectIdx);
        if object.numberOfGmComponents <= 0
            continue;
        end
        label = [object.birthTime; object.birthLocation];
        if isempty(labels) || ...
                ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
end

function object = objectByLabel(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    candidate = objects(objectIdx);
    if candidate.numberOfGmComponents > 0 && ...
            candidate.birthTime == label(1) && ...
            candidate.birthLocation == label(2)
        object = candidate;
        return;
    end
end
end

function value = objectExistence(object)
value = 0;
if ~isempty(object) && isfield(object, 'r') && ...
        isscalar(object.r) && isfinite(object.r)
    value = min(max(object.r, 0), 1);
end
end

function value = objectAssociation(object)
value = 0;
if ~isempty(object) && ...
        isfield(object, 'detectionAssociationMass') && ...
        isscalar(object.detectionAssociationMass) && ...
        isfinite(object.detectionAssociationMass)
    value = min(max(object.detectionAssociationMass, 0), 1);
end
end

function record = emptyRecord(template)
record = struct( ...
    'label', zeros(2, 1), 'object', {template}, ...
    'referenceObject', {template}, 'role', '', 'kind', '', ...
    'advantage', 0, 'kept', false, 'initiallyKept', false, ...
    'referenceRescue', false, 'revertible', false, ...
    'bytesSavingIfReverted', 0);
end

function value = resolvePositionScale(model)
value = 100;
if isfield(model, 'ospaParameters') && ...
        isfield(model.ospaParameters, 'eC') && ...
        isfinite(model.ospaParameters.eC) && ...
        model.ospaParameters.eC > 0
    value = model.ospaParameters.eC;
end
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = mean(values);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
