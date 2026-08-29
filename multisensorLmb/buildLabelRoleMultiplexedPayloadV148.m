function [payload, details] = buildLabelRoleMultiplexedPayloadV148( ...
        workingObjects, referenceObjects, model, eventType, ...
        updateDiagnostics, options)
% BUILDLABELROLEMULTIPLEXEDPAYLOADV148 One full-GM object per selected label.

if nargin < 6 || isempty(options)
    options = struct();
end
protocol = getLabelRoleMultiplexedSinglePayloadV148Protocol();
config = protocol.labelRole;
workingObjects = reshape(workingObjects, 1, []);
referenceObjects = reshape(referenceObjects, 1, []);
labels = collectLabels({workingObjects, referenceObjects});
records = repmat(emptyRecord(model.object), 1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    working = objectByLabel(workingObjects, label);
    reference = objectByLabel(referenceObjects, label);
    [selected, role, priority, mandatory, rescue] = chooseObject( ...
        working, reference, model, config);
    records(labelIdx) = struct( ...
        'label', label, 'object', selected, 'role', role, ...
        'priority', priority, 'mandatory', mandatory, ...
        'referenceRescue', rescue, 'kept', true);
end

referenceStats = estimateLmbPayloadSize( ...
    referenceObjects, model, eventType, updateDiagnostics);
capBytes = referenceStats.estimatedBytes;
[payload, candidateStats] = materialize(records, model, eventType, ...
    updateDiagnostics);
droppedCount = 0;
while candidateStats.estimatedBytes > capBytes + 1e-9
    optional = find([records.kept] & ~[records.mandatory]);
    if isempty(optional)
        payload = referenceObjects;
        candidateStats = referenceStats;
        fallbackUsed = true;
        break;
    end
    ranking = [[records(optional).priority]', optional'];
    [~, order] = sortrows(ranking, [1, 2]);
    records(optional(order(1))).kept = false;
    droppedCount = droppedCount + 1;
    [payload, candidateStats] = materialize( ...
        records, model, eventType, updateDiagnostics);
end
if ~exist('fallbackUsed', 'var')
    fallbackUsed = false;
end
if candidateStats.estimatedBytes > capBytes + 1e-9
    error('LabelRoleV148:ByteProjectionFailure', ...
        'The projected label-role payload exceeds its reference edge cap.');
end

kept = [records.kept];
if fallbackUsed
    selectedWorkingLabelCount = 0;
    selectedReferenceLabelCount = numel(payload);
    referenceRescueLabelCount = nnz([records.referenceRescue]);
    droppedCount = 0;
else
    selectedWorkingLabelCount = nnz(kept & ...
        strcmp({records.role}, 'working'));
    selectedReferenceLabelCount = nnz(kept & ...
        strcmp({records.role}, 'reference'));
    referenceRescueLabelCount = nnz(kept & ...
        [records.referenceRescue]);
end
details = struct( ...
    'contractVersion', 'v148-label-role-payload-details-v1', ...
    'candidateLabelCount', numel(records), ...
    'selectedLabelCount', numel(payload), ...
    'selectedWorkingLabelCount', selectedWorkingLabelCount, ...
    'selectedReferenceLabelCount', selectedReferenceLabelCount, ...
    'referenceRescueLabelCount', referenceRescueLabelCount, ...
    'droppedOptionalLabelCount', droppedCount, ...
    'referencePayloadBytes', capBytes, ...
    'candidatePayloadBytes', candidateStats.estimatedBytes, ...
    'fallbackUsed', fallbackUsed, ...
    'truthUsed', false, 'futureMeasurementUsed', false, ...
    'futureOutcomeUsed', false);
if getField(options, 'requireNonemptyWhenReferenceNonempty', true) && ...
        ~isempty(referenceObjects) && isempty(payload)
    error('LabelRoleV148:EmptyProjectedPayload', ...
        'A nonempty reference edge projected to an empty payload.');
end
end

function [selected, role, priority, mandatory, rescue] = ...
        chooseObject(working, reference, model, config)
workingPresent = ~isempty(working);
referencePresent = ~isempty(reference);
workingExistence = objectExistence(working);
referenceExistence = objectExistence(reference);
workingAssociation = objectAssociation(working);
referenceAssociation = objectAssociation(reference);
workingPositive = workingPresent && ...
    workingAssociation >= config.positiveAssociationThreshold - 1e-12;
referenceProtected = referencePresent && ( ...
    referenceExistence >= ...
        config.decisionExistenceThreshold - 1e-12 || ...
    referenceAssociation >= ...
        config.positiveAssociationThreshold - 1e-12);
rescue = referencePresent && ...
    referenceExistence >= config.decisionExistenceThreshold - 1e-12 && ...
    (~workingPresent || workingExistence < ...
        config.decisionExistenceThreshold - 1e-12);
if rescue
    selected = reference;
    role = 'reference';
    priority = config.referenceRescuePriority + ...
        referenceExistence + referenceAssociation;
elseif workingPositive && (~referencePresent || ...
        workingAssociation >= referenceAssociation - 1e-12)
    selected = working;
    role = 'working';
    priority = config.positiveWorkingPriority + ...
        workingExistence + workingAssociation;
elseif workingPresent && referencePresent
    workingCredibility = objectCredibility(working, model);
    referenceCredibility = objectCredibility(reference, model);
    if referenceCredibility > workingCredibility + 1e-12
        selected = reference;
        role = 'reference';
    else
        selected = working;
        role = 'working';
    end
    priority = config.sharedLabelPriority + ...
        max(workingCredibility, referenceCredibility);
elseif workingPresent
    selected = working;
    role = 'working';
    priority = config.uniqueLabelPriority + ...
        objectCredibility(working, model);
else
    selected = reference;
    role = 'reference';
    priority = config.uniqueLabelPriority + ...
        objectCredibility(reference, model);
end
mandatory = rescue || workingPositive || referenceProtected;
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

function value = objectCredibility(object, model)
if isempty(object) || object.numberOfGmComponents <= 0
    value = 0;
    return;
end
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(size(weights));
end
weights = weights / sum(weights);
meanVector = zeros(model.xDimension, 1);
for componentIdx = 1:numel(weights)
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(model.xDimension);
for componentIdx = 1:numel(weights)
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ( ...
        object.Sigma{componentIdx} + delta * delta');
end
positionScale = model.ospaParameters.eC;
uncertainty = trace(covariance(1:2, 1:2)) / ...
    max(positionScale ^ 2, eps);
spatialCredibility = 0.25 / (1 + max(uncertainty, 0));
value = objectExistence(object) * ...
    max(objectAssociation(object), spatialCredibility);
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
if ~isempty(object) && isfield(object, 'detectionAssociationMass') && ...
        isscalar(object.detectionAssociationMass) && ...
        isfinite(object.detectionAssociationMass)
    value = min(max(object.detectionAssociationMass, 0), 1);
end
end

function record = emptyRecord(template)
record = struct( ...
    'label', zeros(2, 1), 'object', {template}, 'role', '', ...
    'priority', -inf, 'mandatory', false, ...
    'referenceRescue', false, 'kept', false);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
