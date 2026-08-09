function projection = projectSelectedLmbLabelFusionRetention( ...
        receiverObject, senderObjects, senderEvidence, selectedSenderMask, ...
        model, triggerConfig, options)
% PROJECTSELECTEDLMBLABELFUSIONRETENTION Post-receipt V54 safety layer.
%
% Selected payload bytes have already been spent. The receiver executes the
% installed fusion and removes harmful selected inputs until a supported
% local label retains the configured existence floor. Receiver-only fallback
% makes the existence guarantee independent of a learned utility model.

if nargin < 7 || isempty(options)
    options = struct();
end
if ~iscell(senderObjects) || ~iscell(senderEvidence) || ...
        numel(senderObjects) ~= numel(senderEvidence)
    error('LmbRetentionProjection:InvalidSenders', ...
        'Sender objects and evidence must be matching cell arrays.');
end
senderCount = numel(senderObjects);
selectedSenderMask = logical(reshape(selectedSenderMask, 1, []));
if numel(selectedSenderMask) ~= senderCount
    error('LmbRetentionProjection:InvalidMask', ...
        'selectedSenderMask must contain one value per sender.');
end

receiverSupportThreshold = getUnitOption( ...
    options, 'receiverSupportThreshold', 0.50);
retentionFraction = getUnitOption( ...
    options, 'retentionFraction', 0.95);
spatialWeights = resolveWeights( ...
    options, 'spatialWeights', senderCount + 1);
existenceWeights = resolveWeights( ...
    options, 'existenceWeights', senderCount + 1);

initialSelectedMask = selectedSenderMask;
candidateObject = fuseSubset(receiverObject, senderObjects, ...
    selectedSenderMask, spatialWeights, existenceWeights, model, ...
    triggerConfig);
floorIsActive = receiverObject.r >= receiverSupportThreshold;
retentionFloor = retentionFraction * receiverObject.r;
trace = repmat(struct( ...
    'removedSenderIndex', 0, ...
    'removedEvidenceType', '', ...
    'existenceBefore', NaN, ...
    'existenceAfter', NaN), 1, 0);

while floorIsActive && candidateObject.r + 1e-12 < retentionFloor && ...
        any(selectedSenderMask)
    selectedIndices = find(selectedSenderMask);
    trialExistence = -Inf(1, numel(selectedIndices));
    negativePreference = false(1, numel(selectedIndices));
    trialObjects = cell(1, numel(selectedIndices));
    for localIdx = 1:numel(selectedIndices)
        senderIdx = selectedIndices(localIdx);
        trialMask = selectedSenderMask;
        trialMask(senderIdx) = false;
        trialObjects{localIdx} = fuseSubset( ...
            receiverObject, senderObjects, trialMask, spatialWeights, ...
            existenceWeights, model, triggerConfig);
        trialExistence(localIdx) = trialObjects{localIdx}.r;
        negativePreference(localIdx) = strcmpi( ...
            getEvidenceType(senderEvidence{senderIdx}), ...
            'credible-negative');
    end
    bestExistence = max(trialExistence);
    bestCandidates = find(abs(trialExistence - bestExistence) <= 1e-12);
    preferred = bestCandidates(negativePreference(bestCandidates));
    if ~isempty(preferred)
        chosenLocalIdx = preferred(1);
    else
        chosenLocalIdx = bestCandidates(1);
    end
    removedSenderIdx = selectedIndices(chosenLocalIdx);
    existenceBefore = candidateObject.r;
    selectedSenderMask(removedSenderIdx) = false;
    candidateObject = trialObjects{chosenLocalIdx};
    trace(end+1) = struct( ...
        'removedSenderIndex', removedSenderIdx, ...
        'removedEvidenceType', ...
            getEvidenceType(senderEvidence{removedSenderIdx}), ...
        'existenceBefore', existenceBefore, ...
        'existenceAfter', candidateObject.r); %#ok<AGROW>
end

projection = struct();
projection.contractVersion = 'selected-lmb-label-retention-projection-v1';
projection.fusedObject = candidateObject;
projection.initialSelectedSenderMask = initialSelectedMask;
projection.acceptedSenderMask = selectedSenderMask;
projection.rejectedSenderMask = ...
    initialSelectedMask & ~selectedSenderMask;
projection.receiverExistence = receiverObject.r;
projection.retentionFloorIsActive = floorIsActive;
projection.retentionFloor = retentionFloor;
projection.retentionSatisfied = ~floorIsActive || ...
    candidateObject.r + 1e-12 >= retentionFloor;
projection.removalTrace = trace;
projection.payloadBytesReclaimed = 0;
projection.isPostReceiptProjection = true;
projection.guaranteeDependsOnLearnedUtility = false;
end

function fusedObject = fuseSubset(receiverObject, senderObjects, mask, ...
        spatialWeights, existenceWeights, model, triggerConfig)
selected = find(mask);
if isempty(selected)
    fusedObject = receiverObject;
    return;
end
sourceIndices = [1, selected + 1];
fusionDetails = struct('eventType', [0, 2 * ones(1, numel(selected))]);
fused = fuseLmbPosteriorsByLabel( ...
    [{receiverObject}, senderObjects(selected)], ...
    spatialWeights(sourceIndices), model, ...
    existenceWeights(sourceIndices), fusionDetails, triggerConfig);
if numel(fused) ~= 1
    error('LmbRetentionProjection:UnexpectedFusionOutput', ...
        'Single-label retention projection requires one fused label.');
end
fusedObject = fused(1);
end

function type = getEvidenceType(evidence)
if isstruct(evidence) && isfield(evidence, 'type')
    type = evidence.type;
elseif isstruct(evidence) && isfield(evidence, 'evidence') && ...
        isstruct(evidence.evidence) && isfield(evidence.evidence, 'type')
    type = evidence.evidence.type;
else
    type = 'unknown';
end
end

function weights = resolveWeights(options, fieldName, sourceCount)
weights = getField(options, fieldName, ones(1, sourceCount));
weights = reshape(weights, 1, []);
if numel(weights) ~= sourceCount || any(~isfinite(weights)) || ...
        any(weights < 0) || sum(weights) <= 0
    error('LmbRetentionProjection:InvalidWeights', ...
        '%s must provide nonnegative weights for every source.', fieldName);
end
end

function value = getUnitOption(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('LmbRetentionProjection:InvalidUnitOption', ...
        '%s must be a finite scalar in [0, 1].', fieldName);
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
