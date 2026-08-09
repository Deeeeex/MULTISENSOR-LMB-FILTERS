function [fusedObjects, details] = projectReceiverSafeFusionRetention( ...
        fusedObjects, fusionInputs, spatialWeights, existenceWeights, ...
        fusionDetails, selectiveSourceMask, sourceEvidence, model, ...
        triggerConfig, options)
% PROJECTRECEIVERSAFEFUSIONRETENTION Enforce V54 label retention post receipt.
%
% Only label inputs from V54-selective residual sources may be removed.  If
% fixed backbone inputs still push a locally supported label below its
% floor, that label alone falls back to the receiver posterior.  Previously
% transmitted bytes are never reclaimed.

if nargin < 10 || isempty(options)
    options = struct();
end
sourceCount = numel(fusionInputs);
selectiveSourceMask = logical(reshape(selectiveSourceMask, 1, []));
if ~iscell(fusionInputs) || sourceCount < 1 || ...
        numel(spatialWeights) ~= sourceCount || ...
        numel(existenceWeights) ~= sourceCount || ...
        numel(selectiveSourceMask) ~= sourceCount || ...
        ~iscell(sourceEvidence) || numel(sourceEvidence) ~= sourceCount
    error('ReceiverSafeRetention:InvalidInput', ...
        'Fusion inputs, weights, source mask, and evidence must align.');
end
selectiveSourceMask(1) = false;
supportThreshold = getUnitOption( ...
    options, 'receiverSupportThreshold', 0.50);
maximumReceiverLogOddsDrop = getNonnegativeOption( ...
    options, 'maximumReceiverLogOddsDrop', log(4));
requirePositiveReceiverEvidence = logical(getField( ...
    options, 'requirePositiveReceiverEvidence', true));

details = struct();
details.contractVersion = 'receiver-safe-fusion-retention-v1';
details.supportedLabelCount = 0;
details.violationCountBeforeProjection = 0;
details.removedSelectiveInputCount = 0;
details.receiverOnlyFallbackCount = 0;
details.existenceClampCount = 0;
details.unresolvedViolationCount = 0;
details.payloadBytesReclaimed = 0;
details.labelTraces = repmat(makeTrace(), 1, 0);

receiverObjects = fusionInputs{1};
for receiverObjectIdx = 1:numel(receiverObjects)
    receiverObject = receiverObjects(receiverObjectIdx);
    label = [receiverObject.birthTime; receiverObject.birthLocation];
    receiverEvidence = findEvidence(sourceEvidence{1}, label);
    hasPositiveReceiverEvidence = strcmpi( ...
        getEvidenceType(receiverEvidence), 'positive-support');
    if receiverObject.numberOfGmComponents < 1 || ...
            receiverObject.r < supportThreshold || ...
            (requirePositiveReceiverEvidence && ...
             ~hasPositiveReceiverEvidence)
        continue;
    end
    details.supportedLabelCount = details.supportedLabelCount + 1;
    currentObject = findObject(fusedObjects, label);
    floorValue = logOddsRetentionFloor( ...
        receiverObject.r, maximumReceiverLogOddsDrop);
    if ~isempty(currentObject) && currentObject.r + 1e-12 >= floorValue
        continue;
    end
    details.violationCountBeforeProjection = ...
        details.violationCountBeforeProjection + 1;

    mutableInputs = fusionInputs;
    availableSelective = selectiveSourceMask;
    removalOrder = zeros(1, 0);
    while any(availableSelective)
        currentObject = fuseAndFind( ...
            mutableInputs, spatialWeights, existenceWeights, ...
            fusionDetails, model, triggerConfig, label);
        if ~isempty(currentObject) && ...
                currentObject.r + 1e-12 >= floorValue
            break;
        end
        candidates = find(availableSelective);
        trialExistence = -Inf(1, numel(candidates));
        credibleNegative = false(1, numel(candidates));
        for candidateIdx = 1:numel(candidates)
            sourceIdx = candidates(candidateIdx);
            trialInputs = mutableInputs;
            trialInputs{sourceIdx} = removeLabel( ...
                trialInputs{sourceIdx}, label);
            trialObject = fuseAndFind( ...
                trialInputs, spatialWeights, existenceWeights, ...
                fusionDetails, model, triggerConfig, label);
            if ~isempty(trialObject)
                trialExistence(candidateIdx) = trialObject.r;
            end
            evidence = findEvidence(sourceEvidence{sourceIdx}, label);
            credibleNegative(candidateIdx) = strcmpi( ...
                getEvidenceType(evidence), 'credible-negative');
        end
        bestExistence = max(trialExistence);
        best = find(abs(trialExistence - bestExistence) <= 1e-12);
        preferred = best(credibleNegative(best));
        if ~isempty(preferred)
            chosen = preferred(1);
        else
            chosen = best(1);
        end
        sourceIdx = candidates(chosen);
        mutableInputs{sourceIdx} = removeLabel( ...
            mutableInputs{sourceIdx}, label);
        availableSelective(sourceIdx) = false;
        removalOrder(end+1) = sourceIdx; %#ok<AGROW>
    end

    projectedObject = fuseAndFind( ...
        mutableInputs, spatialWeights, existenceWeights, ...
        fusionDetails, model, triggerConfig, label);
    receiverOnlyFallback = isempty(projectedObject);
    existenceClamped = false;
    if receiverOnlyFallback
        projectedObject = receiverObject;
        details.receiverOnlyFallbackCount = ...
            details.receiverOnlyFallbackCount + 1;
    elseif projectedObject.r + 1e-12 < floorValue
        % Bernoulli projection: retain the unconstrained KLA spatial density
        % and clamp only the existence coordinate to the feasible boundary.
        projectedObject.r = floorValue;
        existenceClamped = true;
        details.existenceClampCount = details.existenceClampCount + 1;
    end
    fusedObjects = replaceOrAppendObject( ...
        fusedObjects, projectedObject, label);
    details.removedSelectiveInputCount = ...
        details.removedSelectiveInputCount + numel(removalOrder);
    satisfied = projectedObject.r + 1e-12 >= floorValue;
    details.unresolvedViolationCount = ...
        details.unresolvedViolationCount + double(~satisfied);
    trace = makeTrace();
    trace.birthTime = label(1);
    trace.birthLocation = label(2);
    trace.receiverExistence = receiverObject.r;
    trace.retentionFloor = floorValue;
    trace.removedSourceIndices = removalOrder;
    trace.receiverOnlyFallback = receiverOnlyFallback;
    trace.existenceClamped = existenceClamped;
    trace.projectedExistence = projectedObject.r;
    trace.retentionSatisfied = satisfied;
    details.labelTraces(end+1) = trace; %#ok<AGROW>
end
end

function object = fuseAndFind( ...
        inputs, spatialWeights, existenceWeights, fusionDetails, ...
        model, triggerConfig, label)
objects = fuseLmbPosteriorsByLabel( ...
    inputs, spatialWeights, model, existenceWeights, ...
    fusionDetails, triggerConfig);
object = findObject(objects, label);
end

function objects = removeLabel(objects, label)
if isempty(objects)
    return;
end
keep = true(1, numel(objects));
for objectIdx = 1:numel(objects)
    keep(objectIdx) = ~( ...
        objects(objectIdx).birthTime == label(1) && ...
        objects(objectIdx).birthLocation == label(2));
end
objects = objects(keep);
end

function objects = replaceOrAppendObject(objects, object, label)
index = findObjectIndex(objects, label);
if index > 0
    objects(index) = object;
else
    objects(end+1) = object;
end
end

function object = findObject(objects, label)
object = [];
index = findObjectIndex(objects, label);
if index > 0
    object = objects(index);
end
end

function index = findObjectIndex(objects, label)
index = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        index = objectIdx;
        return;
    end
end
end

function evidence = findEvidence(evidenceArray, label)
evidence = struct();
for evidenceIdx = 1:numel(evidenceArray)
    if evidenceArray(evidenceIdx).birthTime == label(1) && ...
            evidenceArray(evidenceIdx).birthLocation == label(2)
        evidence = evidenceArray(evidenceIdx);
        return;
    end
end
end

function type = getEvidenceType(evidence)
if isstruct(evidence) && isfield(evidence, 'type')
    type = evidence.type;
else
    type = 'unknown';
end
end

function trace = makeTrace()
trace = struct( ...
    'birthTime', 0, ...
    'birthLocation', 0, ...
    'receiverExistence', NaN, ...
    'retentionFloor', NaN, ...
    'removedSourceIndices', [], ...
    'receiverOnlyFallback', false, ...
    'existenceClamped', false, ...
    'projectedExistence', NaN, ...
    'retentionSatisfied', false);
end

function value = getUnitOption(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('ReceiverSafeRetention:InvalidOption', ...
        '%s must lie in [0, 1].', fieldName);
end
end

function value = getNonnegativeOption(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0
    error('ReceiverSafeRetention:InvalidOption', ...
        '%s must be a finite nonnegative scalar.', fieldName);
end
end

function floorValue = logOddsRetentionFloor(existence, maximumDrop)
existence = min(max(existence, eps), 1 - eps);
receiverLogOdds = log(existence / (1 - existence));
floorOdds = exp(receiverLogOdds - maximumDrop);
floorValue = floorOdds / (1 + floorOdds);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
