function result = buildContextAwareReceiverSafeLabelSubsetOptions( ...
        receiverObject, fixedSenderObjects, selectableSenderObjects, ...
        selectableSenderEvidence, model, triggerConfig, options)
% BUILDCONTEXTAWARERECEIVERSAFELABELSUBSETOPTIONS V55 label options.
%
% Every option contains the same fixed V46 fusion inputs.  The reference also
% contains every present cross-residual label that the full V46 message would
% supply; evidence type is not used to delete labels from the reference.
% Receiver-supported existence projection may be enabled for the combined
% arm or disabled for selection-only attribution.

if nargin < 7 || isempty(options)
    options = struct();
end
receiverPresent = ~isempty(receiverObject);
if receiverPresent && ...
        (~isstruct(receiverObject) || ~isscalar(receiverObject) || ...
         ~isfield(receiverObject, 'r'))
    error('ContextAwareReceiverSafeOptions:InvalidReceiver', ...
        'The receiver label must be empty or a scalar object.');
end
if ~iscell(fixedSenderObjects) || ...
        ~iscell(selectableSenderObjects) || ...
        ~iscell(selectableSenderEvidence) || ...
        numel(selectableSenderObjects) ~= ...
            numel(selectableSenderEvidence)
    error('ContextAwareReceiverSafeOptions:InvalidSources', ...
        'Fixed and selectable label inputs must be matching cell arrays.');
end

fixedCount = numel(fixedSenderObjects);
selectableCount = numel(selectableSenderObjects);
maximumEnumeratedSenders = getField( ...
    options, 'maximumEnumeratedSenders', 8);
if selectableCount > maximumEnumeratedSenders
    error('ContextAwareReceiverSafeOptions:TooManySelectableSources', ...
        'At most %d selectable senders may be enumerated.', ...
        maximumEnumeratedSenders);
end

referenceMask = false(1, selectableCount);
for senderIdx = 1:selectableCount
    referenceMask(senderIdx) = ...
        ~isempty(selectableSenderObjects{senderIdx});
end
enumeratedIndices = find(referenceMask);

sourceCount = 1 + fixedCount + selectableCount;
spatialWeights = resolveSourceWeights( ...
    options, 'spatialWeights', sourceCount);
existenceWeights = resolveSourceWeights( ...
    options, 'existenceWeights', sourceCount);
referenceObject = fuseContext( ...
    receiverObject, fixedSenderObjects, selectableSenderObjects, ...
    referenceMask, spatialWeights, existenceWeights, model, ...
    triggerConfig);

supportThreshold = getUnitOption( ...
    options, 'supportedExistenceThreshold', 0.50);
receiverHasPositiveSupport = logical(getField( ...
    options, 'receiverHasPositiveSupport', true));
maximumReceiverLogOddsDrop = getNonnegativeOption( ...
    options, 'maximumReceiverLogOddsDrop', log(4));
existenceProjectionEnabled = logical(getField( ...
    options, 'existenceProjectionEnabled', true));
if receiverPresent
    receiverExistence = receiverObject.r;
    retentionFloor = logOddsRetentionFloor( ...
        receiverExistence, maximumReceiverLogOddsDrop);
else
    receiverExistence = NaN;
    retentionFloor = NaN;
end
floorIsActive = existenceProjectionEnabled && receiverPresent && ...
    receiverHasPositiveSupport && ...
    receiverObject.r >= supportThreshold;
[projectedReference, referenceWasProjected] = ...
    projectExistenceToFloor( ...
        referenceObject, retentionFloor, floorIsActive);

selectedPayloadMetadataScalars = getField( ...
    options, 'selectedPayloadMetadataScalars', 0);
if ~isscalar(selectedPayloadMetadataScalars) || ...
        ~isfinite(selectedPayloadMetadataScalars) || ...
        selectedPayloadMetadataScalars < 0 || ...
        selectedPayloadMetadataScalars ~= ...
            round(selectedPayloadMetadataScalars)
    error('ContextAwareReceiverSafeOptions:InvalidMetadataCost', ...
        'selectedPayloadMetadataScalars must be a nonnegative integer.');
end

optionCount = 2^numel(enumeratedIndices);
labelOptions = repmat(makeOption(selectableCount), 1, optionCount);
for optionIdx = 1:optionCount
    localSelection = logical(arrayfun( ...
        @(bitIdx) bitget(optionIdx - 1, bitIdx), ...
        1:numel(enumeratedIndices)));
    senderSubset = false(1, selectableCount);
    senderSubset(enumeratedIndices(localSelection)) = true;
    rawCandidate = fuseContext( ...
        receiverObject, fixedSenderObjects, selectableSenderObjects, ...
        senderSubset, spatialWeights, existenceWeights, model, ...
            triggerConfig);
    if isempty(rawCandidate)
        rawCandidate = projectedReference;
        rawCandidate.r = 0;
    end
    [candidateObject, candidateWasProjected] = ...
        projectExistenceToFloor( ...
            rawCandidate, retentionFloor, floorIsActive);
    [spatialKld, spatialDetails] = ...
        approximateLmbSpatialKldCubature( ...
            projectedReference, candidateObject);
    terms = computeLmbBernoulliKldTerms( ...
        projectedReference.r, candidateObject.r, spatialKld);

    % Bernoulli KLD is nonnegative; suppress only roundoff below zero.
    labelOptions(optionIdx).distortion = max(terms.total, 0);
    labelOptions(optionIdx).variablePayloadBytes = ...
        selectedLabelPayloadBytes( ...
            selectableSenderObjects, senderSubset, model.xDimension, ...
            selectedPayloadMetadataScalars);
    labelOptions(optionIdx).activeEdgeMask = senderSubset;
    labelOptions(optionIdx).existenceRetentionSatisfied = true;
    labelOptions(optionIdx).senderSubset = senderSubset;
    labelOptions(optionIdx).selectedSenderCount = sum(senderSubset);
    labelOptions(optionIdx).referenceExistence = ...
        projectedReference.r;
    labelOptions(optionIdx).candidateExistence = candidateObject.r;
    labelOptions(optionIdx).rawCandidateExistence = rawCandidate.r;
    labelOptions(optionIdx).existenceRetentionFloor = retentionFloor;
    labelOptions(optionIdx).existenceProjectionActive = floorIsActive;
    labelOptions(optionIdx).existenceWasProjected = ...
        candidateWasProjected;
    labelOptions(optionIdx).spatialKld = spatialKld;
    labelOptions(optionIdx).spatialKldDetails = spatialDetails;
    labelOptions(optionIdx).bernoulliKldTerms = terms;
    labelOptions(optionIdx).candidateObject = candidateObject;
end

result = struct();
result.contractVersion = ...
    'context-aware-receiver-safe-label-subset-options-v2';
result.referenceObject = projectedReference;
result.rawReferenceObject = referenceObject;
result.referenceWasProjected = referenceWasProjected;
result.fixedSourceCount = fixedCount;
result.fixedPresentSourceCount = sum(cellfun( ...
    @(object) ~isempty(object), fixedSenderObjects));
result.selectableSourceCount = selectableCount;
result.referenceSelectableMask = referenceMask;
result.admissibleSelectableMask = referenceMask;
result.excludedSelectableMask = ~referenceMask;
result.selectableEvidenceUsedForReference = false;
result.labelOptions = labelOptions;
result.receiverHasPositiveSupport = receiverHasPositiveSupport;
result.receiverExistence = receiverExistence;
result.existenceRetentionFloor = retentionFloor;
result.existenceProjectionActive = floorIsActive;
result.existenceProjectionEnabled = existenceProjectionEnabled;
result.maximumReceiverLogOddsDrop = maximumReceiverLogOddsDrop;
result.selectedPayloadMetadataScalars = ...
    selectedPayloadMetadataScalars;
result.controlSynopsisCostIncluded = false;
result.spatialMetricIsExactGmKld = false;
result.fixedContextIncluded = true;
result.receiverApproximation = 'installed-powered-gm-lmb-kla';
result.requiresFullSenderPosteriors = true;
result.intendedUse = 'offline-context-aware-teacher';
end

function fusedObject = fuseContext( ...
        receiverObject, fixedSenderObjects, selectableSenderObjects, ...
        selectableMask, spatialWeights, existenceWeights, model, ...
        triggerConfig)
fixedPresentMask = cellfun(@(object) ~isempty(object), ...
    fixedSenderObjects);
selectedPresentMask = logical(reshape(selectableMask, 1, [])) & ...
    cellfun(@(object) ~isempty(object), selectableSenderObjects);

distributions = cell(1, 0);
sourceIndices = zeros(1, 0);
eventTypes = zeros(1, 0);
if ~isempty(receiverObject)
    distributions{end+1} = receiverObject;
    sourceIndices(end+1) = 1;
    eventTypes(end+1) = 0;
end
for fixedIdx = find(fixedPresentMask)
    distributions{end+1} = fixedSenderObjects{fixedIdx}; %#ok<AGROW>
    sourceIndices(end+1) = 1 + fixedIdx; %#ok<AGROW>
    eventTypes(end+1) = 2; %#ok<AGROW>
end
fixedCount = numel(fixedSenderObjects);
for selectableIdx = find(selectedPresentMask)
    distributions{end+1} = ...
        selectableSenderObjects{selectableIdx}; %#ok<AGROW>
    sourceIndices(end+1) = ...
        1 + fixedCount + selectableIdx; %#ok<AGROW>
    eventTypes(end+1) = 2; %#ok<AGROW>
end

if isempty(distributions)
    fusedObject = [];
    return;
end
if numel(distributions) == 1
    fusedObject = distributions{1};
    return;
end
fusionDetails = struct('eventType', eventTypes);
fused = fuseLmbPosteriorsByLabel( ...
    distributions, spatialWeights(sourceIndices), model, ...
    existenceWeights(sourceIndices), fusionDetails, triggerConfig);
if numel(fused) ~= 1
    error('ContextAwareReceiverSafeOptions:UnexpectedFusionOutput', ...
        'One context-aware label option must produce exactly one label.');
end
fusedObject = fused(1);
end

function [object, wasProjected] = ...
        projectExistenceToFloor(object, floorValue, floorIsActive)
wasProjected = floorIsActive && object.r + 1e-12 < floorValue;
if wasProjected
    object.r = floorValue;
end
end

function bytes = selectedLabelPayloadBytes( ...
        senderObjects, senderSubset, stateDimension, metadataScalars)
bytes = 0;
for senderIdx = find(senderSubset)
    object = senderObjects{senderIdx};
    componentCount = object.numberOfGmComponents;
    scalarCount = 3 + metadataScalars + componentCount * ...
        (1 + stateDimension + stateDimension * stateDimension);
    bytes = bytes + 8 * scalarCount;
end
end

function weights = resolveSourceWeights( ...
        options, fieldName, sourceCount)
weights = getField(options, fieldName, ones(1, sourceCount));
weights = reshape(weights, 1, []);
if numel(weights) ~= sourceCount || ...
        any(~isfinite(weights)) || any(weights < 0) || ...
        sum(weights) <= 0
    error('ContextAwareReceiverSafeOptions:InvalidSourceWeights', ...
        '%s must contain nonnegative weights for every source.', ...
        fieldName);
end
end

function value = getUnitOption(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('ContextAwareReceiverSafeOptions:InvalidUnitOption', ...
        '%s must be a finite scalar in [0, 1].', fieldName);
end
end

function value = getNonnegativeOption( ...
        options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0
    error('ContextAwareReceiverSafeOptions:InvalidNonnegativeOption', ...
        '%s must be a finite nonnegative scalar.', fieldName);
end
end

function floorValue = logOddsRetentionFloor(existence, maximumDrop)
existence = min(max(existence, eps), 1 - eps);
receiverLogOdds = log(existence / (1 - existence));
floorOdds = exp(receiverLogOdds - maximumDrop);
floorValue = floorOdds / (1 + floorOdds);
end

function option = makeOption(senderCount)
option = struct( ...
    'distortion', Inf, ...
    'variablePayloadBytes', Inf, ...
    'activeEdgeMask', false(1, senderCount), ...
    'existenceRetentionSatisfied', false, ...
    'senderSubset', false(1, senderCount), ...
    'selectedSenderCount', 0, ...
    'referenceExistence', NaN, ...
    'candidateExistence', NaN, ...
    'rawCandidateExistence', NaN, ...
    'existenceRetentionFloor', NaN, ...
    'existenceProjectionActive', false, ...
    'existenceWasProjected', false, ...
    'spatialKld', Inf, ...
    'spatialKldDetails', struct(), ...
    'bernoulliKldTerms', struct(), ...
    'candidateObject', struct());
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
