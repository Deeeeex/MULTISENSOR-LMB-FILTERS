function result = buildReceiverSafeLabelSubsetOptions( ...
        receiverObject, senderObjects, senderEvidence, ...
        model, triggerConfig, options)
% BUILDRECEIVERSAFELABELSUBSETOPTIONS Enumerate one label's safe inputs.
%
% The reference fuses the receiver with every sender carrying positive or
% credible-negative evidence. Candidate options enumerate subsets of those
% senders with the installed powered-GM receiver approximation.

if nargin < 6 || isempty(options)
    options = struct();
end
if ~isstruct(receiverObject) || ~isscalar(receiverObject) || ...
        ~isfield(receiverObject, 'r')
    error('ReceiverSafeOptions:InvalidReceiver', ...
        'A scalar receiver label is required.');
end
if ~iscell(senderObjects) || ~iscell(senderEvidence) || ...
        numel(senderObjects) ~= numel(senderEvidence)
    error('ReceiverSafeOptions:InvalidSenders', ...
        'Sender objects and evidence must be matching cell arrays.');
end
senderCount = numel(senderObjects);
maximumEnumeratedSenders = getField(options, ...
    'maximumEnumeratedSenders', 8);
if senderCount > maximumEnumeratedSenders
    error('ReceiverSafeOptions:TooManySenders', ...
        'The bounded-degree exact teacher supports at most %d senders.', ...
        maximumEnumeratedSenders);
end

safeSenderMask = false(1, senderCount);
for senderIdx = 1:senderCount
    evidence = senderEvidence{senderIdx};
    safeSenderMask(senderIdx) = ~isempty(senderObjects{senderIdx}) && ...
        isstruct(evidence) && isscalar(evidence) && ...
        isfield(evidence, 'isAdmissibleToSafeReference') && ...
        logical(evidence.isAdmissibleToSafeReference);
end
safeSenderIndices = find(safeSenderMask);

spatialWeights = resolveSourceWeights( ...
    options, 'spatialWeights', senderCount + 1);
existenceWeights = resolveSourceWeights( ...
    options, 'existenceWeights', senderCount + 1);
referenceObject = fuseSubset(receiverObject, senderObjects, ...
    safeSenderMask, spatialWeights, existenceWeights, model, triggerConfig);

supportedExistenceThreshold = getUnitOption( ...
    options, 'supportedExistenceThreshold', 0.50);
existenceRetentionFraction = getUnitOption( ...
    options, 'existenceRetentionFraction', 0.95);
labelMetadataScalars = getField(options, 'labelMetadataScalars', 4);
if ~isscalar(labelMetadataScalars) || ...
        ~isfinite(labelMetadataScalars) || ...
        labelMetadataScalars < 0 || ...
        labelMetadataScalars ~= round(labelMetadataScalars)
    error('ReceiverSafeOptions:InvalidMetadataCost', ...
        'labelMetadataScalars must be a nonnegative integer.');
end

safeCount = numel(safeSenderIndices);
optionCount = 2^safeCount;
template = makeOption(senderCount);
labelOptions = repmat(template, 1, optionCount);
for optionIdx = 1:optionCount
    localSelection = logical(bitget(optionIdx - 1, 1:safeCount));
    senderSubset = false(1, senderCount);
    senderSubset(safeSenderIndices(localSelection)) = true;
    candidateObject = fuseSubset(receiverObject, senderObjects, ...
        senderSubset, spatialWeights, existenceWeights, model, triggerConfig);
    [spatialKld, spatialDetails] = ...
        approximateLmbSpatialKldCubature(referenceObject, candidateObject);
    terms = computeLmbBernoulliKldTerms( ...
        referenceObject.r, candidateObject.r, spatialKld);
    isSupported = referenceObject.r >= supportedExistenceThreshold;
    retentionFloor = existenceRetentionFraction * referenceObject.r;
    retentionSatisfied = ~isSupported || ...
        candidateObject.r + 1e-12 >= retentionFloor;

    labelOptions(optionIdx).distortion = terms.total;
    labelOptions(optionIdx).variablePayloadBytes = ...
        selectedLabelPayloadBytes( ...
            senderObjects, senderSubset, model.xDimension, ...
            labelMetadataScalars);
    labelOptions(optionIdx).activeEdgeMask = senderSubset;
    labelOptions(optionIdx).existenceRetentionSatisfied = ...
        retentionSatisfied;
    labelOptions(optionIdx).senderSubset = senderSubset;
    labelOptions(optionIdx).selectedSenderCount = sum(senderSubset);
    labelOptions(optionIdx).referenceExistence = referenceObject.r;
    labelOptions(optionIdx).candidateExistence = candidateObject.r;
    labelOptions(optionIdx).existenceRetentionFloor = retentionFloor;
    labelOptions(optionIdx).spatialKld = spatialKld;
    labelOptions(optionIdx).spatialKldDetails = spatialDetails;
    labelOptions(optionIdx).bernoulliKldTerms = terms;
    labelOptions(optionIdx).candidateObject = candidateObject;
end

result = struct();
result.contractVersion = 'receiver-safe-label-subset-options-v1';
result.referenceObject = referenceObject;
result.safeSenderMask = safeSenderMask;
result.excludedSenderMask = ~safeSenderMask;
result.labelOptions = labelOptions;
result.labelMetadataScalars = labelMetadataScalars;
result.spatialMetricIsExactGmKld = false;
result.receiverApproximation = 'installed-powered-gm-lmb-kla';
end

function fusedObject = fuseSubset(receiverObject, senderObjects, ...
        senderSubset, spatialWeights, existenceWeights, model, triggerConfig)
selectedSenders = find(senderSubset);
if isempty(selectedSenders)
    fusedObject = receiverObject;
    return;
end
distributions = [{receiverObject}, senderObjects(selectedSenders)];
sourceIndices = [1, selectedSenders + 1];
fusionDetails = struct('eventType', [0, 2 * ones(1, numel(selectedSenders))]);
fused = fuseLmbPosteriorsByLabel( ...
    distributions, spatialWeights(sourceIndices), model, ...
    existenceWeights(sourceIndices), fusionDetails, triggerConfig);
if numel(fused) ~= 1
    error('ReceiverSafeOptions:UnexpectedFusionOutput', ...
        'Single-label subset fusion must return exactly one label.');
end
fusedObject = fused(1);
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

function weights = resolveSourceWeights(options, fieldName, sourceCount)
weights = getField(options, fieldName, ones(1, sourceCount));
weights = reshape(weights, 1, []);
if numel(weights) ~= sourceCount || ...
        any(~isfinite(weights)) || any(weights < 0) || sum(weights) <= 0
    error('ReceiverSafeOptions:InvalidSourceWeights', ...
        '%s must contain nonnegative weights for every source.', fieldName);
end
end

function value = getUnitOption(options, fieldName, defaultValue)
value = getField(options, fieldName, defaultValue);
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('ReceiverSafeOptions:InvalidUnitOption', ...
        '%s must be a finite scalar in [0, 1].', fieldName);
end
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
    'existenceRetentionFloor', NaN, ...
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
