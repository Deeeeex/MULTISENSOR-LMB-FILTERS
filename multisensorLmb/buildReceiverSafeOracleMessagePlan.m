function plan = buildReceiverSafeOracleMessagePlan( ...
        localPosteriorBySensor, labelEvidenceBySensor, selectiveEdgeMask, ...
        topologyWeights, updateDiagnostics, model, triggerConfig)
% BUILDRECEIVERSAFEORACLEMESSAGEPLAN Offline V54 label-payload teacher.
%
% The V46 route remains fixed.  On selected cross-formation residual edges,
% this teacher pays the observable synopsis cost and then chooses full-GM
% labels under the bytes that the corresponding V46 full messages would
% have consumed.  Full sender posteriors are used only to measure oracle
% headroom; this function is not an online policy.

sensorCount = numel(localPosteriorBySensor);
if ~iscell(localPosteriorBySensor) || ...
        ~iscell(labelEvidenceBySensor) || ...
        numel(labelEvidenceBySensor) ~= sensorCount || ...
        ~isequal(size(selectiveEdgeMask), [sensorCount, sensorCount]) || ...
        ~isequal(size(topologyWeights), [sensorCount, sensorCount]) || ...
        ~iscell(updateDiagnostics) || numel(updateDiagnostics) ~= sensorCount
    error('ReceiverSafeOracle:InvalidInput', ...
        'Posterior, evidence, edge, weight, and diagnostic inputs disagree.');
end
selectiveEdgeMask = logical(selectiveEdgeMask);
selectiveEdgeMask(1:sensorCount+1:end) = false;

plan = struct();
plan.contractVersion = 'receiver-safe-oracle-message-plan-v1';
plan.mode = 'offline-full-posterior-oracle';
plan.selectiveEdgeMask = selectiveEdgeMask;
plan.payloadByReceiverSender = cell(sensorCount);
plan.synopsisByReceiverSender = cell(sensorCount);
plan.receiverDetails = repmat(makeReceiverDetails(), 1, sensorCount);
plan.activeLabelCountBySender = zeros(1, sensorCount);
plan.usesFullSenderPosteriors = true;
plan.isOnlineDeployable = false;
plan.evidenceLabelInputCounts = struct( ...
    'positiveSupport', 0, ...
    'credibleNegative', 0, ...
    'unsupportedAbsence', 0, ...
    'ambiguous', 0, ...
    'missing', 0);

synopsisOptions = getField( ...
    triggerConfig, 'receiverSafeSynopsisOptions', struct());
for senderIdx = find(any(selectiveEdgeMask, 2))'
    synopsis = buildLmbLabelControlSynopsis( ...
        localPosteriorBySensor{senderIdx}, ...
        labelEvidenceBySensor{senderIdx}, model, synopsisOptions);
    plan.activeLabelCountBySender(senderIdx) = synopsis.labelCount;
    receivers = find(selectiveEdgeMask(senderIdx, :));
    for receiverIdx = receivers
        plan.synopsisByReceiverSender{receiverIdx, senderIdx} = synopsis;
        plan.evidenceLabelInputCounts = accumulateEvidenceCounts( ...
            plan.evidenceLabelInputCounts, ...
            localPosteriorBySensor{senderIdx}, ...
            labelEvidenceBySensor{senderIdx}, ...
            triggerConfig.payloadExistenceThreshold);
    end
end

for receiverIdx = find(any(selectiveEdgeMask, 1))
    senders = reshape(find(selectiveEdgeMask(:, receiverIdx)), 1, []);
    plan.receiverDetails(receiverIdx) = buildReceiverPlan( ...
        receiverIdx, senders, localPosteriorBySensor, ...
        labelEvidenceBySensor, plan.synopsisByReceiverSender, ...
        topologyWeights, updateDiagnostics, model, triggerConfig);
    details = plan.receiverDetails(receiverIdx);
    for localSenderIdx = 1:numel(senders)
        senderIdx = senders(localSenderIdx);
        plan.payloadByReceiverSender{receiverIdx, senderIdx} = ...
            details.payloadByLocalSender{localSenderIdx};
    end
end

details = plan.receiverDetails;
plan.totalBaselineFullBytes = sum([details.baselineFullBytes]);
plan.totalControlSynopsisBytes = sum([details.controlSynopsisBytes]);
plan.totalSelectedPayloadBytes = sum([details.selectedPayloadBytes]);
plan.totalAttemptedBytes = sum([details.totalAttemptedBytes]);
plan.totalSelectedLabelCount = sum([details.selectedLabelCount]);
plan.totalCandidateLabelCount = sum([details.candidateLabelCount]);
plan.byteBudgetSatisfied = all([details.byteBudgetSatisfied]);
if ~plan.byteBudgetSatisfied
    error('ReceiverSafeOracle:ByteBudgetExceeded', ...
        'The selected synopsis plus GM labels exceed the V46 byte budget.');
end
end

function counts = accumulateEvidenceCounts( ...
        counts, objects, evidenceArray, threshold)
objects = activeObjects(objects, threshold);
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; objects(objectIdx).birthLocation];
    evidence = findEvidence(evidenceArray, label);
    switch lower(getEvidenceType(evidence))
        case 'positive-support'
            counts.positiveSupport = counts.positiveSupport + 1;
        case 'credible-negative'
            counts.credibleNegative = counts.credibleNegative + 1;
        case 'unsupported-absence'
            counts.unsupportedAbsence = counts.unsupportedAbsence + 1;
        case 'ambiguous'
            counts.ambiguous = counts.ambiguous + 1;
        otherwise
            counts.missing = counts.missing + 1;
    end
end
end

function details = buildReceiverPlan( ...
        receiverIdx, senders, posteriors, evidenceBySensor, ...
        synopsisByReceiverSender, topologyWeights, updateDiagnostics, ...
        model, triggerConfig)
senderCount = numel(senders);
receiverObjects = activeObjects( ...
    posteriors{receiverIdx}, triggerConfig.payloadExistenceThreshold);
senderObjects = cell(1, senderCount);
senderEvidence = cell(1, senderCount);
baselineFullBytes = zeros(1, senderCount);
controlSynopsisBytes = zeros(1, senderCount);
edgeHeaderBytes = zeros(1, senderCount);
for localSenderIdx = 1:senderCount
    senderIdx = senders(localSenderIdx);
    senderObjects{localSenderIdx} = activeObjects( ...
        posteriors{senderIdx}, triggerConfig.payloadExistenceThreshold);
    senderEvidence{localSenderIdx} = evidenceBySensor{senderIdx};
    fullStats = estimateLmbPayloadSize( ...
        senderObjects{localSenderIdx}, model, 2, ...
        updateDiagnostics{senderIdx});
    baselineFullBytes(localSenderIdx) = fullStats.estimatedBytes;
    synopsis = synopsisByReceiverSender{receiverIdx, senderIdx};
    controlSynopsisBytes(localSenderIdx) = synopsis.estimatedBytes;
    labelBytes = collectionLabelBytes( ...
        senderObjects{localSenderIdx}, model.xDimension);
    edgeHeaderBytes(localSenderIdx) = max( ...
        fullStats.estimatedBytes - labelBytes, 0);
end

labels = collectLabels(receiverObjects, senderObjects);
optionSets = cell(1, size(labels, 2));
optionMetadata = cell(1, size(labels, 2));
sourceWeights = [topologyWeights(receiverIdx, receiverIdx), ...
    topologyWeights(receiverIdx, senders)];
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    receiverObject = findObject(receiverObjects, label);
    objectsForLabel = cell(1, senderCount);
    evidenceForLabel = cell(1, senderCount);
    for localSenderIdx = 1:senderCount
        objectsForLabel{localSenderIdx} = findObject( ...
            senderObjects{localSenderIdx}, label);
        evidenceForLabel{localSenderIdx} = findEvidence( ...
            senderEvidence{localSenderIdx}, label);
    end
    if isempty(receiverObject)
        [optionSets{labelIdx}, optionMetadata{labelIdx}] = ...
            buildReceiverMissingLabelOption( ...
                objectsForLabel, evidenceForLabel, model.xDimension);
    else
        options = struct();
        options.maximumEnumeratedSenders = getField(triggerConfig, ...
            'receiverSafeMaximumEnumeratedSenders', 8);
        options.supportedExistenceThreshold = getField(triggerConfig, ...
            'receiverSafeSupportedExistenceThreshold', 0.50);
        options.maximumReceiverLogOddsDrop = getField(triggerConfig, ...
            'receiverSafeMaximumLogOddsDrop', log(4));
        receiverEvidence = findEvidence( ...
            evidenceBySensor{receiverIdx}, label);
        options.receiverHasPositiveSupport = strcmpi( ...
            getEvidenceType(receiverEvidence), 'positive-support');
        options.spatialWeights = sourceWeights;
        options.existenceWeights = sourceWeights;
        result = buildReceiverSafeLabelSubsetOptions( ...
            receiverObject, objectsForLabel, evidenceForLabel, ...
            model, triggerConfig, options);
        optionSets{labelIdx} = result.labelOptions;
        optionMetadata{labelIdx} = result;
    end
end

budgetFraction = getField( ...
    triggerConfig, 'receiverSafeOracleByteBudgetFraction', 1);
byteBudget = budgetFraction * sum(baselineFullBytes);
selectorOptions = struct();
selectorOptions.edgeCount = senderCount;
selectorOptions.edgeHeaderBytes = edgeHeaderBytes;
selectorOptions.initialCostBytes = sum(controlSynopsisBytes);
selectorOptions.byteBudget = byteBudget;
selection = selectReceiverSafeLabelOptionsExact( ...
    optionSets, selectorOptions);
if ~selection.isFeasible
    error('ReceiverSafeOracle:NoFeasibleSelection', ...
        ['Receiver %d has no feasible label selection after paying ', ...
         'the control synopsis.'], receiverIdx);
end

payloadByLocalSender = cell(1, senderCount);
for localSenderIdx = 1:senderCount
    payloadByLocalSender{localSenderIdx} = ...
        senderObjects{localSenderIdx}([]);
end
selectedLabelCount = 0;
for labelIdx = 1:size(labels, 2)
    selectedOption = optionSets{labelIdx}( ...
        selection.selectedOptionIndices(labelIdx));
    for localSenderIdx = find(selectedOption.activeEdgeMask)
        object = findObject(senderObjects{localSenderIdx}, labels(:, labelIdx));
        if ~isempty(object)
            payloadByLocalSender{localSenderIdx}(end+1) = object; %#ok<AGROW>
            selectedLabelCount = selectedLabelCount + 1;
        end
    end
end

selectedPayloadBytes = zeros(1, senderCount);
for localSenderIdx = 1:senderCount
    stats = estimateLmbPayloadSize( ...
        payloadByLocalSender{localSenderIdx}, model, 2, ...
        updateDiagnostics{senders(localSenderIdx)});
    selectedPayloadBytes(localSenderIdx) = stats.estimatedBytes;
end
totalAttemptedBytes = sum(controlSynopsisBytes) + ...
    sum(selectedPayloadBytes);

details = makeReceiverDetails();
details.receiverIdx = receiverIdx;
details.senders = senders;
details.labels = labels;
details.optionMetadata = optionMetadata;
details.selection = selection;
details.payloadByLocalSender = payloadByLocalSender;
details.baselineFullBytesBySender = baselineFullBytes;
details.controlSynopsisBytesBySender = controlSynopsisBytes;
details.selectedPayloadBytesBySender = selectedPayloadBytes;
details.baselineFullBytes = sum(baselineFullBytes);
details.controlSynopsisBytes = sum(controlSynopsisBytes);
details.selectedPayloadBytes = sum(selectedPayloadBytes);
details.totalAttemptedBytes = totalAttemptedBytes;
details.selectedLabelCount = selectedLabelCount;
details.candidateLabelCount = size(labels, 2) * senderCount;
details.byteBudget = byteBudget;
details.byteBudgetSatisfied = totalAttemptedBytes <= byteBudget + 1e-9;
end

function [options, metadata] = buildReceiverMissingLabelOption( ...
        senderObjects, senderEvidence, stateDimension)
senderCount = numel(senderObjects);
mandatoryMask = false(1, senderCount);
for senderIdx = 1:senderCount
    mandatoryMask(senderIdx) = ~isempty(senderObjects{senderIdx}) && ...
        strcmpi(getEvidenceType(senderEvidence{senderIdx}), ...
            'positive-support');
end
option = struct( ...
    'distortion', 0, ...
    'variablePayloadBytes', collectionSubsetLabelBytes( ...
        senderObjects, mandatoryMask, stateDimension), ...
    'activeEdgeMask', mandatoryMask, ...
    'existenceRetentionSatisfied', true, ...
    'senderSubset', mandatoryMask, ...
    'selectedSenderCount', sum(mandatoryMask), ...
    'referenceExistence', NaN, ...
    'candidateExistence', NaN, ...
    'existenceRetentionFloor', NaN, ...
    'spatialKld', 0, ...
    'spatialKldDetails', struct(), ...
    'bernoulliKldTerms', struct(), ...
    'candidateObject', struct());
options = option;
metadata = struct( ...
    'contractVersion', 'receiver-missing-positive-label-rule-v1', ...
    'mandatoryPositiveSenderMask', mandatoryMask, ...
    'credibleNegativePropagatedWithoutReceiverLabel', false);
end

function objects = activeObjects(objects, threshold)
if isempty(objects)
    return;
end
keep = [objects.r] > threshold & [objects.numberOfGmComponents] > 0;
objects = objects(keep);
end

function labels = collectLabels(receiverObjects, senderObjects)
labels = zeros(2, 0);
labels = appendLabels(labels, receiverObjects);
for senderIdx = 1:numel(senderObjects)
    labels = appendLabels(labels, senderObjects{senderIdx});
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function labels = appendLabels(labels, objects)
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; objects(objectIdx).birthLocation];
    if isempty(labels) || ~any(all(labels == label, 1))
        labels(:, end+1) = label; %#ok<AGROW>
    end
end
end

function object = findObject(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
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

function bytes = collectionLabelBytes(objects, stateDimension)
bytes = 0;
for objectIdx = 1:numel(objects)
    bytes = bytes + objectLabelBytes(objects(objectIdx), stateDimension);
end
end

function bytes = collectionSubsetLabelBytes( ...
        senderObjects, mask, stateDimension)
bytes = 0;
for senderIdx = find(mask)
    bytes = bytes + objectLabelBytes( ...
        senderObjects{senderIdx}, stateDimension);
end
end

function bytes = objectLabelBytes(object, stateDimension)
scalarCount = 3 + object.numberOfGmComponents * ...
    (1 + stateDimension + stateDimension * stateDimension);
bytes = 8 * scalarCount;
end

function details = makeReceiverDetails()
details = struct( ...
    'receiverIdx', 0, ...
    'senders', [], ...
    'labels', zeros(2, 0), ...
    'optionMetadata', {{}}, ...
    'selection', struct(), ...
    'payloadByLocalSender', {{}}, ...
    'baselineFullBytesBySender', [], ...
    'controlSynopsisBytesBySender', [], ...
    'selectedPayloadBytesBySender', [], ...
    'baselineFullBytes', 0, ...
    'controlSynopsisBytes', 0, ...
    'selectedPayloadBytes', 0, ...
    'totalAttemptedBytes', 0, ...
    'selectedLabelCount', 0, ...
    'candidateLabelCount', 0, ...
    'byteBudget', 0, ...
    'byteBudgetSatisfied', true);
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
