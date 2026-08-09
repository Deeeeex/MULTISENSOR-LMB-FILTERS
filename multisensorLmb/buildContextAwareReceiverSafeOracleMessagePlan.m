function plan = buildContextAwareReceiverSafeOracleMessagePlan( ...
        localPosteriorBySensor, labelEvidenceBySensor, ...
        selectiveEdgeMask, currentDirectedEdgeMask, topologyWeights, ...
        updateDiagnostics, model, triggerConfig)
% BUILDCONTEXTAWARERECEIVERSAFEORACLEMESSAGEPLAN Offline V55 teacher.
%
% The byte budget and selectable edges are inherited from V54.  Every label
% option is evaluated together with the fixed incoming V46 sources that are
% common to all options for the receiver.

sensorCount = numel(localPosteriorBySensor);
if ~iscell(localPosteriorBySensor) || ...
        ~iscell(labelEvidenceBySensor) || ...
        numel(labelEvidenceBySensor) ~= sensorCount || ...
        ~isequal(size(selectiveEdgeMask), [sensorCount, sensorCount]) || ...
        ~isequal(size(currentDirectedEdgeMask), ...
            [sensorCount, sensorCount]) || ...
        ~isequal(size(topologyWeights), [sensorCount, sensorCount]) || ...
        ~iscell(updateDiagnostics) || ...
        numel(updateDiagnostics) ~= sensorCount
    error('ContextAwareReceiverSafeOracle:InvalidInput', ...
        'Posterior, evidence, topology, and diagnostic inputs disagree.');
end
selectiveEdgeMask = logical(selectiveEdgeMask);
currentDirectedEdgeMask = logical(currentDirectedEdgeMask);
selectiveEdgeMask(1:sensorCount+1:end) = false;
currentDirectedEdgeMask(1:sensorCount+1:end) = false;
if any(selectiveEdgeMask(:) & ~currentDirectedEdgeMask(:))
    error('ContextAwareReceiverSafeOracle:InactiveSelectiveEdge', ...
        'Every selectable edge must be active in the V46 route.');
end

plan = struct();
plan.contractVersion = ...
    'context-aware-receiver-safe-oracle-message-plan-v2';
plan.mode = 'offline-full-posterior-context-aware-teacher';
plan.selectiveEdgeMask = selectiveEdgeMask;
plan.currentDirectedEdgeMask = currentDirectedEdgeMask;
plan.payloadByReceiverSender = cell(sensorCount);
plan.synopsisByReceiverSender = cell(sensorCount);
plan.receiverDetails = repmat(makeReceiverDetails(), 1, sensorCount);
plan.activeLabelCountBySender = zeros(1, sensorCount);
plan.usesFullSenderPosteriors = true;
plan.isOnlineDeployable = false;
plan.fixedFusionContextIncluded = true;
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
    for receiverIdx = find(selectiveEdgeMask(senderIdx, :))
        plan.synopsisByReceiverSender{receiverIdx, senderIdx} = synopsis;
        plan.evidenceLabelInputCounts = accumulateEvidenceCounts( ...
            plan.evidenceLabelInputCounts, ...
            localPosteriorBySensor{senderIdx}, ...
            labelEvidenceBySensor{senderIdx}, ...
            triggerConfig.payloadExistenceThreshold);
    end
end

for receiverIdx = find(any(selectiveEdgeMask, 1))
    selectableSenders = reshape( ...
        find(selectiveEdgeMask(:, receiverIdx)), 1, []);
    fixedSenders = reshape(find( ...
        currentDirectedEdgeMask(:, receiverIdx) & ...
        ~selectiveEdgeMask(:, receiverIdx)), 1, []);
    plan.receiverDetails(receiverIdx) = buildReceiverPlan( ...
        receiverIdx, fixedSenders, selectableSenders, ...
        localPosteriorBySensor, labelEvidenceBySensor, ...
        plan.synopsisByReceiverSender, topologyWeights, ...
        updateDiagnostics, model, triggerConfig);
    details = plan.receiverDetails(receiverIdx);
    for localSenderIdx = 1:numel(selectableSenders)
        senderIdx = selectableSenders(localSenderIdx);
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
plan.totalFixedContextSourceCount = ...
    sum([details.fixedContextSourceCount]);
plan.totalProjectedReferenceLabelCount = ...
    sum([details.projectedReferenceLabelCount]);
plan.totalProjectedCandidateOptionCount = ...
    sum([details.projectedCandidateOptionCount]);
plan.byteBudgetSatisfied = all([details.byteBudgetSatisfied]);
if ~plan.byteBudgetSatisfied
    error('ContextAwareReceiverSafeOracle:ByteBudgetExceeded', ...
        'Synopsis plus selected GM labels exceed the paired V46 budget.');
end
end

function details = buildReceiverPlan( ...
        receiverIdx, fixedSenders, selectableSenders, posteriors, ...
        evidenceBySensor, synopsisByReceiverSender, topologyWeights, ...
        updateDiagnostics, model, triggerConfig)
selectableCount = numel(selectableSenders);
receiverObjects = activeObjects( ...
    posteriors{receiverIdx}, triggerConfig.payloadExistenceThreshold);
fixedObjects = cell(1, numel(fixedSenders));
for fixedIdx = 1:numel(fixedSenders)
    fixedObjects{fixedIdx} = activeObjects( ...
        posteriors{fixedSenders(fixedIdx)}, ...
        triggerConfig.payloadExistenceThreshold);
end
selectableObjects = cell(1, selectableCount);
selectableEvidence = cell(1, selectableCount);
baselineFullBytes = zeros(1, selectableCount);
controlSynopsisBytes = zeros(1, selectableCount);
edgeHeaderBytes = zeros(1, selectableCount);
for localSenderIdx = 1:selectableCount
    senderIdx = selectableSenders(localSenderIdx);
    selectableObjects{localSenderIdx} = activeObjects( ...
        posteriors{senderIdx}, triggerConfig.payloadExistenceThreshold);
    selectableEvidence{localSenderIdx} = evidenceBySensor{senderIdx};
    fullStats = estimateLmbPayloadSize( ...
        selectableObjects{localSenderIdx}, model, 2, ...
        updateDiagnostics{senderIdx});
    baselineFullBytes(localSenderIdx) = fullStats.estimatedBytes;
    synopsis = synopsisByReceiverSender{receiverIdx, senderIdx};
    controlSynopsisBytes(localSenderIdx) = synopsis.estimatedBytes;
    labelBytes = collectionLabelBytes( ...
        selectableObjects{localSenderIdx}, model.xDimension);
    edgeHeaderBytes(localSenderIdx) = max( ...
        fullStats.estimatedBytes - labelBytes, 0);
end

labels = collectAllLabels( ...
    receiverObjects, fixedObjects, selectableObjects);
optionSets = cell(1, size(labels, 2));
optionMetadata = cell(1, size(labels, 2));
sourceWeights = [topologyWeights(receiverIdx, receiverIdx), ...
    topologyWeights(receiverIdx, fixedSenders), ...
    topologyWeights(receiverIdx, selectableSenders)];
projectedReferenceLabelCount = 0;
projectedCandidateOptionCount = 0;
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    receiverObject = findObject(receiverObjects, label);
    fixedForLabel = cell(1, numel(fixedSenders));
    for fixedIdx = 1:numel(fixedSenders)
        fixedForLabel{fixedIdx} = findObject( ...
            fixedObjects{fixedIdx}, label);
    end
    selectableForLabel = cell(1, selectableCount);
    evidenceForLabel = cell(1, selectableCount);
    for localSenderIdx = 1:selectableCount
        selectableForLabel{localSenderIdx} = findObject( ...
            selectableObjects{localSenderIdx}, label);
        evidenceForLabel{localSenderIdx} = findEvidence( ...
            selectableEvidence{localSenderIdx}, label);
    end

    options = struct();
    options.maximumEnumeratedSenders = getField(triggerConfig, ...
        'receiverSafeMaximumEnumeratedSenders', 8);
    options.supportedExistenceThreshold = getField(triggerConfig, ...
        'receiverSafeSupportedExistenceThreshold', 0.50);
    options.maximumReceiverLogOddsDrop = getField(triggerConfig, ...
        'receiverSafeMaximumLogOddsDrop', log(4));
    options.existenceProjectionEnabled = strcmpi(getField( ...
        triggerConfig, 'receiverSafeLabelFusionMode', ...
        'context-aware-combined'), 'context-aware-combined');
    receiverEvidence = findEvidence( ...
        evidenceBySensor{receiverIdx}, label);
    options.receiverHasPositiveSupport = ...
        ~isempty(receiverObject) && strcmpi( ...
            getEvidenceType(receiverEvidence), 'positive-support');
    options.spatialWeights = sourceWeights;
    options.existenceWeights = sourceWeights;
    metadata = buildContextAwareReceiverSafeLabelSubsetOptions( ...
        receiverObject, fixedForLabel, selectableForLabel, ...
        evidenceForLabel, model, triggerConfig, options);
    optionSets{labelIdx} = metadata.labelOptions;
    optionMetadata{labelIdx} = metadata;
    projectedReferenceLabelCount = projectedReferenceLabelCount + ...
        double(metadata.referenceWasProjected);
    projectedCandidateOptionCount = projectedCandidateOptionCount + ...
        sum([metadata.labelOptions.existenceWasProjected]);
end

budgetFraction = getField( ...
    triggerConfig, 'receiverSafeOracleByteBudgetFraction', 1);
byteBudget = budgetFraction * sum(baselineFullBytes);
selectorOptions = struct();
selectorOptions.edgeCount = selectableCount;
selectorOptions.edgeHeaderBytes = edgeHeaderBytes;
selectorOptions.initialCostBytes = sum(controlSynopsisBytes);
selectorOptions.byteBudget = byteBudget;
selection = selectReceiverSafeLabelOptionsExact( ...
    optionSets, selectorOptions);
if ~selection.isFeasible
    error('ContextAwareReceiverSafeOracle:NoFeasibleSelection', ...
        ['Receiver %d has no context-aware feasible selection after ', ...
         'paying the control synopsis.'], receiverIdx);
end

payloadByLocalSender = cell(1, selectableCount);
for localSenderIdx = 1:selectableCount
    payloadByLocalSender{localSenderIdx} = ...
        selectableObjects{localSenderIdx}([]);
end
selectedLabelCount = 0;
for labelIdx = 1:size(labels, 2)
    selectedOption = optionSets{labelIdx}( ...
        selection.selectedOptionIndices(labelIdx));
    for localSenderIdx = find(selectedOption.activeEdgeMask)
        object = findObject( ...
            selectableObjects{localSenderIdx}, labels(:, labelIdx));
        if ~isempty(object)
            payloadByLocalSender{localSenderIdx}(end+1) = object; ...
                %#ok<AGROW>
            selectedLabelCount = selectedLabelCount + 1;
        end
    end
end

selectedPayloadBytes = zeros(1, selectableCount);
for localSenderIdx = 1:selectableCount
    stats = estimateLmbPayloadSize( ...
        payloadByLocalSender{localSenderIdx}, model, 2, ...
        updateDiagnostics{selectableSenders(localSenderIdx)});
    selectedPayloadBytes(localSenderIdx) = stats.estimatedBytes;
end
totalAttemptedBytes = sum(controlSynopsisBytes) + ...
    sum(selectedPayloadBytes);

details = makeReceiverDetails();
details.receiverIdx = receiverIdx;
details.fixedSenders = fixedSenders;
details.senders = selectableSenders;
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
details.candidateLabelCount = size(labels, 2) * selectableCount;
details.fixedContextSourceCount = numel(fixedSenders);
details.projectedReferenceLabelCount = projectedReferenceLabelCount;
details.projectedCandidateOptionCount = projectedCandidateOptionCount;
details.byteBudget = byteBudget;
details.byteBudgetSatisfied = totalAttemptedBytes <= byteBudget + 1e-9;
end

function counts = accumulateEvidenceCounts( ...
        counts, objects, evidenceArray, threshold)
objects = activeObjects(objects, threshold);
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; ...
        objects(objectIdx).birthLocation];
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

function objects = activeObjects(objects, threshold)
if isempty(objects)
    return;
end
keep = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
objects = objects(keep);
end

function labels = collectAllLabels( ...
        receiverObjects, fixedObjects, selectableObjects)
labels = zeros(2, 0);
labels = appendLabels(labels, receiverObjects);
for sourceIdx = 1:numel(fixedObjects)
    labels = appendLabels(labels, fixedObjects{sourceIdx});
end
for sourceIdx = 1:numel(selectableObjects)
    labels = appendLabels(labels, selectableObjects{sourceIdx});
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function labels = appendLabels(labels, objects)
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; ...
        objects(objectIdx).birthLocation];
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
    bytes = bytes + objectLabelBytes( ...
        objects(objectIdx), stateDimension);
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
    'fixedSenders', [], ...
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
    'fixedContextSourceCount', 0, ...
    'projectedReferenceLabelCount', 0, ...
    'projectedCandidateOptionCount', 0, ...
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
