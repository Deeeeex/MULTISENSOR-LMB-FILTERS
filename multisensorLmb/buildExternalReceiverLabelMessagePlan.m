function plan = buildExternalReceiverLabelMessagePlan( ...
        localPosteriorBySensor, selectiveEdgeMask, updateDiagnostics, ...
        model, triggerConfig)
% BUILDEXTERNALRECEIVERLABELMESSAGEPLAN Execute a frozen V61 label mask.
%
% The externally supplied plan is built before future tracking outcomes are
% scored.  It filters complete sender Bernoulli mixtures.  In the explicit
% omission mode, every dropped local label is named in a charged control
% envelope so the receiver can distinguish deliberate omission from natural
% label absence.

sensorCount = numel(localPosteriorBySensor);
dropPlan = getField(triggerConfig, ...
    'receiverSafeExternalDropLabelsByReceiverSender', cell(sensorCount));
if ~iscell(localPosteriorBySensor) || ...
        ~isequal(size(selectiveEdgeMask), [sensorCount, sensorCount]) || ...
        ~iscell(updateDiagnostics) || ...
        numel(updateDiagnostics) ~= sensorCount || ...
        ~iscell(dropPlan) || ...
        ~isequal(size(dropPlan), [sensorCount, sensorCount])
    error('ExternalReceiverLabelPlan:InvalidInput', ...
        'The posterior, edge mask, diagnostics, or drop plan is invalid.');
end
selectiveEdgeMask = logical(selectiveEdgeMask);
selectiveEdgeMask(1:sensorCount+1:end) = false;
activeThreshold = triggerConfig.payloadExistenceThreshold;
supportThreshold = getField(triggerConfig, ...
    'receiverSafeExternalPositiveSupportThreshold', 0.20);
explicitOmissionEnabled = logical(getField(triggerConfig, ...
    'receiverSafeExternalExplicitOmissionEnabled', false));
positiveSupportOmissionAllowed = logical(getField(triggerConfig, ...
    'receiverSafeExternalPositiveSupportOmissionAllowed', false));

plan = struct();
if explicitOmissionEnabled
    plan.contractVersion = ...
        'external-explicit-label-omission-message-plan-v1';
else
    plan.contractVersion = 'external-receiver-label-message-plan-v61-v1';
end
plan.mode = 'external-label-plan';
plan.selectiveEdgeMask = selectiveEdgeMask;
plan.abstainFromFusionEdgeMask = false(sensorCount);
plan.labelWhitelistRestrictedEdgeMask = false(sensorCount);
plan.explicitLabelOmissionRegisteredEdgeMask = false(sensorCount);
plan.explicitOmittedLabelsByReceiverSender = cell(sensorCount);
plan.fusionTrustFactorByReceiverSender = ones(sensorCount);
plan.eventTypeByReceiverSender = 2 * ones(sensorCount);
plan.payloadByReceiverSender = cell(sensorCount);
plan.synopsisByReceiverSender = cell(sensorCount);
plan.receiverDetails = repmat(makeReceiverDetails(), 1, sensorCount);
plan.activeLabelCountBySender = zeros(1, sensorCount);
plan.usesFullSenderPosteriors = false;
plan.isOnlineDeployable = false;
plan.evidenceLabelInputCounts = struct( ...
    'positiveSupport', 0, ...
    'credibleNegative', 0, ...
    'unsupportedAbsence', 0, ...
    'ambiguous', 0, ...
    'missing', 0);

for receiverIdx = reshape(find(any(selectiveEdgeMask, 1)), 1, [])
    senders = reshape(find(selectiveEdgeMask(:, receiverIdx)), 1, []);
    details = makeReceiverDetails();
    details.receiverIdx = receiverIdx;
    details.senders = senders;
    for localSenderIdx = 1:numel(senders)
        senderIdx = senders(localSenderIdx);
        objects = activeObjects( ...
            localPosteriorBySensor{senderIdx}, activeThreshold);
        labelsToDrop = validateDropLabels( ...
            dropPlan{receiverIdx, senderIdx});
        if ~positiveSupportOmissionAllowed
            validateSenderSupport(objects, labelsToDrop, supportThreshold);
        end
        payload = dropLabels(objects, labelsToDrop);
        baselineStats = estimateLmbPayloadSize( ...
            objects, model, 2, updateDiagnostics{senderIdx});
        selectedStats = estimateLmbPayloadSize( ...
            payload, model, 2, updateDiagnostics{senderIdx});
        if explicitOmissionEnabled
            if isempty(labelsToDrop)
                synopsis = emptySynopsis();
            else
                synopsis = explicitOmissionEnvelopeSynopsis(labelsToDrop);
                plan.explicitLabelOmissionRegisteredEdgeMask( ...
                    senderIdx, receiverIdx) = true;
                plan.explicitOmittedLabelsByReceiverSender{ ...
                    receiverIdx, senderIdx} = labelsToDrop;
            end
        else
            synopsis = compactSynopsis(numel(objects), model.xDimension);
        end
        plan.payloadByReceiverSender{receiverIdx, senderIdx} = payload;
        plan.synopsisByReceiverSender{receiverIdx, senderIdx} = synopsis;
        plan.activeLabelCountBySender(senderIdx) = max( ...
            plan.activeLabelCountBySender(senderIdx), numel(objects));
        plan.evidenceLabelInputCounts = accumulateSupportCounts( ...
            plan.evidenceLabelInputCounts, objects, supportThreshold);
        details.baselineFullBytesBySender(localSenderIdx) = ...
            baselineStats.estimatedBytes;
        details.controlSynopsisBytesBySender(localSenderIdx) = ...
            synopsis.estimatedBytes;
        details.selectedPayloadBytesBySender(localSenderIdx) = ...
            selectedStats.estimatedBytes;
        details.candidateLabelCount = ...
            details.candidateLabelCount + numel(objects);
        details.selectedLabelCount = ...
            details.selectedLabelCount + numel(payload);
    end
    details.baselineFullBytes = sum(details.baselineFullBytesBySender);
    details.controlSynopsisBytes = sum( ...
        details.controlSynopsisBytesBySender);
    details.selectedPayloadBytes = sum( ...
        details.selectedPayloadBytesBySender);
    details.totalAttemptedBytes = ...
        details.controlSynopsisBytes + details.selectedPayloadBytes;
    details.byteBudget = details.baselineFullBytes;
    details.byteBudgetSatisfied = ...
        details.totalAttemptedBytes <= details.byteBudget + 1e-9;
    plan.receiverDetails(receiverIdx) = details;
end

details = plan.receiverDetails;
plan.totalBaselineFullBytes = sum([details.baselineFullBytes]);
plan.totalControlSynopsisBytes = sum([details.controlSynopsisBytes]);
plan.totalSelectedPayloadBytes = sum([details.selectedPayloadBytes]);
plan.totalAttemptedBytes = sum([details.totalAttemptedBytes]);
plan.totalSelectedLabelCount = sum([details.selectedLabelCount]);
plan.totalCandidateLabelCount = sum([details.candidateLabelCount]);
plan.byteBudgetSatisfied = plan.totalAttemptedBytes <= ...
    plan.totalBaselineFullBytes + 1e-9;
if ~plan.byteBudgetSatisfied
    error('ExternalReceiverLabelPlan:ByteBudgetExceeded', ...
        'The selected labels plus synopsis exceed the reference payload.');
end
end

function synopsis = emptySynopsis()
synopsis = struct( ...
    'contractVersion', 'lmb-no-separate-control-envelope-v1', ...
    'labelCount', 0, ...
    'estimatedBytes', 0, ...
    'scalarCount', 0);
end

function synopsis = explicitOmissionEnvelopeSynopsis(labels)
fixedEnvelopeBytes = 16;
labelIdentifierBytes = 8 * size(labels, 2);
estimatedBytes = fixedEnvelopeBytes + labelIdentifierBytes;
synopsis = struct( ...
    'contractVersion', 'lmb-explicit-omission-envelope-v1', ...
    'labelCount', size(labels, 2), ...
    'estimatedBytes', estimatedBytes, ...
    'scalarCount', estimatedBytes / 8);
end

function labels = validateDropLabels(labels)
if isempty(labels)
    labels = zeros(2, 0);
    return;
end
if ~isnumeric(labels) || size(labels, 1) ~= 2 || ...
        any(~isfinite(labels(:))) || ...
        size(unique(labels', 'rows'), 1) ~= size(labels, 2)
    error('ExternalReceiverLabelPlan:InvalidLabels', ...
        'Every drop list must be a unique 2-by-K label matrix.');
end
end

function validateSenderSupport(objects, labels, threshold)
for labelIdx = 1:size(labels, 2)
    object = findObject(objects, labels(:, labelIdx));
    if isempty(object)
        error('ExternalReceiverLabelPlan:MissingSenderLabel', ...
            'A requested drop label is absent from the sender payload.');
    end
    support = getField(object, 'detectionAssociationMass', 0);
    if isfinite(support) && support >= threshold - 1e-12
        error('ExternalReceiverLabelPlan:PositiveSupportDrop', ...
            'A sender label with current positive support cannot be dropped.');
    end
end
end

function objects = dropLabels(objects, labels)
keep = true(1, numel(objects));
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; ...
        objects(objectIdx).birthLocation];
    keep(objectIdx) = isempty(labels) || ...
        ~any(all(bsxfun(@eq, labels, label), 1));
end
objects = objects(keep);
end

function synopsis = compactSynopsis(labelCount, stateDimension)
positionDimension = min(2, stateDimension);
velocityDimension = stateDimension - positionDimension;
positionCovarianceScalars = ...
    positionDimension * (positionDimension + 1) / 2;
floatCount = 5 + stateDimension + ...
    positionCovarianceScalars + velocityDimension;
rawPerLabelBytes = 4 + 4 * floatCount + 2;
perLabelBytes = 4 * ceil(rawPerLabelBytes / 4);
estimatedBytes = 16 + labelCount * perLabelBytes;
synopsis = struct( ...
    'contractVersion', 'lmb-label-control-synopsis-v2-compact-cost', ...
    'labelCount', labelCount, ...
    'estimatedBytes', estimatedBytes, ...
    'scalarCount', estimatedBytes / 8);
end

function counts = accumulateSupportCounts(counts, objects, threshold)
for objectIdx = 1:numel(objects)
    support = getField(objects(objectIdx), ...
        'detectionAssociationMass', 0);
    if isscalar(support) && isfinite(support) && ...
            support >= threshold - 1e-12
        counts.positiveSupport = counts.positiveSupport + 1;
    else
        counts.ambiguous = counts.ambiguous + 1;
    end
end
end

function objects = activeObjects(objects, threshold)
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
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

function details = makeReceiverDetails()
details = struct( ...
    'receiverIdx', 0, ...
    'senders', [], ...
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

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
