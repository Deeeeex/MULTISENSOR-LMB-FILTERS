function [adjacency, fusionWeights, details] = ...
    buildFormationB4V47GatewayDebtSchedule(context, referenceDetails)
% BUILDFormationB4V47GATEWAYDEBTSCHEDULE Causal residual-budget dispatch.
%
% Every receiver keeps its dominant local input.  The exact N residual
% opportunities available over four steps are spent first on the
% cross-formation residual layer.  A deadline projection prevents a stable
% current gateway edge from aging beyond its scale-derived service window;
% remaining choices use only past ACK age, current posterior disagreement,
% current link reliability and immutable physical UIDs.

protocol = getFormationGatewayDebtV47Protocol();
validateInputs(context, referenceDetails, protocol);
nodeCount = numel(context.localPosteriorBySensor);
period = protocol.period;
phase = mod(context.currentTime - 1, period) + 1;
quotaByPhase = diff(floor((0:period) * nodeCount / period));
residualQuota = quotaByPhase(phase);

dominant = logical(referenceDetails.dominantAdjacency);
residual = logical(referenceDetails.residualAdjacency);
formationUids = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
crossMask = formationUids(:) ~= formationUids(:)';
crossResidual = residual & crossMask;
localResidual = residual & ~crossMask;
crossReceivers = find(sum(crossResidual, 2) == 1)';
localReceivers = find(sum(localResidual, 2) == 1)';

minimumQuota = min(quotaByPhase);
crossServiceHorizon = max(1, ceil( ...
    numel(crossReceivers) / max(minimumQuota, 1)));
localSlotsByPhase = max(quotaByPhase - numel(crossReceivers), 0);
positiveLocalSlots = localSlotsByPhase(localSlotsByPhase > 0);
if isempty(positiveLocalSlots) || isempty(localReceivers)
    localServiceHorizon = inf;
else
    localServiceHorizon = max(1, ceil( ...
        numel(localReceivers) / min(positiveLocalSlots)));
end

summaries = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    summaries{sensorIdx} = summarizeLmbPosteriorForDisagreement( ...
        context.localPosteriorBySensor{sensorIdx}, context.model);
end
features = buildFeatures( ...
    context, residual, summaries, crossServiceHorizon, protocol);

crossSlots = min(residualQuota, numel(crossReceivers));
selectedCrossReceivers = selectReceivers( ...
    crossReceivers, crossSlots, features, crossServiceHorizon);
localSlots = residualQuota - numel(selectedCrossReceivers);
selectedLocalReceivers = selectReceivers( ...
    localReceivers, localSlots, features, localServiceHorizon);
selectedReceivers = [selectedCrossReceivers, selectedLocalReceivers];
selectedResidual = false(nodeCount);
for receiverIdx = selectedReceivers
    selectedResidual(receiverIdx, :) = residual(receiverIdx, :);
end

crossDutyRate = classDutyRate( ...
    numel(crossReceivers), ...
    arrayfun(@(quota) min(quota, numel(crossReceivers)), ...
        quotaByPhase), period);
localDutyRate = classDutyRate( ...
    numel(localReceivers), localSlotsByPhase, period);
crossWeight = activeClassWeight(crossDutyRate, protocol);
localWeight = activeClassWeight(localDutyRate, protocol);
fusionWeights = zeros(nodeCount);
fusionWeights(dominant) = protocol.dominantWeight;
fusionWeights(selectedResidual & crossMask) = crossWeight;
fusionWeights(selectedResidual & ~crossMask) = localWeight;
fusionWeights(1:nodeCount+1:end) = ...
    1 - sum(fusionWeights, 2)';
adjacency = dominant | selectedResidual;

[rollingMature, rollingFormationStrong, rollingUnion] = ...
    evaluateRollingCrossService( ...
        context, adjacency, formationUids, crossServiceHorizon);
fallbackUsed = rollingMature && ~rollingFormationStrong;
if fallbackUsed
    adjacency = logical(referenceDetails.referenceAdjacency);
    fusionWeights = referenceDetails.referenceFusionWeights;
end

referenceAdjacency = logical(referenceDetails.referenceAdjacency);
if any(adjacency(:) & ~logical(context.physicalAdjacency(:))) || ...
        any(adjacency(:) & ~referenceAdjacency(:)) || ...
        any(diag(adjacency)) || ...
        any(~isfinite(fusionWeights(:))) || ...
        any(fusionWeights(:) < -1e-12) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        any(any((fusionWeights > 1e-12 & ...
            ~logical(eye(nodeCount))) & ~adjacency)) || ...
        (~fallbackUsed && nnz(adjacency) ~= nodeCount + residualQuota)
    error('FormationGatewayDebtV47:InvalidSchedule', ...
        'The selected V47 residual schedule violates a hard invariant.');
end

payload = struct();
payload.contractVersion = ...
    'formation-b4-v47-gateway-debt-schedule-v1';
payload.protocolId = protocol.id;
payload.protocolCanonicalSha256 = protocol.canonicalSha256;
payload.currentTime = context.currentTime;
payload.period = period;
payload.currentAbsolutePhase = phase;
payload.quotaByPhase = quotaByPhase;
payload.residualQuota = residualQuota;
payload.crossResidualCount = numel(crossReceivers);
payload.localResidualCount = numel(localReceivers);
payload.crossServiceHorizon = crossServiceHorizon;
payload.localServiceHorizon = localServiceHorizon;
payload.selectedCrossReceivers = selectedCrossReceivers;
payload.selectedLocalReceivers = selectedLocalReceivers;
payload.selectedReceiverPhysicalUids = ...
    context.sensorPhysicalUids(selectedReceivers);
payload.crossDutyRate = crossDutyRate;
payload.localDutyRate = localDutyRate;
payload.crossActiveResidualWeight = crossWeight;
payload.localActiveResidualWeight = localWeight;
payload.attemptAgeByReceiver = features.attemptAge;
payload.deliveryAgeByReceiver = features.deliveryAge;
payload.posteriorDisagreementByReceiver = ...
    features.posteriorDisagreement;
payload.currentReliabilityByReceiver = features.reliability;
payload.informationDebtScoreByReceiver = features.debtScore;
payload.deadlineMandatoryByReceiver = ...
    features.attemptAge >= crossServiceHorizon & ...
    sum(crossResidual, 2)' == 1;
payload.rollingCrossServiceMature = rollingMature;
payload.rollingFormationStrong = rollingFormationStrong;
payload.rollingCrossUnion = rollingUnion;
payload.referenceFallbackUsed = fallbackUsed;
payload.currentMessageCount = nnz(adjacency);
payload.referenceMessageCount = nnz(referenceAdjacency);
payload.messageSavingFraction = ...
    (payload.referenceMessageCount - payload.currentMessageCount) / ...
        payload.referenceMessageCount;
payload.dominantLayerAlwaysAttempted = ...
    all(dominant(:) <= adjacency(:));
payload.crossFormationResidualPriority = true;
payload.currentPosteriorUsed = true;
payload.pastSelectedTopologyUsed = true;
payload.pastSuccessfulDeliveryUsed = true;
payload.currentLinkReliabilityUsed = true;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.futurePageUsed = false;
payload.futureOutcomeUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.perEdgeMassEquivalenceClaimAllowed = false;
payload.trackingOutcomeScored = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
details = payload;
details.fusionWeightMatrix = fusionWeights;
details.referenceAdjacency = referenceAdjacency;
details.dominantAdjacency = dominant;
details.residualAdjacency = residual;
details.crossResidualAdjacency = crossResidual;
details.localResidualAdjacency = localResidual;
details.selectedResidualAdjacency = selectedResidual;
details.canonicalSha256 = computeCanonicalValueSha256(payload);
end

function validateInputs(context, referenceDetails, protocol)
requiredContext = { ...
    'localPosteriorBySensor', 'model', 'commConfig', ...
    'currentTime', 'physicalAdjacency', 'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor', ...
    'previousAdjacencyHistory', 'previousAdjacencyHistoryTimes', ...
    'previousDeliveryHistory', 'previousDeliveryHistoryTimes', ...
    'observableInputContract'};
requiredReference = { ...
    'dominantAdjacency', 'residualAdjacency', ...
    'referenceAdjacency', 'referenceFusionWeights'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, requiredContext)) || ...
        ~isstruct(referenceDetails) || ~isscalar(referenceDetails) || ...
        ~all(isfield(referenceDetails, requiredReference)) || ...
        ~isstruct(context.observableInputContract) || ...
        ~context.observableInputContract.passed || ...
        ~strcmp(context.observableInputContract.contractVersion, ...
            protocol.observableContractVersion) || ...
        ~context.observableInputContract.deliveryHistoryPresent || ...
        ~context.observableInputContract.pastDeliveryHistoryOnly || ...
        ~context.observableInputContract. ...
            deliveryHistoryAlignedToTopologyHistory || ...
        ~context.observableInputContract.deliveryHistoryAttemptSubset
    error('FormationGatewayDebtV47:InvalidContext', ...
        'The V47 causal observable context or reference route is invalid.');
end
nodeCount = numel(context.localPosteriorBySensor);
dominant = logical(referenceDetails.dominantAdjacency);
residual = logical(referenceDetails.residualAdjacency);
reference = logical(referenceDetails.referenceAdjacency);
weights = referenceDetails.referenceFusionWeights;
if nodeCount < protocol.period || ...
        ~isequal(size(dominant), [nodeCount, nodeCount]) || ...
        ~isequal(size(residual), [nodeCount, nodeCount]) || ...
        ~isequal(size(reference), [nodeCount, nodeCount]) || ...
        ~isequal(reference, dominant | residual) || ...
        any(dominant(:) & residual(:)) || ...
        any(sum(dominant, 2) ~= 1) || ...
        any(sum(residual, 2) ~= 1) || ...
        nnz(reference) ~= 2 * nodeCount || ...
        ~isequal(size(weights), [nodeCount, nodeCount]) || ...
        numel(context.sensorPhysicalUids) ~= nodeCount || ...
        numel(context.formationPhysicalUidsBySensor) ~= nodeCount || ...
        ~isequal(context.previousAdjacencyHistoryTimes, ...
            context.previousDeliveryHistoryTimes) || ...
        ~all(~logical(context.previousDeliveryHistory(:)) | ...
            logical(context.previousAdjacencyHistory(:)))
    error('FormationGatewayDebtV47:InvalidReference', ...
        'The V47 reference route or causal history is malformed.');
end
end

function features = buildFeatures( ...
        context, residual, summaries, serviceHorizon, protocol)
nodeCount = size(residual, 1);
attemptAge = inf(1, nodeCount);
deliveryAge = inf(1, nodeCount);
posteriorDisagreement = zeros(1, nodeCount);
reliability = zeros(1, nodeCount);
debtScore = zeros(1, nodeCount);
receiverPhysicalUid = reshape(context.sensorPhysicalUids, 1, []);
senderPhysicalUid = zeros(1, nodeCount);
normalizationAge = max([protocol.period, serviceHorizon, ...
    numel(context.previousAdjacencyHistoryTimes) + 1]);
for receiverIdx = 1:nodeCount
    senderIdx = find(residual(receiverIdx, :), 1);
    senderPhysicalUid(receiverIdx) = ...
        context.sensorPhysicalUids(senderIdx);
    attemptAge(receiverIdx) = lastEventAge( ...
        context.previousAdjacencyHistory, ...
        context.previousAdjacencyHistoryTimes, ...
        receiverIdx, senderIdx, context.currentTime);
    deliveryAge(receiverIdx) = lastEventAge( ...
        context.previousDeliveryHistory, ...
        context.previousDeliveryHistoryTimes, ...
        receiverIdx, senderIdx, context.currentTime);
    disagreement = computeLmbPosteriorSummaryDisagreement( ...
        summaries{receiverIdx}, summaries{senderIdx});
    posteriorDisagreement(receiverIdx) = disagreement.combined;
    reliability(receiverIdx) = min(max(1 - ...
        context.commConfig.pDropByEdge(senderIdx, receiverIdx), 0), 1);
    deliveryDebt = min(deliveryAge(receiverIdx), ...
        normalizationAge) / normalizationAge;
    novelty = posteriorDisagreement(receiverIdx) / ...
        (1 + posteriorDisagreement(receiverIdx));
    debtScore(receiverIdx) = ...
        protocol.deliveryAgeScoreWeight * deliveryDebt + ...
        protocol.posteriorDisagreementScoreWeight * novelty + ...
        protocol.currentReliabilityScoreWeight * reliability(receiverIdx);
end
features = struct( ...
    'attemptAge', attemptAge, ...
    'deliveryAge', deliveryAge, ...
    'posteriorDisagreement', posteriorDisagreement, ...
    'reliability', reliability, ...
    'debtScore', debtScore, ...
    'receiverPhysicalUid', receiverPhysicalUid, ...
    'senderPhysicalUid', senderPhysicalUid);
end

function age = lastEventAge(history, times, receiverIdx, senderIdx, now)
age = inf;
if isempty(times)
    return;
end
pages = reshape(logical(history(receiverIdx, senderIdx, :)), 1, []);
lastIdx = find(pages, 1, 'last');
if ~isempty(lastIdx)
    age = now - times(lastIdx);
end
end

function selected = selectReceivers( ...
        receivers, slotCount, features, serviceHorizon)
selected = zeros(1, 0);
if slotCount <= 0 || isempty(receivers)
    return;
end
slotCount = min(slotCount, numel(receivers));
mandatory = features.attemptAge(receivers) >= serviceHorizon;
keys = [ ...
    -double(mandatory(:)), ...
    -reshape(features.debtScore(receivers), [], 1), ...
    -reshape(min(features.attemptAge(receivers), 1e9), [], 1), ...
    reshape(features.receiverPhysicalUid(receivers), [], 1), ...
    reshape(features.senderPhysicalUid(receivers), [], 1)];
[~, order] = sortrows(keys, 1:size(keys, 2));
selected = receivers(order(1:slotCount));
end

function rate = classDutyRate(classCount, slotsByPhase, period)
if classCount <= 0
    rate = 0;
else
    rate = min(sum(slotsByPhase) / (period * classCount), 1);
end
end

function weight = activeClassWeight(dutyRate, protocol)
if dutyRate <= 0
    weight = 0;
else
    weight = min(protocol.referenceResidualWeight / dutyRate, ...
        protocol.maximumActiveResidualWeight);
end
end

function [mature, strong, unionAdjacency] = ...
        evaluateRollingCrossService( ...
            context, currentAdjacency, formationUids, horizon)
history = logical(context.previousAdjacencyHistory);
times = reshape(context.previousAdjacencyHistoryTimes, 1, []);
neededPast = max(horizon - 1, 0);
mature = neededPast == 0;
pages = reshape(logical(currentAdjacency), ...
    size(currentAdjacency, 1), size(currentAdjacency, 2), 1);
if neededPast > 0 && numel(times) >= neededPast
    selectedTimes = times(end-neededPast+1:end);
    mature = isequal(selectedTimes, ...
        (context.currentTime-neededPast):(context.currentTime-1));
    if mature
        pages = cat(3, history(:, :, end-neededPast+1:end), pages);
    end
end
unionAdjacency = any(pages, 3);
formationAdjacency = collapseToFormations( ...
    unionAdjacency, formationUids);
strong = isStronglyConnected(formationAdjacency);
end

function formationAdjacency = collapseToFormations(adjacency, uids)
formationIds = sort(unique(uids));
formationAdjacency = false(numel(formationIds));
for receiverFormation = 1:numel(formationIds)
    receivers = uids == formationIds(receiverFormation);
    for senderFormation = 1:numel(formationIds)
        if receiverFormation == senderFormation
            continue;
        end
        senders = uids == formationIds(senderFormation);
        formationAdjacency(receiverFormation, senderFormation) = ...
            any(any(adjacency(receivers, senders)));
    end
end
end

function connected = isStronglyConnected(adjacency)
connected = reachableAll(adjacency) && reachableAll(adjacency');
end

function passed = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end
