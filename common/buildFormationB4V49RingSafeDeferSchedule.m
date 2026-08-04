function [adjacencySequence, fusionWeightSequence, details] = ...
    buildFormationB4V49RingSafeDeferSchedule( ...
        context, referenceDetails, options)
% BUILDFORMATIONB4V49RINGSAFEDEFERSCHEDULE
% Keep one directed formation cycle at the synchronized residual burst and
% move a UID-canonical subset of the opposite cycle by exactly one phase.

if nargin < 3 || isempty(options)
    options = struct();
end
allowed = {'period', 'dominantWeight', 'activeResidualWeight', ...
    'missingNeighborWeightMode', 'maximumIncomingCount', ...
    'improvementTolerance', ...
    'minimumRelativeImprovementVsBestSynchronized', 'pulsePhase'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowed))
    error('FormationB4V49:InvalidOptions', ...
        'The ring-safe defer options are malformed.');
end
period = getField(options, 'period', 4);
dominantWeight = getField(options, 'dominantWeight', 0.70);
activeResidualWeight = getField( ...
    options, 'activeResidualWeight', 0.20);
missingMode = lower(char(getField( ...
    options, 'missingNeighborWeightMode', 'renormalize')));
maximumIncomingCount = getField(options, 'maximumIncomingCount', 2);
tolerance = getField(options, 'improvementTolerance', 1e-12);
minimumRelativeImprovement = getField(options, ...
    'minimumRelativeImprovementVsBestSynchronized', 0.01);
pulsePhase = getField(options, 'pulsePhase', 1);
if ~isscalar(period) || period ~= 4 || ...
        ~isscalar(dominantWeight) || ~isfinite(dominantWeight) || ...
        ~isscalar(activeResidualWeight) || ...
        ~isfinite(activeResidualWeight) || ...
        dominantWeight <= 0 || activeResidualWeight <= 0 || ...
        dominantWeight + activeResidualWeight >= 1 || ...
        ~ismember(missingMode, {'renormalize', 'self'}) || ...
        maximumIncomingCount ~= 2 || ...
        ~isscalar(tolerance) || ~isfinite(tolerance) || tolerance < 0 || ...
        ~isscalar(minimumRelativeImprovement) || ...
        ~isfinite(minimumRelativeImprovement) || ...
        minimumRelativeImprovement < 0 || ...
        minimumRelativeImprovement > 1 || ...
        ~isscalar(pulsePhase) || ~isfinite(pulsePhase) || ...
        pulsePhase < 1 || pulsePhase > period || ...
        pulsePhase ~= round(pulsePhase)
    error('FormationB4V49:InvalidOptions', ...
        'The frozen B4 score or materiality rule is invalid.');
end

requiredContext = {'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor', 'physicalAdjacency', ...
    'commConfig', 'currentTime'};
requiredReference = {'dominantAdjacency', 'residualAdjacency', ...
    'referenceAdjacency', 'referenceFusionWeights'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~hasExactFields(context, requiredContext) || ...
        ~isstruct(context.commConfig) || ...
        ~hasExactFields(context.commConfig, {'pDropByEdge'}) || ...
        ~isstruct(referenceDetails) || ~isscalar(referenceDetails) || ...
        ~hasExactFields(referenceDetails, requiredReference)
    error('FormationB4V49:InvalidContext', ...
        'Only the graph-only context and exact reference are accepted.');
end

sensorUids = reshape(context.sensorPhysicalUids, 1, []);
formationUidsBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
nodeCount = numel(sensorUids);
dominant = logical(referenceDetails.dominantAdjacency);
residual = logical(referenceDetails.residualAdjacency);
reference = logical(referenceDetails.referenceAdjacency);
reliability = 1 - context.commConfig.pDropByEdge';
if nodeCount < 4 || ~isValidUidVector(sensorUids, nodeCount) || ...
        ~isValidUidVector(formationUidsBySensor, nodeCount) || ...
        numel(unique(formationUidsBySensor)) < 3 || ...
        ~isBinaryMatrix(context.physicalAdjacency, nodeCount) || ...
        any(diag(logical(context.physicalAdjacency))) || ...
        ~isequal(size(dominant), [nodeCount, nodeCount]) || ...
        ~isequal(size(residual), [nodeCount, nodeCount]) || ...
        ~isequal(reference, dominant | residual) || ...
        any(dominant(:) & residual(:)) || ...
        any(sum(dominant, 2) ~= 1) || ...
        any(sum(residual, 2) ~= 1) || ...
        nnz(reference) ~= 2 * nodeCount || ...
        ndims(context.commConfig.pDropByEdge) ~= 2 || ...
        ~isequal(size(reliability), [nodeCount, nodeCount]) || ...
        any(~isfinite(reliability(:))) || ...
        any(reliability(:) < 0) || any(reliability(:) > 1) || ...
        any(reference(:) & ~logical(context.physicalAdjacency(:)))
    error('FormationB4V49:InvalidReference', ...
        'The V49 reference or current reliability page is invalid.');
end

cycle = decomposeBidirectionalFormationCycle( ...
    residual, sensorUids, formationUidsBySensor);
certificateOptions = struct( ...
    'missingNeighborWeightMode', missingMode, ...
    'maximumIncomingCount', maximumIncomingCount);
syncScores = nan(1, period);
for burstPhase = 1:period
    [syncAdjacency, syncWeights] = materializeSchedule( ...
        dominant, residual, false(nodeCount), burstPhase, period, ...
        dominantWeight, activeResidualWeight);
    syncScores(burstPhase) = scoreSchedule( ...
        syncAdjacency, syncWeights, reliability, ...
        period, certificateOptions);
end
bestSyncPhase = pulsePhase;
bestSyncScore = syncScores(bestSyncPhase);

formationCount = numel(cycle.formationPhysicalUids);
candidateCount = 1 + 2 * (2^formationCount - 1);
candidateScores = nan(1, candidateCount);
candidateMandatoryCycleOrdinals = nan(1, candidateCount);
candidateDeferMaskOrdinals = nan(1, candidateCount);
allCandidateBurstSensorStrong = true;
allCandidateBurstFormationStrong = true;
allCandidateSameMessageBudget = true;
cursor = 1;
candidateScores(cursor) = bestSyncScore;
candidateMandatoryCycleOrdinals(cursor) = 0;
candidateDeferMaskOrdinals(cursor) = 0;
for mandatoryOrdinal = 1:2
    optionalOrdinal = 3 - mandatoryOrdinal;
    optionalEdges = cycle.directedCycleMasks(:, :, optionalOrdinal);
    optionalLinear = cycle.directedCycleLinearByFlow{optionalOrdinal};
    for maskOrdinal = 1:(2^formationCount - 1)
        cursor = cursor + 1;
        deferred = false(nodeCount);
        selected = logical(bitget(maskOrdinal, 1:formationCount));
        deferred(optionalLinear(selected)) = true;
        if any(deferred(:) & ~optionalEdges(:))
            error('FormationB4V49:InternalContract', ...
                'The defer mask escaped the optional directed cycle.');
        end
        [candidateAdjacency, candidateWeights] = materializeSchedule( ...
            dominant, residual, deferred, bestSyncPhase, period, ...
            dominantWeight, activeResidualWeight);
        candidateScores(cursor) = scoreSchedule( ...
            candidateAdjacency, candidateWeights, reliability, ...
            period, certificateOptions);
        candidateMandatoryCycleOrdinals(cursor) = mandatoryOrdinal;
        candidateDeferMaskOrdinals(cursor) = maskOrdinal;
        burst = candidateAdjacency(:, :, bestSyncPhase);
        burstFormation = collapseToFormations( ...
            burst, formationUidsBySensor);
        allCandidateBurstSensorStrong = ...
            allCandidateBurstSensorStrong && isStronglyConnected(burst);
        allCandidateBurstFormationStrong = ...
            allCandidateBurstFormationStrong && ...
                isStronglyConnected(burstFormation);
        counts = reshape(sum(sum(candidateAdjacency, 1), 2), 1, []);
        allCandidateSameMessageBudget = ...
            allCandidateSameMessageBudget && sum(counts) == 5 * nodeCount;
    end
end
keys = [reshape(candidateScores, [], 1), ...
    reshape(candidateMandatoryCycleOrdinals, [], 1), ...
    reshape(candidateDeferMaskOrdinals, [], 1)];
[~, order] = sortrows(keys, 1:size(keys, 2));
rawIndex = order(1);
rawScore = candidateScores(rawIndex);
rawMandatoryOrdinal = candidateMandatoryCycleOrdinals(rawIndex);
rawMaskOrdinal = candidateDeferMaskOrdinals(rawIndex);
rawDeferredCount = nnz(bitget( ...
    rawMaskOrdinal, 1:formationCount));
rawRelativeImprovement = safeRelativeImprovement( ...
    bestSyncScore, rawScore);
materialityGateTriggered = rawDeferredCount > 0 && ...
    rawRelativeImprovement + tolerance < minimumRelativeImprovement;
if materialityGateTriggered
    selectedMandatoryOrdinal = 0;
    selectedMaskOrdinal = 0;
    selectedDeferred = false(nodeCount);
    selectedScore = bestSyncScore;
elseif rawDeferredCount == 0
    selectedMandatoryOrdinal = 0;
    selectedMaskOrdinal = 0;
    selectedDeferred = false(nodeCount);
    selectedScore = bestSyncScore;
else
    selectedMandatoryOrdinal = rawMandatoryOrdinal;
    selectedMaskOrdinal = rawMaskOrdinal;
    optionalOrdinal = 3 - selectedMandatoryOrdinal;
    optionalLinear = cycle.directedCycleLinearByFlow{optionalOrdinal};
    selected = logical(bitget( ...
        selectedMaskOrdinal, 1:formationCount));
    selectedDeferred = false(nodeCount);
    selectedDeferred(optionalLinear(selected)) = true;
    selectedScore = rawScore;
end

[adjacencySequence, fusionWeightSequence] = materializeSchedule( ...
    dominant, residual, selectedDeferred, bestSyncPhase, period, ...
    dominantWeight, activeResidualWeight);
selectedCertificate = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacencySequence, fusionWeightSequence, ...
        repmat(reliability, 1, 1, period), certificateOptions);
messageCountByPhase = reshape(sum(sum( ...
    adjacencySequence, 1), 2), 1, []);
burstAdjacency = adjacencySequence(:, :, bestSyncPhase);
burstFormationAdjacency = collapseToFormations( ...
    burstAdjacency, formationUidsBySensor);
recoveryPhase = mod(bestSyncPhase, period) + 1;
if cursor ~= candidateCount || ...
        ~allCandidateBurstSensorStrong || ...
        ~allCandidateBurstFormationStrong || ...
        ~allCandidateSameMessageBudget || ...
        sum(messageCountByPhase) ~= 5 * nodeCount || ...
        ~isequal(any(adjacencySequence, 3), reference) || ...
        ~isStronglyConnected(burstAdjacency) || ...
        ~isStronglyConnected(burstFormationAdjacency) || ...
        any(abs(reshape(sum(fusionWeightSequence, 2), [], 1) - 1) ...
            > 1e-12) || selectedScore > bestSyncScore + tolerance
    error('FormationB4V49:InvalidSchedule', ...
        'The selected ring-safe schedule violates a hard invariant.');
end

[sortedSensorUids, physicalOrder] = sort(sensorUids);
payload = struct();
payload.contractVersion = ...
    'formation-b4-v49-ring-safe-defer-schedule-v1';
payload.currentTime = context.currentTime;
payload.period = period;
payload.burstPhase = bestSyncPhase;
payload.recoveryPhase = recoveryPhase;
payload.periodicCycleBoundaryAssumed = true;
payload.sensorPhysicalUids = sortedSensorUids;
payload.formationPhysicalUids = cycle.formationPhysicalUids;
payload.directedFormationCycleOrders = cycle.directedCycleOrders;
payload.bestSynchronizedSquaredFactorByPulsePhase = syncScores;
payload.bestSynchronizedPulsePhase = bestSyncPhase;
payload.pulsePhaseFixedByCaller = true;
payload.deferPulsePhaseFrozenToComparator = true;
payload.candidatePulsePhaseReoptimized = false;
payload.bestSynchronizedB4SquaredFactor = bestSyncScore;
payload.candidateCount = candidateCount;
payload.candidateSquaredFactors = candidateScores;
payload.candidateMandatoryCycleOrdinals = ...
    candidateMandatoryCycleOrdinals;
payload.candidateDeferMaskOrdinals = candidateDeferMaskOrdinals;
payload.unconstrainedSelectedCandidateIndex = rawIndex;
payload.unconstrainedSelectedSquaredFactor = rawScore;
payload.unconstrainedSelectedMandatoryCycleOrdinal = ...
    rawMandatoryOrdinal;
payload.unconstrainedSelectedDeferMaskOrdinal = rawMaskOrdinal;
payload.unconstrainedSelectedDeferredCrossEdgeCount = rawDeferredCount;
payload.unconstrainedRelativeSquaredFactorImprovementVsBestSynchronized = ...
    rawRelativeImprovement;
payload.minimumRelativeImprovementVsBestSynchronized = ...
    minimumRelativeImprovement;
payload.materialityGateTriggered = materialityGateTriggered;
payload.selectedSquaredFactor = selectedScore;
payload.selectedRelativeSquaredFactorImprovementVsBestSynchronized = ...
    safeRelativeImprovement(bestSyncScore, selectedScore);
payload.selectedMandatoryCycleOrdinal = selectedMandatoryOrdinal;
payload.selectedDeferMaskOrdinal = selectedMaskOrdinal;
payload.selectedDeferredCrossEdgeCount = nnz(selectedDeferred);
payload.selectedDeferredCrossEdgesPhysicalUid = edgeUidTable( ...
    selectedDeferred, sensorUids, formationUidsBySensor);
payload.messageCountByPhase = messageCountByPhase;
payload.messagesPerPeriod = sum(messageCountByPhase);
payload.referenceMessagesPerPeriod = period * nnz(reference);
payload.posteriorMessageSavingFraction = ...
    (payload.referenceMessagesPerPeriod - payload.messagesPerPeriod) / ...
        payload.referenceMessagesPerPeriod;
payload.localResidualsRemainAtBurst = ...
    ~any(selectedDeferred(:) & cycle.localResidualMask(:));
payload.onlyOppositeCycleEdgesDeferred = true;
payload.mandatoryDirectedFormationCycleAtBurst = true;
payload.burstSensorStrongConnected = true;
payload.burstFormationStrongConnected = true;
payload.periodUnionEqualsReference = true;
payload.allCandidateBurstSensorStrongConnected = ...
    allCandidateBurstSensorStrong;
payload.allCandidateBurstFormationStrongConnected = ...
    allCandidateBurstFormationStrong;
payload.allCandidateSamePosteriorMessageBudget = ...
    allCandidateSameMessageBudget;
payload.selectedNoWorseThanBestSynchronizedProxy = true;
payload.nonSynchronizedExecutionRequiresMaterialImprovement = true;
payload.currentGraphUsed = true;
payload.currentLinkReliabilityUsed = true;
payload.posteriorUsed = false;
payload.posteriorSummaryUsed = false;
payload.deliveryAcknowledgmentUsed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.futurePageUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.trackingOutcomeScored = false;
payload.runtimeProtocolImplemented = false;
payload.sameTotalByteClaimAllowed = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
physicalCertificate = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacencySequence(physicalOrder, physicalOrder, :), ...
        fusionWeightSequence(physicalOrder, physicalOrder, :), ...
        repmat(reliability(physicalOrder, physicalOrder), ...
            1, 1, period), certificateOptions);
payload.selectedPhysicalUidOrderCertificateCanonicalSha256 = ...
    computeCanonicalValueSha256(physicalCertificate);
payload.canonicalSha256 = computeCanonicalValueSha256(payload);
details = payload;
details.selectedCertificate = selectedCertificate;
details.referenceAdjacency = reference;
details.dominantAdjacency = dominant;
details.residualAdjacency = residual;
details.localResidualAdjacency = cycle.localResidualMask;
details.crossResidualAdjacency = cycle.crossResidualMask;
details.directedCycleMasks = cycle.directedCycleMasks;
details.selectedDeferredCrossAdjacency = selectedDeferred;
details.adjacencySequence = adjacencySequence;
details.fusionWeightSequence = fusionWeightSequence;
end

function cycle = decomposeBidirectionalFormationCycle( ...
        residual, sensorUids, formationUidsBySensor)
formationUids = sort(unique(formationUidsBySensor));
formationCount = numel(formationUids);
crossMask = formationUidsBySensor(:) ~= formationUidsBySensor(:)';
crossResidual = residual & crossMask;
localResidual = residual & ~crossMask;
formationAdjacency = false(formationCount);
for receiverIdx = 1:formationCount
    receivers = formationUidsBySensor == formationUids(receiverIdx);
    for senderIdx = 1:formationCount
        senders = formationUidsBySensor == formationUids(senderIdx);
        formationAdjacency(receiverIdx, senderIdx) = any(any( ...
            crossResidual(receivers, senders)));
    end
end
if any(diag(formationAdjacency)) || ...
        ~isequal(formationAdjacency, formationAdjacency') || ...
        any(sum(formationAdjacency, 2) ~= 2) || ...
        nnz(crossResidual) ~= 2 * formationCount || ...
        ~isStronglyConnected(formationAdjacency)
    error('FormationB4V49:NoBidirectionalFormationCycle', ...
        ['Cross residuals must be the two directed orientations of ', ...
         'one simple formation cycle.']);
end

root = 1;
neighbors = find(formationAdjacency(root, :));
neighbors = sort(neighbors);
orders = zeros(2, formationCount);
for orientation = 1:2
    orders(orientation, :) = walkCycle( ...
        formationAdjacency, root, neighbors(orientation));
end
if ~isequal(orders(2, :), ...
        [orders(1, 1), fliplr(orders(1, 2:end))])
    error('FormationB4V49:NoBidirectionalFormationCycle', ...
        'The two canonical walks are not reverse cycle orientations.');
end

directedMasks = false(size(residual, 1), size(residual, 2), 2);
linearByFlow = cell(1, 2);
for orientation = 1:2
    order = orders(orientation, :);
    linear = zeros(1, formationCount);
    for flowIdx = 1:formationCount
        senderFormation = order(flowIdx);
        receiverFormation = order(mod(flowIdx, formationCount) + 1);
        receiverMask = formationUidsBySensor == ...
            formationUids(receiverFormation);
        senderMask = formationUidsBySensor == ...
            formationUids(senderFormation);
        [receivers, senders] = find( ...
            crossResidual & receiverMask(:) & senderMask(:)');
        if numel(receivers) ~= 1 || numel(senders) ~= 1
            error('FormationB4V49:NoBidirectionalFormationCycle', ...
                'Every directed formation arc must map to one residual edge.');
        end
        linear(flowIdx) = sub2ind(size(residual), ...
            receivers, senders);
    end
    directedMasks(:, :, orientation) = false(size(residual));
    directedMasks(linear + ...
        (orientation - 1) * numel(residual)) = true;
    linearByFlow{orientation} = linear;
end
if any(any(directedMasks(:, :, 1) & directedMasks(:, :, 2))) || ...
        ~isequal(any(directedMasks, 3), crossResidual)
    error('FormationB4V49:NoBidirectionalFormationCycle', ...
        'The two directed cycle masks do not partition cross residuals.');
end

cycle = struct();
cycle.formationPhysicalUids = formationUids;
cycle.formationAdjacency = formationAdjacency;
cycle.directedCycleOrders = formationUids(orders);
cycle.directedCycleMasks = directedMasks;
cycle.directedCycleLinearByFlow = linearByFlow;
cycle.localResidualMask = localResidual;
cycle.crossResidualMask = crossResidual;
cycle.crossResidualPhysicalUidTable = edgeUidTable( ...
    crossResidual, sensorUids, formationUidsBySensor);
end

function order = walkCycle(adjacency, root, firstNeighbor)
formationCount = size(adjacency, 1);
order = zeros(1, formationCount);
order(1:2) = [root, firstNeighbor];
for position = 3:formationCount
    candidates = find(adjacency(order(position - 1), :));
    candidates(candidates == order(position - 2)) = [];
    candidates(ismember(candidates, order(1:(position - 1)))) = [];
    if numel(candidates) ~= 1
        error('FormationB4V49:NoBidirectionalFormationCycle', ...
            'The formation graph is not a simple cycle.');
    end
    order(position) = candidates;
end
if ~adjacency(order(end), root) || numel(unique(order)) ~= formationCount
    error('FormationB4V49:NoBidirectionalFormationCycle', ...
        'The canonical formation walk does not close exactly once.');
end
end

function [adjacency, weights] = materializeSchedule( ...
        dominant, residual, deferred, burstPhase, period, ...
        dominantWeight, activeResidualWeight)
nodeCount = size(dominant, 1);
recoveryPhase = mod(burstPhase, period) + 1;
adjacency = false(nodeCount, nodeCount, period);
weights = zeros(nodeCount, nodeCount, period);
for phase = 1:period
    activeResidual = false(nodeCount);
    if phase == burstPhase
        activeResidual = residual & ~deferred;
    end
    if phase == recoveryPhase
        activeResidual = activeResidual | deferred;
    end
    adjacency(:, :, phase) = dominant | activeResidual;
    pageWeights = zeros(nodeCount);
    pageWeights(dominant) = dominantWeight;
    pageWeights(activeResidual) = activeResidualWeight;
    pageWeights(1:nodeCount+1:end) = 1 - sum(pageWeights, 2)';
    weights(:, :, phase) = pageWeights;
end
end

function score = scoreSchedule( ...
        adjacency, weights, reliability, period, options)
certificate = ...
    computeReliableKlaWindowMeanSquareContractionCertificate( ...
        adjacency, weights, repmat(reliability, 1, 1, period), ...
        options);
score = certificate.worstCaseExpectedSquaredContractionFactor;
end

function pages = collapseToFormations(sensorPages, formationUidsBySensor)
uids = sort(unique(formationUidsBySensor));
pages = false(numel(uids), numel(uids), size(sensorPages, 3));
for timeIdx = 1:size(sensorPages, 3)
    for receiverIdx = 1:numel(uids)
        receivers = formationUidsBySensor == uids(receiverIdx);
        for senderIdx = 1:numel(uids)
            senders = formationUidsBySensor == uids(senderIdx);
            pages(receiverIdx, senderIdx, timeIdx) = any(any( ...
                sensorPages(receivers, senders, timeIdx)));
        end
    end
end
end

function table = edgeUidTable(adjacency, sensorUids, formationUids)
[receivers, senders] = find(adjacency);
table = [reshape(formationUids(receivers), [], 1), ...
    reshape(formationUids(senders), [], 1), ...
    reshape(sensorUids(receivers), [], 1), ...
    reshape(sensorUids(senders), [], 1)];
if ~isempty(table)
    table = sortrows(table, 1:4);
end
end

function valid = isValidUidVector(values, count)
valid = isa(values, 'double') && isreal(values) && ...
    numel(values) == count && all(isfinite(values)) && ...
    all(values > 0) && all(values == round(values));
end

function valid = isBinaryMatrix(matrix, count)
valid = (islogical(matrix) || isnumeric(matrix)) && ...
    isreal(matrix) && isequal(size(matrix), [count, count]) && ...
    all(isfinite(matrix(:))) && ...
    all(matrix(:) == 0 | matrix(:) == 1);
end

function passed = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
passed = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
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

function value = safeRelativeImprovement(reference, candidate)
if reference > 0
    value = (reference - candidate) / reference;
elseif candidate == 0
    value = 0;
else
    value = -inf;
end
end

function passed = hasExactFields(structure, expected)
passed = numel(fieldnames(structure)) == numel(expected) && ...
    all(ismember(expected, fieldnames(structure)));
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
