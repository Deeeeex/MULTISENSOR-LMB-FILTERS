function sequence = buildBalancedRecoverySequenceV79( ...
        context, referenceAdjacency, referenceDetails, options)
% BUILDBALANCEDRECOVERYSEQUENCEV79 Select a two-round contraction pair.

if nargin < 4 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
nominalTolerance = getField( ...
    options, 'nominalDoubleStochasticTolerance', 1e-12);
nodeCount = numel(context.localPosteriorBySensor);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
groupSizes = arrayfun(@(groupId) ...
    nnz(groupIds == groupId), groups);
if numel(groupIds) ~= nodeCount || any(groupSizes < 2) || ...
        ~isfield(referenceDetails, 'residualAdjacency') || ...
        ~isfield(referenceDetails, 'fusionWeightMatrix') || ...
        ~isequal(size(referenceAdjacency), [nodeCount, nodeCount])
    error('BalancedRecoveryV79:InvalidContext', ...
        'The balanced recovery context is incomplete.');
end

referenceWeights = referenceDetails.fusionWeightMatrix;
referenceEffective = expectedWeights( ...
    referenceAdjacency, referenceWeights, ...
    context.commConfig, context.currentTime);
centering = eye(nodeCount) - ones(nodeCount) / nodeCount;
referenceMetrics = operatorMetrics( ...
    referenceWeights, referenceEffective, centering);

maximumPhase = max(groupSizes) - 1;
records = repmat(emptyPhaseRecord(), 1, maximumPhase);
valid = false(1, maximumPhase);
for phase = 1:maximumPhase
    try
        [dominantAdjacency, ~] = ...
            selectRegisteredDirectedRoutingPolicy( ...
                context, 'fixed-balanced-cycle', struct( ...
                    'sourceWeight', dominantWeight, ...
                    'phase', phase));
        [adjacency, weights, route] = ...
            buildBackbonePreservingResidualRoute( ...
                context, dominantAdjacency, ...
                referenceDetails.residualAdjacency, ...
                dominantWeight, residualWeight);
        nominalColumnDeviation = ...
            max(abs(sum(weights, 1) - 1));
        exactBudget = nnz(adjacency) == ...
            nnz(referenceAdjacency);
        distinctInputs = route.duplicateSourceFraction == 0;
        if ~exactBudget || ~distinctInputs || ...
                nominalColumnDeviation > nominalTolerance
            continue;
        end
        effective = expectedWeights( ...
            adjacency, weights, context.commConfig, ...
            context.currentTime);
        record = emptyPhaseRecord();
        record.phase = phase;
        record.adjacency = adjacency;
        record.fusionWeights = weights;
        record.effectiveWeights = effective;
        record.messageCount = nnz(adjacency);
        record.duplicateSourceFraction = ...
            route.duplicateSourceFraction;
        record.metrics = operatorMetrics( ...
            weights, effective, centering);
        record.valid = true;
        records(phase) = record;
        valid(phase) = true;
    catch errorInfo
        if ~isExpectedUnavailable(errorInfo)
            rethrow(errorInfo);
        end
    end
end
records = records(valid);
if isempty(records)
    error('BalancedRecoveryV79:NoValidPhase', ...
        'No exact-budget physical balanced recovery phase exists.');
end

pairRows = zeros(numel(records)^2, 6);
pairCursor = 0;
for firstIdx = 1:numel(records)
    firstEffective = records(firstIdx).effectiveWeights;
    firstBound = norm(centering * firstEffective * centering, 2);
    for secondIdx = 1:numel(records)
        secondEffective = records(secondIdx).effectiveWeights;
        terminalBound = norm(centering * secondEffective * ...
            firstEffective * centering, 2);
        pairCursor = pairCursor + 1;
        pairRows(pairCursor, :) = [ ...
            max(firstBound, terminalBound), terminalBound, ...
            records(firstIdx).phase, records(secondIdx).phase, ...
            firstIdx, secondIdx];
    end
end
pairRows = pairRows(1:pairCursor, :);
pairRows = sortrows(pairRows, 1:4);
selected = pairRows(1, :);
firstRecord = records(selected(5));
secondRecord = records(selected(6));

sequence = struct();
sequence.contractVersion = ...
    'balanced-recovery-sequence-v79-v1';
sequence.referenceAdjacency = referenceAdjacency;
sequence.referenceFusionWeights = referenceWeights;
sequence.referenceEffectiveWeights = referenceEffective;
sequence.referenceMetrics = referenceMetrics;
sequence.phaseRecords = records;
sequence.selectedPhasePair = selected(3:4);
sequence.selectedPeakCenteredLinearBound = selected(1);
sequence.selectedTerminalCenteredLinearBound = selected(2);
sequence.recoveryAdjacencyByRound = { ...
    firstRecord.adjacency, secondRecord.adjacency};
sequence.recoveryFusionWeightsByRound = { ...
    firstRecord.fusionWeights, secondRecord.fusionWeights};
sequence.recoveryEffectiveWeightsByRound = { ...
    firstRecord.effectiveWeights, secondRecord.effectiveWeights};
sequence.recoveryMetricsByRound = [ ...
    firstRecord.metrics, secondRecord.metrics];
sequence.referenceMessageCount = nnz(referenceAdjacency);
sequence.recoveryMessageCountByRound = [ ...
    firstRecord.messageCount, secondRecord.messageCount];
sequence.exactMessageCountParity = all( ...
    sequence.recoveryMessageCountByRound == ...
        sequence.referenceMessageCount);
sequence.nominalDoubleStochastic = all(arrayfun( ...
    @(record) record.metrics.nominalColumnDeviation <= ...
        nominalTolerance, [firstRecord, secondRecord]));
sequence.posteriorUsed = false;
sequence.measurementUsed = false;
sequence.currentPhysicalLinksUsed = true;
sequence.currentLinkReliabilityUsed = true;
sequence.truthUsed = false;
sequence.futureLinkUsed = false;
sequence.futureOutcomeUsed = false;
if ~sequence.exactMessageCountParity || ...
        ~sequence.nominalDoubleStochastic
    error('BalancedRecoveryV79:InvalidSelection', ...
        'The selected recovery pair violates a hard invariant.');
end
end

function metrics = operatorMetrics(nominal, effective, centering)
metrics = struct();
metrics.nominalColumnDeviation = ...
    max(abs(sum(nominal, 1) - 1));
metrics.effectiveColumnDeviation = ...
    max(abs(sum(effective, 1) - 1));
metrics.centeredSpectralNorm = ...
    norm(centering * effective * centering, 2);
metrics.nonNormality = ...
    norm(effective' * effective - ...
        effective * effective', 'fro');
end

function effective = expectedWeights( ...
        adjacency, weights, commConfig, currentTime)
nodeCount = size(weights, 1);
effective = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    sources = [receiverIdx, reshape(find( ...
        adjacency(receiverIdx, :)), 1, [])];
    localWeights = weights(receiverIdx, sources);
    for sourceCursor = 2:numel(sources)
        senderIdx = sources(sourceCursor);
        localWeights(sourceCursor) = ...
            localWeights(sourceCursor) * (1 - edgeDrop( ...
                commConfig, senderIdx, receiverIdx, currentTime));
    end
    localWeights = localWeights / sum(localWeights);
    effective(receiverIdx, sources) = localWeights;
end
end

function probability = edgeDrop( ...
        config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function record = emptyPhaseRecord()
record = struct( ...
    'phase', NaN, ...
    'adjacency', false(0), ...
    'fusionWeights', zeros(0), ...
    'effectiveWeights', zeros(0), ...
    'messageCount', 0, ...
    'duplicateSourceFraction', NaN, ...
    'metrics', struct(), ...
    'valid', false);
end

function value = isExpectedUnavailable(errorInfo)
value = strcmp(errorInfo.identifier, ...
    'PhysicalFormationTreeReference:NoSensorTour') || ...
    isempty(errorInfo.identifier);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
