function [assignment, details] = ...
        projectScaleEquivariantGatewayAssignmentV254( ...
            context, edgeValues, referenceAssignment, options)
% PROJECTSCALEEQUIVARIANTGATEWAYASSIGNMENTV254 Exact safe gateway matching.
%
% edgeValues(receiverIndex, senderIndex) is the learned value of the
% directed physical message sender -> receiver.  The formation-level tree
% is inherited from referenceAssignment.  For every receiving formation,
% the projector assigns distinct receiver sensors to its incoming directed
% tree arcs and chooses the best sender for each assigned receiver.

if nargin < 4 || isempty(options)
    options = struct();
end
switchingPenalty = getField(options, 'switchingPenalty', 0);
tieTolerance = getField(options, 'tieTolerance', 1e-10);
if ~isscalar(switchingPenalty) || ~isfinite(switchingPenalty) || ...
        switchingPenalty < 0 || ~isscalar(tieTolerance) || ...
        ~isfinite(tieTolerance) || tieTolerance < 0
    error('ScaleEquivariantGatewayV254:InvalidProjectionOptions', ...
        'Switching penalty and tie tolerance must be finite nonnegative scalars.');
end

identity = buildIdentity(context);
if ~isnumeric(edgeValues) || ...
        ~isequal(size(edgeValues), [identity.nodeCount, identity.nodeCount])
    error('ScaleEquivariantGatewayV254:InvalidEdgeValues', ...
        'Edge values must use the receiver-row, sender-column convention.');
end
reference = normalizeAssignment(referenceAssignment);
validateReference(reference, context, identity);

projected = reference;
receivingFormations = unique(reference(:, 2), 'stable');
local = repmat(emptyLocalProjection(), 1, ...
    numel(receivingFormations));
candidateScore = 0;
candidateAvailable = true;
uniqueOptimum = true;
fallbackReason = '';
for formationIdx = 1:numel(receivingFormations)
    receiverFormation = receivingFormations(formationIdx);
    rows = find(reference(:, 2) == receiverFormation);
    receivers = find(identity.formationBySensor == receiverFormation);
    [scoreTable, senderTable, senderAmbiguous] = ...
        buildLocalScoreTable(context, edgeValues, reference, rows, ...
            receivers, identity, switchingPenalty, tieTolerance);
    [selectedColumns, localScore, localUnique, localFeasible] = ...
        solveUniqueMaximumMatching(scoreTable, tieTolerance);

    record = emptyLocalProjection();
    record.receiverFormationUid = receiverFormation;
    record.directedPairRows = reshape(rows, 1, []);
    record.receiverSensorUids = identity.sensorUids(receivers);
    record.scoreTable = scoreTable;
    record.feasible = localFeasible;
    record.uniqueOptimum = localUnique;
    record.selectedReceiverColumns = selectedColumns;
    if ~localFeasible
        candidateAvailable = false;
        uniqueOptimum = false;
        fallbackReason = 'no-complete-local-matching';
        local(formationIdx) = record;
        break;
    end

    selectedLinear = sub2ind(size(scoreTable), ...
        (1:numel(rows))', selectedColumns(:));
    if any(senderAmbiguous(selectedLinear))
        localUnique = false;
    end
    record.uniqueOptimum = localUnique;
    record.selectedSenderSensorUids = ...
        identity.sensorUids(senderTable(selectedLinear));
    record.selectedReceiverSensorUids = ...
        identity.sensorUids(receivers(selectedColumns));
    record.adjustedScore = localScore;
    local(formationIdx) = record;

    projected(rows, 3) = ...
        reshape(record.selectedSenderSensorUids, [], 1);
    projected(rows, 4) = ...
        reshape(record.selectedReceiverSensorUids, [], 1);
    candidateScore = candidateScore + localScore;
    if ~localUnique
        uniqueOptimum = false;
        fallbackReason = 'nonunique-observable-optimum';
    end
end

referenceScore = scoreAssignment( ...
    edgeValues, reference, identity, reference, switchingPenalty);
if ~isfinite(referenceScore)
    candidateAvailable = false;
    uniqueOptimum = false;
    fallbackReason = 'reference-score-unavailable';
end
if candidateAvailable
    projected = normalizeAssignment(projected);
    validateProjected(projected, reference, context, identity);
end

fallbackUsed = ~candidateAvailable || ~uniqueOptimum;
if fallbackUsed
    assignment = reference;
    appliedScore = referenceScore;
    appliedAdvantage = 0;
else
    assignment = projected;
    appliedScore = candidateScore;
    appliedAdvantage = candidateScore - referenceScore;
end
changedArcs = 0;
if candidateAvailable
    changedArcs = nnz(any(projected(:, 3:4) ~= ...
        reference(:, 3:4), 2));
end

details = struct();
details.contractVersion = ...
    'scale-equivariant-safe-gateway-v254-projection-v1';
details.edgeValueConvention = 'receiver-row-sender-column';
details.referenceAssignment = reference;
details.candidateAssignment = projected;
details.appliedAssignment = assignment;
details.localProjections = local;
details.referenceScore = referenceScore;
details.candidateAdjustedScore = candidateScore;
details.candidatePredictedAdvantage = ...
    candidateScore - referenceScore;
details.appliedAdjustedScore = appliedScore;
details.appliedPredictedAdvantage = appliedAdvantage;
details.changedArcCount = changedArcs;
details.switchingPenalty = switchingPenalty;
details.tieTolerance = tieTolerance;
details.candidateAvailable = candidateAvailable;
details.uniqueObservableOptimum = uniqueOptimum;
details.projectionFallbackUsed = fallbackUsed;
details.projectionFallbackReason = fallbackReason;
details.expectedMessageCount = identity.nodeCount + ...
    2 * (identity.formationCount - 1);
details.physicallyFeasible = true;
details.receiverInjective = true;
details.formationTreePreserved = true;
details.sensorPermutationEquivariant = true;
details.formationPermutationEquivariant = true;
details.numericSensorIdentifiersUsedAsScores = false;
details.numericFormationIdentifiersUsedAsScores = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function [scores, senders, ambiguous] = buildLocalScoreTable( ...
        context, edgeValues, reference, rows, receivers, identity, ...
        switchingPenalty, tieTolerance)
slotCount = numel(rows);
scores = -Inf(slotCount, numel(receivers));
senders = zeros(size(scores));
ambiguous = false(size(scores));
physical = logical(context.physicalAdjacency);
for slotIdx = 1:slotCount
    rowIdx = rows(slotIdx);
    senderFormation = reference(rowIdx, 1);
    senderCandidates = find( ...
        identity.formationBySensor == senderFormation);
    referenceSenderUid = reference(rowIdx, 3);
    referenceReceiverUid = reference(rowIdx, 4);
    for receiverColumn = 1:numel(receivers)
        receiver = receivers(receiverColumn);
        available = senderCandidates( ...
            physical(receiver, senderCandidates) & ...
            isfinite(edgeValues(receiver, senderCandidates)));
        if isempty(available)
            continue;
        end
        raw = edgeValues(receiver, available);
        isReferenceEdge = ...
            identity.sensorUids(available) == referenceSenderUid & ...
            identity.sensorUids(receiver) == referenceReceiverUid;
        adjusted = raw - switchingPenalty * (~isReferenceEdge);
        best = max(adjusted);
        tied = find(closeEnough(adjusted, best, tieTolerance));
        referenceTie = tied(isReferenceEdge(tied));
        if ~isempty(referenceTie)
            chosen = referenceTie(1);
        else
            chosen = tied(1);
            ambiguous(slotIdx, receiverColumn) = numel(tied) > 1;
        end
        scores(slotIdx, receiverColumn) = adjusted(chosen);
        senders(slotIdx, receiverColumn) = available(chosen);
    end
end
end

function [columns, objective, uniqueOptimum, feasible] = ...
        solveUniqueMaximumMatching(scores, tieTolerance)
rowCount = size(scores, 1);
columns = zeros(1, rowCount);
objective = -Inf;
uniqueOptimum = false;
feasible = rowCount <= size(scores, 2) && ...
    all(any(isfinite(scores), 2));
if ~feasible
    return;
end
cost = buildFiniteSolverCost(scores);
[solverMask, ~] = munkres(cost');
mask = solverMask';
[feasible, columns, objective] = ...
    decodeMatching(mask, scores);
if ~feasible
    return;
end

uniqueOptimum = true;
for rowIdx = 1:rowCount
    alternativeScores = scores;
    alternativeScores(rowIdx, columns(rowIdx)) = -Inf;
    alternativeCost = buildFiniteSolverCost(alternativeScores);
    [alternativeSolverMask, ~] = munkres(alternativeCost');
    alternativeMask = alternativeSolverMask';
    [alternativeFeasible, ~, alternativeObjective] = ...
        decodeMatching(alternativeMask, alternativeScores);
    if alternativeFeasible && closeEnough( ...
            alternativeObjective, objective, tieTolerance)
        uniqueOptimum = false;
        return;
    end
end
end

function cost = buildFiniteSolverCost(scores)
valid = isfinite(scores);
finiteValues = scores(valid);
if isempty(finiteValues)
    cost = ones(size(scores));
    return;
end
cost = zeros(size(scores));
cost(valid) = max(finiteValues) - finiteValues;
maximumFiniteCost = max(cost(valid));
invalidCost = (size(scores, 1) + 1) * ...
    (maximumFiniteCost + 1);
cost(~valid) = invalidCost;
end

function [feasible, columns, objective] = decodeMatching(mask, scores)
rowCount = size(scores, 1);
columns = zeros(1, rowCount);
feasible = isequal(size(mask), size(scores)) && ...
    all(sum(mask, 2) == 1) && all(sum(mask, 1) <= 1);
if ~feasible
    objective = -Inf;
    return;
end
for rowIdx = 1:rowCount
    columns(rowIdx) = find(mask(rowIdx, :), 1);
end
selected = sub2ind(size(scores), ...
    (1:rowCount)', columns(:));
feasible = all(isfinite(scores(selected)));
if feasible
    objective = sum(scores(selected));
else
    objective = -Inf;
end
end

function score = scoreAssignment( ...
        edgeValues, assignment, identity, reference, switchingPenalty)
score = 0;
for rowIdx = 1:size(assignment, 1)
    sender = find(identity.sensorUids == assignment(rowIdx, 3), 1);
    receiver = find(identity.sensorUids == assignment(rowIdx, 4), 1);
    value = edgeValues(receiver, sender);
    if ~isfinite(value)
        score = NaN;
        return;
    end
    changed = any(assignment(rowIdx, 3:4) ~= ...
        reference(rowIdx, 3:4));
    score = score + value - switchingPenalty * changed;
end
end

function validateReference(reference, context, identity)
expectedRows = 2 * (identity.formationCount - 1);
pairs = reference(:, 1:2);
valid = size(reference, 1) == expectedRows && ...
    size(reference, 2) == 4 && ...
    size(unique(pairs, 'rows'), 1) == expectedRows && ...
    all(pairs(:, 1) ~= pairs(:, 2));
undirected = sort(pairs, 2);
[uniqueUndirected, ~, pairIndex] = unique(undirected, 'rows');
if valid
    counts = accumarray(pairIndex, 1);
    valid = size(uniqueUndirected, 1) == identity.formationCount - 1 && ...
        all(counts == 2) && formationTreeConnected( ...
            uniqueUndirected, identity.formationUids);
end
receiverIndices = zeros(1, size(reference, 1));
if valid
    for rowIdx = 1:size(reference, 1)
        sender = find(identity.sensorUids == reference(rowIdx, 3), 1);
        receiver = find(identity.sensorUids == reference(rowIdx, 4), 1);
        valid = ~isempty(sender) && ~isempty(receiver) && ...
            identity.formationBySensor(sender) == reference(rowIdx, 1) && ...
            identity.formationBySensor(receiver) == reference(rowIdx, 2) && ...
            logical(context.physicalAdjacency(receiver, sender));
        if ~valid
            break;
        end
        receiverIndices(rowIdx) = receiver;
    end
end
if valid
    for formationUid = identity.formationUids
        rows = reference(:, 2) == formationUid;
        valid = numel(unique(receiverIndices(rows))) == nnz(rows);
        if ~valid
            break;
        end
    end
end
if ~valid
    error('ScaleEquivariantGatewayV254:InvalidReference', ...
        'The V254 reference is not a physical injective bidirectional tree embedding.');
end
end

function validateProjected(projected, reference, context, identity)
valid = isequal(projected(:, 1:2), reference(:, 1:2));
receiverIndices = zeros(1, size(projected, 1));
if valid
    for rowIdx = 1:size(projected, 1)
        sender = find(identity.sensorUids == projected(rowIdx, 3), 1);
        receiver = find(identity.sensorUids == projected(rowIdx, 4), 1);
        valid = ~isempty(sender) && ~isempty(receiver) && ...
            identity.formationBySensor(sender) == projected(rowIdx, 1) && ...
            identity.formationBySensor(receiver) == projected(rowIdx, 2) && ...
            logical(context.physicalAdjacency(receiver, sender));
        if ~valid
            break;
        end
        receiverIndices(rowIdx) = receiver;
    end
end
if valid
    for formationUid = identity.formationUids
        rows = projected(:, 2) == formationUid;
        valid = numel(unique(receiverIndices(rows))) == nnz(rows);
        if ~valid
            break;
        end
    end
end
if ~valid
    error('ScaleEquivariantGatewayV254:UnsafeProjection', ...
        'The V254 projected assignment violated a hard topology invariant.');
end
end

function connected = formationTreeConnected(edges, formationUids)
adjacency = false(numel(formationUids));
for edgeIdx = 1:size(edges, 1)
    left = find(formationUids == edges(edgeIdx, 1), 1);
    right = find(formationUids == edges(edgeIdx, 2), 1);
    if isempty(left) || isempty(right)
        connected = false;
        return;
    end
    adjacency(left, right) = true;
    adjacency(right, left) = true;
end
visited = false(1, numel(formationUids));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ...
        ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end

function identity = buildIdentity(context)
required = {'physicalAdjacency', 'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required))
    error('ScaleEquivariantGatewayV254:InvalidContext', ...
        'The V254 projection context is incomplete.');
end
identity = struct();
identity.sensorUids = reshape(context.sensorPhysicalUids, 1, []);
identity.formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
identity.nodeCount = numel(identity.sensorUids);
identity.formationUids = sort(unique(identity.formationBySensor));
identity.formationCount = numel(identity.formationUids);
if identity.nodeCount == 0 || identity.formationCount < 2 || ...
        numel(identity.formationBySensor) ~= identity.nodeCount || ...
        numel(unique(identity.sensorUids)) ~= identity.nodeCount || ...
        ~isequal(size(context.physicalAdjacency), ...
            [identity.nodeCount, identity.nodeCount])
    error('ScaleEquivariantGatewayV254:InvalidIdentity', ...
        'The V254 physical identity or adjacency is malformed.');
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || size(assignment, 2) ~= 4 || ...
        isempty(assignment) || any(~isfinite(assignment(:)))
    error('ScaleEquivariantGatewayV254:InvalidAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function result = closeEnough(values, target, tolerance)
scale = max(1, max(abs([values(:); target])));
result = abs(values - target) <= tolerance * scale;
end

function value = emptyLocalProjection()
value = struct('receiverFormationUid', 0, ...
    'directedPairRows', zeros(1, 0), ...
    'receiverSensorUids', zeros(1, 0), ...
    'scoreTable', zeros(0), 'feasible', false, ...
    'uniqueOptimum', false, ...
    'selectedReceiverColumns', zeros(1, 0), ...
    'selectedSenderSensorUids', zeros(1, 0), ...
    'selectedReceiverSensorUids', zeros(1, 0), ...
    'adjustedScore', -Inf);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
