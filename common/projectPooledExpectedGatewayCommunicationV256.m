function projection = projectPooledExpectedGatewayCommunicationV256( ...
        nodePayloadBytes, denseAdjacency, v242Adjacency, ...
        referenceAssignment, candidateAssignments, sensorPhysicalUids, ...
        physicalAdjacency, controlBytes, horizonSteps, protocol)
% PROJECTPOOLEDEXPECTEDGATEWAYCOMMUNICATIONV256 Deterministic byte credit.

% The projection uses only current posterior sizes.  It compares the V242
% sparse route and each one-arc replacement with the current physical V240
% two-input route over the registered hold.  The controller exchange is
% charged once per hold, not once per posterior page.

nodePayloadBytes = reshape(nodePayloadBytes, 1, []);
sensorPhysicalUids = reshape(sensorPhysicalUids, 1, []);
nodeCount = numel(nodePayloadBytes);
candidateCount = numel(candidateAssignments);
valid = nodeCount > 0 && numel(sensorPhysicalUids) == nodeCount && ...
    numel(unique(sensorPhysicalUids)) == nodeCount && ...
    all(isfinite(nodePayloadBytes)) && all(nodePayloadBytes >= 0) && ...
    isequal(size(denseAdjacency), [nodeCount, nodeCount]) && ...
    isequal(size(v242Adjacency), [nodeCount, nodeCount]) && ...
    isequal(size(physicalAdjacency), [nodeCount, nodeCount]) && ...
    iscell(candidateAssignments) && candidateCount > 0 && ...
    isfinite(controlBytes) && controlBytes >= 0 && ...
    isfinite(horizonSteps) && horizonSteps >= 1 && ...
    horizonSteps == floor(horizonSteps) && ...
    isstruct(protocol) && isscalar(protocol) && ...
    isfield(protocol, 'communicationCreditFraction') && ...
    isfield(protocol, 'minimumRetainedDenseCausalSavingFraction');
if ~valid
    error('PooledExpectedGatewayV256:InvalidCommunicationProjection', ...
        'The deterministic V256 communication projection is malformed.');
end

denseAdjacency = logical(denseAdjacency);
v242Adjacency = logical(v242Adjacency);
physicalAdjacency = logical(physicalAdjacency);
if any(v242Adjacency(:) & ~denseAdjacency(:)) || ...
        any(denseAdjacency(:) & ~physicalAdjacency(:)) || ...
        ~isStronglyConnected(denseAdjacency) || ...
        ~isStronglyConnected(v242Adjacency)
    error('PooledExpectedGatewayV256:InvalidCommunicationReference', ...
        'The V240/V242 communication references are not nested and feasible.');
end

reference = normalizeAssignment(referenceAssignment);
referenceCrossMask = assignmentMask( ...
    reference, sensorPhysicalUids, nodeCount);
if any(referenceCrossMask(:) & ~v242Adjacency(:))
    error('PooledExpectedGatewayV256:CommunicationReferenceDrift', ...
        'The teacher reference gateways do not match the V242 route.');
end
localBackbone = v242Adjacency & ~referenceCrossMask;

densePageBytes = adjacencyBytes(denseAdjacency, nodePayloadBytes);
v242PageBytes = adjacencyBytes(v242Adjacency, nodePayloadBytes);
denseHoldBytes = horizonSteps * densePageBytes;
v242HoldBytes = horizonSteps * v242PageBytes;
denseToV242SavingBytes = denseHoldBytes - v242HoldBytes;
if denseToV242SavingBytes <= 0
    error('PooledExpectedGatewayV256:NonpositiveCommunicationCredit', ...
        'The current V240 route supplies no positive byte credit over V242.');
end
availableCreditBytes = protocol.communicationCreditFraction * ...
    denseToV242SavingBytes;

candidatePageBytes = zeros(1, candidateCount);
candidateHoldBytes = zeros(1, candidateCount);
candidateIncrementalBytes = zeros(1, candidateCount);
retainedSavingFraction = zeros(1, candidateCount);
feasible = false(1, candidateCount);
candidateMessageCount = zeros(1, candidateCount);
for candidateIdx = 1:candidateCount
    candidate = normalizeAssignment(candidateAssignments{candidateIdx});
    if ~isequal(candidate(:, 1:2), reference(:, 1:2)) || ...
            nnz(any(candidate(:, 3:4) ~= reference(:, 3:4), 2)) ~= 1
        error('PooledExpectedGatewayV256:CommunicationActionScope', ...
            'Every V256 communication projection must change one gateway arc.');
    end
    candidateCrossMask = assignmentMask( ...
        candidate, sensorPhysicalUids, nodeCount);
    candidateAdjacency = localBackbone | candidateCrossMask;
    if any(candidateAdjacency(:) & ~physicalAdjacency(:)) || ...
            nnz(candidateAdjacency) ~= nnz(v242Adjacency) || ...
            ~isStronglyConnected(candidateAdjacency)
        error('PooledExpectedGatewayV256:InfeasibleCommunicationAction', ...
            'A projected V256 gateway action violates topology invariants.');
    end
    candidateMessageCount(candidateIdx) = nnz(candidateAdjacency);
    candidatePageBytes(candidateIdx) = ...
        adjacencyBytes(candidateAdjacency, nodePayloadBytes);
    candidateHoldBytes(candidateIdx) = ...
        horizonSteps * candidatePageBytes(candidateIdx) + controlBytes;
    candidateIncrementalBytes(candidateIdx) = ...
        candidateHoldBytes(candidateIdx) - v242HoldBytes;
    retainedSavingFraction(candidateIdx) = ...
        (denseHoldBytes - candidateHoldBytes(candidateIdx)) / ...
        denseToV242SavingBytes;
    feasible(candidateIdx) = ...
        candidateIncrementalBytes(candidateIdx) <= ...
            availableCreditBytes + 1e-9 && ...
        retainedSavingFraction(candidateIdx) + 1e-12 >= ...
            protocol.minimumRetainedDenseCausalSavingFraction;
end

projection = struct();
projection.contractVersion = ...
    'pooled-expected-gateway-v256-communication-projection-v1';
projection.referenceMode = protocol.communicationCreditReferenceMode;
projection.estimateMode = protocol.communicationCreditEstimateMode;
projection.horizonSteps = horizonSteps;
projection.nodePayloadBytes = nodePayloadBytes;
projection.denseReferenceMessageCount = nnz(denseAdjacency);
projection.v242MessageCount = nnz(v242Adjacency);
projection.candidateMessageCount = candidateMessageCount;
projection.denseReferenceEstimatedPageBytes = densePageBytes;
projection.v242EstimatedPageBytes = v242PageBytes;
projection.candidateEstimatedPageBytes = candidatePageBytes;
projection.denseReferenceEstimatedHoldBytes = denseHoldBytes;
projection.v242EstimatedHoldBytes = v242HoldBytes;
projection.candidateEstimatedHoldBytes = candidateHoldBytes;
projection.controlAttemptedBytesPerAction = controlBytes;
projection.denseToV242SavingBytes = denseToV242SavingBytes;
projection.availableCreditBytes = availableCreditBytes;
projection.candidateIncrementalBytesRelativeV242 = ...
    candidateIncrementalBytes;
projection.candidateRetainedDenseSavingFraction = ...
    retainedSavingFraction;
projection.candidateFeasible = feasible;
projection.communicationCreditFraction = ...
    protocol.communicationCreditFraction;
projection.minimumRetainedDenseSavingFraction = ...
    protocol.minimumRetainedDenseCausalSavingFraction;
projection.truthUsed = false;
projection.futureInformationUsed = false;
end

function bytes = adjacencyBytes(adjacency, nodePayloadBytes)
bytes = sum(sum(bsxfun(@times, ...
    double(adjacency), nodePayloadBytes)));
end

function mask = assignmentMask(assignment, sensorUids, nodeCount)
mask = false(nodeCount);
for rowIdx = 1:size(assignment, 1)
    sender = find(sensorUids == assignment(rowIdx, 3), 1);
    receiver = find(sensorUids == assignment(rowIdx, 4), 1);
    if isempty(sender) || isempty(receiver)
        error('PooledExpectedGatewayV256:UnknownCommunicationSensor', ...
            'A V256 gateway assignment references an unknown sensor UID.');
    end
    mask(receiver, sender) = true;
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('PooledExpectedGatewayV256:InvalidCommunicationAssignment', ...
        'V256 gateway assignments must be finite K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end
