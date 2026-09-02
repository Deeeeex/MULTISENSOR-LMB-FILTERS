function [adjacency, details] = ...
        selectCausalGatewayEmbeddingV250Policy(context, requestedAssignment)
% SELECTCAUSALGATEWAYEMBEDDINGV250POLICY Project one gateway assignment.
%
% requestedAssignment rows are:
% [senderFormationUid, receiverFormationUid, ...
%  senderSensorUid, receiverSensorUid].

protocol = getCausalGatewayEmbeddingV250Protocol();
[referenceAdjacency, reference] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
identity = buildIdentity(context);
referenceAssignment = assignmentFromCrossAdjacency( ...
    reference.crossResidualAdjacency, identity);
if nargin < 2 || isempty(requestedAssignment)
    requested = referenceAssignment;
else
    requested = normalizeAssignment(requestedAssignment);
end
[validStructure, requestedCross, reason] = ...
    validateAndBuildCross(requested, reference, context, identity);
requestedApplied = validStructure;
if requestedApplied
    cross = requestedCross;
    applied = requested;
else
    cross = logical(reference.crossResidualAdjacency);
    applied = referenceAssignment;
end

weights = reference.fusionWeightMatrix;
oldCross = logical(reference.crossResidualAdjacency);
for receiver = 1:identity.nodeCount
    oldSenders = find(oldCross(receiver, :));
    if ~isempty(oldSenders)
        returned = sum(weights(receiver, oldSenders));
        weights(receiver, oldSenders) = 0;
        weights(receiver, receiver) = ...
            weights(receiver, receiver) + returned;
    end
end
residualWeight = getCausalMinimumFormationBackboneV242Protocol();
residualWeight = residualWeight.crossResidualWeight;
for receiver = 1:identity.nodeCount
    sender = find(cross(receiver, :));
    if isempty(sender), continue; end
    if numel(sender) ~= 1 || ...
            weights(receiver, receiver) < residualWeight - 1e-12
        error('CausalGatewayEmbeddingV250:WeightProjection', ...
            'The selected receiver cannot fund one residual KLA input.');
    end
    weights(receiver, receiver) = ...
        weights(receiver, receiver) - residualWeight;
    weights(receiver, sender) = residualWeight;
end

dominant = logical(reference.dominantAdjacency);
adjacency = dominant | cross;
expectedMessages = identity.nodeCount + ...
    2 * (identity.formationCount - 1);
positiveAllowed = adjacency | logical(eye(identity.nodeCount));
hardGate = nnz(adjacency) == expectedMessages && ...
    ~any(adjacency(:) & ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(adjacency) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('CausalGatewayEmbeddingV250:ProjectionFailed', ...
        'The projected gateway embedding violates a hard invariant.');
end

details = reference;
details.contractVersion = ...
    'causal-gateway-embedding-v250-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'fixed-formation-tree-sensor-gateway-embedding';
details.backboneMode = details.mode;
details.referenceAdjacency = logical(referenceAdjacency);
details.referenceGatewayAssignment = referenceAssignment;
details.requestedGatewayAssignment = requested;
details.appliedGatewayAssignment = applied;
details.requestedGatewayAssignmentApplied = requestedApplied;
details.gatewayFallbackUsed = ~requestedApplied;
details.gatewayFallbackReason = reason;
details.crossResidualAdjacency = cross;
details.selectedResidualAdjacency = cross;
details.omittedResidualAdjacency = ...
    logical(reference.localResidualAdjacency);
details.fusionWeightMatrix = weights;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.architectureMinimumPassed = true;
details.instantaneousSensorStrong = true;
details.instantaneousFormationStrong = true;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'causal-gateway-embedding-v250-schedule-v1';
schedule.phase = 'v250-gateway-embedding';
schedule.referenceGatewayAssignment = referenceAssignment;
schedule.requestedGatewayAssignment = requested;
schedule.appliedGatewayAssignment = applied;
schedule.requestedGatewayAssignmentApplied = requestedApplied;
schedule.gatewayFallbackUsed = ~requestedApplied;
schedule.gatewayFallbackReason = reason;
schedule.currentMessageCount = nnz(adjacency);
details.scheduleCertificate = schedule;
end

function identity = buildIdentity(context)
required = {'localPosteriorBySensor', 'model', 'physicalAdjacency', ...
    'sensorPhysicalUids', 'formationPhysicalUidsBySensor'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required))
    error('CausalGatewayEmbeddingV250:InvalidContext', ...
        'The gateway policy context is incomplete.');
end
identity = struct();
identity.nodeCount = numel(context.localPosteriorBySensor);
identity.sensorUids = reshape(context.sensorPhysicalUids, 1, []);
identity.formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
identity.formationUids = sort(unique(identity.formationBySensor));
identity.formationCount = numel(identity.formationUids);
identity.members = cell(1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    identity.members{formationIdx} = find( ...
        identity.formationBySensor == identity.formationUids(formationIdx));
end
if numel(identity.sensorUids) ~= identity.nodeCount || ...
        numel(unique(identity.sensorUids)) ~= identity.nodeCount || ...
        numel(identity.formationBySensor) ~= identity.nodeCount
    error('CausalGatewayEmbeddingV250:InvalidIdentity', ...
        'The physical identity vectors are malformed.');
end
end

function assignment = assignmentFromCrossAdjacency(cross, identity)
[receivers, senders] = find(logical(cross));
assignment = zeros(numel(receivers), 4);
for edgeIdx = 1:numel(receivers)
    assignment(edgeIdx, :) = [ ...
        identity.formationBySensor(senders(edgeIdx)), ...
        identity.formationBySensor(receivers(edgeIdx)), ...
        identity.sensorUids(senders(edgeIdx)), ...
        identity.sensorUids(receivers(edgeIdx))];
end
assignment = normalizeAssignment(assignment);
end

function [valid, cross, reason] = validateAndBuildCross( ...
        assignment, reference, context, identity)
cross = false(identity.nodeCount);
reason = '';
expectedRows = 2 * (identity.formationCount - 1);
if size(assignment, 1) ~= expectedRows || ...
        size(unique(assignment(:, 1:2), 'rows'), 1) ~= expectedRows
    valid = false;
    reason = 'directed-formation-edge-set';
    return;
end
referencePairs = normalizeDirectedFormationPairs( ...
    reference.currentFormationTreePairs);
if ~isequal(assignment(:, 1:2), referencePairs)
    valid = false;
    reason = 'formation-tree-drift';
    return;
end
receiverIndices = zeros(1, expectedRows);
for edgeIdx = 1:expectedRows
    sender = find(identity.sensorUids == assignment(edgeIdx, 3));
    receiver = find(identity.sensorUids == assignment(edgeIdx, 4));
    if numel(sender) ~= 1 || numel(receiver) ~= 1 || ...
            identity.formationBySensor(sender) ~= assignment(edgeIdx, 1) || ...
            identity.formationBySensor(receiver) ~= assignment(edgeIdx, 2) || ...
            ~logical(context.physicalAdjacency(receiver, sender))
        valid = false;
        reason = 'physical-sensor-edge';
        return;
    end
    receiverIndices(edgeIdx) = receiver;
    cross(receiver, sender) = true;
end
for formationUid = identity.formationUids
    rows = assignment(:, 2) == formationUid;
    if numel(unique(receiverIndices(rows))) ~= nnz(rows)
        valid = false;
        reason = 'receiver-conflict';
        return;
    end
end
valid = true;
end

function directed = normalizeDirectedFormationPairs(undirected)
undirected = sortrows(sort(undirected, 2), [1, 2]);
directed = [undirected; undirected(:, [2, 1])];
directed = sortrows(directed, [2, 1]);
end

function assignment = normalizeAssignment(assignment)
if isempty(assignment)
    assignment = zeros(0, 4);
    return;
end
if ~isnumeric(assignment) || size(assignment, 2) ~= 4 || ...
        any(~isfinite(assignment(:)))
    error('CausalGatewayEmbeddingV250:InvalidAssignment', ...
        'Gateway assignments must be finite K-by-4 matrices.');
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
