function [features, names, details] = ...
        computeScaleEquivariantGatewayAssignmentFeaturesV254( ...
            context, candidateAssignments, referenceAssignment)
% COMPUTESCALEEQUIVARIANTGATEWAYASSIGNMENTFEATURESV254 Sum edge values.

if isnumeric(candidateAssignments)
    candidateAssignments = {candidateAssignments};
end
if ~iscell(candidateAssignments) || isempty(candidateAssignments)
    error('ScaleEquivariantGatewayV254:InvalidCandidateAssignments', ...
        'V254 requires at least one complete gateway assignment.');
end

[edgeFeatures, names, edgeDetails] = ...
    computeScaleEquivariantGatewayEdgeFeaturesV254( ...
        context, referenceAssignment);
reference = edgeDetails.referenceAssignment;
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
features = zeros(numel(candidateAssignments), numel(names));
for candidateIdx = 1:numel(candidateAssignments)
    assignment = normalizeAssignment(candidateAssignments{candidateIdx});
    if ~isequal(assignment(:, 1:2), reference(:, 1:2))
        error('ScaleEquivariantGatewayV254:CandidateTreeDrift', ...
            'A V254 candidate changed the directed formation tree.');
    end
    receivers = zeros(size(assignment, 1), 1);
    senders = zeros(size(assignment, 1), 1);
    for rowIdx = 1:size(assignment, 1)
        senders(rowIdx) = findUid( ...
            sensorUids, assignment(rowIdx, 3));
        receivers(rowIdx) = findUid( ...
            sensorUids, assignment(rowIdx, 4));
        if ~edgeDetails.availableEdgeMask( ...
                receivers(rowIdx), senders(rowIdx))
            error('ScaleEquivariantGatewayV254:UnavailableCandidateEdge', ...
                'A V254 candidate uses an unavailable gateway edge.');
        end
    end
    for receiverUid = unique(assignment(:, 2))'
        rows = assignment(:, 2) == receiverUid;
        if numel(unique(receivers(rows))) ~= nnz(rows)
            error('ScaleEquivariantGatewayV254:CandidateReceiverConflict', ...
                'A V254 candidate reuses one receiving sensor.');
        end
    end
    selected = sub2ind(size(edgeDetails.availableEdgeMask), ...
        receivers, senders);
    for featureIdx = 1:numel(names)
        page = edgeFeatures(:, :, featureIdx);
        features(candidateIdx, featureIdx) = sum(page(selected));
    end
end
if any(~isfinite(features(:)))
    error('ScaleEquivariantGatewayV254:InvalidAssignmentFeatures', ...
        'The V254 assignment representation is not finite.');
end

details = edgeDetails;
details.contractVersion = ...
    'scale-equivariant-gateway-v254-additive-assignment-features-v1';
details.candidateCount = numel(candidateAssignments);
details.referenceAssignmentFeatures = ...
    sumReference(edgeFeatures, reference, sensorUids);
details.assignmentRepresentationIsProjectable = true;
end

function row = sumReference(edgeFeatures, reference, sensorUids)
row = zeros(1, size(edgeFeatures, 3));
for edgeIdx = 1:size(reference, 1)
    sender = findUid(sensorUids, reference(edgeIdx, 3));
    receiver = findUid(sensorUids, reference(edgeIdx, 4));
    row = row + reshape(edgeFeatures(receiver, sender, :), 1, []);
end
end

function index = findUid(uids, requested)
index = find(uids == requested, 1);
if isempty(index)
    error('ScaleEquivariantGatewayV254:UnknownCandidateSensor', ...
        'A V254 assignment references an unknown sensor UID.');
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('ScaleEquivariantGatewayV254:InvalidCandidateAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end
