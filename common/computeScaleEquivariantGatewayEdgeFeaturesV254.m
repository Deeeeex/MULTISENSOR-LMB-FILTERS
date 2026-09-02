function [features, names, details] = ...
        computeScaleEquivariantGatewayEdgeFeaturesV254( ...
            context, referenceAssignment)
% COMPUTESCALEEQUIVARIANTGATEWAYEDGEFEATURESV254 Additive edge values.
%
% features(receiverIndex, senderIndex, :) contains the contribution of one
% physically available cross-formation gateway edge.  Summing these rows
% over a complete gateway assignment yields the assignment representation
% used for learning.  The same linear readout can therefore be optimized
% exactly by projectScaleEquivariantGatewayAssignmentV254.

requiredContext = {'localPosteriorBySensor', 'model', ...
    'physicalAdjacency', 'positions', 'commConfig', 'currentTime', ...
    'sensorPhysicalUids', 'formationPhysicalUidsBySensor'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, requiredContext))
    error('ScaleEquivariantGatewayV254:InvalidFeatureContext', ...
        'The V254 edge-feature context is incomplete.');
end

nodeCount = numel(context.localPosteriorBySensor);
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
formationUids = unique(formationBySensor, 'stable');
physical = logical(context.physicalAdjacency);
if nodeCount == 0 || numel(sensorUids) ~= nodeCount || ...
        numel(unique(sensorUids)) ~= nodeCount || ...
        numel(formationBySensor) ~= nodeCount || ...
        ~isequal(size(physical), [nodeCount, nodeCount]) || ...
        ~isequal(size(context.positions), [2, nodeCount])
    error('ScaleEquivariantGatewayV254:InvalidFeatureIdentity', ...
        'The V254 physical identity or current page is malformed.');
end

reference = normalizeAssignment(referenceAssignment);
validateReferenceAssignment( ...
    reference, sensorUids, formationBySensor, formationUids, physical);
arcCount = size(reference, 1);

[directedFeatures, directedNames] = ...
    computeDirectedRoutingFeatures(context, struct());
selectedNames = { ...
    'link_reliability', ...
    'normalized_sensor_distance', ...
    'source_expected_cardinality', ...
    'receiver_expected_cardinality', ...
    'source_cardinality_variance', ...
    'receiver_cardinality_variance', ...
    'source_position_variance', ...
    'receiver_position_variance', ...
    'source_association_confidence', ...
    'receiver_association_confidence', ...
    'source_detection_mass', ...
    'receiver_detection_mass', ...
    'positive_existence_gain', ...
    'negative_existence_gap', ...
    'positive_precision_gain', ...
    'negative_precision_gap', ...
    'normalized_state_discrepancy', ...
    'active_label_overlap', ...
    'node_quality_advantage', ...
    'receiver_need'};
pairwiseLabelNames = { ...
    'positive_existence_gain', ...
    'negative_existence_gap', ...
    'positive_precision_gain', ...
    'negative_precision_gap', ...
    'normalized_state_discrepancy', ...
    'active_label_overlap'};
selectedIndices = zeros(1, numel(selectedNames));
for featureIdx = 1:numel(selectedNames)
    selectedIndices(featureIdx) = findFeature( ...
        directedNames, selectedNames{featureIdx});
end

history = getField(context, 'previousAdjacencyHistory', ...
    false(nodeCount, nodeCount, 0));
if isempty(history)
    recentSelection = zeros(nodeCount);
elseif size(history, 1) ~= nodeCount || size(history, 2) ~= nodeCount
    error('ScaleEquivariantGatewayV254:InvalidFeatureHistory', ...
        'The V254 adjacency history has incompatible dimensions.');
else
    recentSelection = mean(double(history), 3);
end

[nodePayloadBytes, referenceAttemptedBytes] = ...
    estimateReferencePayload(context, reference, sensorUids);
names = cell(1, numel(selectedNames) + 2);
for featureIdx = 1:numel(selectedNames)
    names{featureIdx} = ['assignment_mean_', selectedNames{featureIdx}];
end
names{end - 1} = 'assignment_mean_recent_selection_fraction';
names{end} = 'attempted_byte_ratio_contribution';

features = nan(nodeCount, nodeCount, numel(names));
availableMask = false(nodeCount);
directedFormationPairs = reference(:, 1:2);
for receiver = 1:nodeCount
    for sender = 1:nodeCount
        if receiver == sender || ~physical(receiver, sender)
            continue;
        end
        senderFormation = formationBySensor(sender);
        receiverFormation = formationBySensor(receiver);
        if ~any(directedFormationPairs(:, 1) == senderFormation & ...
                directedFormationPairs(:, 2) == receiverFormation)
            continue;
        end
        row = reshape(directedFeatures( ...
            receiver, sender, selectedIndices), 1, []);
        if any(~isfinite(row))
            error('ScaleEquivariantGatewayV254:InvalidDirectedFeature', ...
                'A current physical gateway has a non-finite edge feature.');
        end
        features(receiver, sender, :) = reshape([ ...
            row / arcCount, ...
            recentSelection(receiver, sender) / arcCount, ...
            nodePayloadBytes(sender) / referenceAttemptedBytes], ...
            1, 1, []);
        availableMask(receiver, sender) = true;
    end
end

referenceMask = assignmentMask(reference, sensorUids, nodeCount);
if any(referenceMask(:) & ~availableMask(:))
    error('ScaleEquivariantGatewayV254:UnavailableReferenceFeature', ...
        'A reference gateway lacks a current V254 edge representation.');
end

details = struct();
details.contractVersion = ...
    'scale-equivariant-gateway-v254-additive-edge-features-v1';
details.featureNames = names;
details.selectedDirectedEdgeFeatureNames = selectedNames;
compactBaseMask = ~ismember(selectedNames, pairwiseLabelNames);
details.compactTelemetryMask = [compactBaseMask, true, true];
details.richPairwiseMask = true(1, numel(names));
details.compactTelemetryFeatureCount = ...
    nnz(details.compactTelemetryMask);
details.richPairwiseFeatureCount = nnz(details.richPairwiseMask);
details.pairwiseLabelFeatureNames = pairwiseLabelNames;
details.availableEdgeMask = availableMask;
details.referenceEdgeMask = referenceMask;
details.referenceAssignment = reference;
details.directedFormationArcCount = arcCount;
details.nodeFullLmbPayloadBytes = nodePayloadBytes;
details.referenceEstimatedAttemptedBytes = referenceAttemptedBytes;
details.assignmentEmbeddingOperation = 'sum-selected-edge-contributions';
details.nonPayloadContributionsNormalizeToAssignmentMean = true;
details.payloadDifferenceEqualsEstimatedAttemptedByteRatioDifference = true;
details.sensorPermutationEquivariant = true;
details.formationPermutationEquivariant = true;
details.numericSensorIdentifiersUsedAsFeatures = false;
details.numericFormationIdentifiersUsedAsFeatures = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function [nodeBytes, referenceTotal] = ...
        estimateReferencePayload(context, reference, sensorUids)
nodeCount = numel(sensorUids);
nodeBytes = zeros(1, nodeCount);
for sensorIdx = 1:nodeCount
    estimate = estimateLmbPayloadSize( ...
        context.localPosteriorBySensor{sensorIdx}, ...
        context.model, 2, struct());
    nodeBytes(sensorIdx) = estimate.estimatedBytes;
end
referenceTotal = sum(nodeBytes);
for rowIdx = 1:size(reference, 1)
    sender = find(sensorUids == reference(rowIdx, 3), 1);
    referenceTotal = referenceTotal + nodeBytes(sender);
end
if any(~isfinite(nodeBytes)) || any(nodeBytes < 0) || ...
        ~isfinite(referenceTotal) || referenceTotal <= 0
    error('ScaleEquivariantGatewayV254:InvalidPayloadFeature', ...
        'The current full-LMB payload estimate is invalid.');
end
end

function validateReferenceAssignment( ...
        reference, sensorUids, formationBySensor, formationUids, physical)
expectedRows = 2 * (numel(formationUids) - 1);
valid = size(reference, 1) == expectedRows && ...
    size(unique(reference(:, 1:2), 'rows'), 1) == expectedRows;
if valid
    undirected = sort(reference(:, 1:2), 2);
    [pairs, ~, pairIndex] = unique(undirected, 'rows');
    counts = accumarray(pairIndex, 1);
    valid = size(pairs, 1) == numel(formationUids) - 1 && ...
        all(counts == 2);
end
if valid
    for rowIdx = 1:size(reference, 1)
        sender = find(sensorUids == reference(rowIdx, 3), 1);
        receiver = find(sensorUids == reference(rowIdx, 4), 1);
        valid = ~isempty(sender) && ~isempty(receiver) && ...
            formationBySensor(sender) == reference(rowIdx, 1) && ...
            formationBySensor(receiver) == reference(rowIdx, 2) && ...
            physical(receiver, sender);
        if ~valid
            break;
        end
    end
end
if ~valid
    error('ScaleEquivariantGatewayV254:InvalidFeatureReference', ...
        'The reference is not a current physical bidirectional tree embedding.');
end
end

function mask = assignmentMask(assignment, sensorUids, nodeCount)
mask = false(nodeCount);
for rowIdx = 1:size(assignment, 1)
    sender = find(sensorUids == assignment(rowIdx, 3), 1);
    receiver = find(sensorUids == assignment(rowIdx, 4), 1);
    mask(receiver, sender) = true;
end
end

function index = findFeature(names, requested)
index = find(strcmp(names, requested), 1);
if isempty(index)
    error('ScaleEquivariantGatewayV254:MissingDirectedFeature', ...
        'The required directed feature %s is unavailable.', requested);
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('ScaleEquivariantGatewayV254:InvalidFeatureAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
