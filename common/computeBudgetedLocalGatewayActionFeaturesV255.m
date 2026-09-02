function [features, names, details] = ...
        computeBudgetedLocalGatewayActionFeaturesV255( ...
            context, candidateAssignments, referenceAssignment)
% COMPUTEBUDGETEDLOCALGATEWAYACTIONFEATURESV255 Encode one edge replacement.

if isnumeric(candidateAssignments)
    candidateAssignments = {candidateAssignments};
end
if ~iscell(candidateAssignments) || isempty(candidateAssignments)
    error('BudgetedLocalGatewayRepairV255:InvalidFeatureCandidates', ...
        'V255 requires at least one single-arc candidate assignment.');
end

[edgeFeatures, edgeNames, edgeDetails] = ...
    computeScaleEquivariantGatewayEdgeFeaturesV254( ...
        context, referenceAssignment);
compactMask = logical(edgeDetails.compactTelemetryMask);
compactNames = edgeNames(compactMask);
names = [prefixNames(compactNames, 'candidate_'), ...
    prefixNames(compactNames, 'incumbent_')];
features = zeros(numel(candidateAssignments), numel(names));
changedRows = zeros(1, numel(candidateAssignments));
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
reference = normalizeAssignment(referenceAssignment);
arcCount = edgeDetails.directedFormationArcCount;

for candidateIdx = 1:numel(candidateAssignments)
    candidate = normalizeAssignment(candidateAssignments{candidateIdx});
    if ~isequal(candidate(:, 1:2), reference(:, 1:2))
        error('BudgetedLocalGatewayRepairV255:FeatureTreeDrift', ...
            'A V255 action changed the directed formation tree.');
    end
    changed = find(any(candidate(:, 3:4) ~= reference(:, 3:4), 2));
    if numel(changed) ~= 1
        error('BudgetedLocalGatewayRepairV255:FeatureScope', ...
            'Every V255 learned action must replace exactly one gateway arc.');
    end
    changedRows(candidateIdx) = changed;
    candidateRow = edgeRow(edgeFeatures, candidate(changed, :), ...
        sensorUids, compactMask, arcCount);
    incumbentRow = edgeRow(edgeFeatures, reference(changed, :), ...
        sensorUids, compactMask, arcCount);
    features(candidateIdx, :) = [candidateRow, incumbentRow];
end

if any(~isfinite(features(:)))
    error('BudgetedLocalGatewayRepairV255:InvalidFeatureValues', ...
        'A V255 local action feature is not finite.');
end
details = struct();
details.contractVersion = ...
    'budgeted-local-gateway-action-features-v255-v1';
details.sourceFeatureContractVersion = edgeDetails.contractVersion;
details.featureNames = names;
details.compactSourceFeatureNames = compactNames;
details.featureCount = numel(names);
details.candidateCount = numel(candidateAssignments);
details.changedReferenceRows = changedRows;
details.maximumChangedDirectedGatewayArcs = 1;
details.actionRepresentation = ...
    'candidate-edge-concatenated-with-incumbent-edge';
details.nonPayloadEdgeNormalizationRemoved = true;
details.payloadFeatureRemainsReferenceRatio = true;
details.sensorPermutationEquivariant = true;
details.formationPermutationEquivariant = true;
details.numericSensorIdentifiersUsedAsFeatures = false;
details.numericFormationIdentifiersUsedAsFeatures = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function row = edgeRow( ...
        tensor, assignmentRow, sensorUids, compactMask, arcCount)
sender = find(sensorUids == assignmentRow(3), 1);
receiver = find(sensorUids == assignmentRow(4), 1);
if isempty(sender) || isempty(receiver)
    error('BudgetedLocalGatewayRepairV255:UnknownFeatureSensor', ...
        'A V255 local action references an unknown sensor UID.');
end
row = reshape(tensor(receiver, sender, compactMask), 1, []);
row(1:end-1) = row(1:end-1) * arcCount;
end

function names = prefixNames(source, prefix)
names = cell(size(source));
for nameIdx = 1:numel(source)
    names{nameIdx} = [prefix, source{nameIdx}];
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('BudgetedLocalGatewayRepairV255:InvalidFeatureAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end
