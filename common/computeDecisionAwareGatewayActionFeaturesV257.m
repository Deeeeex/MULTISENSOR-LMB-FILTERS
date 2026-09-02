function [features, names, details] = ...
        computeDecisionAwareGatewayActionFeaturesV257( ...
            context, candidateAssignments, referenceAssignment)
% COMPUTEDECISIONAWAREGATEWAYACTIONFEATURESV257 Rich information upper bound.

if isnumeric(candidateAssignments)
    candidateAssignments = {candidateAssignments};
end
if ~iscell(candidateAssignments) || isempty(candidateAssignments)
    error('DecisionAwareGatewayV257:InvalidFeatureCandidates', ...
        'V257 requires at least one one-arc action.');
end
[edgeFeatures, edgeNames, edgeDetails] = ...
    computeScaleEquivariantGatewayEdgeFeaturesV254( ...
        context, referenceAssignment);
richMask = logical(edgeDetails.richPairwiseMask);
compactMask = logical(edgeDetails.compactTelemetryMask);
richNames = edgeNames(richMask);
compactNames = edgeNames(compactMask);
richCount = nnz(richMask);
names = [prefixNames(richNames, 'delta_'), ...
    prefixNames(richNames, 'incumbent_')];
features = zeros(numel(candidateAssignments), 2 * richCount);
sensorUids = reshape(context.sensorPhysicalUids, 1, []);
reference = normalizeAssignment(referenceAssignment);
arcCount = edgeDetails.directedFormationArcCount;

for candidateIdx = 1:numel(candidateAssignments)
    candidate = normalizeAssignment(candidateAssignments{candidateIdx});
    if ~isequal(candidate(:, 1:2), reference(:, 1:2))
        error('DecisionAwareGatewayV257:FeatureTreeDrift', ...
            'A V257 action changed the directed formation tree.');
    end
    changed = find(any(candidate(:, 3:4) ~= reference(:, 3:4), 2));
    if numel(changed) ~= 1
        error('DecisionAwareGatewayV257:FeatureScope', ...
            'Every V257 action must replace exactly one gateway arc.');
    end
    candidateRow = edgeRow(edgeFeatures, candidate(changed, :), ...
        sensorUids, richMask, arcCount);
    incumbentRow = edgeRow(edgeFeatures, reference(changed, :), ...
        sensorUids, richMask, arcCount);
    features(candidateIdx, :) = [ ...
        candidateRow - incumbentRow, incumbentRow];
end

compactInRich = find(ismember(richNames, compactNames));
compactProjection = [compactInRich, richCount + compactInRich];
details = struct();
details.contractVersion = ...
    'decision-aware-gateway-v257-action-features-v1';
details.sourceFeatureContractVersion = edgeDetails.contractVersion;
details.featureNames = names;
details.featureCount = numel(names);
details.compactFeatureNames = [ ...
    prefixNames(compactNames, 'delta_'), ...
    prefixNames(compactNames, 'incumbent_')];
details.compactProjectionIndices = compactProjection;
details.compactFeatures = features(:, compactProjection);
details.pairwiseLabelFeatureNames = ...
    edgeDetails.pairwiseLabelFeatureNames;
details.pairwiseFeatureCountPerEdge = ...
    numel(edgeDetails.pairwiseLabelFeatureNames);
details.compactControlCosted = true;
details.richPairwiseControlCosted = false;
details.truthUsed = false;
details.futureInformationUsed = false;
if size(features, 2) ~= 44 || ...
        size(details.compactFeatures, 2) ~= 32 || ...
        any(~isfinite(features(:)))
    error('DecisionAwareGatewayV257:FeatureCount', ...
        'The V257 rich/compact feature projection drifted.');
end
end

function row = edgeRow( ...
        tensor, assignmentRow, sensorUids, mask, arcCount)
sender = find(sensorUids == assignmentRow(3), 1);
receiver = find(sensorUids == assignmentRow(4), 1);
if isempty(sender) || isempty(receiver)
    error('DecisionAwareGatewayV257:UnknownFeatureSensor', ...
        'A V257 action references an unknown sensor UID.');
end
row = reshape(tensor(receiver, sender, mask), 1, []);
row(1:end-1) = row(1:end-1) * arcCount;
end

function names = prefixNames(source, prefix)
names = cell(size(source));
for idx = 1:numel(source)
    names{idx} = [prefix, source{idx}];
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('DecisionAwareGatewayV257:InvalidAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end
