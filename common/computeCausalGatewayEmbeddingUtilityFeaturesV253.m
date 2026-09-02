function [features, names, details] = ...
        computeCausalGatewayEmbeddingUtilityFeaturesV253( ...
            context, candidateAssignments, referenceAssignment, ...
            identity, options)
% COMPUTECAUSALGATEWAYEMBEDDINGUTILITYFEATURESV253 Preserve edge tails.

if nargin < 5
    options = struct();
end
if isnumeric(candidateAssignments)
    candidateAssignments = {candidateAssignments};
end
[meanFeatures, meanNames, source] = ...
    computeCausalGatewayEmbeddingCandidateFeaturesV251( ...
        context, candidateAssignments, referenceAssignment, ...
        identity, options);
baseNames = source.selectedDirectedEdgeFeatureNames;
statistics = {'std', 'min', 'max'};
edgeNames = cell(1, 2 * numel(statistics) * numel(baseNames));
cursor = 0;
for statIdx = 1:numel(statistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        edgeNames{cursor} = sprintf('assignment_%s_%s', ...
            statistics{statIdx}, baseNames{featureIdx});
    end
end
for statIdx = 1:numel(statistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        edgeNames{cursor} = sprintf('changed_delta_%s_%s', ...
            statistics{statIdx}, baseNames{featureIdx});
    end
end

formationStatistics = {'mean', 'std', 'min', 'max'};
formationNames = cell(1, ...
    2 * numel(formationStatistics) * numel(baseNames));
cursor = 0;
for statIdx = 1:numel(formationStatistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        formationNames{cursor} = sprintf( ...
            'receiver_formation_%s_%s', ...
            formationStatistics{statIdx}, baseNames{featureIdx});
    end
end
for statIdx = 1:numel(formationStatistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        formationNames{cursor} = sprintf( ...
            'receiver_formation_delta_%s_%s', ...
            formationStatistics{statIdx}, baseNames{featureIdx});
    end
end

edgeFeatures = zeros(size(meanFeatures, 1), numel(edgeNames));
formationFeatures = zeros( ...
    size(meanFeatures, 1), numel(formationNames));
referenceRows = source.referenceEdgeRows;
for candidateIdx = 1:size(meanFeatures, 1)
    rows = source.edgeRowsByCandidate{candidateIdx};
    changed = source.changedRowsByCandidate{candidateIdx};
    assignmentStats = [stableStd(rows); min(rows, [], 1); max(rows, [], 1)];
    if isempty(changed)
        deltaStats = zeros(3, size(rows, 2));
    else
        delta = rows(changed, :) - referenceRows(changed, :);
        deltaStats = [stableStd(delta); min(delta, [], 1); max(delta, [], 1)];
    end
    edgeFeatures(candidateIdx, :) = [ ...
        reshape(assignmentStats', 1, []), ...
        reshape(deltaStats', 1, [])];

    assignment = sortrows(candidateAssignments{candidateIdx}, ...
        [2, 1, 4, 3]);
    receiverFormations = unique(assignment(:, 2), 'stable');
    perFormation = zeros(numel(receiverFormations), size(rows, 2));
    perFormationDelta = zeros(size(perFormation));
    allDelta = rows - referenceRows;
    for formationIdx = 1:numel(receiverFormations)
        memberRows = assignment(:, 2) == ...
            receiverFormations(formationIdx);
        perFormation(formationIdx, :) = mean(rows(memberRows, :), 1);
        perFormationDelta(formationIdx, :) = ...
            mean(allDelta(memberRows, :), 1);
    end
    currentFormationStats = [mean(perFormation, 1); ...
        stableStd(perFormation); min(perFormation, [], 1); ...
        max(perFormation, [], 1)];
    deltaFormationStats = [mean(perFormationDelta, 1); ...
        stableStd(perFormationDelta); ...
        min(perFormationDelta, [], 1); ...
        max(perFormationDelta, [], 1)];
    formationFeatures(candidateIdx, :) = [ ...
        reshape(currentFormationStats', 1, []), ...
        reshape(deltaFormationStats', 1, [])];
end
[payloadRatio, payloadDetails] = estimateAssignmentPayloadRatios( ...
    context, candidateAssignments, referenceAssignment, identity);
payloadNames = {'estimated_attempted_byte_ratio_to_reference'};
formationTailCount = numel(meanNames) + numel(edgeNames) + ...
    numel(formationNames);
features = [meanFeatures, edgeFeatures, formationFeatures, ...
    payloadRatio];
names = [meanNames, edgeNames, formationNames, payloadNames];
if any(~isfinite(features(:)))
    error('CausalGatewayEmbeddingV253:InvalidFeatureValues', ...
        'The distribution-preserving feature matrix is not finite.');
end

details = source;
details.contractVersion = ...
    'causal-gateway-embedding-v253-utility-features-v1';
details.featureNames = names;
details.meanFeatureCount = numel(meanNames);
details.edgeDistributionFeatureCount = ...
    numel(meanNames) + numel(edgeNames);
details.formationTailFeatureCount = formationTailCount;
details.payloadAwareFeatureCount = numel(names);
details.meanOnlyMask = [true(1, numel(meanNames)), ...
    false(1, numel(edgeNames) + numel(formationNames) + 1)];
details.edgeDistributionMask = [ ...
    true(1, numel(meanNames) + numel(edgeNames)), ...
    false(1, numel(formationNames) + 1)];
details.formationTailMask = [true(1, formationTailCount), false];
details.edgeTailStatistics = statistics;
details.receiverFormationStatistics = formationStatistics;
details.payloadEstimate = payloadDetails;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function [ratios, details] = estimateAssignmentPayloadRatios( ...
        context, candidateAssignments, referenceAssignment, identity)
nodeCount = numel(context.localPosteriorBySensor);
sensorUids = reshape(identity.sensorPhysicalUids, 1, []);
if numel(sensorUids) ~= nodeCount || ...
        ~isfield(context, 'model')
    error('CausalGatewayEmbeddingV253:InvalidPayloadContext', ...
        'Payload-aware features require the current LMBs and sensor identities.');
end
nodeBytes = zeros(1, nodeCount);
for sensorIdx = 1:nodeCount
    stats = estimateLmbPayloadSize( ...
        context.localPosteriorBySensor{sensorIdx}, ...
        context.model, 2, struct());
    nodeBytes(sensorIdx) = stats.estimatedBytes;
end
reference = sortrows(referenceAssignment, [2, 1, 4, 3]);
localCycleBytes = sum(nodeBytes);
referenceCrossBytes = assignmentCrossBytes( ...
    reference, sensorUids, nodeBytes);
referenceTotalBytes = localCycleBytes + referenceCrossBytes;
if ~isfinite(referenceTotalBytes) || referenceTotalBytes <= 0
    error('CausalGatewayEmbeddingV253:InvalidPayloadEstimate', ...
        'The V242 reference payload estimate is not positive and finite.');
end
ratios = zeros(numel(candidateAssignments), 1);
for candidateIdx = 1:numel(candidateAssignments)
    assignment = sortrows(candidateAssignments{candidateIdx}, ...
        [2, 1, 4, 3]);
    candidateBytes = localCycleBytes + assignmentCrossBytes( ...
        assignment, sensorUids, nodeBytes);
    ratios(candidateIdx) = candidateBytes / referenceTotalBytes;
end
details = struct();
details.contractVersion = ...
    'v253-causal-current-full-lmb-payload-estimate-v1';
details.nodeFullLmbPayloadBytes = nodeBytes;
details.localCyclePayloadBytes = localCycleBytes;
details.referenceCrossPayloadBytes = referenceCrossBytes;
details.referenceEstimatedAttemptedBytes = referenceTotalBytes;
details.architectureMessageFormula = 'N+2(F-1)';
details.truthUsed = false;
details.futureInformationUsed = false;
end

function bytes = assignmentCrossBytes(assignment, sensorUids, nodeBytes)
bytes = 0;
for rowIdx = 1:size(assignment, 1)
    sender = find(sensorUids == assignment(rowIdx, 3), 1);
    if isempty(sender)
        error('CausalGatewayEmbeddingV253:UnknownPayloadSender', ...
            'A gateway payload estimate references an unknown sender UID.');
    end
    bytes = bytes + nodeBytes(sender);
end
end

function value = stableStd(rows)
if size(rows, 1) <= 1
    value = zeros(1, size(rows, 2));
else
    value = std(rows, 0, 1);
end
end
