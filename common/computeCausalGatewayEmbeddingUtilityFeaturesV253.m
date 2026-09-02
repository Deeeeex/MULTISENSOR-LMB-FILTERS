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
features = [meanFeatures, edgeFeatures, formationFeatures];
names = [meanNames, edgeNames, formationNames];
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
details.formationTailFeatureCount = numel(names);
details.meanOnlyMask = [true(1, numel(meanNames)), ...
    false(1, numel(edgeNames) + numel(formationNames))];
details.edgeDistributionMask = [ ...
    true(1, numel(meanNames) + numel(edgeNames)), ...
    false(1, numel(formationNames))];
details.edgeTailStatistics = statistics;
details.receiverFormationStatistics = formationStatistics;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function value = stableStd(rows)
if size(rows, 1) <= 1
    value = zeros(1, size(rows, 2));
else
    value = std(rows, 0, 1);
end
end
