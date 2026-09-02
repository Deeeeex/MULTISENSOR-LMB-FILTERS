function [features, names, details] = ...
        computeCausalGatewayEmbeddingUtilityFeaturesV253( ...
            context, candidateAssignments, referenceAssignment, ...
            identity, options)
% COMPUTECAUSALGATEWAYEMBEDDINGUTILITYFEATURESV253 Preserve edge tails.

if nargin < 5
    options = struct();
end
[meanFeatures, meanNames, source] = ...
    computeCausalGatewayEmbeddingCandidateFeaturesV251( ...
        context, candidateAssignments, referenceAssignment, ...
        identity, options);
baseNames = source.selectedDirectedEdgeFeatureNames;
statistics = {'std', 'min', 'max'};
extraNames = cell(1, 2 * numel(statistics) * numel(baseNames));
cursor = 0;
for statIdx = 1:numel(statistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        extraNames{cursor} = sprintf('assignment_%s_%s', ...
            statistics{statIdx}, baseNames{featureIdx});
    end
end
for statIdx = 1:numel(statistics)
    for featureIdx = 1:numel(baseNames)
        cursor = cursor + 1;
        extraNames{cursor} = sprintf('changed_delta_%s_%s', ...
            statistics{statIdx}, baseNames{featureIdx});
    end
end

extra = zeros(size(meanFeatures, 1), numel(extraNames));
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
    extra(candidateIdx, :) = [ ...
        reshape(assignmentStats', 1, []), ...
        reshape(deltaStats', 1, [])];
end
features = [meanFeatures, extra];
names = [meanNames, extraNames];
if any(~isfinite(features(:)))
    error('CausalGatewayEmbeddingV253:InvalidFeatureValues', ...
        'The distribution-preserving feature matrix is not finite.');
end

details = source;
details.contractVersion = ...
    'causal-gateway-embedding-v253-utility-features-v1';
details.featureNames = names;
details.meanFeatureCount = numel(meanNames);
details.distributionFeatureCount = numel(names);
details.meanOnlyMask = [true(1, numel(meanNames)), ...
    false(1, numel(extraNames))];
details.edgeTailStatistics = statistics;
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
