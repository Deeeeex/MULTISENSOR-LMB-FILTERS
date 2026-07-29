function [X, names] = buildFormationTreeContextFeatures( ...
        rawX, rawNames, receiverIndices, senderIndices, ...
        groupIds, mode)
% BUILDFORMATIONTREECONTEXTFEATURES Add truth-free graph-relative context.

rawNames = reshape(rawNames, 1, []);
mode = lower(strrep(char(mode), '_', '-'));
switch mode
    case 'raw'
        X = rawX;
        names = rawNames;
    case 'graph-context'
        receiverRelative = normalizeWithinGroups( ...
            rawX, receiverIndices);
        receiverGroups = groupIds(receiverIndices);
        senderGroups = groupIds(senderIndices);
        groupCount = numel(unique(groupIds, 'stable'));
        formationPair = receiverGroups + ...
            groupCount * (senderGroups - 1);
        formationPairRelative = normalizeWithinGroups( ...
            rawX, formationPair);
        blockRelative = normalizeWithinGroups( ...
            rawX, ones(size(receiverIndices)));
        X = [ ...
            rawX, receiverRelative, ...
            formationPairRelative, blockRelative];
        names = [ ...
            rawNames, prefixNames(rawNames, 'receiver_rel_'), ...
            prefixNames(rawNames, 'formation_pair_rel_'), ...
            prefixNames(rawNames, 'block_rel_')];
    case 'relative-only'
        receiverRelative = normalizeWithinGroups( ...
            rawX, receiverIndices);
        receiverGroups = groupIds(receiverIndices);
        senderGroups = groupIds(senderIndices);
        groupCount = numel(unique(groupIds, 'stable'));
        formationPair = receiverGroups + ...
            groupCount * (senderGroups - 1);
        formationPairRelative = normalizeWithinGroups( ...
            rawX, formationPair);
        blockRelative = normalizeWithinGroups( ...
            rawX, ones(size(receiverIndices)));
        X = [ ...
            receiverRelative, formationPairRelative, blockRelative];
        names = [ ...
            prefixNames(rawNames, 'receiver_rel_'), ...
            prefixNames(rawNames, 'formation_pair_rel_'), ...
            prefixNames(rawNames, 'block_rel_')];
    otherwise
        error('Unknown graph-context feature mode: %s', mode);
end
end

function normalized = normalizeWithinGroups(X, groupKeys)
normalized = zeros(size(X));
keys = unique(reshape(groupKeys, 1, []), 'stable');
for key = keys
    mask = reshape(groupKeys == key, [], 1);
    groupX = X(mask, :);
    location = mean(groupX, 1);
    scale = std(groupX, 0, 1);
    scale(~isfinite(scale) | scale <= eps) = 1;
    normalized(mask, :) = bsxfun(@rdivide, ...
        bsxfun(@minus, groupX, location), scale);
end
end

function names = prefixNames(rawNames, prefix)
names = cell(size(rawNames));
for nameIdx = 1:numel(rawNames)
    names{nameIdx} = [prefix, rawNames{nameIdx}];
end
end
