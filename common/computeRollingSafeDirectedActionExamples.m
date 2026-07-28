function [X, featureNames, receiverIndices, senderIndices, metadata] = ...
    computeRollingSafeDirectedActionExamples( ...
        context, sourceWeight, options)
% COMPUTEROLLINGSAFEDIRECTEDACTIONEXAMPLES Truth-free rolling edge features.
%
% One row is emitted for every physically available cross-formation
% receiver-sender action. The representation combines the existing
% scale-invariant posterior/link features with executed-history and
% rolling-phase features needed by a B=3 policy. No target truth is read.

if nargin < 2 || isempty(sourceWeight)
    sourceWeight = 0.70;
end
if nargin < 3 || isempty(options)
    options = struct();
end
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('Rolling-safe action sourceWeight must lie inside (0,1).');
end
featureContextMode = lower(strrep(char(getField( ...
    options, 'featureContextMode', 'raw')), '_', '-'));

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('Rolling-safe action features need an S-by-S physical graph.');
end
physical(1:nodeCount+1:end) = false;
groupIds = resolveGroupIds(context.model, nodeCount);

[featureTensor, baseFeatureNames] = ...
    computeScaleInvariantDirectedActionFeatures( ...
        context, sourceWeight);
[baseX, receiverIndices, senderIndices] = ...
    unpackCrossFormationExamples( ...
        featureTensor, physical, groupIds);
history = resolveHistory(context, nodeCount);
historyX = computeHistoryFeatures( ...
    history, physical, groupIds, receiverIndices, ...
    senderIndices, context.currentTime);
historyFeatureNames = { ...
    'history_edge_frequency', ...
    'history_edge_ever_used', ...
    'receiver_history_source_diversity', ...
    'formation_pair_history_frequency', ...
    'latest_formation_pair_active', ...
    'receiver_cross_degree_fraction', ...
    'sender_cross_reach_fraction', ...
    'history_depth_fraction', ...
    'rolling_phase_sin', ...
    'rolling_phase_cos'};

rawX = [baseX, historyX];
rawNames = [reshape(baseFeatureNames, 1, []), ...
    historyFeatureNames];
[X, featureNames] = buildFormationTreeContextFeatures( ...
    rawX, rawNames, receiverIndices, senderIndices, ...
    groupIds, featureContextMode);
if any(~isfinite(X(:)))
    error('Rolling-safe action features contain non-finite values.');
end

metadata = struct();
metadata.contractVersion = ...
    'rolling-safe-directed-action-features-v1';
metadata.sourceWeight = sourceWeight;
metadata.featureContextMode = featureContextMode;
metadata.rawFeatureNames = rawNames;
metadata.featureNames = featureNames;
metadata.posteriorUsed = true;
metadata.currentLinkReliabilityUsed = true;
metadata.historyUsed = true;
metadata.currentGeometryUsed = true;
metadata.truthUsed = false;
metadata.groundTruthUsed = false;
metadata.futureOutcomeUsed = false;
metadata.previousAdjacencyConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
metadata.historyDepth = size(history, 3);
metadata.nodeCount = nodeCount;
metadata.crossFormationExampleCount = numel(receiverIndices);
metadata.candidateEdgeKeys = receiverIndices + ...
    nodeCount * (senderIndices - 1);
metadata.featureContractSha256 = computeTextSha256(sprintf( ...
    '%s|%.17g|%s', metadata.contractVersion, ...
    sourceWeight, strjoin(featureNames, char(31))));
end

function [X, receiverIndices, senderIndices] = ...
        unpackCrossFormationExamples(tensor, physical, groupIds)
nodeCount = size(physical, 1);
featureCount = size(tensor, 4);
X = zeros(0, featureCount);
receiverIndices = zeros(0, 1);
senderIndices = zeros(0, 1);
for receiverIdx = 1:nodeCount
    senders = reshape(find(physical(receiverIdx, :)), 1, []);
    for senderIdx = senders
        if groupIds(receiverIdx) == groupIds(senderIdx)
            continue;
        end
        row = reshape(tensor( ...
            receiverIdx, senderIdx, 1, :), 1, []);
        if any(~isfinite(row))
            continue;
        end
        X(end + 1, :) = row; %#ok<AGROW>
        receiverIndices(end + 1, 1) = receiverIdx; %#ok<AGROW>
        senderIndices(end + 1, 1) = senderIdx; %#ok<AGROW>
    end
end
if isempty(receiverIndices)
    error('No physical cross-formation rolling action is available.');
end
end

function historyX = computeHistoryFeatures( ...
        history, physical, groupIds, receiverIndices, ...
        senderIndices, currentTime)
nodeCount = size(physical, 1);
exampleCount = numel(receiverIndices);
historyDepth = size(history, 3);
historyX = zeros(exampleCount, 10);
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
receiverSourceDiversity = zeros(1, nodeCount);
if historyDepth > 0
    unionHistory = any(history, 3);
    receiverSourceDiversity = ...
        sum(unionHistory, 2)' / max(historyDepth, 1);
end
formationHistory = false(groupCount, groupCount, historyDepth);
for historyIdx = 1:historyDepth
    page = history(:, :, historyIdx);
    for receiverIdx = 1:nodeCount
        receiverGroup = find( ...
            groups == groupIds(receiverIdx), 1);
        for senderIdx = reshape(find(page(receiverIdx, :)), 1, [])
            senderGroup = find(groups == groupIds(senderIdx), 1);
            if receiverGroup ~= senderGroup
                formationHistory( ...
                    receiverGroup, senderGroup, historyIdx) = true;
            end
        end
    end
end

crossPhysical = false(nodeCount);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        crossPhysical(receiverIdx, senderIdx) = ...
            groupIds(receiverIdx) ~= groupIds(senderIdx);
    end
end
receiverCrossDegree = sum(crossPhysical, 2) / max(nodeCount - 1, 1);
senderCrossReach = sum(crossPhysical, 1)' / max(nodeCount - 1, 1);
phase = mod(round(currentTime), 3);
phaseAngle = 2 * pi * phase / 3;
for exampleIdx = 1:exampleCount
    receiverIdx = receiverIndices(exampleIdx);
    senderIdx = senderIndices(exampleIdx);
    receiverGroup = find( ...
        groups == groupIds(receiverIdx), 1);
    senderGroup = find(groups == groupIds(senderIdx), 1);
    if historyDepth > 0
        edgeTrace = reshape( ...
            history(receiverIdx, senderIdx, :), 1, []);
        pairTrace = reshape(formationHistory( ...
            receiverGroup, senderGroup, :), 1, []);
        edgeFrequency = mean(edgeTrace);
        pairFrequency = mean(pairTrace);
        latestPairActive = double(pairTrace(end));
    else
        edgeFrequency = 0;
        pairFrequency = 0;
        latestPairActive = 0;
    end
    historyX(exampleIdx, :) = [ ...
        edgeFrequency, ...
        double(edgeFrequency > 0), ...
        receiverSourceDiversity(receiverIdx), ...
        pairFrequency, latestPairActive, ...
        receiverCrossDegree(receiverIdx), ...
        senderCrossReach(senderIdx), ...
        min(historyDepth / 2, 1), ...
        sin(phaseAngle), cos(phaseAngle)];
end
end

function history = resolveHistory(context, nodeCount)
if isfield(context, 'previousAdjacencyHistory') && ...
        ~isempty(context.previousAdjacencyHistory)
    history = logical(context.previousAdjacencyHistory);
elseif isfield(context, 'previousAdjacency') && ...
        ~isempty(context.previousAdjacency)
    history = logical(context.previousAdjacency);
else
    history = false(nodeCount, nodeCount, 0);
end
if ndims(history) == 2
    history = reshape(history, nodeCount, nodeCount, 1);
end
if size(history, 1) ~= nodeCount || size(history, 2) ~= nodeCount
    error('Rolling-safe action history must be S-by-S-by-H.');
end
for historyIdx = 1:size(history, 3)
    page = history(:, :, historyIdx);
    page(1:nodeCount+1:end) = false;
    history(:, :, historyIdx) = page;
end
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Rolling-safe action features require sensorGroupIds.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('Rolling-safe action feature sensorGroupIds are invalid.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
