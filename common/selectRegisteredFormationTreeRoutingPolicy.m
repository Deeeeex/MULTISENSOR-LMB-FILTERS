function [adjacency, details] = ...
    selectRegisteredFormationTreeRoutingPolicy(context, mode, options)
% SELECTREGISTEREDFORMATIONTREEROUTINGPOLICY Matched v4 controls.
%
% The controls share the candidate's fixed-index backbone, exact rooted
% tree projector, one-cross-send-per-sensor cap and optional two-step
% strong-connectivity constraint. They never read posterior contents or
% target truth.
%
% Supported modes:
%   scheduled-path    deterministic rotating-root path schedule
%   link-aware-tree   current link reliability followed by exact projection

if nargin < 2 || isempty(mode)
    mode = 'scheduled-path';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
sourceWeight = getField(options, 'sourceWeight', 0.70);
rootPhase = max(1, round(getField(options, 'rootPhase', ...
    getField(options, 'phase', 1))));
endpointPhase = max(1, round(getField( ...
    options, 'endpointPhase', 1)));
anchorTime = round(getField(options, 'anchorTime', 1));
requirePreviousUnionStrongConnectivity = logical(getField( ...
    options, 'requirePreviousUnionStrongConnectivity', true));
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('Formation-tree control sourceWeight must be in (0,1).');
end

nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context.model, nodeCount);
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('Formation-tree controls need at least two formations.');
end
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;

[baselineAdjacency, baselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', sourceWeight));
selectedSources = baselineDetails.selectedSourcesByReceiver;

receiverIndices = zeros(0, 1);
senderIndices = zeros(0, 1);
scores = zeros(0, 1);
desiredParents = desiredScheduledPath( ...
    groupCount, context.currentTime, anchorTime, rootPhase);
endpointRound = floor( ...
    (context.currentTime - anchorTime) / groupCount);
for receiverIdx = 1:nodeCount
    receiverGroup = find(groups == groupIds(receiverIdx), 1);
    receiverMembers = find(groupIds == groupIds(receiverIdx));
    receiverRole = find(receiverMembers == receiverIdx, 1);
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        senderGroup = find(groups == groupIds(senderIdx), 1);
        if receiverGroup == senderGroup
            continue;
        end
        senderMembers = find(groupIds == groupIds(senderIdx));
        senderRole = find(senderMembers == senderIdx, 1);
        switch mode
            case 'scheduled-path'
                desiredPair = ...
                    desiredParents(receiverGroup) == senderGroup;
                targetSenderRole = 1 + mod( ...
                    endpointRound + senderGroup - 1, ...
                    numel(senderMembers));
                targetReceiverRole = 1 + mod( ...
                    endpointRound + senderGroup - 1 + ...
                        endpointPhase - 1, ...
                    numel(receiverMembers));
                score = 1000 * desiredPair + ...
                    0.1 * (senderRole == targetSenderRole) + ...
                    0.01 * (receiverRole == targetReceiverRole);
            case 'link-aware-tree'
                score = 1 - edgeDrop( ...
                    context.commConfig, senderIdx, receiverIdx, ...
                    context.currentTime);
            otherwise
                error('Unknown registered formation-tree mode: %s', ...
                    mode);
        end
        score = score - 1e-8 * senderIdx - 1e-10 * receiverIdx;
        receiverIndices(end + 1, 1) = receiverIdx; %#ok<AGROW>
        senderIndices(end + 1, 1) = senderIdx; %#ok<AGROW>
        scores(end + 1, 1) = score; %#ok<AGROW>
    end
end

projectionOptions = struct();
if requirePreviousUnionStrongConnectivity
    if ~isfield(context, 'previousAdjacency') || ...
            ~isequal(size(context.previousAdjacency), ...
                [nodeCount, nodeCount])
        error([ ...
            'Matched formation-tree controls require the previous ', ...
            'sensor adjacency.']);
    end
    if any(context.previousAdjacency(:))
        projectionOptions.requiredUnionFormationAdjacency = ...
            formationAdjacencyFromSensorAdjacency( ...
                context.previousAdjacency, groupIds);
    end
end
selection = selectRootedFormationTreeEdges( ...
    groupIds, receiverIndices, senderIndices, scores, ...
    projectionOptions);
selectedSources(selection.receiverIndices) = ...
    selection.senderIndices;

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nodeCount > messageBudget
    error([ ...
        'Formation-tree controls need one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error('Formation-tree control selected a non-physical route.');
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

overrideMask = false(1, nodeCount);
overrideMask(selection.receiverIndices) = true;
details = struct();
details.mode = ['registered-formation-tree-', mode];
details.objective = -selection.predictedObjective;
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = max(scores) - min(scores);
details.validCandidateCount = numel(scores);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.baselineSourcesByReceiver = ...
    baselineDetails.selectedSourcesByReceiver;
details.overrideMask = overrideMask;
details.overrideFraction = mean(overrideMask);
details.crossFormationMessageCount = nnz(overrideMask);
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
details.rootPhase = rootPhase;
details.endpointPhase = endpointPhase;
details.anchorTime = anchorTime;
details.rootFormation = selection.rootFormation;
details.formationParents = selection.formationParents;
details.desiredFormationParents = desiredParents;
details.maximumCrossSourceLoad = selection.maximumSourceLoad;
details.requirePreviousUnionStrongConnectivity = ...
    requirePreviousUnionStrongConnectivity;
details.formationUnionStrongConnected = ...
    ~requirePreviousUnionStrongConnectivity || ...
    ~any(context.previousAdjacency(:)) || ...
    selection.formationUnionStrongConnected;
details.posteriorUsed = false;
details.truthUsed = false;
details.currentLinkReliabilityUsed = ...
    strcmp(mode, 'link-aware-tree');
details.currentPhysicalActionSetUsed = true;
details.baselineAdjacency = baselineAdjacency;
end

function parents = desiredScheduledPath( ...
        groupCount, currentTime, anchorTime, rootPhase)
root = 1 + mod( ...
    currentTime - anchorTime + rootPhase - 1, groupCount);
order = circshift(1:groupCount, [0, -(root - 1)]);
parents = zeros(1, groupCount);
for cursor = 2:groupCount
    parents(order(cursor)) = order(cursor - 1);
end
end

function adjacency = formationAdjacencyFromSensorAdjacency( ...
        sensorAdjacency, groupIds)
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
adjacency = false(groupCount);
sensorAdjacency = logical(sensorAdjacency);
for receiverIdx = 1:numel(groupIds)
    receiverGroup = find(groups == groupIds(receiverIdx), 1);
    for senderIdx = reshape(find(sensorAdjacency(receiverIdx, :)), 1, [])
        senderGroup = find(groups == groupIds(senderIdx), 1);
        if receiverGroup ~= senderGroup
            adjacency(senderGroup, receiverGroup) = true;
        end
    end
end
end

function probability = edgeDrop( ...
        config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Formation-tree controls need sensorGroupIds.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('Formation-tree control sensorGroupIds are invalid.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
