function [adjacency, details] = ...
    selectStructuredFormationTreeRoutingPolicy(context, modelOrPath)
% SELECTSTRUCTUREDFORMATIONTREEROUTINGPOLICY Deploy a truth-free tree ranker.
%
% A fixed-index intra-formation route supplies one message per receiver.
% The learned scores are projected exactly onto a rooted cross-formation
% tree, optionally constrained so its union with the preceding formation
% graph is strongly connected.

timerId = tic;
model = resolveModel(modelOrPath);
validateStructuredFormationTreeModelContract(model);
nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context.model, nodeCount);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;

[baselineAdjacency, baselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, model.baselineMode, ...
        struct('sourceWeight', model.sourceWeight));
selectedSources = baselineDetails.selectedSourcesByReceiver;

[actionFeatures, rawFeatureNames] = ...
    computeScaleInvariantDirectedActionFeatures( ...
        context, model.sourceWeight);
[rawX, receiverIndices, senderIndices] = ...
    unpackCrossFormationExamples( ...
        actionFeatures, groupIds, physical);
[X, featureNames] = buildFormationTreeContextFeatures( ...
    rawX, rawFeatureNames, receiverIndices, senderIndices, ...
    groupIds, model.featureContextMode);
if ~isequal(featureNames, reshape(model.allFeatureNames, 1, []))
    error('Structured formation-tree feature contract mismatch.');
end
featureMask = logical(reshape(model.featureMask, 1, []));
if numel(featureMask) ~= size(X, 2) || ~any(featureMask)
    error('Structured formation-tree retained-feature mask is invalid.');
end
scores = scoreStructuredFormationTreeRanker( ...
    model.ranker, X(:, featureMask));

projectionOptions = struct();
if model.requirePreviousUnionStrongConnectivity
    if ~isfield(context, 'previousAdjacency') || ...
            ~isequal(size(context.previousAdjacency), ...
                [nodeCount, nodeCount])
        error([ ...
            'Structured joint-tree routing requires the previous ', ...
            'sensor adjacency.']);
    end
    projectionOptions.requiredUnionFormationAdjacency = ...
        formationAdjacencyFromSensorAdjacency( ...
            context.previousAdjacency, groupIds);
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
        'Structured formation-tree routing needs one message per ', ...
        'receiver; the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~isfinite(senderIdx) || ~physical(receiverIdx, senderIdx)
        error([ ...
            'Structured formation-tree routing selected a non-physical ', ...
            'route at t=%d.'], context.currentTime);
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = ...
        1 - model.sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = model.sourceWeight;
    selectedWeights(receiverIdx) = model.sourceWeight;
end

overrideMask = false(1, nodeCount);
overrideMask(selection.receiverIndices) = true;
details = struct();
details.mode = 'structured-formation-tree-routing-v1';
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
details.sourceWeight = model.sourceWeight;
details.rootFormation = selection.rootFormation;
details.formationParents = selection.formationParents;
details.selectedPredictedScores = ...
    reshape(scores(selection.exampleIndices), 1, []);
details.maximumCrossSourceLoad = selection.maximumSourceLoad;
details.requirePreviousUnionStrongConnectivity = ...
    model.requirePreviousUnionStrongConnectivity;
details.formationUnionStrongConnected = ...
    ~model.requirePreviousUnionStrongConnectivity || ...
    selection.formationUnionStrongConnected;
details.posteriorUsed = true;
details.truthUsed = false;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.modelKind = model.kind;
details.modelTrainingBehaviorTruthUsed = logical(getField( ...
    model, 'trainingBehaviorTruthUsed', false));
details.baselineAdjacency = baselineAdjacency;
end

function [X, receiverIndices, senderIndices] = ...
        unpackCrossFormationExamples( ...
            features, groupIds, physical)
featureCount = size(features, 4);
X = zeros(0, featureCount);
receiverIndices = zeros(0, 1);
senderIndices = zeros(0, 1);
nodeCount = numel(groupIds);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        if groupIds(receiverIdx) == groupIds(senderIdx)
            continue;
        end
        row = reshape(features( ...
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
    error('No physical cross-formation action is available.');
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

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Structured formation-tree routing needs sensorGroupIds.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('Structured formation-tree sensorGroupIds are invalid.');
end
end

function model = resolveModel(modelOrPath)
if isstruct(modelOrPath)
    model = modelOrPath;
    return;
end
if isempty(modelOrPath) || ~ischar(modelOrPath)
    error('A structured tree-routing model struct or MAT path is required.');
end
loaded = load(modelOrPath, 'model');
if ~isfield(loaded, 'model')
    error('Structured tree-routing model file does not contain model.');
end
model = loaded.model;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
