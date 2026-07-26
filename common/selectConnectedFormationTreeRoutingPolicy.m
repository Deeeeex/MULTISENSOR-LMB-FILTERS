function [adjacency, details] = ...
    selectConnectedFormationTreeRoutingPolicy(context, options)
% SELECTCONNECTEDFORMATIONTREEROUTINGPOLICY Safe mixed-weight tree routing.
%
% A balanced intra-formation round-robin supplies one route per receiver.
% Cross-formation edge scores are projected onto a rooted spanning tree.
% High-confidence information edges use the registered KLA source weight;
% connectivity-only bridge edges use a smaller positive source weight.

if nargin < 2 || isempty(options)
    options = buildFormationGatewayRoutingOptions( ...
        'connected-tree-v2');
elseif ~isfield(options, 'presetName')
    options = buildFormationGatewayRoutingOptions( ...
        'connected-tree-v2', options);
end
if ~isfield(options, 'requirePreviousUnionStrongConnectivity')
    options.requirePreviousUnionStrongConnectivity = false;
end
validateOptions(options);
timerId = tic;

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;
groupIds = resolveGroupIds(context.model, nodeCount);

if isfield(options, 'actionFeatures') && ...
        isfield(options, 'actionFeatureNames')
    if ~isfield(options, 'actionFeatureMetadata')
        error([ ...
            'Injected connected-tree features require explicit ', ...
            'actionFeatureMetadata provenance.']);
    end
    actionFeatures = options.actionFeatures;
    featureNames = reshape(options.actionFeatureNames, 1, []);
    featureMetadata = options.actionFeatureMetadata;
else
    [actionFeatures, featureNames, featureMetadata] = ...
        computeScaleInvariantDirectedActionFeatures( ...
            context, options.sourceWeight);
end
if ~isfield(featureMetadata, 'truthUsed') || ...
        featureMetadata.truthUsed
    error([ ...
        'Connected-tree feature provenance must explicitly certify ', ...
        'truthUsed=false.']);
end
if size(actionFeatures, 1) ~= nodeCount || ...
        size(actionFeatures, 2) ~= nodeCount
    error('Connected-tree action features must be S-by-S-by-W-by-F.');
end
weightIdx = resolveWeightIndex( ...
    featureMetadata, options.sourceWeight);

reliability = feature( ...
    actionFeatures, featureNames, 'link_reliability', weightIdx);
positiveExistence = feature( ...
    actionFeatures, featureNames, ...
    'positive_existence_gain', weightIdx);
positivePrecision = feature( ...
    actionFeatures, featureNames, ...
    'positive_precision_gain', weightIdx);
negativeExistence = feature( ...
    actionFeatures, featureNames, ...
    'negative_existence_gap', weightIdx);
negativePrecision = feature( ...
    actionFeatures, featureNames, ...
    'negative_precision_gap', weightIdx);
discrepancy = feature( ...
    actionFeatures, featureNames, ...
    'normalized_state_discrepancy', weightIdx);
overlap = feature( ...
    actionFeatures, featureNames, ...
    'active_label_overlap', weightIdx);
receiverNeed = feature( ...
    actionFeatures, featureNames, 'receiver_need', weightIdx);
alpha = options.sourceWeight;
positiveInformation = ...
    options.positiveExistenceWeight * positiveExistence + ...
    options.positivePrecisionWeight * positivePrecision;
utility = reliability .* alpha .* ...
    (1 + options.receiverNeedMultiplier * receiverNeed) .* ...
    positiveInformation ...
    - alpha * ( ...
        options.negativeExistencePenalty * negativeExistence + ...
        options.negativePrecisionPenalty * negativePrecision) ...
    - options.discrepancyPenalty * alpha^2 .* ...
        overlap .* discrepancy;
utility(~physical) = NaN;

[~, baselineDetails] = selectRegisteredDirectedRoutingPolicy( ...
    context, options.baselineMode, struct( ...
        'sourceWeight', alpha, ...
        'phase', options.baselinePhase, ...
        'anchorTime', options.anchorTime));
baselineSources = ...
    baselineDetails.selectedSourcesByReceiver;
baselineUtility = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    baselineUtility(receiverIdx) = ...
        utility(receiverIdx, baselineSources(receiverIdx));
end

previous = false(nodeCount);
if isfield(context, 'previousAdjacency') && ...
        isequal(size(context.previousAdjacency), ...
            [nodeCount, nodeCount])
    previous = logical(context.previousAdjacency);
end
receiverIndices = zeros(0, 1);
senderIndices = zeros(0, 1);
marginScores = zeros(0, 1);
highConfidence = false(0, 1);
for receiverIdx = 1:nodeCount
    for senderIdx = reshape(find(physical(receiverIdx, :)), 1, [])
        if groupIds(receiverIdx) == groupIds(senderIdx)
            continue;
        end
        margin = utility(receiverIdx, senderIdx) - ...
            baselineUtility(receiverIdx);
        requiredMargin = options.marginThreshold + ...
            options.switchHysteresis * ...
                ~previous(receiverIdx, senderIdx);
        receiverIndices(end + 1, 1) = receiverIdx; %#ok<AGROW>
        senderIndices(end + 1, 1) = senderIdx; %#ok<AGROW>
        marginScores(end + 1, 1) = margin; %#ok<AGROW>
        highConfidence(end + 1, 1) = ...
            positiveExistence(receiverIdx, senderIdx) >= ...
                options.minimumPositiveExistence && ...
            margin > requiredMargin; %#ok<AGROW>
    end
end
switch options.projectionMode
    case 'weak-tree'
        projectionOptions = struct();
        if options.requirePreviousUnionStrongConnectivity && ...
                isfield(context, 'previousAdjacency') && ...
                isequal(size(context.previousAdjacency), ...
                    [nodeCount, nodeCount])
            projectionOptions.requiredUnionFormationAdjacency = ...
                formationAdjacencyFromSensorAdjacency( ...
                    context.previousAdjacency, groupIds);
        end
        selection = selectRootedFormationTreeEdges( ...
            groupIds, receiverIndices, senderIndices, marginScores, ...
            projectionOptions);
        if strcmp(options.presetName, 'joint-tree-v4')
            policyMode = 'jointly-strong-formation-tree-v4';
        else
            policyMode = 'connected-formation-tree-v2';
        end
    case 'strong-cycle'
        selection = selectStrongFormationCycleEdges( ...
            groupIds, receiverIndices, senderIndices, marginScores);
        policyMode = 'strong-formation-cycle-v3';
    otherwise
        error('Unknown formation projection mode: %s', ...
            options.projectionMode);
end

selectedSources = baselineSources;
selectedWeights = alpha * ones(1, nodeCount);
overrideMask = false(1, nodeCount);
bridgeMask = false(1, nodeCount);
selectedMargins = zeros(1, nodeCount);
for cursor = 1:numel(selection.exampleIndices)
    exampleIdx = selection.exampleIndices(cursor);
    receiverIdx = receiverIndices(exampleIdx);
    selectedSources(receiverIdx) = senderIndices(exampleIdx);
    overrideMask(receiverIdx) = true;
    selectedMargins(receiverIdx) = marginScores(exampleIdx);
    if ~highConfidence(exampleIdx)
        selectedWeights(receiverIdx) = ...
            options.bridgeSourceWeight;
        bridgeMask(receiverIdx) = true;
    end
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nodeCount > messageBudget
    error([ ...
        'Connected-tree routing needs one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error([ ...
            'Connected-tree routing selected a non-physical route at ', ...
            't=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
    sourceWeight = selectedWeights(receiverIdx);
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
end

details = struct();
details.mode = policyMode;
details.objective = -sum(selectedMargins(overrideMask));
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = sum(selectedMargins(overrideMask));
details.taskRiskSpread = max(marginScores) - min(marginScores);
details.validCandidateCount = numel(marginScores);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.baselineSourcesByReceiver = baselineSources;
details.overrideMask = overrideMask;
details.bridgeMask = bridgeMask;
details.highConfidenceOverrideMask = overrideMask & ~bridgeMask;
details.overrideFraction = mean(overrideMask);
details.bridgeFraction = mean(bridgeMask);
details.supportedCandidateFraction = ...
    mean(overrideMask & ~bridgeMask);
details.crossFormationMessageCount = nnz(overrideMask);
details.messageCount = nnz(adjacency);
details.sourceWeight = alpha;
details.bridgeSourceWeight = options.bridgeSourceWeight;
details.rootFormation = getField(selection, 'rootFormation', NaN);
details.formationParents = getField( ...
    selection, 'formationParents', []);
details.formationCycle = getField(selection, 'formationCycle', []);
details.maximumCrossSourceLoad = ...
    selection.maximumSourceLoad;
details.posteriorUsed = true;
details.truthUsed = false;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.options = options;
end

function validateOptions(options)
finiteFields = { ...
    'sourceWeight', 'bridgeSourceWeight', ...
    'baselinePhase', 'anchorTime', ...
    'positiveExistenceWeight', 'positivePrecisionWeight', ...
    'negativeExistencePenalty', 'negativePrecisionPenalty', ...
    'discrepancyPenalty', 'receiverNeedMultiplier', ...
    'marginThreshold', 'minimumPositiveExistence', ...
    'switchHysteresis'};
for fieldIdx = 1:numel(finiteFields)
    fieldName = finiteFields{fieldIdx};
    if ~isfield(options, fieldName) || ...
            ~isscalar(options.(fieldName)) || ...
            ~isfinite(options.(fieldName))
        error('Connected-tree option %s must be finite.', fieldName);
    end
end
if options.sourceWeight <= 0 || ...
        options.sourceWeight >= 1 || ...
        options.bridgeSourceWeight <= 0 || ...
        options.bridgeSourceWeight >= ...
            options.sourceWeight
    error([ ...
        'Connected-tree weights require ', ...
        '0 < bridgeSourceWeight < sourceWeight < 1.']);
end
if ~isfield(options, 'projectionMode') || ...
        ~ischar(options.projectionMode) || ...
        ~any(strcmp(options.projectionMode, ...
            {'weak-tree', 'strong-cycle'}))
    error('Connected-tree projectionMode is invalid.');
end
if ~isfield(options, 'baselineMode') || ...
        ~ischar(options.baselineMode) || ...
        ~any(strcmp(options.baselineMode, { ...
            'fixed-index-star', 'fixed-balanced-cycle', ...
            'round-robin-balanced'}))
    error('Connected-tree baselineMode is invalid.');
end
if ~isfield(options, 'requirePreviousUnionStrongConnectivity')
    options.requirePreviousUnionStrongConnectivity = false;
end
if ~isscalar(options.requirePreviousUnionStrongConnectivity) || ...
        ~islogical(options.requirePreviousUnionStrongConnectivity)
    error([ ...
        'Connected-tree requirePreviousUnionStrongConnectivity ', ...
        'must be logical scalar.']);
end
nonnegative = setdiff(finiteFields, ...
    {'sourceWeight', 'bridgeSourceWeight', 'anchorTime'});
for fieldIdx = 1:numel(nonnegative)
    if options.(nonnegative{fieldIdx}) < 0
        error('Connected-tree option %s must be nonnegative.', ...
            nonnegative{fieldIdx});
    end
end
if mod(options.baselinePhase, 1) ~= 0 || ...
        options.baselinePhase < 1
    error('Connected-tree baselinePhase must be a positive integer.');
end
end

function index = resolveWeightIndex(metadata, sourceWeight)
index = 1;
if ~isfield(metadata, 'sourceWeightGrid')
    return;
end
[difference, index] = min(abs( ...
    reshape(metadata.sourceWeightGrid, 1, []) - sourceWeight));
if difference > 1e-12
    error('The requested source weight is absent from action features.');
end
end

function matrix = feature(features, names, name, weightIdx)
featureIdx = find(strcmp(names, name), 1);
if isempty(featureIdx)
    error('Missing connected-tree action feature: %s', name);
end
matrix = reshape( ...
    features(:, :, weightIdx, featureIdx), ...
    size(features, 1), size(features, 2));
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Connected-tree routing requires sensorGroupIds metadata.');
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(mod(groupIds, 1) ~= 0)
    error('sensorGroupIds must contain one positive integer per sensor.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
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
