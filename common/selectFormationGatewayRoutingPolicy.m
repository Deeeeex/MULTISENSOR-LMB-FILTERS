function [adjacency, details] = ...
    selectFormationGatewayRoutingPolicy(context, options)
% SELECTFORMATIONGATEWAYROUTINGPOLICY Novelty-gated gateway handovers.
%
% Every receiver starts from a balanced intra-formation round-robin route.
% A truth-free pairwise utility may replace at most one route per formation
% with a physical cross-formation route. Total message count stays fixed.

if nargin < 2 || isempty(options)
    options = buildFormationGatewayRoutingOptions();
elseif ~isfield(options, 'presetName')
    options = buildFormationGatewayRoutingOptions( ...
        'novelty-gated-v1', options);
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
            'Injected formation-gateway features require explicit ', ...
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
if ~isfield(featureMetadata, 'truthUsed') || featureMetadata.truthUsed
    error([ ...
        'Formation-gateway feature provenance must explicitly certify ', ...
        'truthUsed=false.']);
end
if size(actionFeatures, 1) ~= nodeCount || ...
        size(actionFeatures, 2) ~= nodeCount
    error('Formation-gateway action features must be S-by-S-by-W-by-F.');
end
weightIdx = resolveWeightIndex(featureMetadata, options.sourceWeight);

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
    actionFeatures, featureNames, 'active_label_overlap', weightIdx);
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

baselineSources = roundRobinSources( ...
    groupIds, context.currentTime, ...
    options.anchorTime, options.baselinePhase);
baselineUtility = nan(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = baselineSources(receiverIdx);
    if ~isfinite(senderIdx) || ~physical(receiverIdx, senderIdx)
        error([ ...
            'Formation-gateway baseline selected a non-physical route ', ...
            'at t=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
    baselineUtility(receiverIdx) = utility(receiverIdx, senderIdx);
end

previous = false(nodeCount);
if isfield(context, 'previousAdjacency') && ...
        isequal(size(context.previousAdjacency), [nodeCount, nodeCount])
    previous = logical(context.previousAdjacency);
end
proposals = repmat(struct( ...
    'receiver', NaN, 'sender', NaN, 'formation', NaN, ...
    'margin', -inf, 'novelty', NaN), 1, 0);
supportedReceiver = false(1, nodeCount);
bestCrossSenderByReceiver = nan(1, nodeCount);
bestCrossMarginByReceiver = -inf(1, nodeCount);
for receiverIdx = 1:nodeCount
    cross = isfinite(utility(receiverIdx, :)) & ...
        groupIds ~= groupIds(receiverIdx) & ...
        positiveExistence(receiverIdx, :) >= ...
            options.minimumPositiveExistence;
    senders = find(cross);
    if isempty(senders)
        continue;
    end
    supportedReceiver(receiverIdx) = true;
    [bestUtility, bestCursor] = max(utility(receiverIdx, senders));
    senderIdx = senders(bestCursor);
    margin = bestUtility - baselineUtility(receiverIdx);
    bestCrossSenderByReceiver(receiverIdx) = senderIdx;
    bestCrossMarginByReceiver(receiverIdx) = margin;
    requiredMargin = options.marginThreshold + ...
        options.switchHysteresis * ~previous(receiverIdx, senderIdx);
    if margin <= requiredMargin
        continue;
    end
    proposals(end + 1) = struct( ... %#ok<AGROW>
        'receiver', receiverIdx, ...
        'sender', senderIdx, ...
        'formation', groupIds(receiverIdx), ...
        'margin', margin, ...
        'novelty', positiveExistence(receiverIdx, senderIdx));
end

selectedSources = baselineSources;
overrideMask = false(1, nodeCount);
selectedMargins = zeros(1, nodeCount);
if ~isempty(proposals)
    [~, order] = sort([proposals.margin], 'descend');
    formationLoad = zeros(1, max(groupIds));
    sourceLoad = zeros(1, nodeCount);
    for cursor = reshape(order, 1, [])
        proposal = proposals(cursor);
        if formationLoad(proposal.formation) >= ...
                options.maxCrossPerFormation || ...
                sourceLoad(proposal.sender) >= ...
                options.maxCrossSourceLoad
            continue;
        end
        selectedSources(proposal.receiver) = proposal.sender;
        overrideMask(proposal.receiver) = true;
        selectedMargins(proposal.receiver) = proposal.margin;
        formationLoad(proposal.formation) = ...
            formationLoad(proposal.formation) + 1;
        sourceLoad(proposal.sender) = sourceLoad(proposal.sender) + 1;
    end
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nodeCount > messageBudget
    error([ ...
        'Formation-gateway routing needs one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error([ ...
            'Formation-gateway routing selected a non-physical route ', ...
            'at t=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - alpha;
    fusionWeights(receiverIdx, senderIdx) = alpha;
    selectedWeights(receiverIdx) = alpha;
end

details = struct();
details.mode = ['formation-gateway-', options.presetName];
details.objective = sum(selectedMargins);
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = numel(proposals);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.baselineSourcesByReceiver = baselineSources;
details.bestCrossSenderByReceiver = bestCrossSenderByReceiver;
details.bestCrossMarginByReceiver = bestCrossMarginByReceiver;
details.overrideMask = overrideMask;
details.overrideFraction = mean(overrideMask);
details.supportedCandidateFraction = mean(supportedReceiver);
details.crossFormationMessageCount = nnz(overrideMask);
details.messageCount = nnz(adjacency);
details.sourceWeight = alpha;
details.posteriorUsed = true;
details.truthUsed = false;
details.currentLinkReliabilityUsed = true;
details.currentPhysicalActionSetUsed = true;
details.options = options;
end

function validateOptions(options)
finiteScalarFields = { ...
    'sourceWeight', 'baselinePhase', 'anchorTime', ...
    'positiveExistenceWeight', 'positivePrecisionWeight', ...
    'negativeExistencePenalty', 'negativePrecisionPenalty', ...
    'discrepancyPenalty', 'receiverNeedMultiplier', ...
    'marginThreshold', 'minimumPositiveExistence', ...
    'maxCrossPerFormation', 'maxCrossSourceLoad', ...
    'switchHysteresis'};
for fieldIdx = 1:numel(finiteScalarFields)
    fieldName = finiteScalarFields{fieldIdx};
    if ~isfield(options, fieldName) || ...
            ~isscalar(options.(fieldName)) || ...
            ~isfinite(options.(fieldName))
        error('Formation-gateway option %s must be a finite scalar.', ...
            fieldName);
    end
end
if options.sourceWeight <= 0 || options.sourceWeight >= 1
    error('sourceWeight must be strictly between zero and one.');
end
if options.baselinePhase < 1 || ...
        mod(options.baselinePhase, 1) ~= 0 || ...
        options.maxCrossPerFormation < 1 || ...
        mod(options.maxCrossPerFormation, 1) ~= 0 || ...
        options.maxCrossSourceLoad < 1 || ...
        mod(options.maxCrossSourceLoad, 1) ~= 0
    error('Gateway phases and capacity limits must be positive integers.');
end
nonnegative = setdiff(finiteScalarFields, ...
    {'sourceWeight', 'baselinePhase', 'anchorTime'});
for fieldIdx = 1:numel(nonnegative)
    if options.(nonnegative{fieldIdx}) < 0
        error('Formation-gateway option %s must be nonnegative.', ...
            nonnegative{fieldIdx});
    end
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
    error('Missing formation-gateway action feature: %s', name);
end
matrix = reshape( ...
    features(:, :, weightIdx, featureIdx), ...
    size(features, 1), size(features, 2));
end

function sources = roundRobinSources( ...
    groupIds, currentTime, anchorTime, phase)
nodeCount = numel(groupIds);
sources = nan(1, nodeCount);
groups = unique(groupIds, 'stable');
for groupIdx = 1:numel(groups)
    members = reshape(find(groupIds == groups(groupIdx)), 1, []);
    groupSize = numel(members);
    if groupSize < 2
        error('Every formation needs at least two sensors.');
    end
    shift = 1 + mod( ...
        currentTime - anchorTime + phase - 1, groupSize - 1);
    sources(members) = circshift(members, [0, -shift]);
end
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Formation-gateway routing requires sensorGroupIds metadata.');
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
