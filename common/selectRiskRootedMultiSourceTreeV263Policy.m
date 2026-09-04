function [adjacency, details] = ...
        selectRiskRootedMultiSourceTreeV263Policy(context)
% SELECTRISKROOTEDMULTISOURCETREEV263POLICY Shortest paths to risk root.
%
% A breadth-first tree rooted at the at-risk formation minimizes the vector
% of formation-to-root hop distances componentwise.  Thus every supported
% lower-risk source reaches the target in the fewest physically possible
% KLA rounds while the cross-formation edge and message counts remain fixed.

protocol = getRiskRootedMultiSourceTreeV263Protocol();
[minimumAdjacency, minimum] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
nodeCount = size(minimumAdjacency, 1);
[groupIds, formationIds, formationUids, members] = ...
    resolveFormationIdentity(context, nodeCount);
formationCount = numel(formationIds);
summary = summarizeFormationLmbRiskModesV259( ...
    context.localPosteriorBySensor, groupIds, context.model);

localization = reshape( ...
    summary.localizationTailRiskByFormation, 1, []);
formationMedian = median(localization);
relative = localization / max(formationMedian, eps);
eligibleTarget = localization >= ...
    protocol.localizationAbsoluteThreshold & ...
    relative >= protocol.localizationRelativeThreshold;
targetPosition = selectMaximum(localization, eligibleTarget);

physicalFormation = formationAdjacency( ...
    context.physicalAdjacency, members);
formationReliability = formationPairReliability( ...
    context.commConfig, context.currentTime, members, ...
    context.physicalAdjacency, physicalFormation);
minimumTree = treeFromUidPairs( ...
    minimum.currentFormationTreePairs, formationUids);
rootedTree = minimumTree;
rootedLayers = inf(1, formationCount);
requestedPairs = normalizePairs(minimum.currentFormationTreePairs);
selectedLabelIndex = 0;
selectedLabel = zeros(2, 0);
targetLabelRisk = NaN;
sourceLabelRisk = nan(1, formationCount);
sourceCoverage = zeros(1, formationCount);
sourceExistence = zeros(1, formationCount);
eligibleSourceMask = false(1, formationCount);
baseHopDistances = inf(1, formationCount);
rootedHopDistances = inf(1, formationCount);
physicalHopDistances = inf(1, formationCount);
shortenedSourceMask = false(1, formationCount);
multiSourceRiskSupported = false;
requestAttempted = false;
requestApplied = false;
rootedInitiated = false;

if targetPosition > 0
    selectedLabelIndex = summary. ...
        highestLocalizationRiskLabelIndexByFormation(targetPosition);
end
if selectedLabelIndex > 0
    selectedLabel = summary.labels(:, selectedLabelIndex);
    sourceLabelRisk = reshape( ...
        summary.labelLocalizationRiskByFormation(:, ...
            selectedLabelIndex), 1, []);
    sourceCoverage = reshape( ...
        summary.formationActiveCoverage(:, selectedLabelIndex), 1, []);
    sourceExistence = reshape( ...
        summary.formationMedianExistence(:, selectedLabelIndex), 1, []);
    targetLabelRisk = sourceLabelRisk(targetPosition);
    eligibleSourceMask = isfinite(sourceLabelRisk) & ...
        sourceCoverage >= protocol.minimumSourceCoverage & ...
        sourceExistence >= protocol.minimumSourceExistence & ...
        sourceLabelRisk <= ...
            protocol.maximumSourceToTargetRiskRatio * targetLabelRisk;
    eligibleSourceMask(targetPosition) = false;
    multiSourceRiskSupported = nnz(eligibleSourceMask) >= ...
        protocol.minimumEligibleSourceCount;
end
if multiSourceRiskSupported
    [rootedTree, rootedLayers] = buildRootedShortestPathTree( ...
        physicalFormation, formationReliability, minimumTree, ...
        targetPosition, formationUids);
    requestedPairs = uidPairsFromTree(rootedTree, formationUids);
    for formationPosition = 1:formationCount
        baseHopDistances(formationPosition) = shortestHopDistance( ...
            minimumTree, formationPosition, targetPosition);
        rootedHopDistances(formationPosition) = shortestHopDistance( ...
            rootedTree, formationPosition, targetPosition);
        physicalHopDistances(formationPosition) = shortestHopDistance( ...
            physicalFormation, formationPosition, targetPosition);
    end
    shortenedSourceMask = eligibleSourceMask & ...
        rootedHopDistances < baseHopDistances;
    requestAttempted = ~isequal(rootedTree, minimumTree) && ...
        nnz(shortenedSourceMask) >= ...
            protocol.minimumInitiallyShortenedSourceCount;
end

adjacency = minimumAdjacency;
details = minimum;
if requestAttempted
    [projectedAdjacency, projected] = ...
        selectHorizonValueProjectedMinimumTreeV249Policy( ...
            context, requestedPairs);
    requestApplied = projected.requestedTreeApplied;
    if requestApplied
        adjacency = projectedAdjacency;
        details = projected;
        rootedInitiated = true;
    end
end

currentTree = treeFromUidPairs( ...
    details.currentFormationTreePairs, formationUids);
rootedActive = multiSourceRiskSupported && ...
    isequal(currentTree, rootedTree);
componentwiseOptimal = rootedActive && ...
    all(rootedHopDistances == physicalHopDistances);

weights = details.fusionWeightMatrix;
expectedMessages = nodeCount + 2 * (formationCount - 1);
positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
hardGate = nnz(adjacency) == expectedMessages && ...
    ~any(logical(adjacency(:)) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(logical(adjacency)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('RiskRootedMultiSourceV263:ProjectionFailed', ...
        'The risk-rooted route violates a graph, weight or budget gate.');
end

details.contractVersion = ...
    'risk-rooted-multi-source-tree-v263-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'risk-rooted-multi-source-shortest-path-tree';
details.backboneMode = details.mode;
details.minimumBackboneAdjacency = logical(minimumAdjacency);
details.minimumFormationTreePairs = normalizePairs( ...
    minimum.currentFormationTreePairs);
details.requestedFormationTreePairs = requestedPairs;
details.appliedFormationTreePairs = normalizePairs( ...
    details.currentFormationTreePairs);
details.localizationTailRiskByFormation = localization;
details.localizationRelativeRiskByFormation = relative;
details.localizationRiskFormationMedian = formationMedian;
details.localizationRiskGateByFormation = eligibleTarget;
details.selectedLocalizationFormationPosition = targetPosition;
details.selectedLocalizationFormationId = scalarOrZero( ...
    targetPosition, formationIds);
details.selectedLocalizationLabel = selectedLabel;
details.targetLabelLocalizationRisk = targetLabelRisk;
details.sourceLabelLocalizationRisk = sourceLabelRisk;
details.sourceCoverage = sourceCoverage;
details.sourceExistence = sourceExistence;
details.eligibleSourceMask = eligibleSourceMask;
details.eligibleSourceFormationIds = ...
    positionsToIds(find(eligibleSourceMask), formationIds);
details.eligibleSourceCount = nnz(eligibleSourceMask);
details.minimumEligibleSourceCount = ...
    protocol.minimumEligibleSourceCount;
details.baseSourceHopDistances = baseHopDistances;
details.rootedSourceHopDistances = rootedHopDistances;
details.physicalSourceHopDistances = physicalHopDistances;
details.shortenedSourceMask = shortenedSourceMask;
details.shortenedSourceFormationIds = ...
    positionsToIds(find(shortenedSourceMask), formationIds);
details.initiallyShortenedSourceCount = nnz(shortenedSourceMask);
details.rootedBreadthFirstLayers = rootedLayers;
details.multiSourceRiskSupported = multiSourceRiskSupported;
details.rootedTreeRequestAttempted = requestAttempted;
details.rootedTreeRequestApplied = requestApplied;
details.riskRootedTreeInitiated = rootedInitiated;
details.riskRootedTreeActive = rootedActive;
details.multiSourceShortestPathPassed = componentwiseOptimal;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = false;
details.currentNetworkPosteriorSynopsisUsed = true;
details.distributedControlSynopsisCostIncluded = false;
details.centralizedDevelopmentController = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.trackingOutcomeScored = false;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'risk-rooted-multi-source-tree-v263-schedule-v1';
schedule.phase = conditionalValue(rootedActive, ...
    'v263-risk-rooted-multi-source-active', ...
    'v263-minimum-backbone');
schedule.selectedLocalizationFormationId = ...
    details.selectedLocalizationFormationId;
schedule.selectedLocalizationLabel = selectedLabel;
schedule.eligibleSourceFormationIds = ...
    details.eligibleSourceFormationIds;
schedule.eligibleSourceCount = details.eligibleSourceCount;
schedule.shortenedSourceFormationIds = ...
    details.shortenedSourceFormationIds;
schedule.initiallyShortenedSourceCount = ...
    details.initiallyShortenedSourceCount;
schedule.baseSourceHopDistances = baseHopDistances;
schedule.rootedSourceHopDistances = rootedHopDistances;
schedule.physicalSourceHopDistances = physicalHopDistances;
schedule.requestedFormationTreePairs = requestedPairs;
schedule.appliedFormationTreePairs = ...
    details.appliedFormationTreePairs;
schedule.rootedTreeRequestAttempted = requestAttempted;
schedule.rootedTreeRequestApplied = requestApplied;
schedule.riskRootedTreeInitiated = rootedInitiated;
schedule.riskRootedTreeActive = rootedActive;
schedule.multiSourceShortestPathPassed = componentwiseOptimal;
schedule.currentMessageCount = nnz(adjacency);
schedule.centralizedDevelopmentController = true;
schedule.distributedControlSynopsisCostIncluded = false;
details.scheduleCertificate = schedule;
end

function [groupIds, formationIds, formationUids, members] = ...
        resolveFormationIdentity(context, nodeCount)
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
members = cell(1, formationCount);
formationUids = zeros(1, formationCount);
uidsBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
if numel(groupIds) ~= nodeCount || numel(uidsBySensor) ~= nodeCount
    error('RiskRootedMultiSourceV263:InvalidFormationIdentity', ...
        'The formation identity vectors are malformed.');
end
for formationPosition = 1:formationCount
    members{formationPosition} = find( ...
        groupIds == formationIds(formationPosition));
    values = unique(uidsBySensor(members{formationPosition}));
    if isempty(members{formationPosition}) || numel(values) ~= 1
        error('RiskRootedMultiSourceV263:InvalidFormationIdentity', ...
            'Each formation must map to one physical UID.');
    end
    formationUids(formationPosition) = values;
end
end

function adjacency = formationAdjacency(sensorAdjacency, members)
formationCount = numel(members);
adjacency = false(formationCount);
sensorAdjacency = logical(sensorAdjacency);
for left = 1:formationCount-1
    for right = left+1:formationCount
        forward = sensorAdjacency(members{left}, members{right});
        reverse = sensorAdjacency(members{right}, members{left});
        available = any(forward(:)) && any(reverse(:));
        adjacency(left, right) = available;
        adjacency(right, left) = available;
    end
end
end

function reliability = formationPairReliability( ...
        commConfig, currentTime, members, sensorPhysical, physical)
formationCount = numel(members);
reliability = zeros(formationCount);
for left = 1:formationCount-1
    for right = left+1:formationCount
        if ~physical(left, right), continue; end
        forward = edgeReliabilityBlock( ...
            commConfig, currentTime, members{left}, members{right});
        reverse = edgeReliabilityBlock( ...
            commConfig, currentTime, members{right}, members{left});
        forwardMask = logical(sensorPhysical( ...
            members{right}, members{left}))';
        reverseMask = logical(sensorPhysical( ...
            members{left}, members{right}))';
        value = min(max(forward(forwardMask)), ...
            max(reverse(reverseMask)));
        reliability(left, right) = value;
        reliability(right, left) = value;
    end
end
end

function values = edgeReliabilityBlock( ...
        config, currentTime, senders, receivers)
if isfield(config, 'forceDelivery') && config.forceDelivery
    values = ones(numel(senders), numel(receivers));
elseif isfield(config, 'pDropByEdge') && ...
        ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        page = min(currentTime, size(config.pDropByEdge, 3));
        values = 1 - config.pDropByEdge(senders, receivers, page);
    else
        values = 1 - config.pDropByEdge(senders, receivers);
    end
else
    values = ones(numel(senders), numel(receivers));
end
values = min(max(values, 0), 1);
end

function [tree, layers] = buildRootedShortestPathTree( ...
        physical, reliability, incumbent, root, uids)
layers = breadthFirstDistances(physical, root);
if any(~isfinite(layers))
    error('RiskRootedMultiSourceV263:DisconnectedFormationGraph', ...
        'The physical formation graph is disconnected.');
end
count = size(physical, 1);
tree = false(count);
nodes = find((1:count) ~= root);
keys = [layers(nodes)', uids(nodes)', nodes'];
keys = sortrows(keys, [1, 2]);
for row = 1:size(keys, 1)
    node = keys(row, 3);
    parents = find(physical(node, :) & ...
        layers == layers(node) - 1);
    parentKeys = [ ...
        -double(incumbent(node, parents))', ...
        -reliability(node, parents)', uids(parents)', parents'];
    parentKeys = sortrows(parentKeys, [1, 2, 3]);
    parent = parentKeys(1, 4);
    tree(node, parent) = true;
    tree(parent, node) = true;
end
if nnz(triu(tree, 1)) ~= count - 1 || ~isConnected(tree)
    error('RiskRootedMultiSourceV263:InvalidRootedTree', ...
        'The breadth-first parent selection did not form a tree.');
end
end

function distances = breadthFirstDistances(adjacency, root)
distances = inf(1, size(adjacency, 1));
distances(root) = 0;
queue = root;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    for next = reshape(find(adjacency(current, :)), 1, [])
        if isfinite(distances(next)), continue; end
        distances(next) = distances(current) + 1;
        queue(end + 1) = next; %#ok<AGROW>
    end
end
end

function tree = treeFromUidPairs(pairs, formationUids)
tree = false(numel(formationUids));
pairs = normalizePairs(pairs);
for pairIdx = 1:size(pairs, 1)
    left = find(formationUids == pairs(pairIdx, 1), 1);
    right = find(formationUids == pairs(pairIdx, 2), 1);
    if isempty(left) || isempty(right)
        error('RiskRootedMultiSourceV263:UnknownFormationUid', ...
            'A formation-tree pair uses an unknown physical UID.');
    end
    tree(left, right) = true;
    tree(right, left) = true;
end
end

function pairs = uidPairsFromTree(tree, uids)
[left, right] = find(triu(logical(tree), 1));
edges = sortrows([left, right], [1, 2]);
pairs = zeros(size(edges));
for idx = 1:size(edges, 1)
    pairs(idx, :) = sort(uids(edges(idx, :)));
end
pairs = normalizePairs(pairs);
end

function value = shortestHopDistance(adjacency, source, target)
if source <= 0 || target <= 0
    value = Inf;
    return;
end
distances = breadthFirstDistances(adjacency, source);
value = distances(target);
end

function connected = isConnected(adjacency)
connected = all(isfinite(breadthFirstDistances(adjacency, 1)));
end

function connected = isStronglyConnected(adjacency)
connected = all(isfinite(breadthFirstDistances(adjacency, 1))) && ...
    all(isfinite(breadthFirstDistances(adjacency', 1)));
end

function position = selectMaximum(values, mask)
position = 0;
candidates = find(mask);
if isempty(candidates), return; end
[~, local] = max(values(candidates));
position = candidates(local);
end

function value = scalarOrZero(position, values)
if position > 0, value = values(position); else, value = 0; end
end

function values = positionsToIds(positions, ids)
if isempty(positions)
    values = zeros(1, 0);
else
    values = reshape(ids(positions), 1, []);
end
end

function pairs = normalizePairs(pairs)
if isempty(pairs)
    pairs = zeros(0, 2);
else
    pairs = sortrows(sort(pairs, 2), [1, 2]);
end
end

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
end
