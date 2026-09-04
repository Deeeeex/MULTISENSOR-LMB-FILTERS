function [adjacency, details] = ...
        selectRiskTriggeredFormationShortcutV261Policy(context)
% SELECTRISKTRIGGEREDFORMATIONSHORTCUTV261POLICY Shorten one risk path.
%
% V242 preserves a feasible formation tree even when a newly available
% physical edge would carry a better-informed formation to a high-risk
% formation in fewer KLA rounds.  V261 detects that common-mode failure
% from current label moments, requests the shorter formation path, and
% lets the existing V249/V242 projector rebuild the same-budget physical
% sensor route.

protocol = getRiskTriggeredFormationShortcutV261Protocol();
[minimumAdjacency, minimum] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
nodeCount = size(minimumAdjacency, 1);
[groupIds, formationIds, formationUids, members] = ...
    resolveFormationIdentity(context, nodeCount);
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

donorPosition = 0;
selectedLabelIndex = 0;
selectedLabel = zeros(2, 0);
targetLabelRisk = NaN;
donorLabelRisk = NaN;
referenceHopDistance = Inf;
physicalHopDistance = Inf;
hopReduction = 0;
shortcutPath = zeros(1, 0);
requestedTree = minimumTree;
requestedPairs = normalizePairs(minimum.currentFormationTreePairs);
requestAttempted = false;
requestApplied = false;
shortcutInitiated = false;

if targetPosition > 0
    selectedLabelIndex = summary. ...
        highestLocalizationRiskLabelIndexByFormation(targetPosition);
    if selectedLabelIndex > 0
        selectedLabel = summary.labels(:, selectedLabelIndex);
        targetLabelRisk = summary.labelLocalizationRiskByFormation( ...
            targetPosition, selectedLabelIndex);
        [donorPosition, shortcutPath, referenceHopDistance, ...
         physicalHopDistance, donorLabelRisk] = selectDonorAndPath( ...
            summary, selectedLabelIndex, targetPosition, ...
            minimumTree, physicalFormation, formationReliability, ...
            formationUids, protocol);
        if donorPosition > 0
            hopReduction = referenceHopDistance - physicalHopDistance;
            requestedTree = completeShortcutTree( ...
                shortcutPath, minimumTree, physicalFormation, ...
                formationReliability, formationUids);
            requestedPairs = uidPairsFromTree( ...
                requestedTree, formationUids);
            requestAttempted = hopReduction >= ...
                protocol.minimumHopReduction && ...
                ~isequal(requestedTree, minimumTree);
        end
    end
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
        shortcutInitiated = true;
    end
end

currentTree = treeFromUidPairs( ...
    details.currentFormationTreePairs, formationUids);
shortcutActive = donorPosition > 0 && ...
    pathIsContained(currentTree, shortcutPath) && ...
    physicalHopDistance < Inf && ...
    shortestHopDistance(currentTree, donorPosition, targetPosition) <= ...
        physicalHopDistance;

weights = details.fusionWeightMatrix;
expectedMessages = nodeCount + 2 * (numel(formationIds) - 1);
positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
hardGate = nnz(adjacency) == expectedMessages && ...
    ~any(logical(adjacency(:)) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(logical(adjacency)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('RiskTriggeredShortcutV261:ProjectionFailed', ...
        'The selected shortcut violates a graph, weight or budget gate.');
end

details.contractVersion = ...
    'risk-triggered-formation-shortcut-v261-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'risk-triggered-formation-kla-path-shortcut';
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
details.selectedDonorFormationPosition = donorPosition;
details.selectedDonorFormationId = scalarOrZero( ...
    donorPosition, formationIds);
details.selectedLocalizationLabel = selectedLabel;
details.targetLabelLocalizationRisk = targetLabelRisk;
details.donorLabelLocalizationRisk = donorLabelRisk;
details.referenceFormationHopDistance = referenceHopDistance;
details.physicalFormationHopDistance = physicalHopDistance;
details.formationHopReduction = hopReduction;
details.shortcutPathFormationIds = ...
    positionsToIds(shortcutPath, formationIds);
details.shortcutPathFormationUids = ...
    positionsToIds(shortcutPath, formationUids);
details.shortcutRequestAttempted = requestAttempted;
details.shortcutRequestApplied = requestApplied;
details.shortcutInitiated = shortcutInitiated;
details.shortcutActive = shortcutActive;
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
    'risk-triggered-formation-shortcut-v261-schedule-v1';
schedule.phase = conditionalValue(shortcutActive, ...
    'v261-formation-shortcut-active', 'v261-minimum-backbone');
schedule.localizationTailRiskByFormation = localization;
schedule.localizationRelativeRiskByFormation = relative;
schedule.localizationRiskGateByFormation = eligibleTarget;
schedule.selectedLocalizationFormationId = ...
    details.selectedLocalizationFormationId;
schedule.selectedDonorFormationId = ...
    details.selectedDonorFormationId;
schedule.selectedLocalizationLabel = selectedLabel;
schedule.targetLabelLocalizationRisk = targetLabelRisk;
schedule.donorLabelLocalizationRisk = donorLabelRisk;
schedule.referenceFormationHopDistance = referenceHopDistance;
schedule.physicalFormationHopDistance = physicalHopDistance;
schedule.formationHopReduction = hopReduction;
schedule.shortcutPathFormationIds = ...
    details.shortcutPathFormationIds;
schedule.shortcutRequestAttempted = requestAttempted;
schedule.shortcutRequestApplied = requestApplied;
schedule.shortcutInitiated = shortcutInitiated;
schedule.shortcutActive = shortcutActive;
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
if numel(groupIds) ~= nodeCount || ...
        numel(uidsBySensor) ~= nodeCount
    error('RiskTriggeredShortcutV261:InvalidFormationIdentity', ...
        'The formation identity vectors are malformed.');
end
for formationPosition = 1:formationCount
    members{formationPosition} = find( ...
        groupIds == formationIds(formationPosition));
    values = unique(uidsBySensor(members{formationPosition}));
    if isempty(members{formationPosition}) || numel(values) ~= 1
        error('RiskTriggeredShortcutV261:InvalidFormationIdentity', ...
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

function tree = treeFromUidPairs(pairs, formationUids)
tree = false(numel(formationUids));
pairs = normalizePairs(pairs);
for pairIdx = 1:size(pairs, 1)
    left = find(formationUids == pairs(pairIdx, 1), 1);
    right = find(formationUids == pairs(pairIdx, 2), 1);
    if isempty(left) || isempty(right)
        error('RiskTriggeredShortcutV261:UnknownFormationUid', ...
            'A formation-tree pair uses an unknown physical UID.');
    end
    tree(left, right) = true;
    tree(right, left) = true;
end
end

function [donor, path, baseDistance, physicalDistance, donorRisk] = ...
        selectDonorAndPath(summary, labelIdx, target, baseTree, ...
            physical, reliability, formationUids, protocol)
formationCount = numel(summary.formationIds);
labelRisk = reshape( ...
    summary.labelLocalizationRiskByFormation(:, labelIdx), 1, []);
coverage = reshape( ...
    summary.formationActiveCoverage(:, labelIdx), 1, []);
existence = reshape( ...
    summary.formationMedianExistence(:, labelIdx), 1, []);
targetRisk = labelRisk(target);
eligible = isfinite(labelRisk) & ...
    coverage >= protocol.minimumDonorCoverage & ...
    existence >= protocol.minimumDonorExistence & ...
    labelRisk <= protocol.maximumDonorToTargetRiskRatio * targetRisk;
eligible(target) = false;
rows = zeros(0, 7);
paths = cell(1, formationCount);
for candidate = find(eligible)
    [candidatePath, candidatePhysicalDistance, pathCost] = ...
        reliableShortestPath( ...
            physical, reliability, candidate, target, formationUids);
    candidateBaseDistance = shortestHopDistance( ...
        baseTree, candidate, target);
    reduction = candidateBaseDistance - candidatePhysicalDistance;
    if reduction < protocol.minimumHopReduction
        continue;
    end
    paths{candidate} = candidatePath;
    rows(end + 1, :) = [labelRisk(candidate), ...
        -existence(candidate), candidatePhysicalDistance, ...
        pathCost, -reduction, formationUids(candidate), candidate]; ...
        %#ok<AGROW>
end
if isempty(rows)
    donor = 0;
    path = zeros(1, 0);
    baseDistance = Inf;
    physicalDistance = Inf;
    donorRisk = NaN;
    return;
end
rows = sortrows(rows, [1, 2, 3, 4, 5, 6]);
donor = rows(1, 7);
path = paths{donor};
baseDistance = shortestHopDistance(baseTree, donor, target);
physicalDistance = numel(path) - 1;
donorRisk = labelRisk(donor);
end

function [path, distance, cost] = reliableShortestPath( ...
        adjacency, reliability, source, target, uids)
count = size(adjacency, 1);
distanceByNode = inf(1, count);
costByNode = inf(1, count);
predecessor = zeros(1, count);
visited = false(1, count);
distanceByNode(source) = 0;
costByNode(source) = 0;
for iteration = 1:count
    available = find(~visited & isfinite(distanceByNode));
    if isempty(available), break; end
    keys = [distanceByNode(available)', costByNode(available)', ...
        uids(available)', available'];
    keys = sortrows(keys, [1, 2, 3]);
    current = keys(1, 4);
    visited(current) = true;
    if current == target, break; end
    neighbors = find(adjacency(current, :) & ~visited);
    for next = reshape(neighbors, 1, [])
        candidateDistance = distanceByNode(current) + 1;
        candidateCost = costByNode(current) - ...
            log(max(reliability(current, next), realmin));
        better = candidateDistance < distanceByNode(next) || ...
            (candidateDistance == distanceByNode(next) && ...
             candidateCost < costByNode(next) - 1e-12) || ...
            (candidateDistance == distanceByNode(next) && ...
             abs(candidateCost - costByNode(next)) <= 1e-12 && ...
             (predecessor(next) == 0 || ...
              uids(current) < uids(predecessor(next))));
        if better
            distanceByNode(next) = candidateDistance;
            costByNode(next) = candidateCost;
            predecessor(next) = current;
        end
    end
end
distance = distanceByNode(target);
cost = costByNode(target);
if ~isfinite(distance)
    path = zeros(1, 0);
    return;
end
path = target;
while path(1) ~= source
    previous = predecessor(path(1));
    if previous == 0
        error('RiskTriggeredShortcutV261:PathReconstruction', ...
            'The physical shortest path could not be reconstructed.');
    end
    path = [previous, path]; %#ok<AGROW>
end
end

function tree = completeShortcutTree( ...
        path, reference, physical, reliability, uids)
count = size(physical, 1);
tree = false(count);
for idx = 1:numel(path)-1
    tree(path(idx), path(idx + 1)) = true;
    tree(path(idx + 1), path(idx)) = true;
end
referenceEdges = upperEdges(reference);
for idx = 1:size(referenceEdges, 1)
    tree = addIfAcyclic(tree, referenceEdges(idx, :));
end
physicalEdges = upperEdges(physical);
if ~isempty(physicalEdges)
    edgeReliability = zeros(size(physicalEdges, 1), 1);
    edgeUidLeft = zeros(size(physicalEdges, 1), 1);
    edgeUidRight = zeros(size(physicalEdges, 1), 1);
    for idx = 1:size(physicalEdges, 1)
        left = physicalEdges(idx, 1);
        right = physicalEdges(idx, 2);
        edgeReliability(idx) = reliability(left, right);
        pair = sort([uids(left), uids(right)]);
        edgeUidLeft(idx) = pair(1);
        edgeUidRight(idx) = pair(2);
    end
    order = [(1:size(physicalEdges, 1))', -edgeReliability, ...
        edgeUidLeft, edgeUidRight];
    order = sortrows(order, [2, 3, 4]);
    for cursor = 1:size(order, 1)
        tree = addIfAcyclic(tree, ...
            physicalEdges(order(cursor, 1), :));
    end
end
if nnz(triu(tree, 1)) ~= count - 1 || ~isConnected(tree)
    error('RiskTriggeredShortcutV261:TreeCompletion', ...
        'The forced shortcut could not be completed to a tree.');
end
end

function tree = addIfAcyclic(tree, edge)
if nnz(triu(tree, 1)) >= size(tree, 1) - 1 || ...
        reachable(tree, edge(1), edge(2))
    return;
end
tree(edge(1), edge(2)) = true;
tree(edge(2), edge(1)) = true;
end

function edges = upperEdges(adjacency)
[left, right] = find(triu(logical(adjacency), 1));
edges = sortrows([left, right], [1, 2]);
end

function pairs = uidPairsFromTree(tree, uids)
edges = upperEdges(tree);
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
distance = inf(1, size(adjacency, 1));
distance(source) = 0;
queue = source;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    if current == target, break; end
    for next = reshape(find(adjacency(current, :)), 1, [])
        if isfinite(distance(next)), continue; end
        distance(next) = distance(current) + 1;
        queue(end + 1) = next; %#ok<AGROW>
    end
end
value = distance(target);
end

function contained = pathIsContained(tree, path)
contained = ~isempty(path);
for idx = 1:numel(path)-1
    contained = contained && tree(path(idx), path(idx + 1));
end
end

function connected = reachable(adjacency, source, target)
if source == target
    connected = true;
    return;
end
visited = false(1, size(adjacency, 1));
queue = source;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    if visited(current), continue; end
    visited(current) = true;
    queue = [queue, find(adjacency(current, :) & ~visited)]; ...
        %#ok<AGROW>
end
connected = visited(target);
end

function connected = isConnected(adjacency)
connected = reachable(adjacency, 1, size(adjacency, 1)) && ...
    allReachable(adjacency, 1);
end

function passed = allReachable(adjacency, source)
visited = false(1, size(adjacency, 1));
queue = source;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    if visited(current), continue; end
    visited(current) = true;
    queue = [queue, find(adjacency(current, :) & ~visited)]; ...
        %#ok<AGROW>
end
passed = all(visited);
end

function connected = isStronglyConnected(adjacency)
connected = allReachable(adjacency, 1) && ...
    allReachable(adjacency', 1);
end

function position = selectMaximum(values, mask)
position = 0;
candidates = find(mask);
if isempty(candidates), return; end
[~, local] = max(values(candidates));
position = candidates(local);
end

function value = scalarOrZero(position, values)
if position > 0
    value = values(position);
else
    value = 0;
end
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
