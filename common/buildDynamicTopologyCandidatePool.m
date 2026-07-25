function [candidates, metadata] = ...
    buildDynamicTopologyCandidatePool(context, options)
% BUILDDYNAMICTOPOLOGYCANDIDATEPOOL Safe candidate actions at any scale.
%
% D12 uses its registered exhaustive 48-graph family. Larger scenes use a
% deterministic proposal pool: preserve each formation's ring backbone,
% connect the formation graph, fill the common edge budget, and diversify
% the bridge choices with reliability, posterior, geometry and mixed
% scores. The resulting interface lets experiment scripts switch D12,
% M24, and X36 without changing the topology policy.

if nargin < 2 || isempty(options)
    options = struct();
end
scenario = context.model.dynamicTopologyScenario;
registered = getField(scenario, 'candidateAdjacency', []);
if ~isempty(registered)
    candidates = logical(registered);
    metadata = struct( ...
        'source', 'registered-exact', ...
        'isExactFamily', true, ...
        'candidateCount', size(candidates, 3), ...
        'proposalNames', {{'registered'}});
    return;
end

config = scenario.config;
nodeCount = size(context.physicalAdjacency, 1);
groupIds = reshape(config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount
    error('sensorGroupIds must contain one entry per topology node.');
end
edgeBudget = min(round(context.edgeBudget), ...
    nodeCount * (nodeCount - 1) / 2);
maxCandidates = max(1, round(getField( ...
    options, 'maxCandidates', 96)));
diversityBansPerScore = max(0, round(getField( ...
    options, 'diversityBansPerScore', 8)));
maxLocalSwapCandidates = max(0, round(getField( ...
    options, 'maxLocalSwapCandidates', 24)));

physical = logical(context.physicalAdjacency);
physical = physical | physical';
physical(1:nodeCount+1:end) = false;
base = logical(context.baseAdjacency);
base = base | base';
base(1:nodeCount+1:end) = false;
previous = logical(context.previousAdjacency);
previous = previous | previous';
previous(1:nodeCount+1:end) = false;

sameGroup = bsxfun(@eq, groupIds(:), groupIds(:)'); %#ok<NASGU>
backbone = buildRingBackbone(groupIds, physical);

[scoreMatrices, scoreNames] = buildProposalScores( ...
    context, groupIds, physical);
candidates = false(nodeCount, nodeCount, 0);
proposalNames = {};
[candidates, proposalNames] = addCandidateIfUnique( ...
    candidates, proposalNames, base, 'registered-static', ...
    physical, edgeBudget, groupIds, config);
[candidates, proposalNames] = addCandidateIfUnique( ...
    candidates, proposalNames, previous, 'previous', ...
    physical, edgeBudget, groupIds, config);
[candidates, proposalNames] = addLocalSwapNeighborhood( ...
    candidates, proposalNames, previous, backbone, ...
    scoreMatrices{1}, physical, edgeBudget, groupIds, config, ...
    min(maxLocalSwapCandidates, maxCandidates - size(candidates, 3)));

for scoreIdx = 1:numel(scoreMatrices)
    if size(candidates, 3) >= maxCandidates
        break;
    end
    scores = scoreMatrices{scoreIdx};
    candidate = projectBridgeTopology( ...
        backbone, scores, false(nodeCount), physical, ...
        edgeBudget, groupIds, config);
    [candidates, proposalNames] = addCandidateIfUnique( ...
        candidates, proposalNames, candidate, scoreNames{scoreIdx}, ...
        physical, edgeBudget, groupIds, config);

    bridgeEdges = find(triu(candidate & ~sameGroup, 1));
    bridgeScores = scores(bridgeEdges);
    [~, order] = sort(bridgeScores, 'descend');
    order = order(1:min(numel(order), diversityBansPerScore));
    for edgeCursor = reshape(order, 1, [])
        if size(candidates, 3) >= maxCandidates
            break;
        end
        banned = false(nodeCount);
        banned(bridgeEdges(edgeCursor)) = true;
        banned = banned | banned';
        alternative = projectBridgeTopology( ...
            backbone, scores, banned, physical, ...
            edgeBudget, groupIds, config);
        name = sprintf('%s-ban-%d', ...
            scoreNames{scoreIdx}, edgeCursor);
        [candidates, proposalNames] = addCandidateIfUnique( ...
            candidates, proposalNames, alternative, name, ...
            physical, edgeBudget, groupIds, config);
    end
end

if isempty(candidates)
    error(['No feasible topology candidate satisfies the physical, ', ...
        'budget, degree, and formation-connectivity constraints.']);
end
metadata = struct();
metadata.source = 'projected-general';
metadata.isExactFamily = false;
metadata.candidateCount = size(candidates, 3);
metadata.proposalNames = proposalNames;
metadata.backboneEdgeCount = countEdges(backbone);
metadata.bridgeEdgeCount = edgeBudget - metadata.backboneEdgeCount;
end

function [candidates, names] = addLocalSwapNeighborhood( ...
    candidates, names, previous, backbone, scores, physical, ...
    edgeBudget, groupIds, config, maxCandidates)
if maxCandidates <= 0 || isempty(previous) || ~any(previous(:)) || ...
        countEdges(previous) ~= edgeBudget
    return;
end
sameGroup = bsxfun(@eq, groupIds(:), groupIds(:)');
removableMask = previous & ~backbone;
[removeLeft, removeRight] = find(triu(removableMask, 1));
additionMask = physical & ~previous & ~sameGroup;
rankedAdditions = sortedEdges(scores, additionMask);
added = 0;
for removeIdx = 1:numel(removeLeft)
    for additionIdx = 1:size(rankedAdditions, 1)
        candidate = previous;
        leftRemove = removeLeft(removeIdx);
        rightRemove = removeRight(removeIdx);
        candidate(leftRemove, rightRemove) = false;
        candidate(rightRemove, leftRemove) = false;
        leftAdd = rankedAdditions(additionIdx, 1);
        rightAdd = rankedAdditions(additionIdx, 2);
        candidate(leftAdd, rightAdd) = true;
        candidate(rightAdd, leftAdd) = true;
        beforeCount = size(candidates, 3);
        name = sprintf('previous-swap-%d-%d-%d-%d', ...
            leftRemove, rightRemove, leftAdd, rightAdd);
        [candidates, names] = addCandidateIfUnique( ...
            candidates, names, candidate, name, physical, ...
            edgeBudget, groupIds, config);
        if size(candidates, 3) > beforeCount
            added = added + 1;
        end
        if added >= maxCandidates
            return;
        end
    end
end
end

function [scoreMatrices, names] = buildProposalScores( ...
    context, groupIds, physical)
nodeCount = numel(groupIds);
raw = normalizeScores(getField( ...
    context, 'edgeScores', zeros(nodeCount)), physical);
reliability = normalizeScores(buildReliabilityScores( ...
    context.commConfig, context.currentTime, nodeCount), physical);
discrepancy = normalizeScores(buildDiscrepancyScores( ...
    context.localPosteriorBySensor, context.model, nodeCount), physical);
geometry = normalizeScores(buildGeometryScores( ...
    getField(context, 'positions', zeros(2, nodeCount))), physical);

scoreMatrices = { ...
    raw, ...
    reliability, ...
    discrepancy, ...
    geometry, ...
    normalizeScores(0.5 * discrepancy + 0.5 * reliability, physical), ...
    normalizeScores(0.5 * discrepancy + 0.5 * raw, physical), ...
    normalizeScores(0.4 * discrepancy + 0.4 * reliability + ...
        0.2 * geometry, physical)};
names = { ...
    'registered-score', ...
    'reliability', ...
    'posterior-discrepancy', ...
    'geometry', ...
    'discrepancy-reliability', ...
    'discrepancy-registered', ...
    'mixed'};
end

function scores = buildReliabilityScores( ...
    commConfig, currentTime, nodeCount)
scores = zeros(nodeCount);
for leftIdx = 1:nodeCount-1
    for rightIdx = leftIdx+1:nodeCount
        forward = edgeDrop( ...
            commConfig, leftIdx, rightIdx, currentTime);
        reverse = edgeDrop( ...
            commConfig, rightIdx, leftIdx, currentTime);
        scores(leftIdx, rightIdx) = 1 - 0.5 * (forward + reverse);
        scores(rightIdx, leftIdx) = scores(leftIdx, rightIdx);
    end
end
end

function scores = buildDiscrepancyScores(posteriors, model, nodeCount)
scores = zeros(nodeCount);
if isempty(posteriors)
    return;
end
for leftIdx = 1:nodeCount-1
    for rightIdx = leftIdx+1:nodeCount
        value = posteriorDisagreement( ...
            posteriors{leftIdx}, posteriors{rightIdx}, model);
        scores(leftIdx, rightIdx) = value;
        scores(rightIdx, leftIdx) = value;
    end
end
end

function scores = buildGeometryScores(positions)
nodeCount = size(positions, 2);
scores = zeros(nodeCount);
distances = zeros(nodeCount);
for leftIdx = 1:nodeCount-1
    for rightIdx = leftIdx+1:nodeCount
        distance = norm( ...
            positions(:, leftIdx) - positions(:, rightIdx));
        distances(leftIdx, rightIdx) = distance;
        distances(rightIdx, leftIdx) = distance;
    end
end
positive = distances(distances > 0 & isfinite(distances));
if isempty(positive)
    return;
end
scale = median(positive);
scores = exp(-distances / max(scale, eps));
scores(1:nodeCount+1:end) = 0;
end

function normalized = normalizeScores(scores, physical)
scores = double(scores);
scores(~isfinite(scores)) = NaN;
finiteValues = scores(physical & isfinite(scores));
normalized = zeros(size(scores));
if isempty(finiteValues)
    normalized(~physical) = -inf;
    return;
end
minimum = min(finiteValues);
maximum = max(finiteValues);
if maximum > minimum
    normalized(physical) = ...
        (scores(physical) - minimum) / (maximum - minimum);
else
    normalized(physical) = 0.5;
end
normalized(~physical) = -inf;
normalized = (normalized + normalized') / 2;
normalized(1:size(normalized, 1)+1:end) = -inf;
end

function adjacency = projectBridgeTopology( ...
    backbone, scores, banned, physical, edgeBudget, groupIds, config)
nodeCount = size(backbone, 1);
adjacency = logical(backbone);
sameGroup = bsxfun(@eq, groupIds(:), groupIds(:)');
candidateMask = physical & ~sameGroup & ~banned;
candidateMask(1:nodeCount+1:end) = false;
rankedEdges = sortedEdges(scores, candidateMask);
maxNodeDegree = getField(config, 'maxNodeDegree', inf);
maxInterDegree = getField( ...
    config, 'maxInterFormationDegree', inf);
interDegree = sum(adjacency & ~sameGroup, 2);

uniqueGroups = unique(groupIds);
parent = 1:numel(uniqueGroups);
groupLookup = zeros(1, max(uniqueGroups));
groupLookup(uniqueGroups) = 1:numel(uniqueGroups);
for edgeIdx = 1:size(rankedEdges, 1)
    leftIdx = rankedEdges(edgeIdx, 1);
    rightIdx = rankedEdges(edgeIdx, 2);
    leftGroup = groupLookup(groupIds(leftIdx));
    rightGroup = groupLookup(groupIds(rightIdx));
    [parent, leftRoot] = findRoot(parent, leftGroup);
    [parent, rightRoot] = findRoot(parent, rightGroup);
    if leftRoot == rightRoot || ...
            ~canAddEdge(adjacency, interDegree, ...
                leftIdx, rightIdx, maxNodeDegree, maxInterDegree)
        continue;
    end
    adjacency = addEdge(adjacency, leftIdx, rightIdx);
    interDegree([leftIdx, rightIdx]) = ...
        interDegree([leftIdx, rightIdx]) + 1;
    parent(rightRoot) = leftRoot;
    if hasSingleRoot(parent)
        break;
    end
end
if ~hasSingleRoot(parent)
    adjacency = false(nodeCount);
    return;
end

for edgeIdx = 1:size(rankedEdges, 1)
    if countEdges(adjacency) >= edgeBudget
        break;
    end
    leftIdx = rankedEdges(edgeIdx, 1);
    rightIdx = rankedEdges(edgeIdx, 2);
    if adjacency(leftIdx, rightIdx) || ...
            ~canAddEdge(adjacency, interDegree, ...
                leftIdx, rightIdx, maxNodeDegree, maxInterDegree)
        continue;
    end
    adjacency = addEdge(adjacency, leftIdx, rightIdx);
    interDegree([leftIdx, rightIdx]) = ...
        interDegree([leftIdx, rightIdx]) + 1;
end
if countEdges(adjacency) ~= edgeBudget
    adjacency = false(nodeCount);
end
end

function edges = sortedEdges(scores, mask)
[left, right] = find(triu(mask, 1));
if isempty(left)
    edges = zeros(0, 3);
    return;
end
linear = sub2ind(size(scores), left, right);
values = scores(linear);
table = [left, right, values];
table = sortrows(table, [-3, 1, 2]);
edges = table;
end

function allowed = canAddEdge( ...
    adjacency, interDegree, leftIdx, rightIdx, ...
    maxNodeDegree, maxInterDegree)
allowed = sum(adjacency(leftIdx, :)) < maxNodeDegree && ...
    sum(adjacency(rightIdx, :)) < maxNodeDegree && ...
    interDegree(leftIdx) < maxInterDegree && ...
    interDegree(rightIdx) < maxInterDegree;
end

function [candidates, names] = addCandidateIfUnique( ...
    candidates, names, candidate, name, physical, ...
    edgeBudget, groupIds, config)
if isempty(candidate) || ~isCandidateFeasible( ...
        candidate, physical, edgeBudget, groupIds, config)
    return;
end
for candidateIdx = 1:size(candidates, 3)
    if isequal(candidate, candidates(:, :, candidateIdx))
        return;
    end
end
candidates(:, :, end+1) = logical(candidate); %#ok<AGROW>
names{end+1} = name; %#ok<AGROW>
end

function feasible = isCandidateFeasible( ...
    adjacency, physical, edgeBudget, groupIds, config)
nodeCount = size(adjacency, 1);
adjacency = logical(adjacency);
sameGroup = bsxfun(@eq, groupIds(:), groupIds(:)');
maxNodeDegree = getField(config, 'maxNodeDegree', inf);
maxInterDegree = getField( ...
    config, 'maxInterFormationDegree', inf);
feasible = isequal(adjacency, adjacency') && ...
    ~any(diag(adjacency)) && ...
    ~any(adjacency(:) & ~physical(:)) && ...
    countEdges(adjacency) == edgeBudget && ...
    max(sum(adjacency, 2)) <= maxNodeDegree && ...
    max(sum(adjacency & ~sameGroup, 2)) <= maxInterDegree && ...
    isConnected(adjacency) && allGroupsConnected(adjacency, groupIds);
if nodeCount <= 1
    feasible = true;
end
end

function backbone = buildRingBackbone(groupIds, physical)
nodeCount = numel(groupIds);
backbone = false(nodeCount);
for group = unique(groupIds)
    nodes = find(groupIds == group);
    for localIdx = 1:numel(nodes)
        nextIdx = mod(localIdx, numel(nodes)) + 1;
        leftIdx = nodes(localIdx);
        rightIdx = nodes(nextIdx);
        if ~physical(leftIdx, rightIdx)
            error('An intra-formation ring edge is not physical.');
        end
        backbone = addEdge(backbone, leftIdx, rightIdx);
    end
end
end

function connected = allGroupsConnected(adjacency, groupIds)
connected = true;
for group = unique(groupIds)
    nodes = find(groupIds == group);
    if ~isConnected(adjacency(nodes, nodes))
        connected = false;
        return;
    end
end
end

function connected = isConnected(adjacency)
nodeCount = size(adjacency, 1);
if nodeCount <= 1
    connected = true;
    return;
end
visited = false(1, nodeCount);
queue = 1;
visited(1) = true;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    neighbors = find(adjacency(current, :) & ~visited);
    visited(neighbors) = true;
    queue = [queue, neighbors]; %#ok<AGROW>
end
connected = all(visited);
end

function [parent, root] = findRoot(parent, node)
root = node;
while parent(root) ~= root
    root = parent(root);
end
while parent(node) ~= node
    next = parent(node);
    parent(node) = root;
    node = next;
end
end

function value = hasSingleRoot(parent)
roots = zeros(size(parent));
for idx = 1:numel(parent)
    [parent, roots(idx)] = findRoot(parent, idx);
end
value = numel(unique(roots)) == 1;
end

function adjacency = addEdge(adjacency, leftIdx, rightIdx)
adjacency(leftIdx, rightIdx) = true;
adjacency(rightIdx, leftIdx) = true;
end

function count = countEdges(adjacency)
count = nnz(triu(adjacency, 1));
end

function probability = edgeDrop(config, senderIdx, receiverIdx, currentTime)
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

function value = posteriorDisagreement(leftObjects, rightObjects, model)
labels = collectLabels(leftObjects, rightObjects);
if isempty(labels)
    value = 0;
    return;
end
terms = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    left = findLabel(leftObjects, labels(:, labelIdx));
    right = findLabel(rightObjects, labels(:, labelIdx));
    leftExistence = objectExistence(left);
    rightExistence = objectExistence(right);
    existenceTerm = abs(leftExistence - rightExistence);
    spatialTerm = 0;
    if ~isempty(left) && ~isempty(right)
        [leftMean, leftCovariance] = objectMoments( ...
            left, model.xDimension);
        [rightMean, rightCovariance] = objectMoments( ...
            right, model.xDimension);
        scale = sqrt(max(trace( ...
            leftCovariance(1:2, 1:2) + ...
            rightCovariance(1:2, 1:2)), 1));
        spatialTerm = min(norm( ...
            leftMean(1:2) - rightMean(1:2)) / scale, 5);
    end
    terms(labelIdx) = existenceTerm + ...
        min(leftExistence, rightExistence) * spatialTerm;
end
value = mean(terms);
end

function labels = collectLabels(leftObjects, rightObjects)
labels = zeros(2, 0);
collections = {leftObjects, rightObjects};
for collectionIdx = 1:2
    objects = collections{collectionIdx};
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents < 1
            continue;
        end
        label = [objects(objectIdx).birthTime; ...
            objects(objectIdx).birthLocation];
        if isempty(labels) || ~any(all(labels == label, 1))
            labels(:, end+1) = label; %#ok<AGROW>
        end
    end
end
end

function object = findLabel(objects, label)
object = [];
for objectIdx = 1:numel(objects)
    if objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function value = objectExistence(object)
if isempty(object)
    value = 0;
else
    value = min(max(object.r, 0), 1);
end
end

function [meanVector, covariance] = objectMoments( ...
    object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + ...
        weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = (covariance + covariance') / 2;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
