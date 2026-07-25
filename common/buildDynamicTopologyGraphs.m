function graphData = buildDynamicTopologyGraphs(config, sensorTrajectories)
% BUILDDYNAMICTOPOLOGYGRAPHS Physical graph, safe static graph and candidates.

sensorCount = config.numberOfSensors;
timeCount = config.simulationLength;
positions = zeros(2, sensorCount, timeCount);
for sensorIdx = 1:sensorCount
    positions(:, sensorIdx, :) = ...
        reshape(sensorTrajectories{sensorIdx}(1:2, :), 2, 1, timeCount);
end

[distanceByTime, physicalAdjacency] = buildPhysicalGraph( ...
    positions, config.commRange);
intersectionAdjacency = all(physicalAdjacency, 3);
intersectionAdjacency(1:sensorCount+1:end) = false;

switch lower(config.topologyFamily)
    case 'r8-fixed'
        staticAdjacency = buildR8FixedAdjacency(config);
        candidateAdjacency = reshape( ...
            staticAdjacency, sensorCount, sensorCount, 1);
        candidateMetadata = struct('gatewayNodes', {{}});
    case 'd12-enumerated'
        [candidateAdjacency, candidateMetadata] = ...
            enumerateD12Topologies( ...
                config, distanceByTime, intersectionAdjacency);
        staticIdx = chooseRobustStaticCandidate( ...
            candidateAdjacency, distanceByTime);
        staticAdjacency = candidateAdjacency(:, :, staticIdx);
        candidateMetadata.robustStaticCandidateIndex = staticIdx;
    otherwise
        staticAdjacency = buildGenericStaticAdjacency( ...
            config, distanceByTime, intersectionAdjacency);
        candidateAdjacency = zeros(sensorCount, sensorCount, 0);
        candidateMetadata = struct('gatewayNodes', {{}});
end

if any(staticAdjacency(:) & ~intersectionAdjacency(:))
    error('The generated static topology is not physical for all times.');
end

graphData = struct();
graphData.positions = positions;
graphData.distanceByTime = distanceByTime;
graphData.physicalAdjacency = physicalAdjacency;
graphData.intersectionAdjacency = intersectionAdjacency;
graphData.staticAdjacency = logical(staticAdjacency);
graphData.staticNeighborMap = adjacencyToNeighborMap(staticAdjacency);
graphData.candidateAdjacency = logical(candidateAdjacency);
graphData.candidateMetadata = candidateMetadata;
end

function [distanceByTime, physicalAdjacency] = ...
    buildPhysicalGraph(positions, commRange)
sensorCount = size(positions, 2);
timeCount = size(positions, 3);
distanceByTime = zeros(sensorCount, sensorCount, timeCount);
physicalAdjacency = false(sensorCount, sensorCount, timeCount);
for timeIdx = 1:timeCount
    currentPositions = positions(:, :, timeIdx);
    for leftIdx = 1:sensorCount-1
        for rightIdx = leftIdx+1:sensorCount
            distance = norm( ...
                currentPositions(:, leftIdx) - ...
                currentPositions(:, rightIdx));
            distanceByTime(leftIdx, rightIdx, timeIdx) = distance;
            distanceByTime(rightIdx, leftIdx, timeIdx) = distance;
            isPhysical = ~isfinite(commRange) || distance <= commRange;
            physicalAdjacency(leftIdx, rightIdx, timeIdx) = isPhysical;
            physicalAdjacency(rightIdx, leftIdx, timeIdx) = isPhysical;
        end
    end
end
end

function adjacency = buildR8FixedAdjacency(config)
sensorCount = config.numberOfSensors;
if config.formationCount ~= 2 || config.sensorsPerFormation ~= 4
    error('The R8 fixed topology requires two groups of four sensors.');
end
adjacency = false(sensorCount);
leftGroup = 1:4;
rightGroup = 5:8;
adjacency(leftGroup, leftGroup) = true;
adjacency(rightGroup, rightGroup) = true;
for localIdx = 1:4
    adjacency(leftGroup(localIdx), rightGroup(localIdx)) = true;
    adjacency(rightGroup(localIdx), leftGroup(localIdx)) = true;
end
adjacency(1:sensorCount+1:end) = false;
end

function [candidates, metadata] = enumerateD12Topologies( ...
    config, distanceByTime, intersectionAdjacency)
if config.formationCount ~= 3 || config.sensorsPerFormation ~= 4
    error('D12 enumeration requires exactly three groups of four sensors.');
end
sensorCount = config.numberOfSensors;
backbone = false(sensorCount);
groups = cell(1, 3);
for groupIdx = 1:3
    groups{groupIdx} = find(config.sensorGroupIds == groupIdx);
    backbone = addRing(backbone, groups{groupIdx});
end

groupPairs = [1, 2; 1, 3; 2, 3];
gatewayNodes = cell(3, 2);
bridgeOptions = cell(1, 3);
meanDistances = mean(distanceByTime, 3);
for pairIdx = 1:3
    leftGroup = groups{groupPairs(pairIdx, 1)};
    rightGroup = groups{groupPairs(pairIdx, 2)};
    leftScores = mean(meanDistances(leftGroup, rightGroup), 2);
    rightScores = mean(meanDistances(rightGroup, leftGroup), 2);
    [~, leftOrder] = sort(leftScores, 'ascend');
    [~, rightOrder] = sort(rightScores, 'ascend');
    leftGateways = leftGroup(leftOrder(1:2));
    rightGateways = rightGroup(rightOrder(1:2));
    gatewayNodes{pairIdx, 1} = leftGateways;
    gatewayNodes{pairIdx, 2} = rightGateways;
    options = zeros(4, 2);
    cursor = 0;
    for leftIdx = 1:2
        for rightIdx = 1:2
            cursor = cursor + 1;
            options(cursor, :) = ...
                [leftGateways(leftIdx), rightGateways(rightIdx)];
        end
    end
    physical = false(1, size(options, 1));
    for optionIdx = 1:size(options, 1)
        physical(optionIdx) = intersectionAdjacency( ...
            options(optionIdx, 1), options(optionIdx, 2));
    end
    options = options(physical, :);
    if size(options, 1) < 4
        error(['D12 requires four all-time physical bridge options for ', ...
            'each formation pair.']);
    end
    bridgeOptions{pairIdx} = options;
end

% Three formation-level spanning trees. Each selected pair contributes one
% of four gateway edges, yielding 3*4^2 = 48 exact candidates.
treePairIndices = [1, 2; 1, 3; 2, 3];
candidates = false(sensorCount, sensorCount, 48);
candidatePairs = zeros(48, 4);
candidateIdx = 0;
for treeIdx = 1:size(treePairIndices, 1)
    firstPairIdx = treePairIndices(treeIdx, 1);
    secondPairIdx = treePairIndices(treeIdx, 2);
    for firstOptionIdx = 1:4
        for secondOptionIdx = 1:4
            candidateIdx = candidateIdx + 1;
            adjacency = backbone;
            firstEdge = bridgeOptions{firstPairIdx}(firstOptionIdx, :);
            secondEdge = bridgeOptions{secondPairIdx}(secondOptionIdx, :);
            adjacency = addEdge(adjacency, firstEdge(1), firstEdge(2));
            adjacency = addEdge(adjacency, secondEdge(1), secondEdge(2));
            candidates(:, :, candidateIdx) = adjacency;
            candidatePairs(candidateIdx, :) = [firstEdge, secondEdge];
        end
    end
end

metadata = struct();
metadata.groupPairs = groupPairs;
metadata.gatewayNodes = gatewayNodes;
metadata.bridgeOptions = bridgeOptions;
metadata.candidateBridgeEdges = candidatePairs;
metadata.candidateCount = size(candidates, 3);
end

function candidateIdx = chooseRobustStaticCandidate( ...
    candidates, distanceByTime)
meanDistances = mean(distanceByTime, 3);
worstDistances = max(distanceByTime, [], 3);
candidateCount = size(candidates, 3);
scores = inf(1, candidateCount);
for idx = 1:candidateCount
    upperMask = triu(candidates(:, :, idx), 1);
    if any(upperMask(:))
        scores(idx) = mean(meanDistances(upperMask)) + ...
            0.25 * max(worstDistances(upperMask));
    end
end
[~, candidateIdx] = min(scores);
end

function adjacency = buildGenericStaticAdjacency( ...
    config, distanceByTime, intersectionAdjacency)
sensorCount = config.numberOfSensors;
adjacency = false(sensorCount);
groups = cell(1, config.formationCount);
for groupIdx = 1:config.formationCount
    groups{groupIdx} = find(config.sensorGroupIds == groupIdx);
    adjacency = addRing(adjacency, groups{groupIdx});
end

% A formation-level ring supplies an all-time connected static backbone.
meanDistances = mean(distanceByTime, 3);
for groupIdx = 1:config.formationCount
    nextGroup = mod(groupIdx, config.formationCount) + 1;
    adjacency = addBestGroupBridge( ...
        adjacency, groups{groupIdx}, groups{nextGroup}, ...
        meanDistances, intersectionAdjacency, config.maxNodeDegree);
end

% Fill unused budget with the shortest reliable intersection edges while
% retaining the degree cap.
edgeList = [];
for leftIdx = 1:sensorCount-1
    for rightIdx = leftIdx+1:sensorCount
        if intersectionAdjacency(leftIdx, rightIdx) && ...
                ~adjacency(leftIdx, rightIdx)
            edgeList(end+1, :) = [ ...
                leftIdx, rightIdx, meanDistances(leftIdx, rightIdx)]; %#ok<AGROW>
        end
    end
end
if ~isempty(edgeList)
    [~, order] = sort(edgeList(:, 3), 'ascend');
    edgeList = edgeList(order, :);
end
for edgeIdx = 1:size(edgeList, 1)
    if countEdges(adjacency) >= config.edgeBudget
        break;
    end
    leftIdx = edgeList(edgeIdx, 1);
    rightIdx = edgeList(edgeIdx, 2);
    if sum(adjacency(leftIdx, :)) >= config.maxNodeDegree || ...
            sum(adjacency(rightIdx, :)) >= config.maxNodeDegree
        continue;
    end
    adjacency = addEdge(adjacency, leftIdx, rightIdx);
end
end

function adjacency = addBestGroupBridge( ...
    adjacency, leftGroup, rightGroup, distances, physical, maxDegree)
options = [];
for leftIdx = leftGroup
    for rightIdx = rightGroup
        if physical(leftIdx, rightIdx)
            options(end+1, :) = [ ...
                leftIdx, rightIdx, distances(leftIdx, rightIdx)]; %#ok<AGROW>
        end
    end
end
if isempty(options)
    error('No all-time physical edge joins a required formation pair.');
end
[~, order] = sort(options(:, 3), 'ascend');
for optionIdx = reshape(order, 1, [])
    leftIdx = options(optionIdx, 1);
    rightIdx = options(optionIdx, 2);
    if sum(adjacency(leftIdx, :)) < maxDegree && ...
            sum(adjacency(rightIdx, :)) < maxDegree
        adjacency = addEdge(adjacency, leftIdx, rightIdx);
        return;
    end
end
error('Formation bridge cannot satisfy the configured node degree cap.');
end

function adjacency = addRing(adjacency, group)
for localIdx = 1:numel(group)
    nextIdx = mod(localIdx, numel(group)) + 1;
    adjacency = addEdge( ...
        adjacency, group(localIdx), group(nextIdx));
end
end

function adjacency = addEdge(adjacency, leftIdx, rightIdx)
adjacency(leftIdx, rightIdx) = true;
adjacency(rightIdx, leftIdx) = true;
end

function edgeCount = countEdges(adjacency)
edgeCount = nnz(triu(adjacency, 1));
end

function neighborMap = adjacencyToNeighborMap(adjacency)
sensorCount = size(adjacency, 1);
neighborMap = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    neighborMap{sensorIdx} = unique( ...
        [sensorIdx, find(adjacency(sensorIdx, :))]);
end
end
