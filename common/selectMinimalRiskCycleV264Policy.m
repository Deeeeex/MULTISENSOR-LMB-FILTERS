function [adjacency, details] = selectMinimalRiskCycleV264Policy(context)
% SELECTMINIMALRISKCYCLEV264POLICY Restore one displaced incumbent edge.

protocol = getMinimalRiskCycleV264Protocol();
[rootedAdjacency, rooted] = ...
    selectRiskRootedMultiSourceTreeV263Policy(context);
adjacency = rootedAdjacency;
details = rooted;
nodeCount = size(adjacency, 1);
[formationIds, formationUids, members] = ...
    resolveFormationIdentity(context, nodeCount);
formationCount = numel(formationIds);
rootedTree = treeFromUidPairs( ...
    rooted.appliedFormationTreePairs, formationUids);
minimumTree = treeFromUidPairs( ...
    rooted.minimumFormationTreePairs, formationUids);
physicalFormation = formationAdjacency( ...
    context.physicalAdjacency, members);
previousSensorAdjacency = previousAdjacency(context, nodeCount);
previousFormation = formationAdjacency( ...
    previousSensorAdjacency, members);

protectivePairPositions = zeros(1, 0);
protectivePairUids = zeros(1, 0);
protectivePairIds = zeros(1, 0);
protectiveMask = false(nodeCount);
pairwiseDominancePassed = false;
strictDilationRepairPassed = false;
rootedPairwiseDistances = pairwiseHopDistances(rootedTree);
minimumPairwiseDistances = pairwiseHopDistances(minimumTree);
augmentedPairwiseDistances = rootedPairwiseDistances;
candidateCount = 0;

if rooted.riskRootedTreeActive
    sourceGraph = logical(previousSensorAdjacency) | ...
        logical(rooted.minimumBackboneAdjacency);
    candidateFormationEdges = ...
        (previousFormation | minimumTree) & ...
        ~rootedTree & physicalFormation;
    [protectivePairPositions, protectiveMask, ...
        augmentedPairwiseDistances, candidateCount] = ...
        selectProtectiveEdge( ...
            candidateFormationEdges, rootedTree, minimumTree, ...
            sourceGraph, context.physicalAdjacency, members, ...
            context.sensorPhysicalUids, formationUids);
    if ~isempty(protectivePairPositions)
        protectivePairUids = reshape( ...
            formationUids(protectivePairPositions), 1, []);
        protectivePairIds = reshape( ...
            formationIds(protectivePairPositions), 1, []);
        pairwiseDominancePassed = all( ...
            augmentedPairwiseDistances <= ...
                rootedPairwiseDistances) && ...
            all(augmentedPairwiseDistances <= ...
                minimumPairwiseDistances);
        strictDilationRepairPassed = any( ...
            augmentedPairwiseDistances < rootedPairwiseDistances);
    end
end

cycleActive = rooted.riskRootedTreeActive && ...
    nnz(protectiveMask) == protocol.additionalDirectedMessages && ...
    pairwiseDominancePassed && strictDilationRepairPassed;
cycleInitiated = cycleActive && ...
    (rooted.riskRootedTreeInitiated || ...
     nnz(triu(previousFormation, 1)) < formationCount);
weights = rooted.fusionWeightMatrix;
if cycleActive
    adjacency = logical(adjacency) | protectiveMask;
    [receivers, senders] = find(protectiveMask);
    for edgeIdx = 1:numel(receivers)
        receiver = receivers(edgeIdx);
        sender = senders(edgeIdx);
        if weights(receiver, sender) > 0 || ...
                weights(receiver, receiver) + 1e-12 < ...
                    protocol.protectiveEdgeWeight
            error('MinimalRiskCycleV264:WeightCollision', ...
                'The protective edge cannot receive residual KLA mass.');
        end
        weights(receiver, sender) = protocol.protectiveEdgeWeight;
        weights(receiver, receiver) = ...
            weights(receiver, receiver) - ...
                protocol.protectiveEdgeWeight;
    end
end

treeMessages = nodeCount + 2 * (formationCount - 1);
expectedMessages = treeMessages + ...
    protocol.additionalDirectedMessages * double(cycleActive);
positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
inputsPerReceiver = reshape(sum(logical(adjacency), 2), 1, []);
hardGate = nnz(adjacency) == expectedMessages && ...
    nnz(adjacency) <= treeMessages + ...
        protocol.additionalDirectedMessages && ...
    ~any(logical(adjacency(:)) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(logical(adjacency)) && ...
    all(inputsPerReceiver >= 1) && all(inputsPerReceiver <= 3) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('MinimalRiskCycleV264:ProjectionFailed', ...
        'The minimal cycle violates a graph, weight or budget gate.');
end

details.contractVersion = 'minimal-risk-cycle-v264-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'risk-rooted-tree-with-one-protective-cycle-edge';
details.backboneMode = details.mode;
details.fusionWeightMatrix = weights;
details.protectiveFormationPairPositions = protectivePairPositions;
details.protectiveFormationPairUids = protectivePairUids;
details.protectiveFormationPairIds = protectivePairIds;
details.protectiveDirectedAdjacency = protectiveMask;
details.protectiveEdgeWeight = protocol.protectiveEdgeWeight;
details.protectiveCandidateCount = candidateCount;
details.rootedPairwiseHopDistances = rootedPairwiseDistances;
details.minimumPairwiseHopDistances = minimumPairwiseDistances;
details.augmentedPairwiseHopDistances = augmentedPairwiseDistances;
details.pairwiseDistanceDominancePassed = pairwiseDominancePassed;
details.strictDilationRepairPassed = strictDilationRepairPassed;
details.minimalRiskCycleInitiated = cycleInitiated;
details.minimalRiskCycleActive = cycleActive;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = treeMessages;
details.maximumArchitectureMessageCount = ...
    treeMessages + protocol.additionalDirectedMessages;
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
schedule.contractVersion = 'minimal-risk-cycle-v264-schedule-v1';
schedule.phase = conditionalValue(cycleActive, ...
    'v264-minimal-risk-cycle-active', schedule.phase);
schedule.protectiveFormationPairUids = protectivePairUids;
schedule.protectiveFormationPairIds = protectivePairIds;
schedule.protectiveEdgeWeight = protocol.protectiveEdgeWeight;
schedule.protectiveCandidateCount = candidateCount;
schedule.rootedPairwiseHopDistances = rootedPairwiseDistances;
schedule.minimumPairwiseHopDistances = minimumPairwiseDistances;
schedule.augmentedPairwiseHopDistances = augmentedPairwiseDistances;
schedule.pairwiseDistanceDominancePassed = pairwiseDominancePassed;
schedule.strictDilationRepairPassed = strictDilationRepairPassed;
schedule.minimalRiskCycleInitiated = cycleInitiated;
schedule.minimalRiskCycleActive = cycleActive;
schedule.currentMessageCount = nnz(adjacency);
schedule.additionalDirectedMessages = nnz(protectiveMask);
schedule.centralizedDevelopmentController = true;
schedule.distributedControlSynopsisCostIncluded = false;
details.scheduleCertificate = schedule;
end

function [formationIds, formationUids, members] = ...
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
    error('MinimalRiskCycleV264:InvalidFormationIdentity', ...
        'The formation identity vectors are malformed.');
end
for formationPosition = 1:formationCount
    members{formationPosition} = find( ...
        groupIds == formationIds(formationPosition));
    values = unique(uidsBySensor(members{formationPosition}));
    if isempty(members{formationPosition}) || numel(values) ~= 1
        error('MinimalRiskCycleV264:InvalidFormationIdentity', ...
            'Each formation must map to one physical UID.');
    end
    formationUids(formationPosition) = values;
end
end

function previous = previousAdjacency(context, nodeCount)
previous = false(nodeCount);
if isfield(context, 'previousAdjacencyHistory') && ...
        ~isempty(context.previousAdjacencyHistory)
    history = logical(context.previousAdjacencyHistory);
    if size(history, 1) ~= nodeCount || size(history, 2) ~= nodeCount
        error('MinimalRiskCycleV264:InvalidHistory', ...
            'The previous topology history has the wrong shape.');
    end
    previous = history(:, :, end);
end
end

function adjacency = formationAdjacency(sensorAdjacency, members)
count = numel(members);
adjacency = false(count);
sensorAdjacency = logical(sensorAdjacency);
for left = 1:count-1
    for right = left+1:count
        forward = sensorAdjacency(members{left}, members{right});
        reverse = sensorAdjacency(members{right}, members{left});
        available = any(forward(:)) && any(reverse(:));
        adjacency(left, right) = available;
        adjacency(right, left) = available;
    end
end
end

function [pair, mask, bestDistances, candidateCount] = ...
        selectProtectiveEdge( ...
            candidates, rootedTree, minimumTree, sourceGraph, ...
            physicalSensor, members, sensorUids, formationUids)
pair = zeros(1, 0);
mask = false(size(sourceGraph));
bestDistances = pairwiseHopDistances(rootedTree);
candidateCount = 0;
rootedDistances = bestDistances;
minimumDistances = pairwiseHopDistances(minimumTree);
sourceFormation = formationAdjacency(sourceGraph, members);
[left, right] = find(triu(logical(candidates), 1));
rows = zeros(0, 6);
masks = cell(1, 0);
distancePages = cell(1, 0);
for edgeIdx = 1:numel(left)
    candidatePair = [left(edgeIdx), right(edgeIdx)];
    candidateMask = selectGatewayMessages( ...
        candidatePair, sourceGraph, physicalSensor, ...
        members, sensorUids);
    if nnz(candidateMask) ~= 2
        continue;
    end
    augmented = rootedTree;
    augmented(candidatePair(1), candidatePair(2)) = true;
    augmented(candidatePair(2), candidatePair(1)) = true;
    distances = pairwiseHopDistances(augmented);
    if any(distances > rootedDistances) || ...
            any(distances > minimumDistances) || ...
            ~any(distances < rootedDistances)
        continue;
    end
    candidateCount = candidateCount + 1;
    repair = sum(rootedDistances - distances);
    pairUids = sort(formationUids(candidatePair));
    retained = double(sourceFormation( ...
        candidatePair(1), candidatePair(2)));
    rows(end + 1, :) = [-repair, -retained, ...
        pairUids(1), pairUids(2), candidatePair]; %#ok<AGROW>
    masks{end + 1} = candidateMask; %#ok<AGROW>
    distancePages{end + 1} = distances; %#ok<AGROW>
end
if isempty(rows)
    return;
end
[~, order] = sortrows(rows, [1, 2, 3, 4]);
selected = order(1);
pair = rows(selected, 5:6);
mask = masks{selected};
bestDistances = distancePages{selected};
end

function mask = selectGatewayMessages( ...
        pair, sourceGraph, physical, members, sensorUids)
mask = false(size(sourceGraph));
left = members{pair(1)};
right = members{pair(2)};
mask = addOneDirection(mask, sourceGraph, physical, ...
    left, right, sensorUids);
mask = addOneDirection(mask, sourceGraph, physical, ...
    right, left, sensorUids);
end

function mask = addOneDirection( ...
        mask, sourceGraph, physical, receivers, senders, sensorUids)
available = logical(sourceGraph(receivers, senders)) & ...
    logical(physical(receivers, senders));
[receiverLocal, senderLocal] = find(available);
if isempty(receiverLocal)
    return;
end
receiverUids = reshape( ...
    sensorUids(receivers(receiverLocal)), [], 1);
senderUids = reshape(sensorUids(senders(senderLocal)), [], 1);
keys = [receiverUids, senderUids, receiverLocal, senderLocal];
keys = sortrows(keys, [1, 2]);
receiver = receivers(keys(1, 3));
sender = senders(keys(1, 4));
mask(receiver, sender) = true;
end

function distances = pairwiseHopDistances(adjacency)
count = size(adjacency, 1);
distances = zeros(1, count * (count - 1) / 2);
cursor = 0;
for left = 1:count-1
    allDistances = breadthFirstDistances(adjacency, left);
    for right = left+1:count
        cursor = cursor + 1;
        distances(cursor) = allDistances(right);
    end
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
        error('MinimalRiskCycleV264:UnknownFormationUid', ...
            'A formation pair uses an unknown physical UID.');
    end
    tree(left, right) = true;
    tree(right, left) = true;
end
end

function pairs = normalizePairs(pairs)
if isempty(pairs)
    pairs = zeros(0, 2);
else
    pairs = sortrows(sort(pairs, 2), [1, 2]);
end
end

function connected = isStronglyConnected(adjacency)
connected = all(isfinite(breadthFirstDistances(adjacency, 1))) && ...
    all(isfinite(breadthFirstDistances(adjacency', 1)));
end

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
end
