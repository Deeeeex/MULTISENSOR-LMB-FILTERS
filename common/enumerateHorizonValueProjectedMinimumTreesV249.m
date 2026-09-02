function bank = enumerateHorizonValueProjectedMinimumTreesV249( ...
        context, referencePairs, maximumCandidateCount)
% ENUMERATEHORIZONVALUEPROJECTEDMINIMUMTREESV249 Feasible M24 bank.

if nargin < 3 || isempty(maximumCandidateCount)
    protocol = getHorizonValueProjectedMinimumTreeV249Protocol();
    maximumCandidateCount = protocol.maximumCandidateCount;
end
required = {'localPosteriorBySensor', 'physicalAdjacency', ...
    'formationPhysicalUidsBySensor'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~isscalar(maximumCandidateCount) || ...
        maximumCandidateCount < 1
    error('HorizonValueProjectedMinimumTreeV249:InvalidBankInput', ...
        'The tree-bank context or enumeration cap is invalid.');
end
formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
formationUids = sort(unique(formationBySensor));
formationCount = numel(formationUids);
members = cell(1, formationCount);
for formationIdx = 1:formationCount
    members{formationIdx} = find( ...
        formationBySensor == formationUids(formationIdx));
end
physical = logical(context.physicalAdjacency);
available = false(formationCount);
edgeList = zeros(0, 2);
for left = 1:formationCount-1
    for right = left+1:formationCount
        block = physical(members{left}, members{right});
        available(left, right) = any(block(:));
        available(right, left) = available(left, right);
        if available(left, right)
            edgeList(end + 1, :) = [left, right]; %#ok<AGROW>
        end
    end
end
if ~isConnected(available)
    error('HorizonValueProjectedMinimumTreeV249:DisconnectedGraph', ...
        'The current physical formation graph has no spanning tree.');
end
rawCount = nchoosek(size(edgeList, 1), formationCount - 1);
if rawCount > maximumCandidateCount
    error('HorizonValueProjectedMinimumTreeV249:EnumerationCap', ...
        'The raw M24 tree bank exceeds the frozen enumeration cap.');
end
combinations = nchoosek(1:size(edgeList, 1), formationCount - 1);
empty = struct('candidateIndex', 0, 'formationUidPairs', ...
    zeros(0, 2), 'formationTreeAdjacency', false(formationCount), ...
    'sensorAdjacency', false(size(physical)), ...
    'fusionWeightMatrix', zeros(size(physical)), ...
    'messageCount', 0, 'stronglyConnected', false, ...
    'physicallyFeasible', false, 'weightsValid', false, ...
    'requestedTreeApplied', false);
candidates = repmat(empty, 1, 0);
for rawIdx = 1:size(combinations, 1)
    tree = false(formationCount);
    selected = edgeList(combinations(rawIdx, :), :);
    for edgeIdx = 1:size(selected, 1)
        tree(selected(edgeIdx, 1), selected(edgeIdx, 2)) = true;
        tree(selected(edgeIdx, 2), selected(edgeIdx, 1)) = true;
    end
    if ~isTree(tree)
        continue;
    end
    pairs = zeros(formationCount - 1, 2);
    cursor = 0;
    for left = 1:formationCount-1
        for right = left+1:formationCount
            if tree(left, right)
                cursor = cursor + 1;
                pairs(cursor, :) = sort([ ...
                    formationUids(left), formationUids(right)]);
            end
        end
    end
    pairs = sortrows(pairs, [1, 2]);
    [adjacency, details] = ...
        selectHorizonValueProjectedMinimumTreeV249Policy( ...
            context, pairs);
    if ~details.requestedTreeApplied
        continue;
    end
    weights = details.fusionWeightMatrix;
    positiveAllowed = logical(adjacency) | ...
        logical(eye(size(adjacency, 1)));
    candidate = empty;
    candidate.candidateIndex = numel(candidates) + 1;
    candidate.formationUidPairs = pairs;
    candidate.formationTreeAdjacency = tree;
    candidate.sensorAdjacency = logical(adjacency);
    candidate.fusionWeightMatrix = weights;
    candidate.messageCount = nnz(adjacency);
    candidate.stronglyConnected = isStronglyConnected(adjacency);
    candidate.physicallyFeasible = ...
        ~any(adjacency(:) & ~physical(:));
    candidate.weightsValid = ...
        all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
        all(weights(:) >= 0) && ...
        ~any(weights(:) > 0 & ~positiveAllowed(:));
    candidate.requestedTreeApplied = true;
    candidates(end + 1) = candidate; %#ok<AGROW>
end
referencePairs = normalizePairs(referencePairs);
referenceIndex = 0;
for candidateIdx = 1:numel(candidates)
    if isequal(candidates(candidateIdx).formationUidPairs, referencePairs)
        referenceIndex = candidateIdx;
        break;
    end
end
if isempty(candidates) || referenceIndex == 0 || ...
        any(~[candidates.stronglyConnected]) || ...
        any(~[candidates.physicallyFeasible]) || ...
        any(~[candidates.weightsValid])
    error('HorizonValueProjectedMinimumTreeV249:InvalidBank', ...
        'The feasible bank is empty, lacks V242, or violates a hard gate.');
end
bank = struct();
bank.contractVersion = ...
    'horizon-value-projected-minimum-tree-v249-bank-v1';
bank.currentTime = context.currentTime;
bank.formationPhysicalUids = formationUids;
bank.availableFormationAdjacency = available;
bank.rawTreeCount = size(combinations, 1);
bank.feasibleTreeCount = numel(candidates);
bank.referenceFormationUidPairs = referencePairs;
bank.referenceCandidateIndex = referenceIndex;
bank.candidates = candidates;
bank.truthUsed = false;
bank.posteriorUsed = false;
bank.futurePhysicalPageUsed = false;
end

function pairs = normalizePairs(pairs)
if ~isnumeric(pairs) || size(pairs, 2) ~= 2
    error('HorizonValueProjectedMinimumTreeV249:InvalidReferencePairs', ...
        'Reference formation pairs must be K-by-2.');
end
pairs = sortrows(sort(pairs, 2), [1, 2]);
end

function passed = isTree(adjacency)
passed = isequal(adjacency, adjacency') && ...
    nnz(triu(adjacency, 1)) == size(adjacency, 1) - 1 && ...
    isConnected(adjacency);
end

function connected = isStronglyConnected(adjacency)
connected = isReachable(adjacency) && isReachable(adjacency');
end

function connected = isConnected(adjacency)
connected = isReachable(adjacency);
end

function passed = isReachable(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end
