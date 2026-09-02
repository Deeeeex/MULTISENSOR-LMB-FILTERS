function bank = buildCausalGatewayEmbeddingCandidateBankV250( ...
        context, maximumCandidateCount)
% BUILDCAUSALGATEWAYEMBEDDINGCANDIDATEBANKV250 Bounded gateway bank.

protocol = getCausalGatewayEmbeddingV250Protocol();
if nargin < 2 || isempty(maximumCandidateCount)
    maximumCandidateCount = protocol.maximumCandidateCount;
end
if ~isscalar(maximumCandidateCount) || ...
        maximumCandidateCount < protocol.minimumCandidateCount || ...
        maximumCandidateCount > protocol.maximumCandidateCount
    error('CausalGatewayEmbeddingV250:InvalidBankCap', ...
        'The candidate-bank cap violates the frozen V250 protocol.');
end

[referenceAdjacency, reference] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
identity = buildIdentity(context);
referenceAssignment = assignmentFromCrossAdjacency( ...
    reference.crossResidualAdjacency, identity);
localBanks = buildLocalAssignmentBanks( ...
    context, identity, referenceAssignment, protocol);

definitions = repmat(emptyDefinition(), 1, 0);
definitions = appendUniqueDefinition(definitions, ...
    referenceAssignment, 'v242-reference', 0, 0, ...
    maximumCandidateCount);

% Coordinated profiles and receiver-local replacements are inserted before
% single-arc edits so every formation remains represented if the cap binds.
for rankIdx = 1:protocol.globalRankProfileCount
    assignment = referenceAssignment;
    complete = true;
    for formationIdx = 1:numel(localBanks)
        alternatives = localBanks(formationIdx).alternativeAssignments;
        if numel(alternatives) < rankIdx
            complete = false;
            break;
        end
        rows = assignment(:, 2) == ...
            localBanks(formationIdx).receiverFormationUid;
        assignment(rows, :) = alternatives{rankIdx};
    end
    if complete
        definitions = appendUniqueDefinition(definitions, assignment, ...
            'global-rank-profile', 0, rankIdx, ...
            maximumCandidateCount);
    end
end
for rankIdx = 1:protocol.localAssignmentsPerFormation
    for formationIdx = 1:numel(localBanks)
        alternatives = localBanks(formationIdx).alternativeAssignments;
        if numel(alternatives) < rankIdx
            continue;
        end
        assignment = referenceAssignment;
        rows = assignment(:, 2) == ...
            localBanks(formationIdx).receiverFormationUid;
        assignment(rows, :) = alternatives{rankIdx};
        definitions = appendUniqueDefinition(definitions, assignment, ...
            'receiver-local-assignment', ...
            localBanks(formationIdx).receiverFormationUid, rankIdx, ...
            maximumCandidateCount);
    end
end

edgeOptions = buildDirectedEdgeOptions( ...
    context, identity, referenceAssignment(:, 1:2));
for rankIdx = 1:protocol.singleArcAlternativesPerSlot
    for slotIdx = 1:size(referenceAssignment, 1)
        assignment = referenceAssignment;
        receiverFormationUid = assignment(slotIdx, 2);
        occupiedReceivers = assignment( ...
            assignment(:, 2) == receiverFormationUid, 4);
        occupiedReceivers(assignment( ...
            assignment(:, 2) == receiverFormationUid, 1) == ...
            assignment(slotIdx, 1)) = [];
        options = edgeOptions{slotIdx};
        keep = ~ismember(options(:, 4), occupiedReceivers) & ...
            ~(options(:, 3) == assignment(slotIdx, 3) & ...
              options(:, 4) == assignment(slotIdx, 4));
        options = options(keep, :);
        if size(options, 1) < rankIdx
            continue;
        end
        assignment(slotIdx, 3:4) = options(rankIdx, 3:4);
        definitions = appendUniqueDefinition(definitions, assignment, ...
            'single-directed-arc', slotIdx, rankIdx, ...
            maximumCandidateCount);
    end
end

if numel(definitions) < protocol.minimumCandidateCount
    error('CausalGatewayEmbeddingV250:InsufficientCandidates', ...
        'The bounded construction produced too few unique candidates.');
end

candidates = repmat(emptyCandidate(), 1, numel(definitions));
for candidateIdx = 1:numel(definitions)
    definition = definitions(candidateIdx);
    [adjacency, details] = ...
        selectCausalGatewayEmbeddingV250Policy( ...
            context, definition.assignment);
    if ~details.requestedGatewayAssignmentApplied
        error('CausalGatewayEmbeddingV250:CandidateFallback', ...
            'A frozen bank candidate required V242 fallback.');
    end
    weights = details.fusionWeightMatrix;
    positiveAllowed = logical(adjacency) | ...
        logical(eye(identity.nodeCount));
    score = scoreAssignment( ...
        details.appliedGatewayAssignment, context, identity);
    candidate = emptyCandidate();
    candidate.candidateIndex = candidateIdx;
    candidate.candidateType = definition.type;
    candidate.sourceUid = definition.sourceUid;
    candidate.rank = definition.rank;
    candidate.gatewayAssignment = ...
        details.appliedGatewayAssignment;
    candidate.sensorAdjacency = logical(adjacency);
    candidate.fusionWeightMatrix = weights;
    candidate.messageCount = nnz(adjacency);
    candidate.stronglyConnected = isStronglyConnected(adjacency);
    candidate.physicallyFeasible = ...
        ~any(adjacency(:) & ~logical(context.physicalAdjacency(:)));
    candidate.weightsValid = ...
        all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
        all(weights(:) >= -1e-12) && ...
        ~any(weights(:) > 0 & ~positiveAllowed(:));
    candidate.requestedGatewayAssignmentApplied = true;
    candidate.bottleneckReliability = score(1);
    candidate.totalLogReliability = score(2);
    candidate.totalDistance = score(3);
    candidates(candidateIdx) = candidate;
end

expectedMessages = identity.nodeCount + ...
    2 * (identity.formationCount - 1);
coverage = zeros(1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    receiverUid = identity.formationUids(formationIdx);
    represented = zeros(1, 0);
    for candidateIdx = 1:numel(candidates)
        assignment = candidates(candidateIdx).gatewayAssignment;
        represented = [represented, ... %#ok<AGROW>
            reshape(assignment(assignment(:, 2) == receiverUid, 4), ...
                1, [])];
    end
    coverage(formationIdx) = numel(unique(represented));
end
rawLocalCounts = [localBanks.rawAssignmentCount];
rawGlobalCount = prod(rawLocalCounts);
hardGatePassed = numel(candidates) >= ...
        protocol.minimumCandidateCount && ...
    numel(candidates) <= maximumCandidateCount && ...
    all(coverage >= protocol.minimumReceiverCoveragePerFormation) && ...
    all([candidates.messageCount] == expectedMessages) && ...
    all([candidates.stronglyConnected]) && ...
    all([candidates.physicallyFeasible]) && ...
    all([candidates.weightsValid]) && ...
    all([candidates.requestedGatewayAssignmentApplied]);
if ~hardGatePassed
    error('CausalGatewayEmbeddingV250:InvalidBank', ...
        'The V250 gateway bank violates a structural or diversity gate.');
end

bank = struct();
bank.contractVersion = ...
    'causal-gateway-embedding-v250-bank-v1';
bank.currentTime = context.currentTime;
bank.formationPhysicalUids = identity.formationUids;
bank.referenceSensorAdjacency = logical(referenceAdjacency);
bank.referenceGatewayAssignment = referenceAssignment;
bank.referenceCandidateIndex = 1;
bank.referenceFormationUidPairs = ...
    reference.currentFormationTreePairs;
bank.candidateCount = numel(candidates);
bank.candidates = candidates;
bank.localAssignmentBanks = localBanks;
bank.rawLocalAssignmentCountByFormation = rawLocalCounts;
bank.rawGlobalAssignmentCount = rawGlobalCount;
bank.receiverCoverageByFormation = coverage;
bank.minimumReceiverCoverage = min(coverage);
bank.expectedMessageCount = expectedMessages;
bank.hardGatePassed = true;
bank.truthUsed = false;
bank.posteriorUsed = false;
bank.measurementUsed = false;
bank.futurePhysicalPageUsed = false;
bank.trackingOutcomeUsed = false;
end

function identity = buildIdentity(context)
required = {'localPosteriorBySensor', 'physicalAdjacency', ...
    'positions', 'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor', 'commConfig'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~isfield(context.commConfig, 'pDropByEdge')
    error('CausalGatewayEmbeddingV250:InvalidBankContext', ...
        'The candidate-bank context is incomplete.');
end
identity = struct();
identity.nodeCount = numel(context.localPosteriorBySensor);
identity.sensorUids = reshape(context.sensorPhysicalUids, 1, []);
identity.formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
identity.formationUids = sort(unique(identity.formationBySensor));
identity.formationCount = numel(identity.formationUids);
identity.members = cell(1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    identity.members{formationIdx} = find( ...
        identity.formationBySensor == identity.formationUids(formationIdx));
end
if numel(identity.sensorUids) ~= identity.nodeCount || ...
        numel(unique(identity.sensorUids)) ~= identity.nodeCount || ...
        ~isequal(size(context.physicalAdjacency), ...
            [identity.nodeCount, identity.nodeCount]) || ...
        ~isequal(size(context.positions), [2, identity.nodeCount]) || ...
        ~isequal(size(context.commConfig.pDropByEdge), ...
            [identity.nodeCount, identity.nodeCount])
    error('CausalGatewayEmbeddingV250:InvalidBankIdentity', ...
        'The candidate-bank matrices or identities are malformed.');
end
end

function assignment = assignmentFromCrossAdjacency(cross, identity)
[receivers, senders] = find(logical(cross));
assignment = zeros(numel(receivers), 4);
for edgeIdx = 1:numel(receivers)
    assignment(edgeIdx, :) = [ ...
        identity.formationBySensor(senders(edgeIdx)), ...
        identity.formationBySensor(receivers(edgeIdx)), ...
        identity.sensorUids(senders(edgeIdx)), ...
        identity.sensorUids(receivers(edgeIdx))];
end
assignment = normalizeAssignment(assignment);
end

function localBanks = buildLocalAssignmentBanks( ...
        context, identity, referenceAssignment, protocol)
directedPairs = referenceAssignment(:, 1:2);
edgeOptions = buildDirectedEdgeOptions( ...
    context, identity, directedPairs);
maximumAlternatives = max(protocol.localAssignmentsPerFormation, ...
    protocol.globalRankProfileCount);
empty = struct('receiverFormationUid', 0, ...
    'directedPairRows', zeros(1, 0), ...
    'rawAssignmentCount', 0, 'referenceRank', 0, ...
    'alternativeAssignments', {{}});
localBanks = repmat(empty, 1, identity.formationCount);
for formationIdx = 1:identity.formationCount
    receiverUid = identity.formationUids(formationIdx);
    pairRows = find(directedPairs(:, 2) == receiverUid);
    optionCounts = cellfun(@(value) size(value, 1), ...
        edgeOptions(pairRows));
    rawCartesianCount = prod(optionCounts);
    slotCount = numel(pairRows);
    flatAssignments = zeros(rawCartesianCount, 4 * slotCount);
    ranking = zeros(rawCartesianCount, 3 + 4 * slotCount);
    accepted = 0;
    for rawIdx = 0:rawCartesianCount-1
        residual = rawIdx;
        assignment = zeros(slotCount, 4);
        reliability = zeros(1, slotCount);
        distance = zeros(1, slotCount);
        for localSlot = 1:slotCount
            optionCount = optionCounts(localSlot);
            optionIdx = mod(residual, optionCount) + 1;
            residual = floor(residual / optionCount);
            option = edgeOptions{pairRows(localSlot)}(optionIdx, :);
            assignment(localSlot, :) = option(1:4);
            reliability(localSlot) = option(5);
            distance(localSlot) = option(6);
        end
        if numel(unique(assignment(:, 4))) ~= slotCount
            continue;
        end
        accepted = accepted + 1;
        assignment = normalizeAssignment(assignment);
        flat = reshape(assignment', 1, []);
        flatAssignments(accepted, :) = flat;
        ranking(accepted, :) = [-min(reliability), ...
            -sum(log(max(reliability, realmin))), ...
            sum(distance), flat];
    end
    flatAssignments = flatAssignments(1:accepted, :);
    ranking = ranking(1:accepted, :);
    if accepted == 0
        error('CausalGatewayEmbeddingV250:NoLocalAssignment', ...
            'A receiving formation has no injective gateway assignment.');
    end
    [~, order] = sortrows(ranking, 1:size(ranking, 2));
    referenceLocal = normalizeAssignment( ...
        referenceAssignment(pairRows, :));
    alternatives = cell(1, 0);
    referenceRank = 0;
    for orderIdx = 1:numel(order)
        assignment = reshape( ...
            flatAssignments(order(orderIdx), :), 4, [])';
        assignment = normalizeAssignment(assignment);
        if isequal(assignment, referenceLocal)
            referenceRank = orderIdx;
            continue;
        end
        if numel(alternatives) < maximumAlternatives
            alternatives{end + 1} = assignment; %#ok<AGROW>
        end
        if referenceRank > 0 && ...
                numel(alternatives) >= maximumAlternatives
            break;
        end
    end
    if referenceRank == 0 || ...
            numel(alternatives) < maximumAlternatives
        error('CausalGatewayEmbeddingV250:LocalDiversity', ...
            'A receiving formation lacks the frozen local alternatives.');
    end
    localBanks(formationIdx).receiverFormationUid = receiverUid;
    localBanks(formationIdx).directedPairRows = pairRows;
    localBanks(formationIdx).rawAssignmentCount = accepted;
    localBanks(formationIdx).referenceRank = referenceRank;
    localBanks(formationIdx).alternativeAssignments = alternatives;
end
end

function edgeOptions = buildDirectedEdgeOptions( ...
        context, identity, directedPairs)
physical = logical(context.physicalAdjacency);
reliability = 1 - context.commConfig.pDropByEdge';
edgeOptions = cell(1, size(directedPairs, 1));
for pairIdx = 1:size(directedPairs, 1)
    senderFormation = directedPairs(pairIdx, 1);
    receiverFormation = directedPairs(pairIdx, 2);
    senders = find(identity.formationBySensor == senderFormation);
    receivers = find(identity.formationBySensor == receiverFormation);
    options = zeros(0, 6);
    for receiver = receivers
        for sender = senders
            if ~physical(receiver, sender)
                continue;
            end
            distance = norm( ...
                context.positions(:, receiver) - ...
                context.positions(:, sender));
            options(end + 1, :) = [senderFormation, ... %#ok<AGROW>
                receiverFormation, identity.sensorUids(sender), ...
                identity.sensorUids(receiver), ...
                reliability(receiver, sender), distance];
        end
    end
    if isempty(options)
        error('CausalGatewayEmbeddingV250:MissingDirectedEdge', ...
            'A directed formation-tree edge has no physical gateway.');
    end
    key = [-options(:, 5), options(:, 6), ...
        options(:, 3), options(:, 4)];
    [~, order] = sortrows(key, [1, 2, 3, 4]);
    edgeOptions{pairIdx} = options(order, :);
end
end

function definitions = appendUniqueDefinition(definitions, ...
        assignment, type, sourceUid, rank, maximumCandidateCount)
if numel(definitions) >= maximumCandidateCount
    return;
end
assignment = normalizeAssignment(assignment);
for idx = 1:numel(definitions)
    if isequal(definitions(idx).assignment, assignment)
        return;
    end
end
definition = emptyDefinition();
definition.assignment = assignment;
definition.type = type;
definition.sourceUid = sourceUid;
definition.rank = rank;
definitions(end + 1) = definition;
end

function value = emptyDefinition()
value = struct('assignment', zeros(0, 4), 'type', '', ...
    'sourceUid', 0, 'rank', 0);
end

function value = emptyCandidate()
value = struct('candidateIndex', 0, 'candidateType', '', ...
    'sourceUid', 0, 'rank', 0, ...
    'gatewayAssignment', zeros(0, 4), ...
    'sensorAdjacency', false(0), ...
    'fusionWeightMatrix', zeros(0), 'messageCount', 0, ...
    'stronglyConnected', false, 'physicallyFeasible', false, ...
    'weightsValid', false, ...
    'requestedGatewayAssignmentApplied', false, ...
    'bottleneckReliability', 0, 'totalLogReliability', 0, ...
    'totalDistance', 0);
end

function score = scoreAssignment(assignment, context, identity)
reliability = 1 - context.commConfig.pDropByEdge';
q = zeros(1, size(assignment, 1));
distance = zeros(1, size(assignment, 1));
for edgeIdx = 1:size(assignment, 1)
    sender = find(identity.sensorUids == assignment(edgeIdx, 3));
    receiver = find(identity.sensorUids == assignment(edgeIdx, 4));
    q(edgeIdx) = reliability(receiver, sender);
    distance(edgeIdx) = norm(context.positions(:, receiver) - ...
        context.positions(:, sender));
end
score = [min(q), sum(log(max(q, realmin))), sum(distance)];
end

function assignment = normalizeAssignment(assignment)
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, ...
        find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end
