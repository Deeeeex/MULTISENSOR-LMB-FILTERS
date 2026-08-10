function projection = projectSenderBudgetReallocationV95( ...
        candidates, referenceAdjacency, referenceWeights, ...
        physicalAdjacency, groupIds, options)
% PROJECTSENDERBUDGETREALLOCATIONV95 Move residual sender tokens globally.

if nargin < 6 || isempty(options)
    options = struct();
end
nodeCount = numel(groupIds);
groupIds = reshape(groupIds, 1, []);
referenceAdjacency = logical(referenceAdjacency);
physicalAdjacency = logical(physicalAdjacency);
requiredTokenCount = getField(options, ...
    'requiredTokenCount', ceil(numel(unique(groupIds, 'stable')) / 2));
sourceWeight = getField(options, 'sourceWeight', 0.05);
minimumNetUtility = getField(options, 'minimumNetUtilityFraction', 0);
requireStrongConnectivity = logical(getField( ...
    options, 'requireCandidateStrongConnectivity', false));
maximumConnectivityCuts = round(getField( ...
    options, 'maximumConnectivityCuts', 4096));
if ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(referenceWeights), [nodeCount, nodeCount]) || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        requiredTokenCount < 1 || ...
        requiredTokenCount ~= round(requiredTokenCount)
    error('NetworkBudgetV95:InvalidProjectionInput', ...
        'The V95 route matrices or token count are invalid.');
end

projection = emptyProjection(referenceAdjacency, referenceWeights);
if isempty(candidates)
    projection.failureReason = 'no-candidate-token';
    return;
end
requiredFields = { ...
    'targetReceiverIdx', 'donorReceiverIdx', 'senderIdx', ...
    'targetFormationId', 'sourceFormationId', ...
    'targetNoveltyFraction', 'donorUniqueFraction', ...
    'netUtilityFraction'};
if ~isstruct(candidates) || ~all(isfield(candidates, requiredFields))
    error('NetworkBudgetV95:InvalidCandidates', ...
        'V95 candidates do not satisfy the frozen token contract.');
end

target = reshape([candidates.targetReceiverIdx], 1, []);
donor = reshape([candidates.donorReceiverIdx], 1, []);
sender = reshape([candidates.senderIdx], 1, []);
targetFormation = reshape([candidates.targetFormationId], 1, []);
sourceFormation = reshape([candidates.sourceFormationId], 1, []);
utility = reshape([candidates.netUtilityFraction], 1, []);
valid = isfinite(target) & isfinite(donor) & isfinite(sender) & ...
    target == round(target) & donor == round(donor) & ...
    sender == round(sender) & target >= 1 & target <= nodeCount & ...
    donor >= 1 & donor <= nodeCount & sender >= 1 & ...
    sender <= nodeCount & target ~= donor & ...
    isfinite(utility) & utility >= minimumNetUtility - 1e-12;
for idx = find(valid)
    valid(idx) = referenceAdjacency(donor(idx), sender(idx)) && ...
        abs(referenceWeights(donor(idx), sender(idx)) - ...
            sourceWeight) <= 1e-12 && ...
        ~referenceAdjacency(target(idx), sender(idx)) && ...
        physicalAdjacency(target(idx), sender(idx)) && ...
        referenceWeights(target(idx), target(idx)) >= ...
            sourceWeight - 1e-12;
end
candidates = candidates(valid);
target = target(valid);
donor = donor(valid);
sender = sender(valid);
targetFormation = targetFormation(valid);
sourceFormation = sourceFormation(valid);
utility = utility(valid);
candidateCount = numel(candidates);
projection.candidateCount = candidateCount;
if candidateCount < requiredTokenCount
    projection.failureReason = 'insufficient-candidate-tokens';
    return;
end
if exist('glpk', 'file') ~= 2
    error('NetworkBudgetV95:SolverUnavailable', ...
        'V95 requires GLPK for its frozen global token projection.');
end

[A, b, constraintTypes] = buildConstraints( ...
    nodeCount, target, donor, sender, targetFormation, ...
    sourceFormation, requiredTokenCount);
lower = zeros(candidateCount, 1);
upper = ones(candidateCount, 1);
variableTypes = repmat('I', 1, candidateCount);
tieBreak = (candidateCount:-1:1) / max(candidateCount, 1);
objectiveVector = utility(:) + 1e-9 * tieBreak(:);
solverOptions = struct('msglev', 0, 'tmlim', round(getField( ...
    options, 'maximumSolverSeconds', 10)));
workingA = A;
workingB = b;
workingTypes = constraintTypes;
connectivityCutCount = 0;
while true
    [x, objective, errorNumber, extra] = glpk( ...
        objectiveVector, workingA, workingB, lower, upper, ...
        workingTypes, variableTypes, -1, solverOptions);
    status = NaN;
    if isstruct(extra) && isfield(extra, 'status')
        status = extra.status;
    end
    if any(errorNumber == [10, 15]) || any(status == [3, 4])
        if connectivityCutCount > 0
            projection.failureReason = ...
                'strong-connectivity-projection-infeasible';
        else
            projection.failureReason = ...
                'exact-token-projection-infeasible';
        end
        projection.solverErrorNumber = errorNumber;
        projection.solverStatus = status;
        projection.connectivityCutCount = connectivityCutCount;
        return;
    end
    if errorNumber ~= 0 || status ~= 5 || isempty(x) || ...
            any(~isfinite(x))
        error('NetworkBudgetV95:SolverFailure', ...
            'V95 token projection failed (error %d, status %g).', ...
            errorNumber, status);
    end
    selectedIndices = reshape(find(x > 0.5), 1, []);
    if numel(selectedIndices) ~= requiredTokenCount
        error('NetworkBudgetV95:ProjectionCardinalityDrift', ...
            'The certified V95 projection changed its token count.');
    end
    selected = candidates(selectedIndices);
    [donorOnlyAdjacency, donorOnlyWeights, ...
        candidateAdjacency, candidateWeights] = applyTokens( ...
            selected, referenceAdjacency, referenceWeights, ...
            sourceWeight);
    candidateStrongConnectivity = ...
        isStronglyConnected(candidateAdjacency);
    if ~requireStrongConnectivity || candidateStrongConnectivity
        break;
    end
    connectivityCutCount = connectivityCutCount + 1;
    if connectivityCutCount > maximumConnectivityCuts
        projection.failureReason = ...
            'strong-connectivity-cut-limit-exceeded';
        projection.connectivityCutCount = connectivityCutCount;
        return;
    end
    cut = sparse(1, candidateCount);
    cut(selectedIndices) = 1;
    workingA(end + 1, :) = cut;
    workingB(end + 1, 1) = requiredTokenCount - 1;
    workingTypes(end + 1) = 'U';
end

candidatePhysical = ~any(candidateAdjacency(:) & ...
    ~physicalAdjacency(:));
candidateWeightValid = validWeights( ...
    candidateAdjacency, candidateWeights);
donorOnlyWeightValid = validWeights( ...
    donorOnlyAdjacency, donorOnlyWeights);
senderMessageParity = isequal( ...
    sum(candidateAdjacency, 1), sum(referenceAdjacency, 1));
networkMessageParity = nnz(candidateAdjacency) == ...
    nnz(referenceAdjacency);
targetFormationIds = unique( ...
    [selected.targetFormationId], 'stable');
sourceFormationIds = unique( ...
    [selected.sourceFormationId], 'stable');
projection.feasible = candidatePhysical && ...
    candidateWeightValid && donorOnlyWeightValid && ...
    senderMessageParity && networkMessageParity && ...
    (~requireStrongConnectivity || candidateStrongConnectivity);
projection.failureReason = '';
if ~projection.feasible
    projection.failureReason = 'post-projection-invariant-failure';
end
projection.selectedCandidateIndices = selectedIndices;
projection.selectedCandidates = selected;
projection.selectedTokenCount = numel(selected);
projection.selectedTargetFormationIds = targetFormationIds;
projection.selectedSourceFormationIds = sourceFormationIds;
projection.aggregateTargetNoveltyFraction = sum( ...
    [selected.targetNoveltyFraction]);
projection.aggregateDonorUniqueFraction = sum( ...
    [selected.donorUniqueFraction]);
projection.aggregateNetUtilityFraction = sum( ...
    [selected.netUtilityFraction]);
projection.referenceAdjacency = referenceAdjacency;
projection.referenceWeights = referenceWeights;
projection.donorOnlyAdjacency = donorOnlyAdjacency;
projection.donorOnlyWeights = donorOnlyWeights;
projection.candidateAdjacency = candidateAdjacency;
projection.candidateWeights = candidateWeights;
projection.referenceMessageCount = nnz(referenceAdjacency);
projection.donorOnlyMessageCount = nnz(donorOnlyAdjacency);
projection.candidateMessageCount = nnz(candidateAdjacency);
projection.senderMessageParityPassed = senderMessageParity;
projection.networkMessageParityPassed = networkMessageParity;
projection.candidatePhysicalPassed = candidatePhysical;
projection.candidateWeightPassed = candidateWeightValid;
projection.donorOnlyWeightPassed = donorOnlyWeightValid;
projection.candidateStrongConnectivityPassed = ...
    candidateStrongConnectivity;
projection.connectivityCutCount = connectivityCutCount;
projection.solverObjective = objective;
projection.solverErrorNumber = errorNumber;
projection.solverStatus = status;
end

function [donorOnlyAdjacency, donorOnlyWeights, ...
        candidateAdjacency, candidateWeights] = applyTokens( ...
            selected, referenceAdjacency, referenceWeights, sourceWeight)
donorOnlyAdjacency = referenceAdjacency;
donorOnlyWeights = referenceWeights;
candidateAdjacency = referenceAdjacency;
candidateWeights = referenceWeights;
for record = selected
    donorIdx = record.donorReceiverIdx;
    targetIdx = record.targetReceiverIdx;
    senderIdx = record.senderIdx;
    donorOnlyAdjacency(donorIdx, senderIdx) = false;
    donorOnlyWeights(donorIdx, senderIdx) = 0;
    donorOnlyWeights(donorIdx, donorIdx) = ...
        donorOnlyWeights(donorIdx, donorIdx) + sourceWeight;
    candidateAdjacency(donorIdx, senderIdx) = false;
    candidateWeights(donorIdx, senderIdx) = 0;
    candidateWeights(donorIdx, donorIdx) = ...
        candidateWeights(donorIdx, donorIdx) + sourceWeight;
    candidateAdjacency(targetIdx, senderIdx) = true;
    candidateWeights(targetIdx, senderIdx) = sourceWeight;
    candidateWeights(targetIdx, targetIdx) = ...
        candidateWeights(targetIdx, targetIdx) - sourceWeight;
end
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function connected = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
connected = all(visited);
end

function [A, b, types] = buildConstraints( ...
        nodeCount, target, donor, sender, targetFormation, ...
        sourceFormation, requiredTokenCount)
count = numel(target);
A = sparse(0, count);
b = zeros(0, 1);
types = '';
row = 0;
for nodeIdx = 1:nodeCount
    row = row + 1;
    A(row, target == nodeIdx | donor == nodeIdx) = 1;
    b(row, 1) = 1;
    types(end + 1) = 'U'; %#ok<AGROW>
    row = row + 1;
    A(row, sender == nodeIdx) = 1;
    b(row, 1) = 1;
    types(end + 1) = 'U'; %#ok<AGROW>
end
for formationId = unique(targetFormation, 'stable')
    row = row + 1;
    A(row, targetFormation == formationId) = 1;
    b(row, 1) = 1;
    types(end + 1) = 'U'; %#ok<AGROW>
end
for formationId = unique(sourceFormation, 'stable')
    row = row + 1;
    A(row, sourceFormation == formationId) = 1;
    b(row, 1) = 1;
    types(end + 1) = 'U'; %#ok<AGROW>
end
row = row + 1;
A(row, :) = 1;
b(row, 1) = requiredTokenCount;
types(end + 1) = 'S';
end

function valid = validWeights(adjacency, weights)
support = adjacency | logical(eye(size(adjacency, 1)));
valid = all(isfinite(weights(:))) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
end

function projection = emptyProjection(adjacency, weights)
projection = struct( ...
    'feasible', false, ...
    'failureReason', '', ...
    'candidateCount', 0, ...
    'selectedCandidateIndices', zeros(1, 0), ...
    'selectedCandidates', struct([]), ...
    'selectedTokenCount', 0, ...
    'selectedTargetFormationIds', zeros(1, 0), ...
    'selectedSourceFormationIds', zeros(1, 0), ...
    'aggregateTargetNoveltyFraction', 0, ...
    'aggregateDonorUniqueFraction', 0, ...
    'aggregateNetUtilityFraction', 0, ...
    'referenceAdjacency', adjacency, ...
    'referenceWeights', weights, ...
    'donorOnlyAdjacency', adjacency, ...
    'donorOnlyWeights', weights, ...
    'candidateAdjacency', adjacency, ...
    'candidateWeights', weights, ...
    'referenceMessageCount', nnz(adjacency), ...
    'donorOnlyMessageCount', nnz(adjacency), ...
    'candidateMessageCount', nnz(adjacency), ...
    'senderMessageParityPassed', false, ...
    'networkMessageParityPassed', false, ...
    'candidatePhysicalPassed', false, ...
    'candidateWeightPassed', false, ...
    'donorOnlyWeightPassed', false, ...
    'candidateStrongConnectivityPassed', false, ...
    'connectivityCutCount', 0, ...
    'solverObjective', NaN, ...
    'solverErrorNumber', NaN, ...
    'solverStatus', NaN);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
