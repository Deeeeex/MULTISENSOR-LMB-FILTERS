function [reference, details] = ...
    selectFormationB4V50PosteriorAwareCycleReference( ...
        context, incumbentReference, options)
% SELECTFORMATIONB4V50POSTERIORAWARECYCLEREFERENCE
% Rank feasible formation cycles by current-posterior residual-message
% value, then admit only proposals whose synchronized B4 contraction is no
% worse than the V46 incumbent.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getFormationB4V50RuntimeProtocol();
period = getField(options, 'period', protocol.period);
dominantWeight = getField( ...
    options, 'dominantWeight', protocol.dominantWeight);
referenceResidualWeight = getField(options, ...
    'referenceResidualWeight', protocol.referenceResidualWeight);
activeResidualWeight = getField(options, ...
    'activeResidualWeight', protocol.activeResidualWeight);
maximumCycleCount = getField(options, ...
    'maximumCanonicalCycleEnumerationCount', ...
    protocol.maximumCanonicalCycleEnumerationCount);
maximumProposals = getField(options, ...
    'maximumPosteriorProposalEvaluations', ...
    protocol.maximumPosteriorProposalEvaluations);
edgeScoreMode = getField(options, ...
    'posteriorEdgeScoreMode', protocol.posteriorEdgeScoreMode);
minimumObjective = getField(options, ...
    'minimumPosteriorObjectiveImprovement', ...
    protocol.minimumPosteriorObjectiveImprovement);
maximumWorstRegression = getField(options, ...
    'maximumWorstFormationUtilityRegression', ...
    protocol.maximumWorstFormationUtilityRegression);
structuralTolerance = getField(options, ...
    'structuralNoWorseTolerance', ...
    protocol.structuralNoWorseTolerance);

required = {'localPosteriorBySensor', 'model', 'commConfig', ...
    'currentTime', 'physicalAdjacency', 'positions', ...
    'sensorPhysicalUids', 'formationPhysicalUidsBySensor', ...
    'baseAdjacency', 'observableInputContract'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~iscell(context.localPosteriorBySensor) || ...
        ~isstruct(context.observableInputContract) || ...
        ~context.observableInputContract.passed
    error('FormationB4V50CycleReference:InvalidContext', ...
        'A causal observable posterior context is required.');
end
routeContext = buildFormationB4V49GraphOnlyRouteContext(context);
formationUids = reshape( ...
    routeContext.formationPhysicalUidsBySensor, 1, []);
nodeCount = numel(formationUids);
physical = logical(routeContext.physicalAdjacency);
if ~isValidReference(incumbentReference, physical, nodeCount)
    error('FormationB4V50CycleReference:InvalidIncumbent', ...
        'The V46 incumbent route is malformed.');
end

formation = buildFormationView(formationUids, physical);
workUpperBound = boundedFactorialHalf( ...
    formation.count - 1, maximumCycleCount);
enumerationCapped = workUpperBound > maximumCycleCount;
if enumerationCapped
    cycleOrders = zeros(0, formation.count);
else
    cycleOrders = enumerateCanonicalPhysicalCycles( ...
        formation.physicalAdjacency, formation.uids);
end

[edgeUtility, edgeDetails] = scoreEligibleResidualEdges( ...
    context, incumbentReference.dominantAdjacency, physical, ...
    edgeScoreMode);
incumbentReceiverUtility = selectedResidualUtility( ...
    incumbentReference.residualAdjacency, edgeUtility);

cycleCount = size(cycleOrders, 1);
candidateReferences = cell(1, cycleCount);
candidateFeasible = false(1, cycleCount);
candidateObjective = -inf(1, cycleCount);
candidateWorstFormationDelta = -inf(1, cycleCount);
candidateNetworkMeanDelta = -inf(1, cycleCount);
candidateFormationDelta = nan(cycleCount, formation.count);
candidateReceiverDelta = nan(cycleCount, nodeCount);

for candidateIdx = 1:cycleCount
    registered = buildRegisteredCycleAdjacency( ...
        cycleOrders(candidateIdx, :), formation, formationUids, physical);
    try
        candidate = materializeReference( ...
            routeContext, registered, dominantWeight, ...
            referenceResidualWeight);
    catch errorInfo
        if startsWith(errorInfo.identifier, ...
                'IndexEquivariantFormationRoute:')
            continue;
        end
        rethrow(errorInfo);
    end
    if ~isequal(candidate.dominantAdjacency, ...
            incumbentReference.dominantAdjacency)
        continue;
    end
    receiverUtility = selectedResidualUtility( ...
        candidate.residualAdjacency, edgeUtility);
    receiverDelta = receiverUtility - incumbentReceiverUtility;
    formationDelta = aggregateByFormation( ...
        receiverDelta, formation.members, protocol);
    worstDelta = min(formationDelta);
    meanDelta = mean(receiverDelta);
    objective = protocol.worstFormationWeight * worstDelta + ...
        protocol.networkMeanWeight * meanDelta;

    candidateReferences{candidateIdx} = candidate;
    candidateFeasible(candidateIdx) = true;
    candidateObjective(candidateIdx) = objective;
    candidateWorstFormationDelta(candidateIdx) = worstDelta;
    candidateNetworkMeanDelta(candidateIdx) = meanDelta;
    candidateFormationDelta(candidateIdx, :) = formationDelta;
    candidateReceiverDelta(candidateIdx, :) = receiverDelta;
end

feasibleIndices = find(candidateFeasible);
proposalOrder = zeros(1, 0);
if ~isempty(feasibleIndices)
    keys = [-candidateObjective(feasibleIndices)', ...
        -candidateWorstFormationDelta(feasibleIndices)', ...
        -candidateNetworkMeanDelta(feasibleIndices)', ...
        cycleOrders(feasibleIndices, :)];
    [~, order] = sortrows(keys, 1:size(keys, 2));
    proposalOrder = feasibleIndices(order)';
end
proposalOrder = proposalOrder( ...
    1:min(numel(proposalOrder), maximumProposals));

reliability = 1 - routeContext.pDropByEdge';
certificateOptions = struct( ...
    'missingNeighborWeightMode', 'renormalize', ...
    'maximumIncomingCount', 2);
incumbentStructuralScore = scoreSynchronizedReference( ...
    incumbentReference, reliability, period, dominantWeight, ...
    activeResidualWeight, certificateOptions);
candidateStructuralScore = inf(1, cycleCount);
candidateStructuralNoWorse = false(1, cycleCount);
candidatePosteriorGatePassed = false(1, cycleCount);
for proposalIdx = 1:numel(proposalOrder)
    candidateIdx = proposalOrder(proposalIdx);
    candidateStructuralScore(candidateIdx) = ...
        scoreSynchronizedReference( ...
            candidateReferences{candidateIdx}, reliability, period, ...
            dominantWeight, activeResidualWeight, ...
            certificateOptions);
    candidateStructuralNoWorse(candidateIdx) = ...
        candidateStructuralScore(candidateIdx) <= ...
            incumbentStructuralScore + structuralTolerance;
    candidatePosteriorGatePassed(candidateIdx) = ...
        candidateObjective(candidateIdx) > minimumObjective && ...
        candidateWorstFormationDelta(candidateIdx) >= ...
            -maximumWorstRegression;
end

admissible = proposalOrder( ...
    candidateStructuralNoWorse(proposalOrder) & ...
    candidatePosteriorGatePassed(proposalOrder));
selectedIdx = NaN;
if ~isempty(admissible)
    selectedIdx = admissible(1);
end
selectedIsCycle = isfinite(selectedIdx);
if selectedIsCycle
    reference = candidateReferences{selectedIdx};
    fallbackReason = 'none';
    selectedCycleOrder = cycleOrders(selectedIdx, :);
    selectedObjective = candidateObjective(selectedIdx);
    selectedWorstFormationDelta = ...
        candidateWorstFormationDelta(selectedIdx);
    selectedNetworkMeanDelta = ...
        candidateNetworkMeanDelta(selectedIdx);
    selectedFormationDelta = ...
        candidateFormationDelta(selectedIdx, :);
else
    reference = incumbentReference;
    selectedCycleOrder = zeros(1, 0);
    selectedObjective = 0;
    selectedWorstFormationDelta = 0;
    selectedNetworkMeanDelta = 0;
    selectedFormationDelta = zeros(1, formation.count);
    if enumerationCapped
        fallbackReason = 'enumeration-capped';
    elseif cycleCount == 0
        fallbackReason = 'no-physical-cycle';
    elseif isempty(feasibleIndices)
        fallbackReason = 'no-feasible-cycle';
    elseif ~any(candidatePosteriorGatePassed(proposalOrder))
        fallbackReason = 'no-posterior-value-improvement';
    else
        fallbackReason = 'structural-no-worse-gate';
    end
end

details = struct();
details.contractVersion = ...
    'formation-b4-v50-posterior-aware-cycle-reference-v1';
details.currentTime = context.currentTime;
details.nodeCount = nodeCount;
details.formationCount = formation.count;
details.canonicalPhysicalCycleOrders = cycleOrders;
details.cycleEnumerationCapped = enumerationCapped;
details.cycleEnumerationWorkUpperBound = workUpperBound;
details.physicalCycleCandidateCount = cycleCount;
details.feasibleCycleCandidateMask = candidateFeasible;
details.feasibleCycleCandidateCount = nnz(candidateFeasible);
details.posteriorProposalOrder = proposalOrder;
details.maximumPosteriorProposalEvaluations = maximumProposals;
details.candidatePosteriorObjective = candidateObjective;
details.candidateWorstFormationUtilityDelta = ...
    candidateWorstFormationDelta;
details.candidateNetworkMeanUtilityDelta = ...
    candidateNetworkMeanDelta;
details.candidateFormationUtilityDelta = candidateFormationDelta;
details.candidateReceiverUtilityDelta = candidateReceiverDelta;
details.candidateStructuralScore = candidateStructuralScore;
details.candidateStructuralNoWorseMask = ...
    candidateStructuralNoWorse;
details.candidatePosteriorGatePassedMask = ...
    candidatePosteriorGatePassed;
details.incumbentStructuralScore = incumbentStructuralScore;
details.minimumPosteriorObjectiveImprovement = minimumObjective;
details.maximumWorstFormationUtilityRegression = ...
    maximumWorstRegression;
details.selectedIsCyclePreservingReference = selectedIsCycle;
details.selectedUsedIncumbentFallback = ~selectedIsCycle;
details.selectedFallbackReason = fallbackReason;
details.selectedCycleCandidateIndex = selectedIdx;
details.selectedCycleOrder = selectedCycleOrder;
details.selectedPosteriorObjective = selectedObjective;
details.selectedWorstFormationUtilityDelta = ...
    selectedWorstFormationDelta;
details.selectedNetworkMeanUtilityDelta = selectedNetworkMeanDelta;
details.selectedFormationUtilityDelta = selectedFormationDelta;
details.posteriorEdgeScoreMode = edgeScoreMode;
details.posteriorEdgeScoreDetails = edgeDetails;
details.posteriorUsed = true;
details.currentPosteriorUsed = true;
details.posteriorUsedForRoutingOnly = true;
details.fullPosteriorFusionUnchanged = true;
details.currentLinkReliabilityUsed = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
details.validationClaimAllowed = false;
details.developmentEvidenceOnly = true;
details.selectedReference = reference;
details.incumbentReference = incumbentReference;
end

function [utility, details] = scoreEligibleResidualEdges( ...
        context, dominant, physical, mode)
eligible = logical(physical) & ~logical(dominant);
eligible(1:size(eligible, 1)+1:end) = false;
[receivers, senders] = find(eligible);
if isempty(receivers)
    error('FormationB4V50CycleReference:NoResidualEdges', ...
        'No physical residual-message candidates are available.');
end
[scores, scoreDetails] = computeLabelSetDominantRoutingScores( ...
    context, receivers, senders, mode);
utility = nan(size(eligible));
for edgeIdx = 1:numel(scores)
    utility(receivers(edgeIdx), senders(edgeIdx)) = scores(edgeIdx);
end
details = struct( ...
    'eligibleResidualAdjacency', eligible, ...
    'receiverIndices', receivers, ...
    'senderIndices', senders, ...
    'scores', scores, ...
    'scoreDetails', scoreDetails);
end

function values = selectedResidualUtility(residual, utility)
nodeCount = size(residual, 1);
values = zeros(nodeCount, 1);
for receiverIdx = 1:nodeCount
    senderIdx = find(residual(receiverIdx, :), 1);
    if isempty(senderIdx) || ~isfinite(utility(receiverIdx, senderIdx))
        error('FormationB4V50CycleReference:InvalidResidualChoice', ...
            'A residual route selected an ineligible message.');
    end
    values(receiverIdx) = utility(receiverIdx, senderIdx);
end
end

function formationValues = aggregateByFormation( ...
        receiverDelta, members, protocol)
formationValues = zeros(1, numel(members));
for formationIdx = 1:numel(members)
    values = sort(receiverDelta(members{formationIdx}), 'ascend');
    tailCount = max(1, ceil(numel(values) / 4));
    tailValue = mean(values(1:tailCount));
    formationValues(formationIdx) = ...
        protocol.formationMeanWeight * mean(values) + ...
        protocol.formationLowerQuartileWeight * tailValue;
end
end

function formation = buildFormationView(formationUidsBySensor, physical)
uids = sort(unique(formationUidsBySensor));
members = cell(1, numel(uids));
adjacency = false(numel(uids));
for formationIdx = 1:numel(uids)
    members{formationIdx} = find( ...
        formationUidsBySensor == uids(formationIdx));
end
for receiverIdx = 1:numel(uids)
    for senderIdx = 1:numel(uids)
        if receiverIdx ~= senderIdx
            adjacency(receiverIdx, senderIdx) = any(any(physical( ...
                members{receiverIdx}, members{senderIdx})));
        end
    end
end
formation = struct('count', numel(uids), 'uids', uids, ...
    'members', {members}, ...
    'physicalAdjacency', adjacency & adjacency');
end

function orders = enumerateCanonicalPhysicalCycles(adjacency, uids)
count = numel(uids);
tailOrders = perms(2:count);
orders = zeros(0, count);
for rowIdx = 1:size(tailOrders, 1)
    order = [1, tailOrders(rowIdx, :)];
    if uids(order(2)) > uids(order(end))
        continue;
    end
    valid = true;
    for position = 1:count
        valid = valid && adjacency(order(position), ...
            order(mod(position, count) + 1));
    end
    if valid
        orders(end + 1, :) = uids(order); %#ok<AGROW>
    end
end
end

function registered = buildRegisteredCycleAdjacency( ...
        cycleUids, formation, formationUidsBySensor, physical)
cyclePairs = false(formation.count);
for position = 1:formation.count
    first = find(formation.uids == cycleUids(position));
    second = find(formation.uids == ...
        cycleUids(mod(position, formation.count) + 1));
    cyclePairs(first, second) = true;
    cyclePairs(second, first) = true;
end
registered = false(size(physical));
for receiverIdx = 1:formation.count
    receivers = formationUidsBySensor == formation.uids(receiverIdx);
    registered(receivers, receivers) = physical(receivers, receivers);
    for senderIdx = 1:formation.count
        if cyclePairs(receiverIdx, senderIdx)
            senders = formationUidsBySensor == formation.uids(senderIdx);
            registered(receivers, senders) = ...
                physical(receivers, senders);
        end
    end
end
registered = registered & registered';
registered(1:size(registered, 1)+1:end) = false;
end

function reference = materializeReference( ...
        context, registered, dominantWeight, residualWeight)
nodeCount = numel(context.sensorPhysicalUids);
routeContext = struct();
routeContext.localPosteriorBySensor = cell(1, nodeCount);
routeContext.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', context.sensorGroupIds), ...
    'staticAdjacency', registered));
routeContext.baseAdjacency = registered;
routeContext.physicalAdjacency = context.physicalAdjacency;
routeContext.positions = context.positions;
routeContext.sensorPhysicalUids = context.sensorPhysicalUids;
routeContext.formationPhysicalUidsBySensor = ...
    context.formationPhysicalUidsBySensor;
routeContext.commConfig = struct('pDropByEdge', context.pDropByEdge);
routeContext.currentTime = context.currentTime;
routeContext.directedMessageBudget = 2 * nodeCount;
[adjacency, policy] = ...
    selectIndexEquivariantFormationBackbonePolicy( ...
        routeContext, struct('dominantWeight', dominantWeight, ...
            'residualWeight', residualWeight));
reference = struct( ...
    'dominantAdjacency', logical(policy.dominantAdjacency), ...
    'residualAdjacency', logical(policy.residualAdjacency), ...
    'referenceAdjacency', logical(adjacency), ...
    'referenceFusionWeights', policy.fusionWeightMatrix);
end

function score = scoreSynchronizedReference( ...
        reference, reliability, period, dominantWeight, ...
        activeResidualWeight, certificateOptions)
nodeCount = size(reference.referenceAdjacency, 1);
adjacency = false(nodeCount, nodeCount, period);
weights = zeros(nodeCount, nodeCount, period);
for phase = 1:period
    residual = false(nodeCount);
    if phase == 1
        residual = reference.residualAdjacency;
    end
    adjacency(:, :, phase) = reference.dominantAdjacency | residual;
    pageWeights = zeros(nodeCount);
    pageWeights(reference.dominantAdjacency) = dominantWeight;
    pageWeights(residual) = activeResidualWeight;
    pageWeights(1:nodeCount+1:end) = 1 - sum(pageWeights, 2)';
    weights(:, :, phase) = pageWeights;
end
certificate = computeReliableKlaWindowMeanSquareContractionCertificate( ...
    adjacency, weights, repmat(reliability, 1, 1, period), ...
    certificateOptions);
score = certificate.worstCaseExpectedSquaredContractionFactor;
end

function valid = isValidReference(reference, physical, nodeCount)
required = {'dominantAdjacency', 'residualAdjacency', ...
    'referenceAdjacency', 'referenceFusionWeights'};
valid = isstruct(reference) && isscalar(reference) && ...
    all(isfield(reference, required));
if ~valid
    return;
end
dominant = logical(reference.dominantAdjacency);
residual = logical(reference.residualAdjacency);
route = logical(reference.referenceAdjacency);
valid = isequal(size(route), [nodeCount, nodeCount]) && ...
    isequal(route, dominant | residual) && ...
    ~any(dominant(:) & residual(:)) && ...
    all(sum(dominant, 2) == 1) && ...
    all(sum(residual, 2) == 1) && ...
    ~any(route(:) & ~physical(:));
end

function count = boundedFactorialHalf(value, cap)
count = 1;
for factor = 2:value
    count = count * factor;
    if count > 2 * cap
        count = cap + 1;
        return;
    end
end
count = ceil(count / 2);
end

function value = getField(structure, name, defaultValue)
if isstruct(structure) && isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
