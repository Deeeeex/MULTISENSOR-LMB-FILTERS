function [adjacency, details] = ...
    selectPhysicalFormationTreeResidualTourPolicy(context, options)
% SELECTPHYSICALFORMATIONTREERESIDUALTOURPOLICY Geometry-only reference.
%
% The dominant within-formation input is unchanged.  The residual input is
% a deterministic Hamiltonian sensor tour induced by a path tree in the
% current physical formation graph.  Candidate paths maximize, in order,
% the weakest bidirectional sensor-pair support and total support.  No
% posterior, measurement support, link outcome, truth, or future geometry
% is read.

if nargin < 2 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
nodeCount = numel(context.localPosteriorBySensor);
if ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds') || ...
        ~isfield(context, 'physicalAdjacency')
    error('PhysicalFormationTreeReference:InvalidContext', ...
        'Formation metadata and current physical links are required.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
physical = logical(context.physicalAdjacency);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(physical), [nodeCount, nodeCount])
    error('PhysicalFormationTreeReference:InvalidContext', ...
        'Formation metadata or physical graph dimensions are invalid.');
end
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
if formationCount < 2 || formationCount > 8
    error('PhysicalFormationTreeReference:InvalidContext', ...
        'The physical-tree reference supports two to eight formations.');
end

[formationPhysical, support] = collapsePhysical( ...
    physical, groupIds, groups);
candidatePaths = enumerateCandidatePaths(formationPhysical, support);
if isempty(candidatePaths)
    error('PhysicalFormationTreeReference:NoFormationPath', [ ...
        'The current physical formation graph has no Hamiltonian path; ', ...
        'the frozen path-tree reference is unavailable.']);
end

[dominantAdjacency, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
selectedPath = zeros(1, 0);
selectedRegistration = false(nodeCount);
residualAdjacency = false(nodeCount);
tour = struct();
selectedCandidateIndex = NaN;
for candidateIdx = 1:size(candidatePaths, 1)
    path = candidatePaths(candidateIdx, 3:end);
    registration = registerFormationPath(path, groupIds, groups);
    try
        [candidateResidual, candidateTour] = ...
            buildRegisteredFormationBackboneResidualTour( ...
                groupIds, registration, physical);
    catch errorInfo
        if strcmp(errorInfo.identifier, ...
                'FormationBackboneTour:NonphysicalTour')
            continue;
        end
        rethrow(errorInfo);
    end
    selectedPath = path;
    selectedRegistration = registration;
    residualAdjacency = candidateResidual;
    tour = candidateTour;
    selectedCandidateIndex = candidateIdx;
    break;
end
if isempty(selectedPath)
    error('PhysicalFormationTreeReference:NoSensorTour', [ ...
        'No deterministic path-tree expansion produced a fully physical ', ...
        'sensor residual tour.']);
end

[adjacency, fusionWeights, route] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        dominantWeight, residualWeight);
if any(route.dominantSourcesByReceiver == ...
        route.residualSourcesByReceiver) || ...
        route.duplicateSourceFraction ~= 0 || ...
        nnz(adjacency) ~= 2 * nodeCount || ...
        route.maximumMessagesPerReceiver ~= 2 || ...
        (isfield(context, 'directedMessageBudget') && ...
         nnz(adjacency) > context.directedMessageBudget)
    error('PhysicalFormationTreeReference:InvalidReference', [ ...
        'The physical-tree reference must provide two distinct inputs ', ...
        'per receiver within the directed-message budget.']);
end

pathSupport = zeros(1, formationCount - 1);
for edgeIdx = 1:formationCount-1
    pathSupport(edgeIdx) = support( ...
        selectedPath(edgeIdx), selectedPath(edgeIdx + 1));
end
details = route;
details.contractVersion = ...
    'physical-formation-path-tree-residual-tour-policy-v1';
details.mode = 'physical-formation-path-tree-residual-tour';
details.objective = NaN;
details.candidateIndex = selectedCandidateIndex;
details.selectionSeconds = NaN;
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = size(candidatePaths, 1);
details.fusionWeightMatrix = fusionWeights;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.dominantPolicyDetails = dominantDetails;
details.tour = tour;
details.crossReceivers = tour.crossReceivers;
details.crossSenders = tour.crossSenders;
details.incomingCrossReceiversByFormation = ...
    tour.incomingCrossReceiversByFormation;
details.incomingCrossSendersByFormation = ...
    tour.incomingCrossSendersByFormation;
details.crossFormationMessageCount = tour.crossFormationMessageCount;
details.maximumCrossEdges = tour.crossFormationMessageCount;
details.maximumCrossSourceLoad = tour.maximumCrossSourceLoad;
details.maximumCrossReceiverLoad = tour.maximumCrossReceiverLoad;
details.proposalCrossCount = tour.crossFormationMessageCount;
details.formationPhysicalAdjacency = formationPhysical;
details.formationPairSupport = support;
details.selectedFormationPathIndices = selectedPath;
details.selectedFormationPathGroups = groups(selectedPath);
details.selectedPathSupport = pathSupport;
details.selectedPathMinimumSupport = min(pathSupport);
details.selectedPathTotalSupport = sum(pathSupport);
details.selectedTreeRegistration = selectedRegistration;
details.treeSelectionMethod = ...
    'max-bottleneck-then-total-support-hamiltonian-path';
details.repairTriggered = false;
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'fixed-index-plus-current-physical-formation-path-tree';
details.instantaneousSensorStrongConnected = true;
details.instantaneousFormationStrongConnected = true;
details.sensorWindowMature = false;
details.sensorWindowStrongConnected = NaN;
details.formationWindowMature = false;
details.formationWindowStrongConnected = NaN;
details.oneStepTopologyReserveChecked = false;
details.oneStepTopologyReservePassed = NaN;
details.oneStepJointProjectionUsed = false;
details.recursiveSafetyClaimed = false;
details.topologyInfeasible = false;
end

function [adjacency, support] = collapsePhysical(physical, groupIds, groups)
formationCount = numel(groups);
adjacency = false(formationCount);
support = zeros(formationCount);
for leftIdx = 1:formationCount-1
    left = groupIds == groups(leftIdx);
    for rightIdx = leftIdx+1:formationCount
        right = groupIds == groups(rightIdx);
        forward = nnz(physical(left, right));
        reverse = nnz(physical(right, left));
        support(leftIdx, rightIdx) = min(forward, reverse);
        support(rightIdx, leftIdx) = support(leftIdx, rightIdx);
        adjacency(leftIdx, rightIdx) = support(leftIdx, rightIdx) > 0;
        adjacency(rightIdx, leftIdx) = adjacency(leftIdx, rightIdx);
    end
end
end

function candidates = enumerateCandidatePaths(adjacency, support)
formationCount = size(adjacency, 1);
allPaths = perms(1:formationCount);
rows = zeros(0, formationCount + 2);
for pathIdx = 1:size(allPaths, 1)
    path = allPaths(pathIdx, :);
    if path(1) > path(end)
        continue;
    end
    indices = sub2ind(size(adjacency), ...
        path(1:end-1), path(2:end));
    if ~all(adjacency(indices))
        continue;
    end
    edgeSupport = support(indices);
    rows(end + 1, :) = [ ...
        -min(edgeSupport), -sum(edgeSupport), path]; %#ok<AGROW>
end
if isempty(rows)
    candidates = zeros(0, formationCount + 2);
else
    candidates = sortrows(rows, 1:size(rows, 2));
end
end

function registration = registerFormationPath(path, groupIds, groups)
nodeCount = numel(groupIds);
registration = false(nodeCount);
for pathIdx = 1:numel(path)-1
    leftMembers = find(groupIds == groups(path(pathIdx)));
    rightMembers = find(groupIds == groups(path(pathIdx + 1)));
    left = leftMembers(1);
    right = rightMembers(1);
    registration(left, right) = true;
    registration(right, left) = true;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
