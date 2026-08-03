function [adjacency, details] = ...
    selectFormationBackboneResidualTourPolicy(context, options)
% SELECTFORMATIONBACKBONERESIDUALTOURPOLICY Registered-graph reference.
%
% The high-weight, within-formation fixed-index route is retained.  The
% low-weight input is one deterministic Hamiltonian tour whose formation
% transitions cover every edge in the registered formation quotient once
% in each direction.

if nargin < 2 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
if ~isscalar(dominantWeight) || ~isscalar(residualWeight) || ...
        any(~isfinite([dominantWeight, residualWeight])) || ...
        dominantWeight <= 0 || residualWeight <= 0 || ...
        dominantWeight + residualWeight >= 1
    error('FormationBackboneTourPolicy:InvalidWeights', ...
        'Formation-backbone tour fusion weights are invalid.');
end
nodeCount = numel(context.localPosteriorBySensor);
if ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds') || ...
        ~isfield(context, 'baseAdjacency')
    error('FormationBackboneTourPolicy:InvalidContext', ...
        'The registered scene graph is unavailable.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
registeredAdjacency = logical(context.baseAdjacency);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(registeredAdjacency), [nodeCount, nodeCount])
    error('FormationBackboneTourPolicy:InvalidContext', ...
        'Registered scene graph dimensions are invalid.');
end
if isfield(context.model.dynamicTopologyScenario, 'staticAdjacency') && ...
        ~isequal(logical(context.model.dynamicTopologyScenario. ...
            staticAdjacency), registeredAdjacency)
    error('FormationBackboneTourPolicy:InvalidContext', ...
        'The observable base graph disagrees with scene static metadata.');
end

[dominantAdjacency, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
[residualAdjacency, tour] = ...
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registeredAdjacency, context.physicalAdjacency);
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
    error('FormationBackboneTourPolicy:InvalidReference', [ ...
        'The backbone tour must provide two distinct inputs per receiver ', ...
        'within the registered directed-message budget.']);
end

details = route;
details.contractVersion = ...
    'formation-backbone-residual-tour-policy-v1';
details.mode = 'formation-backbone-residual-tour';
details.objective = NaN;
details.candidateIndex = NaN;
details.selectionSeconds = NaN;
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = tour.crossFormationMessageCount;
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
details.crossFormationMessageCount = ...
    tour.crossFormationMessageCount;
details.maximumCrossEdges = tour.crossFormationMessageCount;
details.maximumCrossSourceLoad = maximumLoad( ...
    tour.crossSenders, nodeCount);
details.maximumCrossReceiverLoad = maximumLoad( ...
    tour.crossReceivers, nodeCount);
details.proposalCrossCount = tour.crossFormationMessageCount;
details.repairTriggered = false;
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'fixed-index-plus-registered-formation-backbone-residual-tour';
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

function value = maximumLoad(indices, nodeCount)
if isempty(indices)
    value = 0;
else
    value = max(accumarray( ...
        reshape(indices, [], 1), 1, [nodeCount, 1]));
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
