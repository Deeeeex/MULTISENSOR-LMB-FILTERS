function [adjacency, details] = ...
        selectCausalMinimumFormationBackboneV242Policy(context)
% SELECTCAUSALMINIMUMFORMATIONBACKBONEV242POLICY Remove local redundancy.
%
% A V240 local directed cycle remains inside every formation.  For every
% edge of the selected formation tree, its residual gateway is retained in
% each direction.  Local residual-cycle inputs are omitted and their KLA
% mass is returned to receiver self weight.

protocol = getCausalMinimumFormationBackboneV242Protocol();
nodeCount = numel(context.localPosteriorBySensor);
referenceContext = context;
referenceContext.directedMessageBudget = 2 * nodeCount;
[referenceAdjacency, reference] = ...
    selectCausalMinimalEditFormationTreeV240Policy(referenceContext);
formationUids = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
formationCount = numel(unique(formationUids));
crossMask = formationUids(:) ~= formationUids(:)';
dominant = logical(reference.dominantAdjacency);
residual = logical(reference.residualAdjacency);
crossResidual = residual & crossMask;
localResidual = residual & ~crossMask;
adjacency = dominant | crossResidual;

weights = reference.fusionWeightMatrix;
for receiver = 1:nodeCount
    omitted = find(localResidual(receiver, :));
    returnedMass = sum(weights(receiver, omitted));
    weights(receiver, omitted) = 0;
    weights(receiver, receiver) = ...
        weights(receiver, receiver) + returnedMass;
end

expectedMessages = nodeCount + 2 * (formationCount - 1);
positiveAllowed = adjacency | logical(eye(nodeCount));
inputsPerReceiver = reshape(sum(adjacency, 2), 1, []);
structuralPassed = ...
    all(sum(dominant, 2) == 1) && ...
    nnz(crossResidual) == 2 * (formationCount - 1) && ...
    nnz(adjacency) == expectedMessages && ...
    all(inputsPerReceiver >= 1) && ...
    all(inputsPerReceiver <= 2) && ...
    ~any(adjacency(:) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(adjacency) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= 0) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~structuralPassed
    error('CausalMinimumFormationBackboneV242:InvalidRoute', ...
        'The sparse formation backbone violates its hard invariant.');
end
if isfield(context, 'directedMessageBudget') && ...
        nnz(adjacency) > context.directedMessageBudget
    error('CausalMinimumFormationBackboneV242:BudgetExceeded', ...
        'The sparse formation backbone exceeds its directed budget.');
end

details = reference;
details.contractVersion = ...
    'causal-minimum-formation-backbone-v242-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'causal-minimum-local-cycle-bidirectional-tree';
details.backboneMode = details.mode;
details.referenceAdjacency = logical(referenceAdjacency);
details.referenceFusionWeights = reference.fusionWeightMatrix;
details.dominantAdjacency = dominant;
details.residualAdjacency = residual;
details.crossResidualAdjacency = crossResidual;
details.localResidualAdjacency = localResidual;
details.selectedResidualAdjacency = crossResidual;
details.omittedResidualAdjacency = localResidual;
details.fusionWeightMatrix = weights;
details.formationCount = formationCount;
details.referenceMessageCount = nnz(referenceAdjacency);
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.architectureMinimumPassed = ...
    nnz(adjacency) == expectedMessages;
details.messageSavingCount = ...
    nnz(referenceAdjacency) - nnz(adjacency);
details.messageSavingFraction = ...
    details.messageSavingCount / nnz(referenceAdjacency);
details.minimumInputsPerReceiver = min(inputsPerReceiver);
details.maximumInputsPerReceiver = max(inputsPerReceiver);
details.localCycleMessageCount = nnz(dominant);
details.crossTreeMessageCount = nnz(crossResidual);
details.localResidualMessageCount = nnz(localResidual);
details.omittedResidualMass = sum( ...
    reference.fusionWeightMatrix(localResidual));
details.instantaneousSensorStrong = true;
details.instantaneousFormationStrong = true;
details.localResidualMassReturnedToSelf = true;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
details.referenceFallbackUsed = false;
details.scheduleCertificate = buildSchedule( ...
    context.currentTime, reference.scheduleCertificate, details);
end

function schedule = buildSchedule(currentTime, source, details)
schedule = source;
schedule.contractVersion = ...
    'causal-minimum-formation-backbone-v242-schedule-v1';
schedule.currentTime = currentTime;
schedule.phase = ['minimum-backbone-', source.phase];
schedule.messageParityPassed = ...
    details.currentMessageCount == ...
        details.minimumArchitectureMessageCount;
schedule.rollingSensorStrong = true;
schedule.rollingFormationStrong = true;
schedule.minimumArchitectureMessageCount = ...
    details.minimumArchitectureMessageCount;
schedule.currentMessageCount = details.currentMessageCount;
schedule.referenceMessageCount = details.referenceMessageCount;
schedule.messageSavingFraction = details.messageSavingFraction;
schedule.localCycleMessageCount = details.localCycleMessageCount;
schedule.crossTreeMessageCount = details.crossTreeMessageCount;
schedule.localResidualMassReturnedToSelf = true;
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
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end
