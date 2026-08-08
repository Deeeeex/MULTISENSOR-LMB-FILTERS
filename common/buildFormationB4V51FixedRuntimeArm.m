function [adjacency, details] = ...
    buildFormationB4V51FixedRuntimeArm(context, armId)
% BUILDFORMATIONB4V51FIXEDRUNTIMEARM V46 B4 pulse plus retention gate.

timerId = tic;
protocol = getFormationB4V51RuntimeProtocol();
if nargin ~= 2 || ~strcmp(armId, protocol.candidateArmId)
    error('FormationB4V51Runtime:InvalidArm', ...
        'Only the V51 retention-gated B4 arm is accepted.');
end

routeContext = buildFormationB4V49GraphOnlyRouteContext(context);
parentContext = buildParentContext( ...
    context, routeContext, numel(context.localPosteriorBySensor));
[adjacency, parent] = buildFormationB4V46FixedRuntimeArm( ...
    parentContext, protocol.parentRouteArmId);
parentAdjacency = logical(adjacency);
phase = mod(context.currentTime - 1, protocol.period) + 1;
gateDetails = struct([]);
if phase == 1
    [residual, gateDetails] = ...
        selectFormationB4V51RetentionGatedResidual( ...
            context, parent.dominantAdjacency, ...
            parent.residualAdjacency, protocol);
    adjacency = logical(parent.dominantAdjacency) | residual;
    weights = zeros(size(adjacency));
    weights(parent.dominantAdjacency) = protocol.dominantWeight;
    weights(residual) = protocol.activeResidualWeight;
    weights(1:size(weights, 1)+1:end) = 1 - sum(weights, 2)';
else
    residual = parent.residualAdjacency;
    weights = parent.fusionWeightMatrix;
end

if any(adjacency(:) & ~logical(context.physicalAdjacency(:))) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12) || ...
        any(weights(:) < -1e-12) || ...
        nnz(adjacency) > nnz(parentAdjacency)
    error('FormationB4V51Runtime:InvalidAction', ...
        'The V51 page violates the physical, weight or budget contract.');
end

schedule = struct();
schedule.contractVersion = ...
    'formation-b4-v51-runtime-schedule-v1';
schedule.currentTime = context.currentTime;
schedule.currentAbsolutePhase = phase;
schedule.period = protocol.period;
schedule.cycleSelectionPerformed = false;
schedule.cycleSelected = false;
schedule.referenceFallbackUsed = false;
schedule.fallbackReason = 'v50-rejected-use-v46-route';
schedule.selectedPosteriorObjective = 0;
schedule.retentionGateDetails = gateDetails;
schedule.currentScheduledDirectedEdgeCount = nnz(adjacency);
schedule.sameOrLowerScheduledMessageCountVsV46 = true;
schedule.retentionGateApplied = phase == 1;
schedule.posteriorUsedForRoutingOnly = phase == 1;
schedule.truthUsed = false;
schedule.futureOutcomeUsed = false;
schedule.deferredCrossEdgeCount = 0;
schedule.selectedDeferredFormationIds = zeros(1, 0);
if phase == 1
    schedule.deferredCrossEdgeCount = ...
        gateDetails.deferredCrossEdgeCount;
    schedule.selectedDeferredFormationIds = ...
        gateDetails.selectedDeferredFormationIds;
end

details = parent;
% V51 changes the parent route and weights after the V46 policy returns.
% Its development runner validates the installed adjacency and row-stochastic
% weights directly, so do not carry the now-stale V46 fingerprint forward.
details.routeAndWeightCanonicalSha256 = '';
details.contractVersion = ...
    'formation-b4-v51-fixed-runtime-policy-v1';
details.mode = 'formation-b4-v51-retention-gated-reference-runtime';
details.armId = protocol.candidateArmId;
details.actionName = protocol.candidateArmId;
details.protocolId = protocol.id;
details.fusionWeightMatrix = weights;
details.residualAdjacency = residual;
details.referenceAdjacency = logical(parent.dominantAdjacency) | residual;
details.scheduleCertificate = schedule;
formationUids = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
crossMask = formationUids(:) ~= formationUids(:)';
details.crossResidualCount = nnz(residual & crossMask);
details.localResidualCount = nnz(residual & ~crossMask);
details.currentScheduledDirectedEdgeCount = nnz(adjacency);
details.currentMessageCount = nnz(adjacency);
details.referenceMessageCount = 2 * size(adjacency, 1);
details.referenceScheduledDirectedEdgeCount = ...
    details.referenceMessageCount;
details.scheduledDirectedEdgeSavingFraction = ...
    (details.referenceMessageCount - nnz(adjacency)) / ...
    details.referenceMessageCount;
details.messageSavingFraction = ...
    (details.referenceMessageCount - nnz(adjacency)) / ...
    details.referenceMessageCount;
details.sameScheduledDirectedEdgeCountVsV46 = ...
    nnz(adjacency) == details.referenceMessageCount;
details.sameOrLowerScheduledMessageCountVsV46 = true;
details.retentionGateDetails = gateDetails;
details.selectionSeconds = toc(timerId);
end

function contextOut = buildParentContext( ...
        context, routeContext, nodeCount)
contextOut = struct();
contextOut.localPosteriorBySensor = cell(1, nodeCount);
contextOut.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', routeContext.sensorGroupIds)));
contextOut.baseAdjacency = logical(context.baseAdjacency);
contextOut.physicalAdjacency = logical(routeContext.physicalAdjacency);
contextOut.positions = routeContext.positions;
contextOut.sensorPhysicalUids = routeContext.sensorPhysicalUids;
contextOut.formationPhysicalUidsBySensor = ...
    routeContext.formationPhysicalUidsBySensor;
contextOut.physicalIdentityRegistryCanonicalSha256 = ...
    context.physicalIdentityRegistryCanonicalSha256;
contextOut.commConfig = struct('pDropByEdge', routeContext.pDropByEdge);
contextOut.currentTime = routeContext.currentTime;
contextOut.directedMessageBudget = 2 * nodeCount;
contextOut.triggerConfig = struct( ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount);
contextOut.observableInputContract = context.observableInputContract;
end
