function [adjacency, details] = ...
    buildFormationB4V53FixedRuntimeArm(context, armId)
% BUILDFORMATIONB4V53FIXEDRUNTIMEARM Exact selective cross-input control.

timerId = tic;
protocol = getFormationB4V53RuntimeProtocol();
if nargin ~= 2 || ~strcmp(armId, protocol.candidateArmId)
    error('FormationB4V53Runtime:InvalidArm', ...
        'Only the V53 exact selective cross-pulse arm is accepted.');
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
        selectFormationB4V53ExactSelectiveResidual( ...
            context, parent.dominantAdjacency, ...
            parent.residualAdjacency, protocol);
    adjacency = logical(parent.dominantAdjacency) | residual;
    weights = buildWeights( ...
        parent.dominantAdjacency, residual, protocol);
else
    residual = parent.residualAdjacency;
    weights = parent.fusionWeightMatrix;
end

if any(adjacency(:) & ~logical(context.physicalAdjacency(:))) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12) || ...
        any(weights(:) < -1e-12) || ...
        nnz(adjacency) > nnz(parentAdjacency)
    error('FormationB4V53Runtime:InvalidAction', ...
        'The V53 page violates the physical, weight or budget rule.');
end

schedule = struct();
schedule.contractVersion = ...
    'formation-b4-v53-runtime-schedule-v1';
schedule.currentTime = context.currentTime;
schedule.currentAbsolutePhase = phase;
schedule.period = protocol.period;
schedule.exactSelectiveGateApplied = phase == 1;
schedule.exactSelectiveGateDetails = gateDetails;
schedule.currentScheduledDirectedEdgeCount = nnz(adjacency);
schedule.referenceScheduledDirectedEdgeCount = nnz(parentAdjacency);
schedule.referenceFallbackUsed = false;
schedule.fallbackReason = '';
schedule.deferredCrossEdgeCount = 0;
schedule.selectedDeferredFormationIds = zeros(1, 0);
schedule.currentPosteriorUsed = phase == 1;
schedule.truthUsed = false;
schedule.futureOutcomeUsed = false;
if phase == 1
    schedule.deferredCrossEdgeCount = ...
        gateDetails.deferredCrossEdgeCount;
    schedule.selectedDeferredFormationIds = ...
        gateDetails.selectedDeferredFormationIds;
    if ~gateDetails.previousPulseKnown
        schedule.referenceFallbackUsed = true;
        schedule.fallbackReason = 'missing-previous-pulse-history';
    end
end

details = parent;
% V53 changes the installed route and weights after the V46 policy returns.
% The development filter checks those installed objects directly.
details.routeAndWeightCanonicalSha256 = '';
details.contractVersion = ...
    'formation-b4-v53-fixed-runtime-policy-v1';
details.mode = ...
    'formation-b4-v53-exact-selective-cross-pulse-runtime';
details.armId = protocol.candidateArmId;
details.actionName = protocol.candidateArmId;
details.protocolId = protocol.id;
details.fusionWeightMatrix = weights;
details.residualAdjacency = residual;
details.referenceAdjacency = parent.referenceAdjacency;
details.scheduleCertificate = schedule;
formationUids = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
crossMask = formationUids(:) ~= formationUids(:)';
details.crossResidualCount = nnz(residual & crossMask);
details.localResidualCount = nnz(residual & ~crossMask);
details.currentScheduledDirectedEdgeCount = nnz(adjacency);
details.currentMessageCount = nnz(adjacency);
details.referenceMessageCount = nnz(parentAdjacency);
details.referenceScheduledDirectedEdgeCount = nnz(parentAdjacency);
if nnz(parentAdjacency) > 0
    details.scheduledDirectedEdgeSavingFraction = ...
        (nnz(parentAdjacency) - nnz(adjacency)) / ...
            nnz(parentAdjacency);
else
    details.scheduledDirectedEdgeSavingFraction = 0;
end
details.messageSavingFraction = ...
    details.scheduledDirectedEdgeSavingFraction;
details.sameScheduledDirectedEdgeCountVsV46 = ...
    nnz(adjacency) == nnz(parentAdjacency);
details.sameOrLowerScheduledMessageCountVsV46 = true;
details.exactSelectiveGateDetails = gateDetails;
details.selectionSeconds = toc(timerId);
end

function weights = buildWeights(dominant, residual, protocol)
nodeCount = size(dominant, 1);
weights = zeros(nodeCount);
weights(dominant) = protocol.dominantWeight;
weights(residual) = protocol.activeResidualWeight;
weights(1:nodeCount+1:end) = 1 - sum(weights, 2)';
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
