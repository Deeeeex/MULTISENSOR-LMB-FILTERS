function [adjacency, details] = selectMaturedHandoffV103Policy( ...
        context, startTime, bank, actionEnabled)
% SELECTMATUREDHANDOFFV103POLICY Execute the frozen matured handoff route.

protocol = getMaturedHandoffV103Protocol();
routeOptions = getFormationIsolateReconnectProbeProtocol();
[referenceAdjacency, details] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', routeOptions.dominantWeight, ...
            'residualWeight', routeOptions.residualWeight));
timeIdx = context.currentTime - startTime + 1;
if timeIdx < 1 || timeIdx > protocol.horizonSteps || ...
        ~isfield(bank, 'v103Route') || ...
        ~isequal(logical(referenceAdjacency), ...
            logical(bank.v103Route.referenceAdjacency))
    error('MaturedHandoffV103:RuntimeBoundaryDrift', ...
        'The frozen V103 route no longer matches the runtime reference.');
end
if actionEnabled
    adjacency = logical(bank.v103Route.adjacencyByTime(:, :, timeIdx));
    weights = bank.v103Route.fusionWeightsByTime(:, :, timeIdx);
else
    adjacency = logical(referenceAdjacency);
    weights = details.fusionWeightMatrix;
end
if any(adjacency(:) & ~logical(context.physicalAdjacency(:)))
    error('MaturedHandoffV103:RuntimePhysicalityDrift', ...
        'A frozen handoff edge is not physically reachable at runtime.');
end
details.fusionWeightMatrix = weights;
details.contractVersion = 'matured-handoff-v103-policy-v1';
details.armId = protocol.outcomePolicyName;
details.protocolId = protocol.id;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = false;
details.v103ActionEnabled = logical(actionEnabled);
details.v103TimeIndex = timeIdx;
details.v103HandoffFormationIds = ...
    protocol.handoffFormationIdsByTime{timeIdx};
details.v103ChangedReceiverIndices = ...
    bank.v103Route.changedReceiverIndicesByTime{timeIdx};
end
