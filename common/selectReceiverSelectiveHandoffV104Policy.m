function [adjacency, details] = selectReceiverSelectiveHandoffV104Policy( ...
        context, startTime, bank, actionEnabled)
% SELECTRECEIVERSELECTIVEHANDOFFV104POLICY Execute frozen receiver oracle.

protocol = getReceiverSelectiveHandoffV104Protocol();
routeOptions = getFormationIsolateReconnectProbeProtocol();
[referenceAdjacency, details] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', routeOptions.dominantWeight, ...
            'residualWeight', routeOptions.residualWeight));
timeIdx = context.currentTime - startTime + 1;
if timeIdx < 1 || timeIdx > protocol.horizonSteps || ...
        ~isfield(bank, 'v104Route') || ...
        ~isequal(logical(referenceAdjacency), ...
            logical(bank.v104Route.referenceAdjacency))
    error('ReceiverSelectiveV104:RuntimeBoundaryDrift', ...
        'The frozen V104 route no longer matches the runtime reference.');
end
if actionEnabled
    adjacency = logical(bank.v104Route.adjacencyByTime(:, :, timeIdx));
    weights = bank.v104Route.fusionWeightsByTime(:, :, timeIdx);
else
    adjacency = logical(referenceAdjacency);
    weights = details.fusionWeightMatrix;
end
if any(adjacency(:) & ~logical(context.physicalAdjacency(:)))
    error('ReceiverSelectiveV104:RuntimePhysicalityDrift', ...
        'A frozen receiver-selective edge is not physically reachable.');
end
details.fusionWeightMatrix = weights;
details.contractVersion = 'receiver-selective-v104-policy-v1';
details.armId = protocol.outcomePolicyName;
details.protocolId = protocol.id;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = false;
details.v104ActionEnabled = logical(actionEnabled);
details.v104TimeIndex = timeIdx;
details.v104SelectedReceiverIds = ...
    protocol.selectedReceiverIdsByTime{timeIdx};
end
