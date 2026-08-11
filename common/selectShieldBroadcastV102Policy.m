function [adjacency, details] = selectShieldBroadcastV102Policy( ...
        context, startTime, bank, actionEnabled)
% SELECTSHIELDBROADCASTV102POLICY Execute the frozen shield/broadcast route.

if isfield(bank, 'bankVariant') && strcmp( ...
        bank.bankVariant, 'alternating-shield-broadcast-v111')
    protocol = getAlternatingShieldBroadcastV111Protocol();
else
    protocol = getShieldBroadcastV102Protocol();
end
routeOptions = getFormationIsolateReconnectProbeProtocol();
[referenceAdjacency, details] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', routeOptions.dominantWeight, ...
            'residualWeight', routeOptions.residualWeight));
timeIdx = context.currentTime - startTime + 1;
if timeIdx < 1 || timeIdx > protocol.horizonSteps || ...
        ~isfield(bank, 'v102Route') || ...
        ~isequal(logical(referenceAdjacency), ...
            logical(bank.v102Route.referenceAdjacency))
    error('ShieldBroadcastV102:RuntimeBoundaryDrift', ...
        'The frozen V102 route no longer matches the runtime reference.');
end
if actionEnabled
    adjacency = logical(bank.v102Route.adjacencyByTime(:, :, timeIdx));
    weights = bank.v102Route.fusionWeightsByTime(:, :, timeIdx);
else
    adjacency = logical(referenceAdjacency);
    weights = details.fusionWeightMatrix;
end
if any(adjacency(:) & ~logical(context.physicalAdjacency(:)))
    error('ShieldBroadcastV102:RuntimePhysicalityDrift', ...
        'A frozen broadcast edge is not physically reachable at runtime.');
end
details.fusionWeightMatrix = weights;
details.contractVersion = 'shield-broadcast-v102-policy-v1';
details.armId = protocol.outcomePolicyName;
details.protocolId = protocol.id;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = false;
details.v102ActionEnabled = logical(actionEnabled);
details.v102TimeIndex = timeIdx;
details.v102BroadcastFormationIds = ...
    protocol.broadcastFormationIdsByTime{timeIdx};
details.v102ChangedReceiverIndices = ...
    bank.v102Route.changedReceiverIndicesByTime{timeIdx};
end
