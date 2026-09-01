function registration = ...
        buildInitialFormationBraidRouteV241Registration(inputs)
% BUILDINITIALFORMATIONBRAIDROUTEV241REGISTRATION Select V240 once at t=1.

protocol = getFormationBraidRoutingComparisonV241Protocol();
if ~isstruct(inputs) || ~isscalar(inputs) || ...
        ~all(isfield(inputs, {'config', 'model', 'graphData', ...
            'commConfig', 'seed'})) || ...
        ~ismember(inputs.config.presetName, protocol.allowedPresets) || ...
        ~ismember(inputs.seed, protocol.allowedSeeds)
    error('FormationBraidRoutingV241:InvalidRegistrationInput', ...
        'V241 requires one registered formation-braid scene and seed.');
end
nodeCount = inputs.config.numberOfSensors;
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
context.directedMessageBudget = 2 * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.currentTime = 1;
context.positions = inputs.graphData.positions(:, :, 1);
context.physicalAdjacency = logical( ...
    inputs.graphData.physicalAdjacency(:, :, 1));
context.commConfig = struct('pDropByEdge', ...
    inputs.commConfig.pDropByEdge(:, :, 1));
context.previousAdjacencyHistory = false(nodeCount, nodeCount, 0);
[adjacency, details] = ...
    selectCausalMinimalEditFormationTreeV240Policy(context);
if nnz(adjacency) ~= 2 * nodeCount
    error('FormationBraidRoutingV241:InitialRouteContract', ...
        'The initial V241 route violates the two-input budget.');
end
registration = struct();
registration.contractVersion = ...
    'formation-braid-v241-initial-route-registration-v1';
registration.adjacency = logical(adjacency);
registration.fusionWeightMatrix = details.fusionWeightMatrix;
registration.selectionDetails = details;
registration.messageCount = nnz(adjacency);
registration.selectionTime = 1;
registration.targetTruthUsed = false;
registration.futurePhysicalPageUsed = false;
end
