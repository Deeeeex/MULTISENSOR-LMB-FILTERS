function graph = buildObservableFormationGraphSnapshotV208( ...
        localPosteriorBySensor, groupIds, physicalAdjacency, ...
        selectedAdjacency, previousAdjacency, positions, ...
        commConfig, runtimeModel, currentTime, referencePayloadBytes)
% BUILDOBSERVABLEFORMATIONGRAPHSNAPSHOTV208 Reuse V56 graph representation.
%
% V56 couples observable node/edge features to a routing-mode bank.  V208
% needs only the graph representation, so a synthetic reference-only bank is
% used internally and its mode features are discarded.

nodeCount = numel(localPosteriorBySensor);
groupIds = reshape(groupIds, 1, []);
formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
if ~iscell(localPosteriorBySensor) || nodeCount < 1 || ...
        numel(groupIds) ~= nodeCount || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(selectedAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(previousAdjacency), [nodeCount, nodeCount]) || ...
        size(positions, 2) ~= nodeCount || ...
        ~isstruct(runtimeModel) || ...
        isfield(runtimeModel, 'explicitTargetTrajectories') || ...
        (isfield(runtimeModel, 'dynamicTopologyScenario') && ...
         isfield(runtimeModel.dynamicTopologyScenario, ...
            'targetTrajectories')) || ...
        ~isscalar(referencePayloadBytes) || ...
        ~isfinite(referencePayloadBytes) || referencePayloadBytes < 0
    error('FormationGraphSnapshotV208:InvalidInput', ...
        'A sanitized current observable graph state is required.');
end

commConfig = sanitizeCommunicationConfig(commConfig, currentTime);
context = struct();
context.localPosteriorBySensor = localPosteriorBySensor;
context.model = runtimeModel;
context.commConfig = commConfig;
context.currentTime = currentTime;
context.previousAdjacency = logical(previousAdjacency);
context.physicalAdjacency = logical(physicalAdjacency);
context.positions = positions;

referenceAdjacency = logical(selectedAdjacency);
referenceWeights = referenceFusionWeights(referenceAdjacency);
actionCount = 1 + formationCount;
bank = struct();
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = 2;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.modeTrustWeights = [1, 0.5];
bank.actionFormationIndex = [0; (1:formationCount)'];
bank.actionModeIndex = [1; 2 * ones(formationCount, 1)];
bank.actionAdjacency = repmat( ...
    referenceAdjacency, 1, 1, actionCount);
bank.actionFusionWeights = repmat( ...
    referenceWeights, 1, 1, actionCount);
bank.actionDominantSources = repmat( ...
    1:nodeCount, actionCount, 1);
bank.actionPosteriorProxyAllowed = false(actionCount, 1);
bank.actionPosteriorObjective = zeros(actionCount, 1);
bank.actionPayloadBytes = ...
    referencePayloadBytes * ones(actionCount, 1);
bank.referencePayloadBytes = referencePayloadBytes;
bank.modeNames = {'reference', 'unused'};
bank.modeKindNames = {'reference', 'other'};
bank.modeKindIndex = [1, 2];
bank.truthUsed = false;
bank.futureOutcomeUsed = false;

source = computeTrackingAlignedFormationGraphFeatures( ...
    context, bank, groupIds);
graph = struct();
graph.contractVersion = ...
    'observable-formation-graph-snapshot-v208-v1';
graph.nodeFeatures = source.nodeFeatures;
graph.nodeFeatureNames = source.nodeFeatureNames;
graph.edgeFeatures = source.edgeFeatures;
graph.edgeFeatureNames = source.edgeFeatureNames;
graph.edgeAvailableMask = source.edgeAvailableMask;
graph.formationCount = source.formationCount;
graph.nodeCount = source.nodeCount;
graph.targetCapacity = source.targetCapacity;
graph.formationIds = formationIds;
graph.syntheticModeBankUsedInternally = true;
graph.syntheticModeFeaturesDiscarded = true;
graph.sensorPermutationInvariant = ...
    source.sensorPermutationInvariant;
graph.formationLabelPermutationEquivariant = ...
    source.formationLabelPermutationEquivariant;
graph.scaleNormalized = source.scaleNormalized;
graph.truthUsed = false;
graph.futureInformationUsed = false;
graph.numericFormationIdentifiersUsedAsFeatures = false;
end

function config = sanitizeCommunicationConfig(config, currentTime)
if ~isstruct(config)
    error('FormationGraphSnapshotV208:InvalidCommunication', ...
        'Communication configuration must be a structure.');
end
if isfield(config, 'linkUniforms')
    config = rmfield(config, 'linkUniforms');
end
if isfield(config, 'pDropByEdge') && ...
        ndims(config.pDropByEdge) >= 3
    timeIdx = min(currentTime, size(config.pDropByEdge, 3));
    config.pDropByEdge = config.pDropByEdge(:, :, timeIdx);
end
end

function weights = referenceFusionWeights(adjacency)
nodeCount = size(adjacency, 1);
weights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    sources = unique([receiverIdx, find(adjacency(receiverIdx, :))]);
    weights(receiverIdx, sources) = 1 / numel(sources);
end
end
