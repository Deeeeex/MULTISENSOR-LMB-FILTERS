function bank = buildFormationModeInterventionBank( ...
        projection, groupIds)
% BUILDFORMATIONMODEINTERVENTIONBANK Reference plus local mode changes.
%
% The joint source-trust projector evaluates one full-network routing
% realization for each trust mode.  This helper forms an intervention bank
% in which exactly one formation changes from the registered reference at a
% time.  The bank is intended for offline counterfactual return generation:
% it exposes whether a formation-local action has multi-step value before a
% learned additive or pairwise value model is fitted.

required = { ...
    'formationModeCount', 'modeDominantSourcesByIndex', ...
    'modeAdjacencyByIndex', 'modeFusionWeightsByIndex', ...
    'dynamicModeAllowedMask', 'payloadByFormationMode', ...
    'objectiveByFormationMode', 'projector'};
if ~isstruct(projection) || ~all(isfield(projection, required))
    error('Formation-mode intervention projection is incomplete.');
end
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
modeCount = round(projection.formationModeCount);
sourcesByMode = projection.modeDominantSourcesByIndex;
adjacencyByMode = projection.modeAdjacencyByIndex;
weightsByMode = projection.modeFusionWeightsByIndex;
allowedModeMask = logical(projection.dynamicModeAllowedMask);
payloadByFormationMode = projection.payloadByFormationMode;
objectiveByFormationMode = projection.objectiveByFormationMode;
modeTrustWeights = reshape( ...
    projection.projector.modeTrustWeights, 1, []);
if nodeCount < 1 || formationCount < 1 || modeCount < 2 || ...
        numel(sourcesByMode) ~= modeCount || ...
        numel(adjacencyByMode) ~= modeCount || ...
        numel(weightsByMode) ~= modeCount || ...
        ~isequal(size(allowedModeMask), ...
            [formationCount, modeCount]) || ...
        ~isequal(size(payloadByFormationMode), ...
            [formationCount, modeCount]) || ...
        ~isequal(size(objectiveByFormationMode), ...
            [formationCount, modeCount]) || ...
        numel(modeTrustWeights) ~= modeCount || ...
        any(~isfinite(modeTrustWeights)) || ...
        any(~allowedModeMask(:, 1))
    error('Formation-mode intervention dimensions are invalid.');
end
for modeIdx = 1:modeCount
    sources = reshape(sourcesByMode{modeIdx}, 1, []);
    adjacency = adjacencyByMode{modeIdx};
    weights = weightsByMode{modeIdx};
    weightSupport = logical(adjacency) | eye(nodeCount) > 0;
    if numel(sources) ~= nodeCount || ...
            ~isequal(size(adjacency), [nodeCount, nodeCount]) || ...
            ~isequal(size(weights), [nodeCount, nodeCount]) || ...
            any(~isfinite(sources)) || ...
            any(sources ~= round(sources)) || ...
            any(sources < 1) || any(sources > nodeCount) || ...
            any(~isfinite(weights(:))) || any(weights(:) < 0) || ...
            any(weights(~weightSupport) > 1e-12)
        error('Formation-mode intervention realization is invalid.');
    end
end

actionCount = 1 + formationCount * (modeCount - 1);
actionModes = ones(actionCount, formationCount);
actionFormationIndex = zeros(actionCount, 1);
actionModeIndex = ones(actionCount, 1);
actionAdjacency = false(nodeCount, nodeCount, actionCount);
actionFusionWeights = zeros(nodeCount, nodeCount, actionCount);
actionDominantSources = zeros(actionCount, nodeCount);
actionNames = cell(1, actionCount);
actionPosteriorProxyAllowed = true(actionCount, 1);
actionPosteriorObjective = zeros(actionCount, 1);
actionPayloadBytes = zeros(actionCount, 1);

referenceAdjacency = logical(adjacencyByMode{1});
referenceWeights = weightsByMode{1};
referenceSources = reshape(sourcesByMode{1}, 1, []);
referencePayloadBytes = sum(payloadByFormationMode(:, 1));
actionAdjacency(:, :, 1) = referenceAdjacency;
actionFusionWeights(:, :, 1) = referenceWeights;
actionDominantSources(1, :) = referenceSources;
actionNames{1} = 'reference';
actionPayloadBytes(1) = referencePayloadBytes;

actionIdx = 1;
for formationIdx = 1:formationCount
    members = reshape(find( ...
        groupIds == groups(formationIdx)), 1, []);
    for modeIdx = 2:modeCount
        actionIdx = actionIdx + 1;
        adjacency = referenceAdjacency;
        weights = referenceWeights;
        sources = referenceSources;
        adjacency(members, :) = ...
            logical(adjacencyByMode{modeIdx}(members, :));
        weights(members, :) = ...
            weightsByMode{modeIdx}(members, :);
        modeSources = reshape(sourcesByMode{modeIdx}, 1, []);
        sources(members) = modeSources(members);
        actionModes(actionIdx, formationIdx) = modeIdx;
        actionFormationIndex(actionIdx) = formationIdx;
        actionModeIndex(actionIdx) = modeIdx;
        actionAdjacency(:, :, actionIdx) = adjacency;
        actionFusionWeights(:, :, actionIdx) = weights;
        actionDominantSources(actionIdx, :) = sources;
        actionNames{actionIdx} = sprintf( ...
            'formation-%d-dynamic-trust-%.2f', ...
            formationIdx, modeTrustWeights(modeIdx));
        actionPosteriorProxyAllowed(actionIdx) = ...
            allowedModeMask(formationIdx, modeIdx);
        actionPosteriorObjective(actionIdx) = ...
            objectiveByFormationMode(formationIdx, modeIdx);
        actionPayloadBytes(actionIdx) = ...
            referencePayloadBytes - ...
                payloadByFormationMode(formationIdx, 1) + ...
                payloadByFormationMode(formationIdx, modeIdx);
    end
end
if actionIdx ~= actionCount
    error('Formation-mode intervention enumeration is incomplete.');
end

bank = struct();
bank.contractVersion = ...
    'formation-local-source-trust-intervention-bank-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = modeCount;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.modeTrustWeights = modeTrustWeights;
bank.actionNames = actionNames;
bank.actionModes = actionModes;
bank.actionFormationIndex = actionFormationIndex;
bank.actionModeIndex = actionModeIndex;
bank.actionAdjacency = actionAdjacency;
bank.actionFusionWeights = actionFusionWeights;
bank.actionDominantSources = actionDominantSources;
bank.actionPosteriorProxyAllowed = ...
    actionPosteriorProxyAllowed;
bank.actionPosteriorObjective = actionPosteriorObjective;
bank.actionPayloadBytes = actionPayloadBytes;
bank.referencePayloadBytes = referencePayloadBytes;
bank.actionWithinReferencePayload = ...
    actionPayloadBytes <= referencePayloadBytes + 1e-9;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.trainingTargetStored = false;
end
