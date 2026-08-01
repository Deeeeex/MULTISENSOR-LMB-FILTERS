function bank = buildExhaustiveFormationModeVectorInterventionBank( ...
        projection, groupIds, options)
% BUILDEXHAUSTIVEFORMATIONMODEVECTORINTERVENTIONBANK Joint teacher bank.
%
% Enumerates every per-formation mode vector.  This exponential bank is an
% offline oracle/teacher instrument, not a deployment-time selector.  A
% learned value model and constrained projection are expected to replace
% exhaustive enumeration once safe headroom is established.

if nargin < 3 || isempty(options)
    options = struct();
end
localBank = buildFormationModeInterventionBank( ...
    projection, groupIds);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
nodeCount = numel(groupIds);
modeCount = localBank.modeCount;
maximumActionCount = getField(options, ...
    'maximumActionCount', 4096);
actionCount = modeCount ^ formationCount;
if ~isscalar(maximumActionCount) || ...
        ~isfinite(maximumActionCount) || ...
        maximumActionCount ~= round(maximumActionCount) || ...
        maximumActionCount < 1 || ~isfinite(actionCount) || ...
        actionCount ~= round(actionCount) || ...
        actionCount > maximumActionCount
    error('Exhaustive formation mode-vector bank is too large.');
end

adjacencyByMode = projection.modeAdjacencyByIndex;
weightsByMode = projection.modeFusionWeightsByIndex;
sourcesByMode = projection.modeDominantSourcesByIndex;
allowedModeMask = logical(projection.dynamicModeAllowedMask);
payloadByFormationMode = projection.payloadByFormationMode;
objectiveByFormationMode = projection.objectiveByFormationMode;
modeTrustWeights = reshape(localBank.modeTrustWeights, 1, []);
referencePayloadBytes = sum(payloadByFormationMode(:, 1));

actionModes = zeros(actionCount, formationCount);
actionModeIndices = zeros(actionCount, formationCount);
actionFormationIndices = zeros(actionCount, formationCount);
actionFormationIndex = zeros(actionCount, 1);
actionModeIndex = ones(actionCount, 1);
actionInterventionOrder = zeros(actionCount, 1);
actionAdjacency = false(nodeCount, nodeCount, actionCount);
actionFusionWeights = zeros(nodeCount, nodeCount, actionCount);
actionDominantSources = zeros(actionCount, nodeCount);
actionNames = cell(1, actionCount);
actionPosteriorProxyAllowed = true(actionCount, 1);
actionPosteriorObjective = zeros(actionCount, 1);
actionPayloadBytes = zeros(actionCount, 1);

for actionIdx = 1:actionCount
    modes = actionIndexToModeVector( ...
        actionIdx, modeCount, formationCount);
    actionModes(actionIdx, :) = modes;
    actionModeIndices(actionIdx, :) = modes;
    changed = modes ~= 1;
    actionFormationIndices(actionIdx, :) = ...
        double(changed) .* (1:formationCount);
    actionInterventionOrder(actionIdx) = nnz(changed);
    payload = 0;
    objective = 0;
    allowed = true;
    for formationIdx = 1:formationCount
        members = reshape(find( ...
            groupIds == groups(formationIdx)), 1, []);
        modeIdx = modes(formationIdx);
        adjacency = adjacencyByMode{modeIdx};
        weights = weightsByMode{modeIdx};
        sources = reshape(sourcesByMode{modeIdx}, 1, []);
        actionAdjacency(members, :, actionIdx) = ...
            logical(adjacency(members, :));
        actionFusionWeights(members, :, actionIdx) = ...
            weights(members, :);
        actionDominantSources(actionIdx, members) = sources(members);
        payload = payload + ...
            payloadByFormationMode(formationIdx, modeIdx);
        objective = objective + ...
            objectiveByFormationMode(formationIdx, modeIdx) - ...
            objectiveByFormationMode(formationIdx, 1);
        allowed = allowed && ...
            allowedModeMask(formationIdx, modeIdx);
    end
    actionNames{actionIdx} = sprintf( ...
        'mode-vector-%s', formatModeToken(modes));
    actionPosteriorProxyAllowed(actionIdx) = allowed;
    actionPosteriorObjective(actionIdx) = objective;
    actionPayloadBytes(actionIdx) = payload;
end

bank = struct();
bank.contractVersion = ...
    'formation-exhaustive-mode-vector-intervention-bank-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = modeCount;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.maximumActionCount = maximumActionCount;
bank.modeTrustWeights = modeTrustWeights;
bank.actionNames = actionNames;
bank.actionModes = actionModes;
bank.actionModeIndices = actionModeIndices;
bank.actionFormationIndices = actionFormationIndices;
bank.actionFormationIndex = actionFormationIndex;
bank.actionModeIndex = actionModeIndex;
bank.actionInterventionOrder = actionInterventionOrder;
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

expectedIndices = formationModeVectorToActionIndex( ...
    actionModes, modeCount);
if ~isequal(expectedIndices, (1:actionCount)') || ...
        ~isequal(actionModes(1, :), ones(1, formationCount)) || ...
        ~isequal(actionAdjacency(:, :, 1), ...
            localBank.actionAdjacency(:, :, 1)) || ...
        ~isequal(actionFusionWeights(:, :, 1), ...
            localBank.actionFusionWeights(:, :, 1))
    error('Exhaustive formation mode-vector enumeration is inconsistent.');
end
end

function modes = actionIndexToModeVector( ...
        actionIdx, modeCount, formationCount)
code = actionIdx - 1;
modes = ones(1, formationCount);
for formationIdx = formationCount:-1:1
    modes(formationIdx) = mod(code, modeCount) + 1;
    code = floor(code / modeCount);
end
end

function value = formatModeToken(modes)
value = strtrim(sprintf('%d-', reshape(modes, 1, [])));
value = value(1:(end - 1));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
