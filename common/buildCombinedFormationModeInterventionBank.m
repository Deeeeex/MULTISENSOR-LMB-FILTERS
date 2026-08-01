function bank = buildCombinedFormationModeInterventionBank( ...
        projection, groupIds, options)
% BUILDCOMBINEDFORMATIONMODEINTERVENTIONBANK Local plus pair actions.

if nargin < 3 || isempty(options)
    options = struct();
end
pairTrustWeight = getField(options, 'pairTrustWeight', 0.30);
local = buildFormationModeInterventionBank(projection, groupIds);
pair = buildConservativePairFormationModeInterventionBank( ...
    projection, groupIds, struct( ...
        'pairTrustWeight', pairTrustWeight));
assertReferenceEqual(local, pair);
if local.nodeCount ~= pair.nodeCount || ...
        local.formationCount ~= pair.formationCount || ...
        local.modeCount ~= pair.modeCount || ...
        ~isequal(local.modeTrustWeights, pair.modeTrustWeights)
    error('Combined formation bank dimensions are inconsistent.');
end

pairNonreference = 2:pair.actionCount;
localCount = local.actionCount;
actionCount = localCount + numel(pairNonreference);
localFormationIndices = zeros(localCount, 2);
localFormationIndices(:, 1) = local.actionFormationIndex;
localOrder = double(local.actionFormationIndex > 0);

bank = struct();
bank.contractVersion = ...
    'formation-local-plus-pair-intervention-bank-v1';
bank.nodeCount = local.nodeCount;
bank.formationCount = local.formationCount;
bank.modeCount = local.modeCount;
bank.actionCount = actionCount;
bank.referenceActionIndex = local.referenceActionIndex;
bank.localActionCount = localCount;
bank.pairActionCount = numel(pairNonreference);
bank.pairTrustWeight = pairTrustWeight;
bank.modeTrustWeights = local.modeTrustWeights;
bank.actionNames = [ ...
    local.actionNames, pair.actionNames(pairNonreference)];
bank.actionModes = [ ...
    local.actionModes; pair.actionModes(pairNonreference, :)];
bank.actionFormationIndices = [ ...
    localFormationIndices; ...
    pair.actionFormationIndices(pairNonreference, :)];
bank.actionFormationIndex = [ ...
    local.actionFormationIndex; ...
    pair.actionFormationIndex(pairNonreference)];
bank.actionModeIndex = [ ...
    local.actionModeIndex; pair.actionModeIndex(pairNonreference)];
bank.actionInterventionOrder = [ ...
    localOrder; pair.actionInterventionOrder(pairNonreference)];
bank.actionAdjacency = cat(3, ...
    local.actionAdjacency, ...
    pair.actionAdjacency(:, :, pairNonreference));
bank.actionFusionWeights = cat(3, ...
    local.actionFusionWeights, ...
    pair.actionFusionWeights(:, :, pairNonreference));
bank.actionDominantSources = [ ...
    local.actionDominantSources; ...
    pair.actionDominantSources(pairNonreference, :)];
bank.actionPosteriorProxyAllowed = [ ...
    local.actionPosteriorProxyAllowed; ...
    pair.actionPosteriorProxyAllowed(pairNonreference)];
bank.actionPosteriorObjective = [ ...
    local.actionPosteriorObjective; ...
    pair.actionPosteriorObjective(pairNonreference)];
bank.actionPayloadBytes = [ ...
    local.actionPayloadBytes; ...
    pair.actionPayloadBytes(pairNonreference)];
bank.referencePayloadBytes = local.referencePayloadBytes;
bank.actionWithinReferencePayload = [ ...
    local.actionWithinReferencePayload; ...
    pair.actionWithinReferencePayload(pairNonreference)];
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.trainingTargetStored = false;

if numel(bank.actionNames) ~= actionCount || ...
        size(bank.actionAdjacency, 3) ~= actionCount || ...
        size(bank.actionFusionWeights, 3) ~= actionCount || ...
        size(bank.actionFormationIndices, 1) ~= actionCount
    error('Combined formation bank enumeration is incomplete.');
end
end

function assertReferenceEqual(local, pair)
fields = { ...
    'actionAdjacency', 'actionFusionWeights', ...
    'actionDominantSources', 'actionPayloadBytes'};
for idx = 1:numel(fields)
    name = fields{idx};
    if ndims(local.(name)) == 3
        localValue = local.(name)(:, :, local.referenceActionIndex);
        pairValue = pair.(name)(:, :, pair.referenceActionIndex);
    elseif ismatrix(local.(name)) && ...
            size(local.(name), 1) == local.actionCount
        localValue = local.(name)(local.referenceActionIndex, :);
        pairValue = pair.(name)(pair.referenceActionIndex, :);
    else
        localValue = local.(name)(local.referenceActionIndex);
        pairValue = pair.(name)(pair.referenceActionIndex);
    end
    if ~isequaln(localValue, pairValue)
        error('Combined formation bank reference mismatch in %s.', name);
    end
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
