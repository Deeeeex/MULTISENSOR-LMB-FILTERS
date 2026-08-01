function bank = buildConservativePairFormationModeInterventionBank( ...
        projection, groupIds, options)
% BUILDCONSERVATIVEPAIRFORMATIONMODEINTERVENTIONBANK Pair interventions.
%
% The local H=3 teacher can miss useful actions when a beneficial formation
% change causes a small downstream loss in another formation.  This helper
% adds every unordered pair at one pre-registered conservative trust weight.
% The bank grows quadratically with the formation count and never uses truth
% or future outcomes to choose which pairs are present.

if nargin < 3 || isempty(options)
    options = struct();
end
localBank = buildFormationModeInterventionBank( ...
    projection, groupIds);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
nodeCount = numel(groupIds);
pairTrustWeight = getField(options, 'pairTrustWeight', 0.30);
modeTrustWeights = reshape(localBank.modeTrustWeights, 1, []);
if ~isscalar(pairTrustWeight) || ~isfinite(pairTrustWeight) || ...
        pairTrustWeight <= 0 || formationCount < 2
    error('Conservative pair intervention options are invalid.');
end
matchingModes = find( ...
    (1:numel(modeTrustWeights)) ~= 1 & ...
    abs(modeTrustWeights - pairTrustWeight) <= 1e-12);
if numel(matchingModes) ~= 1
    error('Conservative pair intervention options are invalid.');
end
pairModeIndex = matchingModes(1);

adjacencyByMode = projection.modeAdjacencyByIndex;
weightsByMode = projection.modeFusionWeightsByIndex;
sourcesByMode = projection.modeDominantSourcesByIndex;
allowedModeMask = logical(projection.dynamicModeAllowedMask);
payloadByFormationMode = projection.payloadByFormationMode;
objectiveByFormationMode = projection.objectiveByFormationMode;
referenceAdjacency = logical(adjacencyByMode{1});
referenceWeights = weightsByMode{1};
referenceSources = reshape(sourcesByMode{1}, 1, []);
pairAdjacency = logical(adjacencyByMode{pairModeIndex});
pairWeights = weightsByMode{pairModeIndex};
pairSources = reshape( ...
    sourcesByMode{pairModeIndex}, 1, []);
referencePayloadBytes = sum(payloadByFormationMode(:, 1));

pairCount = formationCount * (formationCount - 1) / 2;
actionCount = 1 + pairCount;
actionModes = ones(actionCount, formationCount);
actionFormationIndices = zeros(actionCount, 2);
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

actionAdjacency(:, :, 1) = referenceAdjacency;
actionFusionWeights(:, :, 1) = referenceWeights;
actionDominantSources(1, :) = referenceSources;
actionNames{1} = 'reference';
actionPayloadBytes(1) = referencePayloadBytes;

actionIdx = 1;
for firstFormation = 1:(formationCount - 1)
    firstMembers = reshape(find( ...
        groupIds == groups(firstFormation)), 1, []);
    for secondFormation = (firstFormation + 1):formationCount
        secondMembers = reshape(find( ...
            groupIds == groups(secondFormation)), 1, []);
        members = [firstMembers, secondMembers];
        actionIdx = actionIdx + 1;
        adjacency = referenceAdjacency;
        weights = referenceWeights;
        sources = referenceSources;
        adjacency(members, :) = pairAdjacency(members, :);
        weights(members, :) = pairWeights(members, :);
        sources(members) = pairSources(members);
        actionModes(actionIdx, ...
            [firstFormation, secondFormation]) = pairModeIndex;
        actionFormationIndices(actionIdx, :) = ...
            [firstFormation, secondFormation];
        actionModeIndex(actionIdx) = pairModeIndex;
        actionInterventionOrder(actionIdx) = 2;
        actionAdjacency(:, :, actionIdx) = adjacency;
        actionFusionWeights(:, :, actionIdx) = weights;
        actionDominantSources(actionIdx, :) = sources;
        actionNames{actionIdx} = sprintf( ...
            'formations-%d-%d-dynamic-trust-%.2f', ...
            firstFormation, secondFormation, pairTrustWeight);
        actionPosteriorProxyAllowed(actionIdx) = all( ...
            allowedModeMask( ...
                [firstFormation, secondFormation], pairModeIndex));
        actionPosteriorObjective(actionIdx) = sum( ...
            objectiveByFormationMode( ...
                [firstFormation, secondFormation], pairModeIndex) - ...
            objectiveByFormationMode( ...
                [firstFormation, secondFormation], 1));
        actionPayloadBytes(actionIdx) = ...
            referencePayloadBytes - sum(payloadByFormationMode( ...
                [firstFormation, secondFormation], 1)) + ...
            sum(payloadByFormationMode( ...
                [firstFormation, secondFormation], pairModeIndex));
    end
end
if actionIdx ~= actionCount
    error('Conservative pair intervention enumeration is incomplete.');
end

bank = struct();
bank.contractVersion = ...
    'formation-conservative-pair-intervention-bank-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = numel(modeTrustWeights);
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.pairTrustWeight = pairTrustWeight;
bank.pairModeIndex = pairModeIndex;
bank.modeTrustWeights = modeTrustWeights;
bank.actionNames = actionNames;
bank.actionModes = actionModes;
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
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
