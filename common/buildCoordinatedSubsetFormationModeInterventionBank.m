function bank = buildCoordinatedSubsetFormationModeInterventionBank( ...
        projection, groupIds, options)
% BUILDCOORDINATEDSUBSETFORMATIONMODEINTERVENTIONBANK Subset actions.
%
% Builds the registered reference plus every formation subset of the
% requested orders at one pre-registered trust mode.  The subset list is
% determined only by formation membership and never by truth or future
% outcomes.  This makes coordination order an explicit experimental axis
% without introducing another trust-weight sweep.

if nargin < 3 || isempty(options)
    options = struct();
end
localBank = buildFormationModeInterventionBank( ...
    projection, groupIds);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
nodeCount = numel(groupIds);
subsetTrustWeight = getField(options, ...
    'subsetTrustWeight', 0.30);
subsetOrders = reshape(getField(options, ...
    'subsetOrders', 3:formationCount), 1, []);
modeTrustWeights = reshape(localBank.modeTrustWeights, 1, []);
if ~isscalar(subsetTrustWeight) || ...
        ~isfinite(subsetTrustWeight) || ...
        subsetTrustWeight <= 0 || isempty(subsetOrders) || ...
        any(~isfinite(subsetOrders)) || ...
        any(subsetOrders ~= round(subsetOrders)) || ...
        any(subsetOrders < 2) || ...
        any(subsetOrders > formationCount) || ...
        numel(unique(subsetOrders)) ~= numel(subsetOrders)
    error('Coordinated subset intervention options are invalid.');
end
subsetOrders = sort(round(subsetOrders));
matchingModes = find( ...
    (1:numel(modeTrustWeights)) ~= 1 & ...
    abs(modeTrustWeights - subsetTrustWeight) <= 1e-12);
if numel(matchingModes) ~= 1
    error('Coordinated subset intervention options are invalid.');
end
subsetModeIndex = matchingModes(1);

adjacencyByMode = projection.modeAdjacencyByIndex;
weightsByMode = projection.modeFusionWeightsByIndex;
sourcesByMode = projection.modeDominantSourcesByIndex;
allowedModeMask = logical(projection.dynamicModeAllowedMask);
payloadByFormationMode = projection.payloadByFormationMode;
objectiveByFormationMode = projection.objectiveByFormationMode;
referenceAdjacency = logical(adjacencyByMode{1});
referenceWeights = weightsByMode{1};
referenceSources = reshape(sourcesByMode{1}, 1, []);
subsetAdjacency = logical(adjacencyByMode{subsetModeIndex});
subsetWeights = weightsByMode{subsetModeIndex};
subsetSources = reshape( ...
    sourcesByMode{subsetModeIndex}, 1, []);
referencePayloadBytes = sum(payloadByFormationMode(:, 1));

subsetCount = 0;
for order = subsetOrders
    subsetCount = subsetCount + nchoosek(formationCount, order);
end
actionCount = 1 + subsetCount;
maximumOrder = max(subsetOrders);
actionModes = ones(actionCount, formationCount);
actionFormationIndices = zeros(actionCount, maximumOrder);
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
for order = subsetOrders
    combinations = nchoosek(1:formationCount, order);
    for combinationIdx = 1:size(combinations, 1)
        selectedFormations = combinations(combinationIdx, :);
        selectedMembers = zeros(1, 0);
        for formationIdx = selectedFormations
            selectedMembers = [selectedMembers, reshape(find( ...
                groupIds == groups(formationIdx)), 1, [])]; %#ok<AGROW>
        end
        actionIdx = actionIdx + 1;
        adjacency = referenceAdjacency;
        weights = referenceWeights;
        sources = referenceSources;
        adjacency(selectedMembers, :) = ...
            subsetAdjacency(selectedMembers, :);
        weights(selectedMembers, :) = ...
            subsetWeights(selectedMembers, :);
        sources(selectedMembers) = subsetSources(selectedMembers);
        actionModes(actionIdx, selectedFormations) = subsetModeIndex;
        actionFormationIndices(actionIdx, 1:order) = ...
            selectedFormations;
        actionModeIndex(actionIdx) = subsetModeIndex;
        actionInterventionOrder(actionIdx) = order;
        actionAdjacency(:, :, actionIdx) = adjacency;
        actionFusionWeights(:, :, actionIdx) = weights;
        actionDominantSources(actionIdx, :) = sources;
        actionNames{actionIdx} = sprintf( ...
            'formations-%s-dynamic-trust-%.2f', ...
            formatFormationToken(selectedFormations), ...
            subsetTrustWeight);
        actionPosteriorProxyAllowed(actionIdx) = all( ...
            allowedModeMask(selectedFormations, subsetModeIndex));
        actionPosteriorObjective(actionIdx) = sum( ...
            objectiveByFormationMode( ...
                selectedFormations, subsetModeIndex) - ...
            objectiveByFormationMode(selectedFormations, 1));
        actionPayloadBytes(actionIdx) = ...
            referencePayloadBytes - sum(payloadByFormationMode( ...
                selectedFormations, 1)) + ...
            sum(payloadByFormationMode( ...
                selectedFormations, subsetModeIndex));
    end
end
if actionIdx ~= actionCount
    error('Coordinated subset intervention enumeration is incomplete.');
end

bank = struct();
bank.contractVersion = ...
    'formation-coordinated-subset-intervention-bank-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = numel(modeTrustWeights);
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.subsetTrustWeight = subsetTrustWeight;
bank.subsetModeIndex = subsetModeIndex;
bank.subsetOrders = subsetOrders;
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

function value = formatFormationToken(indices)
value = strtrim(sprintf('%d-', reshape(indices, 1, [])));
value = value(1:(end - 1));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
