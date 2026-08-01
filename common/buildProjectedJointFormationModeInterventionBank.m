function bank = buildProjectedJointFormationModeInterventionBank( ...
        projection, groupIds)
% BUILDPROJECTEDJOINTFORMATIONMODEINTERVENTIONBANK Exact projected action.
%
% Returns the registered all-reference action and the single coordinated
% multi-formation action selected by the truth-free exact finite-mode
% projector.  Unlike the singleton/pair teacher banks, this probe preserves
% the complete joint mode vector chosen under the current posterior-risk,
% formation-risk, and payload constraints.

required = { ...
    'formationModeCount', 'selectedModeByFormation', ...
    'modeDominantSourcesByIndex', 'modeAdjacencyByIndex', ...
    'modeFusionWeightsByIndex', 'dynamicModeAllowedMask', ...
    'payloadByFormationMode', 'objectiveByFormationMode', ...
    'projector', 'truthUsed'};
if ~isstruct(projection) || ~all(isfield(projection, required)) || ...
        projection.truthUsed
    error('Projected joint formation intervention is incomplete.');
end
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
nodeCount = numel(groupIds);
formationCount = numel(groups);
modeCount = round(projection.formationModeCount);
selectedModes = reshape( ...
    projection.selectedModeByFormation, 1, []);
modeTrustWeights = reshape( ...
    projection.projector.modeTrustWeights, 1, []);
sourcesByMode = projection.modeDominantSourcesByIndex;
adjacencyByMode = projection.modeAdjacencyByIndex;
weightsByMode = projection.modeFusionWeightsByIndex;
allowedModeMask = logical(projection.dynamicModeAllowedMask);
payloadByFormationMode = projection.payloadByFormationMode;
objectiveByFormationMode = projection.objectiveByFormationMode;
if nodeCount < 1 || formationCount < 1 || modeCount < 2 || ...
        numel(selectedModes) ~= formationCount || ...
        any(selectedModes ~= round(selectedModes)) || ...
        any(selectedModes < 1) || any(selectedModes > modeCount) || ...
        numel(modeTrustWeights) ~= modeCount || ...
        numel(sourcesByMode) ~= modeCount || ...
        numel(adjacencyByMode) ~= modeCount || ...
        numel(weightsByMode) ~= modeCount || ...
        ~isequal(size(allowedModeMask), ...
            [formationCount, modeCount]) || ...
        ~isequal(size(payloadByFormationMode), ...
            [formationCount, modeCount]) || ...
        ~isequal(size(objectiveByFormationMode), ...
            [formationCount, modeCount])
    error('Projected joint formation intervention dimensions are invalid.');
end

actionCount = 2;
actionModes = [ones(1, formationCount); selectedModes];
actionAdjacency = false(nodeCount, nodeCount, actionCount);
actionFusionWeights = zeros(nodeCount, nodeCount, actionCount);
actionDominantSources = zeros(actionCount, nodeCount);
for actionIdx = 1:actionCount
    modes = actionModes(actionIdx, :);
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
    end
end

referencePayloadBytes = sum(payloadByFormationMode(:, 1));
selectedPayloadBytes = 0;
selectedObjective = 0;
selectedAllowed = true;
for formationIdx = 1:formationCount
    modeIdx = selectedModes(formationIdx);
    selectedPayloadBytes = selectedPayloadBytes + ...
        payloadByFormationMode(formationIdx, modeIdx);
    selectedObjective = selectedObjective + ...
        objectiveByFormationMode(formationIdx, modeIdx);
    selectedAllowed = selectedAllowed && ...
        allowedModeMask(formationIdx, modeIdx);
end
changedMask = selectedModes ~= 1;
if any(changedMask)
    actionName = sprintf( ...
        'projected-joint-modes-%s', ...
        strrep(strtrim(sprintf('%d-', selectedModes)), ' ', ''));
    if actionName(end) == '-'
        actionName = actionName(1:end-1);
    end
else
    actionName = 'projected-joint-reference';
end

bank = struct();
bank.contractVersion = ...
    'formation-projected-joint-intervention-bank-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = modeCount;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.modeTrustWeights = modeTrustWeights;
bank.actionNames = {'reference', actionName};
bank.actionModes = actionModes;
bank.actionModeIndices = actionModes;
bank.actionFormationIndex = zeros(actionCount, 1);
bank.actionFormationIndices = [ ...
    zeros(1, formationCount); ...
    double(changedMask) .* (1:formationCount)];
bank.actionModeIndex = ones(actionCount, 1);
bank.actionInterventionOrder = [0; nnz(changedMask)];
bank.actionAdjacency = actionAdjacency;
bank.actionFusionWeights = actionFusionWeights;
bank.actionDominantSources = actionDominantSources;
bank.actionPosteriorProxyAllowed = [true; selectedAllowed];
bank.actionPosteriorObjective = [0; selectedObjective];
bank.actionPayloadBytes = [ ...
    referencePayloadBytes; selectedPayloadBytes];
bank.referencePayloadBytes = referencePayloadBytes;
bank.actionWithinReferencePayload = ...
    bank.actionPayloadBytes <= referencePayloadBytes + 1e-9;
bank.selectedModes = selectedModes;
bank.selectedTrustWeights = modeTrustWeights(selectedModes);
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.trainingTargetStored = false;
end
