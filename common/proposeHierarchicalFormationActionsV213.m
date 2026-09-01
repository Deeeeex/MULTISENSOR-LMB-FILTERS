function [proposedCandidates, releaseFormationIds, details] = ...
        proposeHierarchicalFormationActionsV213( ...
            graph, candidates, options)
% PROPOSEHIERARCHICALFORMATIONACTIONSV213 Bounded truth-free proposal set.
%
% V213 first keeps every formation whose observable need is within 90% of
% the maximum, capped at two formations.  It then retains the union of the
% top three supported-label actions for each of three interpretable modes.
% Formation releases are proposed only when the caller explicitly marks
% them executable.  Semantic cooldown is applied before mode ranking so a
% blocked source-label key is replaced by the next eligible candidate.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getHierarchicalDelayedValueControllerV213Protocol();
proposal = protocol.proposal;
validateGraph(graph, proposal);
if ~isstruct(candidates)
    error('HierarchicalFormationProposalV213:InvalidCandidates', ...
        'Candidates must be a struct array.');
end
candidates = reshape(candidates, 1, []);

[candidateFormationIds, candidateIndices] = ...
    candidateKeys(candidates);
[modeFeatureMatrix, availableFeatureNames, candidateKind] = ...
    candidateModeFeatures(candidates, options);
if isempty(candidates)
    modeFeatureMatrix = zeros(0, numel(proposal.modeFeatureNames));
    availableFeatureNames = proposal.modeFeatureNames;
end
modeFeatureIndices = resolveUniqueNames( ...
    availableFeatureNames, proposal.modeFeatureNames);

formationFeatureIdx = resolveUniqueNames( ...
    graph.nodeFeatureNames, {proposal.formationFeatureName});
formationNeed = graph.nodeFeatures(:, formationFeatureIdx);
[selectedFormationRows, needThreshold] = selectFormations( ...
    formationNeed, proposal);
selectedFormationIds = reshape( ...
    graph.formationIds(selectedFormationRows), 1, []);

[eligibleCandidateMask, cooldownDetails] = ...
    applyCandidateCooldown(candidates, candidateFormationIds, ...
        options, proposal.semanticCooldownPages);
eligibleCandidateMask = eligibleCandidateMask & ...
    ismember(candidateFormationIds, selectedFormationIds);

selectedMask = false(1, numel(candidates));
selectedByMode = false(numel(candidates), ...
    numel(proposal.modeFeatureNames));
for formationId = selectedFormationIds
    formationRows = find(eligibleCandidateMask & ...
        candidateFormationIds == formationId);
    for modeIdx = 1:numel(proposal.modeFeatureNames)
        if isempty(formationRows)
            continue;
        end
        score = modeFeatureMatrix( ...
            formationRows, modeFeatureIndices(modeIdx));
        orderTable = [-score(:), candidateIndices(formationRows)'];
        [~, order] = sortrows(orderTable, [1, 2]);
        keepCount = min(proposal.topPerMode, numel(order));
        keepRows = formationRows(order(1:keepCount));
        selectedMask(keepRows) = true;
        selectedByMode(keepRows, modeIdx) = true;
    end
end
selectedIndices = find(selectedMask);
proposedCandidates = candidates(selectedIndices);

[releaseFormationIds, releaseDetails] = ...
    selectFormationReleases(selectedFormationIds, options, ...
        proposal.semanticCooldownPages);
[featureBytes, receiverCounts] = propagationFeatureCost( ...
    graph, proposedCandidates, proposal);

details = struct();
details.contractVersion = proposal.contractVersion;
details.controllerProtocolId = protocol.id;
details.candidateKind = candidateKind;
details.formationFeatureName = proposal.formationFeatureName;
details.formationIds = reshape(graph.formationIds, 1, []);
details.formationNeed = reshape(formationNeed, 1, []);
details.maximumFormationNeed = maximumOrZero(formationNeed);
details.relativeNeedFloor = proposal.relativeNeedFloor;
details.needThreshold = needThreshold;
details.maximumFormationCount = proposal.maximumFormationCount;
details.selectedFormationIds = selectedFormationIds;
details.modeFeatureNames = proposal.modeFeatureNames;
details.topPerMode = proposal.topPerMode;
details.fullCandidateCount = numel(candidates);
details.cooldownEligibleCandidateCount = ...
    nnz(cooldownDetails.eligibleMask);
details.eligibleSelectedFormationCandidateCount = ...
    nnz(eligibleCandidateMask);
details.proposedLabelCandidateCount = numel(proposedCandidates);
details.selectedCandidateIndices = ...
    reshape(candidateIndices(selectedIndices), 1, []);
details.selectedByMode = selectedByMode(selectedIndices, :);
details.releaseFormationIds = releaseFormationIds;
details.releaseProposalCount = numel(releaseFormationIds);
details.proposalCount = numel(proposedCandidates) + ...
    numel(releaseFormationIds);
details.maximumLabelProposalCount = ...
    proposal.maximumLabelProposalCount;
details.maximumProposalCount = proposal.maximumProposalCount;
details.proposalWithinCap = ...
    numel(proposedCandidates) <= proposal.maximumLabelProposalCount && ...
    details.proposalCount <= proposal.maximumProposalCount;
details.semanticCooldownPages = proposal.semanticCooldownPages;
details.cooldown = cooldownDetails;
details.release = releaseDetails;
details.receiverCountsByLabelProposal = receiverCounts;
details.propagationFeatureBytesByLabelProposal = featureBytes;
details.propagationFeatureAttemptedBytes = sum(featureBytes);
details.releaseActionPayloadBytesIncluded = false;
details.propagationFeatureCostChargedForEveryLabelProposal = true;
details.formationEnumeratorOrderUsedOnlyForExactTies = true;
details.candidateEnumeratorOrderUsedOnlyForExactTies = true;
details.truthUsed = false;
details.futureInformationUsed = false;
details.numericFormationIdentifiersUsedAsFeatures = false;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.numericSourceIdentifiersUsedAsFeatures = false;
if ~details.proposalWithinCap
    error('HierarchicalFormationProposalV213:ProposalCap', ...
        'The frozen V213 proposal cap was exceeded.');
end
end

function validateGraph(graph, proposal)
required = {'nodeFeatures', 'nodeFeatureNames', 'formationIds', ...
    'nodeCount', 'truthUsed', 'futureInformationUsed'};
if ~isstruct(graph) || ~isscalar(graph) || ...
        ~all(isfield(graph, required)) || ...
        size(graph.nodeFeatures, 1) ~= numel(graph.formationIds) || ...
        size(graph.nodeFeatures, 2) ~= numel(graph.nodeFeatureNames) || ...
        any(~isfinite(graph.nodeFeatures(:))) || ...
        ~isscalar(graph.nodeCount) || graph.nodeCount < 1 || ...
        graph.nodeCount ~= round(graph.nodeCount) || ...
        graph.truthUsed || graph.futureInformationUsed || ...
        proposal.maximumFormationCount < 1
    error('HierarchicalFormationProposalV213:InvalidGraph', ...
        'A finite truth-free V208 formation graph is required.');
end
end

function [formationIds, indices] = candidateKeys(candidates)
if isempty(candidates)
    formationIds = zeros(1, 0);
    indices = zeros(1, 0);
    return;
end
if all(isfield(candidates, {'formationId', 'candidateIndex', ...
        'sourceId', 'label'}))
    formationIds = [candidates.formationId];
elseif all(isfield(candidates, {'receiverFormationId', ...
        'candidateIndex', 'sourceId', 'label'}))
    formationIds = [candidates.receiverFormationId];
else
    error('HierarchicalFormationProposalV213:CandidateSchema', ...
        'V190 candidates or V208 dataset rows are required.');
end
indices = [candidates.candidateIndex];
if any(~isfinite(formationIds)) || any(formationIds < 1) || ...
        any(formationIds ~= round(formationIds)) || ...
        any(~isfinite(indices)) || any(indices < 1) || ...
        any(indices ~= round(indices)) || ...
        numel(unique(indices)) ~= numel(indices) || ...
        any(arrayfun(@(value) ...
            ~isnumeric(value.label) || numel(value.label) ~= 2 || ...
            any(~isfinite(value.label(:))), candidates))
    error('HierarchicalFormationProposalV213:CandidateKey', ...
        'Candidate routing keys must be finite and unique.');
end
formationIds = reshape(formationIds, 1, []);
indices = reshape(indices, 1, []);
end

function [matrix, names, kind] = ...
        candidateModeFeatures(candidates, options)
if isfield(options, 'modeFeatureMatrix') || ...
        isfield(options, 'modeFeatureNames')
    if ~all(isfield(options, ...
            {'modeFeatureMatrix', 'modeFeatureNames'}))
        error('HierarchicalFormationProposalV213:ModeFeatures', ...
            'The mode feature matrix and names must be supplied together.');
    end
    matrix = options.modeFeatureMatrix;
    names = reshape(options.modeFeatureNames, 1, []);
    kind = 'registered-feature-matrix';
elseif ~isempty(candidates) && isfield(candidates, 'features') && ...
        isfield(options, 'actionFeatureNames')
    matrix = vertcat(candidates.features);
    names = reshape(options.actionFeatureNames, 1, []);
    kind = 'v208-dataset-row';
elseif isempty(candidates)
    matrix = zeros(0, 0);
    names = cell(1, 0);
    kind = 'empty';
else
    matrix = zeros(numel(candidates), 0);
    names = cell(1, 0);
    for candidateIdx = 1:numel(candidates)
        [values, currentNames] = ...
            buildFormationLabelActionModeFeaturesV202( ...
                candidates(candidateIdx));
        if candidateIdx == 1
            matrix = zeros(numel(candidates), numel(values));
            names = reshape(currentNames, 1, []);
        elseif ~isequal(names, reshape(currentNames, 1, []))
            error('HierarchicalFormationProposalV213:FeatureDrift', ...
                'V202 mode features changed between candidates.');
        end
        matrix(candidateIdx, :) = values;
    end
    kind = 'v190-candidate';
end
if size(matrix, 1) ~= numel(candidates) || ...
        size(matrix, 2) ~= numel(names) || ...
        any(~isfinite(matrix(:)))
    error('HierarchicalFormationProposalV213:ModeFeatures', ...
        'The candidate mode feature matrix is malformed.');
end
end

function indices = resolveUniqueNames(names, requested)
indices = zeros(1, numel(requested));
for requestedIdx = 1:numel(requested)
    match = find(strcmp(names, requested{requestedIdx}));
    if numel(match) ~= 1
        error('HierarchicalFormationProposalV213:MissingFeature', ...
            'Required feature is missing: %s.', ...
            requested{requestedIdx});
    end
    indices(requestedIdx) = match;
end
end

function [selectedRows, threshold] = selectFormations(need, proposal)
maximumNeed = maximumOrZero(need);
if isempty(need) || maximumNeed <= 0
    selectedRows = zeros(1, 0);
    threshold = 0;
    return;
end
threshold = proposal.relativeNeedFloor * maximumNeed;
eligibleRows = find(need >= threshold - 1e-12);
orderTable = [-need(eligibleRows), eligibleRows(:)];
[~, order] = sortrows(orderTable, [1, 2]);
keepCount = min(proposal.maximumFormationCount, numel(order));
selectedRows = reshape(eligibleRows(order(1:keepCount)), 1, []);
end

function [eligible, details] = applyCandidateCooldown( ...
        candidates, formationIds, options, cooldownPages)
eligible = true(1, numel(candidates));
blockedKeys = getField(options, 'blockedSemanticKeys', struct([]));
if ~isstruct(blockedKeys)
    error('HierarchicalFormationProposalV213:BlockedKeys', ...
        'blockedSemanticKeys must be a struct array.');
end
for candidateIdx = 1:numel(candidates)
    if semanticKeyBlocked('supported-label-kla', ...
            formationIds(candidateIdx), ...
            candidates(candidateIdx).label, blockedKeys)
        eligible(candidateIdx) = false;
    end
end

history = getField(options, 'appliedHistory', struct([]));
historyDetails = struct();
if ~isstruct(history)
    error('HierarchicalFormationProposalV213:History', ...
        'appliedHistory must be a struct array.');
end
if ~isempty(history) && ~isempty(candidates)
    if ~isfield(options, 'currentTime')
        error('HierarchicalFormationProposalV213:CurrentTime', ...
            'currentTime is required when applied history is supplied.');
    end
    actions = repmat(emptySemanticAction(), 1, numel(candidates));
    for candidateIdx = 1:numel(candidates)
        actions(candidateIdx).actionType = 'supported-label-kla';
        actions(candidateIdx).receiverFormationId = ...
            formationIds(candidateIdx);
        actions(candidateIdx).label = ...
            reshape(candidates(candidateIdx).label, 2, 1);
    end
    [historyEligible, historyDetails] = ...
        projectFormationActionSemanticCooldownV209( ...
            actions, history, options.currentTime, ...
            struct('cooldownPages', cooldownPages));
    eligible = eligible & historyEligible;
end
details = struct();
details.contractVersion = ...
    'hierarchical-formation-cooldown-v213-v1';
details.eligibleMask = eligible;
details.blockedMask = ~eligible;
details.blockedSemanticKeyCount = numel(blockedKeys);
details.appliedHistoryCount = numel(history);
details.historyProjection = historyDetails;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function [releaseIds, details] = selectFormationReleases( ...
        selectedFormationIds, options, cooldownPages)
if ~isfield(options, 'releasableFormationIds')
    error('HierarchicalFormationProposalV213:ReleaseFeasibility', ...
        ['releasableFormationIds must be supplied explicitly; V213 ', ...
         'never invents a full-posterior release.']);
end
releasable = reshape(options.releasableFormationIds, 1, []);
if any(~isfinite(releasable)) || any(releasable < 1) || ...
        any(releasable ~= round(releasable))
    error('HierarchicalFormationProposalV213:ReleaseFeasibility', ...
        'Releasable formation identifiers must be finite integers.');
end
releaseIds = selectedFormationIds( ...
    ismember(selectedFormationIds, releasable));
blockedKeys = getField(options, 'blockedSemanticKeys', struct([]));
blockedMask = false(1, numel(releaseIds));
for releaseIdx = 1:numel(releaseIds)
    blockedMask(releaseIdx) = semanticKeyBlocked( ...
        'formation-release', releaseIds(releaseIdx), ...
        zeros(2, 1), blockedKeys);
end

history = getField(options, 'appliedHistory', struct([]));
historyDetails = struct();
if ~isempty(history) && ~isempty(releaseIds)
    if ~isfield(options, 'currentTime')
        error('HierarchicalFormationProposalV213:CurrentTime', ...
            'currentTime is required when applied history is supplied.');
    end
    actions = repmat(emptySemanticAction(), 1, numel(releaseIds));
    for releaseIdx = 1:numel(releaseIds)
        actions(releaseIdx).actionType = 'formation-release';
        actions(releaseIdx).receiverFormationId = ...
            releaseIds(releaseIdx);
    end
    [historyEligible, historyDetails] = ...
        projectFormationActionSemanticCooldownV209( ...
            actions, history, options.currentTime, ...
            struct('cooldownPages', cooldownPages));
    blockedMask = blockedMask | ~historyEligible;
end
releaseIds = releaseIds(~blockedMask);
details = struct();
details.contractVersion = ...
    'hierarchical-formation-release-proposal-v213-v1';
details.releasableFormationIds = releasable;
details.releaseFormationIds = releaseIds;
details.blockedMaskBeforeRemoval = blockedMask;
details.payloadBytesScoredByActionValueModel = true;
details.propagationDescriptorBytes = 0;
details.historyProjection = historyDetails;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function blocked = semanticKeyBlocked( ...
        actionType, formationId, label, keys)
blocked = false;
for key = reshape(keys, 1, [])
    keyFormationId = NaN;
    if isfield(key, 'receiverFormationId')
        keyFormationId = key.receiverFormationId;
    elseif isfield(key, 'formationId')
        keyFormationId = key.formationId;
    end
    if ~isscalar(keyFormationId) || ~isfinite(keyFormationId) || ...
            keyFormationId ~= formationId
        continue;
    end
    keyType = '';
    if isfield(key, 'actionType')
        keyType = char(key.actionType);
    end
    if ~isempty(keyType) && ~strcmpi(keyType, actionType)
        continue;
    end
    if strcmpi(actionType, 'formation-release')
        if isempty(keyType) && isfield(key, 'label') && ...
                isnumeric(key.label) && numel(key.label) == 2 && ...
                any(key.label(:) ~= 0)
            continue;
        end
        blocked = true;
        return;
    end
    if isfield(key, 'label') && isnumeric(key.label) && ...
            numel(key.label) == 2 && ...
            isequal(reshape(key.label, 2, 1), reshape(label, 2, 1))
        blocked = true;
        return;
    end
end
end

function [bytes, receiverCounts] = propagationFeatureCost( ...
        graph, candidates, proposal)
bytes = zeros(1, numel(candidates));
receiverCounts = zeros(1, numel(candidates));
for candidateIdx = 1:numel(candidates)
    if isfield(candidates, 'receiverIds') && ...
            ~isempty(candidates(candidateIdx).receiverIds)
        receiverCount = numel(candidates(candidateIdx).receiverIds);
    else
        if isfield(candidates, 'receiverFormationId')
            formationId = candidates(candidateIdx).receiverFormationId;
        else
            formationId = candidates(candidateIdx).formationId;
        end
        receiverCount = graphFormationSize(graph, formationId);
    end
    if receiverCount < 1 || receiverCount ~= round(receiverCount)
        error('HierarchicalFormationProposalV213:ReceiverCount', ...
            'Every label proposal must have at least one receiver.');
    end
    receiverCounts(candidateIdx) = receiverCount;
    bytes(candidateIdx) = proposal.sourceSynopsisBytes + ...
        proposal.receiverPropagationBytes * receiverCount;
end
end

function count = graphFormationSize(graph, formationId)
formationRow = find(graph.formationIds == formationId);
if numel(formationRow) ~= 1
    error('HierarchicalFormationProposalV213:FormationLookup', ...
        'A candidate formation is absent from the graph.');
end
sizeFeatureIdx = resolveUniqueNames( ...
    graph.nodeFeatureNames, {'formation_size_fraction'});
count = round(graph.nodeFeatures( ...
    formationRow, sizeFeatureIdx) * graph.nodeCount);
end

function value = maximumOrZero(values)
if isempty(values)
    value = 0;
else
    value = max(values);
end
end

function action = emptySemanticAction()
action = struct('actionType', '', ...
    'receiverFormationId', 0, 'label', zeros(2, 1));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
