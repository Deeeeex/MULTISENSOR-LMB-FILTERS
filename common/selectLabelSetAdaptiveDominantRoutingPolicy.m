function [adjacency, details] = ...
    selectLabelSetAdaptiveDominantRoutingPolicy( ...
        context, mode, options)
% SELECTLABELSETADAPTIVEDOMINANTROUTINGPOLICY Safe variable 0.70 route.
%
% A strong 0.05 residual Hamiltonian cycle supplies the connectivity
% certificate.  The high-weight same-formation input is then allowed to
% change while preserving exactly two dominant/residual duplicate routes
% per formation.  Consequently M24 still attempts exactly 40 payloads:
% 24 dominant plus 24 residual inputs minus eight duplicates.

if nargin < 2 || isempty(mode)
    mode = 'source-quality';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
protocol = getField(options, 'protocol', ...
    getLabelSetSimulatorPolicyProtocol());
if ~protocol.adaptiveDominantRoutingImplementationAuthorized
    error('Adaptive dominant routing is not authorized.');
end
dominantWeight = getField(options, ...
    'dominantWeight', protocol.dominantWeight);
residualWeight = getField(options, ...
    'residualWeight', protocol.localResidualWeight);
orientation = getField(options, ...
    'residualOrientation', 'counter-clockwise');

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
physical(1:nodeCount+1:end) = false;
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(physical), [nodeCount, nodeCount])
    error('Adaptive dominant-routing context is invalid.');
end
[residualBaselineAdjacency, residualBaselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-balanced-cycle', ...
        struct('sourceWeight', residualWeight, 'phase', 1));
residualBaselineSources = reshape( ...
    residualBaselineDetails.selectedSourcesByReceiver, 1, []);

[receiverIndices, senderIndices] = find(physical);
sameFormation = ...
    groupIds(receiverIndices) == groupIds(senderIndices);
receiverIndices = receiverIndices(sameFormation);
senderIndices = senderIndices(sameFormation);
scoreMatrix = -inf(nodeCount);
scoreDetails = struct();
fixedRoot = strncmp(mode, 'fixed-root-role-', ...
    numel('fixed-root-role-'));
if fixedRoot
    rootRole = sscanf(mode, 'fixed-root-role-%d');
    if numel(rootRole) ~= 1
        error('Adaptive dominant fixed-root mode is malformed.');
    end
    dominantSources = buildFixedRootSources( ...
        groupIds, residualBaselineSources, physical, rootRole);
    projectionDetails = struct( ...
        'mode', mode, ...
        'fixedRootRole', rootRole, ...
        'maximumSourceLoad', maxSourceLoad( ...
            dominantSources, groupIds), ...
        'objective', NaN, ...
        'duplicateReceiversByFormation', {{}}, ...
        'truthUsed', false);
    posteriorUsed = false;
    currentLinkReliabilityUsed = false;
else
    [scores, scoreDetails] = ...
        computeLabelSetDominantRoutingScores( ...
            context, receiverIndices, senderIndices, ...
            mode, struct('protocol', protocol));
    for edgeIdx = 1:numel(receiverIndices)
        scoreMatrix(receiverIndices(edgeIdx), ...
            senderIndices(edgeIdx)) = scores(edgeIdx);
    end
    maximumSourceLoad = resolveMaximumSourceLoad( ...
        mode, options, protocol);
    [dominantSources, projectionDetails] = ...
        projectDominantSources( ...
            groupIds, residualBaselineSources, ...
            physical, scoreMatrix, ...
            protocol.adaptiveDominantDuplicateCountPerFormation, ...
            maximumSourceLoad);
    posteriorUsed = true;
    currentLinkReliabilityUsed = true;
end
dominantAdjacency = sourceMapToAdjacency( ...
    dominantSources, nodeCount);
if any(dominantAdjacency(:) & ~physical(:))
    error('Adaptive dominant route selected a nonphysical edge.');
end

crossScoreMatrix = -inf(nodeCount);
[crossReceivers, crossSenders] = find(physical);
crossMask = ...
    groupIds(crossReceivers) ~= groupIds(crossSenders);
crossReceivers = crossReceivers(crossMask);
crossSenders = crossSenders(crossMask);
deterministicScores = -( ...
    crossReceivers * (nodeCount + 1) + crossSenders);
for edgeIdx = 1:numel(crossReceivers)
    crossScoreMatrix(crossReceivers(edgeIdx), ...
        crossSenders(edgeIdx)) = deterministicScores(edgeIdx);
end
splice = selectSplicedResidualFormationCycle( ...
    groupIds, residualBaselineSources, dominantSources, ...
    physical, crossScoreMatrix, orientation);
residualAdjacency = splice.residualAdjacency;
[adjacency, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        dominantWeight, residualWeight);

groupCount = numel(unique(groupIds, 'stable'));
expectedDuplicates = ...
    protocol.adaptiveDominantDuplicateCountPerFormation * ...
        groupCount;
actualDuplicates = nnz( ...
    dominantSources == splice.residualSourcesByReceiver);
expectedMessages = 2 * nodeCount - expectedDuplicates;
if actualDuplicates ~= expectedDuplicates || ...
        nnz(adjacency) ~= expectedMessages || ...
        ~splice.residualSensorStrongConnected || ...
        ~splice.residualFormationStrongConnected
    error(['Adaptive dominant route violated the exact duplicate, ', ...
        'message, or residual-connectivity contract.']);
end

details = routeDetails;
details.mode = ['label-set-adaptive-dominant-', mode];
details.contractVersion = ...
    protocol.adaptiveDominantRoutingContractVersion;
details.objective = -getField( ...
    projectionDetails, 'objective', NaN);
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = finiteSpread(scoreMatrix);
details.validCandidateCount = nnz(isfinite(scoreMatrix));
details.fusionWeightMatrix = fusionWeights;
details.baselineAdjacency = logical( ...
    dominantAdjacency | residualBaselineAdjacency);
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.residualBaselineAdjacency = residualBaselineAdjacency;
details.residualBaselinePolicyDetails = ...
    residualBaselineDetails;
details.dominantSourcesByReceiver = dominantSources;
details.residualSourcesByReceiver = ...
    splice.residualSourcesByReceiver;
details.selectedSourcesByReceiver = ...
    splice.residualSourcesByReceiver;
details.selectedSourceWeightsByReceiver = ...
    residualWeight * ones(1, nodeCount);
details.overrideMask = ...
    splice.residualSourcesByReceiver ~= ...
        residualBaselineSources;
details.overrideFraction = mean(details.overrideMask);
details.spliceSelection = splice;
details.dominantProjection = projectionDetails;
details.dominantScoreDetails = scoreDetails;
details.dominantResidualDuplicateCount = actualDuplicates;
details.dominantResidualDuplicateCountPerFormation = ...
    protocol.adaptiveDominantDuplicateCountPerFormation;
details.expectedSelectedMessageCount = expectedMessages;
details.crossFormationMessageCount = ...
    splice.crossFormationEdgeCount;
details.maximumCrossEdges = ...
    splice.crossFormationEdgeCount;
details.maximumCrossSourceLoad = ...
    splice.maximumCrossSourceLoad;
details.maximumCrossReceiverLoad = ...
    splice.maximumCrossReceiverLoad;
details.proposalCrossCount = ...
    splice.crossFormationEdgeCount;
details.repairTriggered = false;
details.payloadConstraintEnforced = false;
details.payloadLimitPassed = NaN;
details.payloadEmergencyUsed = false;
details.posteriorUsed = posteriorUsed;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureOutcomeUsed = false;
details.currentLinkReliabilityUsed = ...
    currentLinkReliabilityUsed;
details.currentPhysicalActionSetUsed = true;
details.backboneMode = ...
    'adaptive-dominant-plus-spliced-strong-residual-cycle';
details.sourceWeight = residualWeight;
details.sensorWindowMature = true;
details.sensorWindowStrongConnected = true;
details.formationWindowMature = true;
details.formationWindowStrongConnected = true;
details.successorSensorStrongConnected = NaN;
details.oneStepTopologyReserveChecked = false;
details.oneStepTopologyReservePassed = NaN;
details.oneStepJointProjectionUsed = false;
details.recursiveSafetyClaimed = false;
details.topologyInfeasible = false;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.safeProjectorUsed = true;
end

function sources = buildFixedRootSources( ...
        groupIds, baselineSources, physical, rootRole)
groups = unique(groupIds, 'stable');
sources = nan(1, numel(groupIds));
for groupIdx = 1:numel(groups)
    members = reshape(find( ...
        groupIds == groups(groupIdx)), 1, []);
    if rootRole < 1 || rootRole > numel(members)
        error('Fixed dominant root role exceeds a formation size.');
    end
    root = members(rootRole);
    sources(members) = root;
    sources(root) = baselineSources(root);
    for receiver = members
        if sources(receiver) == receiver || ...
                ~physical(receiver, sources(receiver))
            error('Fixed dominant root route is not physical.');
        end
    end
end
end

function [sources, details] = projectDominantSources( ...
        groupIds, baselineSources, physical, scoreMatrix, ...
        duplicateCount, maximumSourceLoad)
groups = unique(groupIds, 'stable');
sources = nan(1, numel(groupIds));
duplicateReceivers = cell(1, numel(groups));
objectives = zeros(1, numel(groups));
loads = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    members = reshape(find( ...
        groupIds == groups(groupIdx)), 1, []);
    [groupSources, duplicateReceivers{groupIdx}, ...
        objectives(groupIdx), loads(groupIdx)] = ...
            solveFormationAssignment( ...
                members, baselineSources, physical, ...
                scoreMatrix, duplicateCount, ...
                maximumSourceLoad);
    sources(members) = groupSources;
end
details = struct();
details.contractVersion = ...
    'label-set-dominant-route-projection-v1';
details.duplicateCountPerFormation = duplicateCount;
details.maximumSourceLoadConstraint = maximumSourceLoad;
details.maximumSourceLoad = max(loads);
details.objectiveByFormation = objectives;
details.objective = sum(objectives);
details.duplicateReceiversByFormation = ...
    duplicateReceivers;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function [bestSources, bestDuplicates, ...
        bestObjective, bestMaximumLoad] = ...
    solveFormationAssignment( ...
        members, baselineSources, physical, scoreMatrix, ...
        duplicateCount, maximumSourceLoad)
groupSize = numel(members);
if duplicateCount < 0 || duplicateCount >= groupSize
    error('Dominant duplicate-count constraint is invalid.');
end
duplicateSets = nchoosek(members, duplicateCount);
bestObjective = -inf;
bestSources = nan(1, groupSize);
bestDuplicates = zeros(1, duplicateCount);
bestMaximumLoad = inf;
for duplicateSetIdx = 1:size(duplicateSets, 1)
    duplicateSet = duplicateSets(duplicateSetIdx, :);
    nonduplicateReceivers = setdiff( ...
        members, duplicateSet, 'stable');
    choices = cell(1, numel(nonduplicateReceivers));
    feasible = true;
    for receiverCursor = 1:numel(nonduplicateReceivers)
        receiver = nonduplicateReceivers(receiverCursor);
        choices{receiverCursor} = members( ...
            members ~= receiver & ...
            members ~= baselineSources(receiver) & ...
            physical(receiver, members) & ...
            isfinite(scoreMatrix(receiver, members)));
        if isempty(choices{receiverCursor})
            feasible = false;
            break;
        end
    end
    if ~feasible
        continue;
    end
    choiceCounts = cellfun(@numel, choices);
    assignmentCount = prod(choiceCounts);
    for assignmentIdx = 0:(assignmentCount - 1)
        groupSources = nan(1, groupSize);
        for receiver = duplicateSet
            receiverPosition = find(members == receiver, 1);
            groupSources(receiverPosition) = ...
                baselineSources(receiver);
        end
        cursor = assignmentIdx;
        for receiverCursor = 1:numel(nonduplicateReceivers)
            receiver = nonduplicateReceivers(receiverCursor);
            receiverPosition = find(members == receiver, 1);
            choiceIdx = 1 + mod(cursor, ...
                choiceCounts(receiverCursor));
            cursor = floor(cursor / ...
                choiceCounts(receiverCursor));
            groupSources(receiverPosition) = ...
                choices{receiverCursor}(choiceIdx);
        end
        sourceLoad = zeros(1, groupSize);
        objective = 0;
        for receiverCursor = 1:groupSize
            receiver = members(receiverCursor);
            sender = groupSources(receiverCursor);
            senderPosition = find(members == sender, 1);
            sourceLoad(senderPosition) = ...
                sourceLoad(senderPosition) + 1;
            objective = objective + ...
                scoreMatrix(receiver, sender);
        end
        maximumLoad = max(sourceLoad);
        if maximumLoad > maximumSourceLoad
            continue;
        end
        if objective > bestObjective + 1e-12
            bestObjective = objective;
            bestSources = groupSources;
            bestDuplicates = duplicateSet;
            bestMaximumLoad = maximumLoad;
        end
    end
end
if ~isfinite(bestObjective) || any(~isfinite(bestSources))
    error('DominantRoute:Infeasible', ...
        'No exact duplicate/load-constrained dominant route exists.');
end
end

function maximumSourceLoad = resolveMaximumSourceLoad( ...
        mode, options, protocol)
if strcmp(mode, 'composite-balanced')
    defaultValue = ...
        protocol.adaptiveDominantBalancedMaximumSourceLoad;
else
    defaultValue = ...
        protocol.adaptiveDominantConcentratedMaximumSourceLoad;
end
maximumSourceLoad = round(getField( ...
    options, 'maximumDominantSourceLoad', defaultValue));
if ~isscalar(maximumSourceLoad) || ...
        ~isfinite(maximumSourceLoad) || ...
        maximumSourceLoad < 1
    error('Adaptive dominant maximum source load is invalid.');
end
end

function adjacency = sourceMapToAdjacency(sources, nodeCount)
adjacency = false(nodeCount);
for receiver = 1:nodeCount
    adjacency(receiver, sources(receiver)) = true;
end
end

function value = maxSourceLoad(sources, groupIds)
groups = unique(groupIds, 'stable');
value = 0;
for groupIdx = 1:numel(groups)
    members = find(groupIds == groups(groupIdx));
    counts = zeros(1, numel(members));
    for receiver = reshape(members, 1, [])
        senderPosition = find( ...
            members == sources(receiver), 1);
        counts(senderPosition) = ...
            counts(senderPosition) + 1;
    end
    value = max(value, max(counts));
end
end

function value = finiteSpread(matrix)
finiteValues = matrix(isfinite(matrix));
if isempty(finiteValues)
    value = NaN;
else
    value = max(finiteValues) - min(finiteValues);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
