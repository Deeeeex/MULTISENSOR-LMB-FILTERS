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
payloadAware = strcmp( ...
    mode, 'composite-balanced-payload-aware');
payloadMarginFraction = getField( ...
    options, 'payloadMarginFraction', 0);
if payloadAware
    if ~protocol.adaptiveDominantPayloadAwareRedesignAuthorized
        error('Payload-aware adaptive dominant routing is not authorized.');
    end
    if ~isscalar(payloadMarginFraction) || ...
            ~isfinite(payloadMarginFraction) || ...
            payloadMarginFraction < 0 || ...
            payloadMarginFraction >= 1
        error('payloadMarginFraction must lie in [0, 1).');
    end
elseif payloadMarginFraction ~= 0
    error(['payloadMarginFraction is available only for the ', ...
        'payload-aware mode.']);
end

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
senderPayloadBytes = nan(1, nodeCount);
referenceDominantSources = nan(1, nodeCount);
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
    scoreMode = mode;
    if payloadAware
        scoreMode = 'composite-balanced';
    end
    [scores, scoreDetails] = ...
        computeLabelSetDominantRoutingScores( ...
            context, receiverIndices, senderIndices, ...
            scoreMode, struct('protocol', protocol));
    for edgeIdx = 1:numel(receiverIndices)
        scoreMatrix(receiverIndices(edgeIdx), ...
            senderIndices(edgeIdx)) = scores(edgeIdx);
    end
    projectionOptions = struct( ...
        'payloadConstraintEnforced', false);
    if payloadAware
        senderPayloadBytes = estimateSenderPayloadBytes( ...
            context, options);
        [~, referenceDominantDetails] = ...
            selectRegisteredDirectedRoutingPolicy( ...
                context, 'fixed-index-star', ...
                struct('sourceWeight', dominantWeight));
        referenceDominantSources = reshape( ...
            referenceDominantDetails. ...
                selectedSourcesByReceiver, 1, []);
        projectionOptions = struct( ...
            'payloadConstraintEnforced', true, ...
            'senderPayloadBytes', senderPayloadBytes, ...
            'referenceSources', referenceDominantSources, ...
            'payloadMarginFraction', payloadMarginFraction);
    end
    maximumSourceLoad = resolveMaximumSourceLoad( ...
        mode, options, protocol);
    [dominantSources, projectionDetails] = ...
        projectDominantSources( ...
            groupIds, residualBaselineSources, ...
            physical, scoreMatrix, ...
            protocol.adaptiveDominantDuplicateCountPerFormation, ...
            maximumSourceLoad, projectionOptions);
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
if payloadAware
    details.contractVersion = ...
        protocol. ...
            adaptiveDominantPayloadAwareRoutingContractVersion;
else
    details.contractVersion = ...
        protocol.adaptiveDominantRoutingContractVersion;
end
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
details.payloadConstraintEnforced = payloadAware;
if payloadAware
    selectedPayloadBytes = topologyPayloadBytes( ...
        adjacency, senderPayloadBytes);
    referencePayloadBytes = ...
        sum(senderPayloadBytes) + ...
        projectionDetails.referenceIncrementalPayloadBytes;
    maximumPayloadBytes = ...
        sum(senderPayloadBytes) + ...
        projectionDetails.maximumIncrementalPayloadBytes;
    predictedSelectedPayloadBytes = ...
        sum(senderPayloadBytes) + ...
        projectionDetails.selectedIncrementalPayloadBytes;
    if abs(selectedPayloadBytes - ...
            predictedSelectedPayloadBytes) > 1e-9
        error(['Payload-aware dominant projection and final route ', ...
            'cost disagree.']);
    end
    details.baselinePayloadBytes = referencePayloadBytes;
    details.selectedPayloadBytes = selectedPayloadBytes;
    details.maximumPayloadBytes = maximumPayloadBytes;
    details.payloadMarginFraction = payloadMarginFraction;
    details.payloadDeltaBytes = ...
        selectedPayloadBytes - referencePayloadBytes;
    details.payloadDeltaFraction = ...
        details.payloadDeltaBytes / ...
            max(referencePayloadBytes, eps);
    details.payloadLimitPassed = ...
        selectedPayloadBytes <= maximumPayloadBytes + 1e-9;
    details.payloadRequestedTargetFeasible = ...
        projectionDetails.allRequestedPayloadTargetsFeasible;
    details.payloadFeasibilityFloorActive = ...
        projectionDetails.anyPayloadFeasibilityFloorActive;
else
    details.payloadLimitPassed = NaN;
end
details.payloadEmergencyUsed = false;
details.posteriorUsed = posteriorUsed;
details.posteriorPayloadMetadataUsed = payloadAware;
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
        duplicateCount, maximumSourceLoad, options)
if nargin < 7 || isempty(options)
    options = struct();
end
groups = unique(groupIds, 'stable');
sources = nan(1, numel(groupIds));
duplicateReceivers = cell(1, numel(groups));
objectives = zeros(1, numel(groups));
loads = zeros(1, numel(groups));
selectedPayload = zeros(1, numel(groups));
referencePayload = zeros(1, numel(groups));
requestedPayload = inf(1, numel(groups));
minimumPayload = zeros(1, numel(groups));
maximumPayload = inf(1, numel(groups));
requestedFeasible = true(1, numel(groups));
floorActive = false(1, numel(groups));
for groupIdx = 1:numel(groups)
    members = reshape(find( ...
        groupIds == groups(groupIdx)), 1, []);
    [groupSources, duplicateReceivers{groupIdx}, ...
        objectives(groupIdx), loads(groupIdx), ...
        payloadDetails] = ...
            solveFormationAssignment( ...
                members, baselineSources, physical, ...
                scoreMatrix, duplicateCount, ...
                maximumSourceLoad, options);
    sources(members) = groupSources;
    selectedPayload(groupIdx) = ...
        payloadDetails.selectedIncrementalPayloadBytes;
    referencePayload(groupIdx) = ...
        payloadDetails.referenceIncrementalPayloadBytes;
    requestedPayload(groupIdx) = ...
        payloadDetails.requestedIncrementalPayloadBytes;
    minimumPayload(groupIdx) = ...
        payloadDetails.minimumFeasibleIncrementalPayloadBytes;
    maximumPayload(groupIdx) = ...
        payloadDetails.maximumIncrementalPayloadBytes;
    requestedFeasible(groupIdx) = ...
        payloadDetails.requestedPayloadTargetFeasible;
    floorActive(groupIdx) = ...
        payloadDetails.payloadFeasibilityFloorActive;
end
details = struct();
if logical(getField(options, ...
        'payloadConstraintEnforced', false))
    details.contractVersion = ...
        'label-set-dominant-route-payload-projection-v2';
else
    details.contractVersion = ...
        'label-set-dominant-route-projection-v1';
end
details.duplicateCountPerFormation = duplicateCount;
details.maximumSourceLoadConstraint = maximumSourceLoad;
details.maximumSourceLoad = max(loads);
details.objectiveByFormation = objectives;
details.objective = sum(objectives);
details.duplicateReceiversByFormation = ...
    duplicateReceivers;
details.payloadConstraintEnforced = logical(getField( ...
    options, 'payloadConstraintEnforced', false));
details.selectedIncrementalPayloadBytesByFormation = ...
    selectedPayload;
details.referenceIncrementalPayloadBytesByFormation = ...
    referencePayload;
details.requestedIncrementalPayloadBytesByFormation = ...
    requestedPayload;
details.minimumFeasibleIncrementalPayloadBytesByFormation = ...
    minimumPayload;
details.maximumIncrementalPayloadBytesByFormation = ...
    maximumPayload;
details.requestedPayloadTargetFeasibleByFormation = ...
    requestedFeasible;
details.payloadFeasibilityFloorActiveByFormation = floorActive;
details.selectedIncrementalPayloadBytes = sum(selectedPayload);
details.referenceIncrementalPayloadBytes = sum(referencePayload);
details.requestedIncrementalPayloadBytes = sum(requestedPayload);
details.minimumFeasibleIncrementalPayloadBytes = ...
    sum(minimumPayload);
details.maximumIncrementalPayloadBytes = sum(maximumPayload);
details.allRequestedPayloadTargetsFeasible = ...
    all(requestedFeasible);
details.anyPayloadFeasibilityFloorActive = any(floorActive);
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function [bestSources, bestDuplicates, ...
        bestObjective, bestMaximumLoad, payloadDetails] = ...
    solveFormationAssignment( ...
        members, baselineSources, physical, scoreMatrix, ...
        duplicateCount, maximumSourceLoad, options)
groupSize = numel(members);
if duplicateCount < 0 || duplicateCount >= groupSize
    error('Dominant duplicate-count constraint is invalid.');
end
duplicateSets = nchoosek(members, duplicateCount);
bestObjective = -inf;
bestSources = nan(1, groupSize);
bestDuplicates = zeros(1, duplicateCount);
bestMaximumLoad = inf;
payloadConstraintEnforced = logical(getField( ...
    options, 'payloadConstraintEnforced', false));
senderPayloadBytes = reshape(getField( ...
    options, 'senderPayloadBytes', []), 1, []);
referenceSources = reshape(getField( ...
    options, 'referenceSources', []), 1, []);
payloadMarginFraction = getField( ...
    options, 'payloadMarginFraction', 0);
if payloadConstraintEnforced
    if numel(senderPayloadBytes) < max(members) || ...
            numel(referenceSources) < max(members) || ...
            any(~isfinite(senderPayloadBytes(members))) || ...
            any(senderPayloadBytes(members) < 0) || ...
            any(~ismember(referenceSources(members), members))
        error('Payload-aware formation metadata is invalid.');
    end
    referenceNonduplicates = ...
        referenceSources(members) ~= baselineSources(members);
    if nnz(referenceNonduplicates) ~= ...
            groupSize - duplicateCount
        error(['Payload reference does not preserve the registered ', ...
            'duplicate count.']);
    end
    referenceIncrementalPayloadBytes = sum( ...
        senderPayloadBytes( ...
            referenceSources(members(referenceNonduplicates))));
    requestedIncrementalPayloadBytes = ...
        (1 - payloadMarginFraction) * ...
            referenceIncrementalPayloadBytes;
else
    referenceIncrementalPayloadBytes = NaN;
    requestedIncrementalPayloadBytes = inf;
end
minimumPayload = inf;
minimumPayloadObjective = -inf;
minimumPayloadSources = nan(1, groupSize);
minimumPayloadDuplicates = zeros(1, duplicateCount);
minimumPayloadMaximumLoad = inf;
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
        nonduplicateMask = ...
            groupSources ~= baselineSources(members);
        incrementalPayload = 0;
        if payloadConstraintEnforced
            incrementalPayload = sum( ...
                senderPayloadBytes( ...
                    groupSources(nonduplicateMask)));
            if incrementalPayload < minimumPayload - 1e-9 || ...
                    (abs(incrementalPayload - minimumPayload) <= 1e-9 && ...
                     objective > minimumPayloadObjective + 1e-12)
                minimumPayload = incrementalPayload;
                minimumPayloadObjective = objective;
                minimumPayloadSources = groupSources;
                minimumPayloadDuplicates = duplicateSet;
                minimumPayloadMaximumLoad = maximumLoad;
            end
            if incrementalPayload > ...
                    requestedIncrementalPayloadBytes + 1e-9
                continue;
            end
        end
        if objective > bestObjective + 1e-12 || ...
                (abs(objective - bestObjective) <= 1e-12 && ...
                 payloadConstraintEnforced && ...
                 incrementalPayload < ...
                    selectedIncrementalPayloadBytes( ...
                        bestSources, baselineSources(members), ...
                        senderPayloadBytes) - 1e-9)
            bestObjective = objective;
            bestSources = groupSources;
            bestDuplicates = duplicateSet;
            bestMaximumLoad = maximumLoad;
        end
    end
end
requestedPayloadTargetFeasible = isfinite(bestObjective);
payloadFeasibilityFloorActive = false;
if payloadConstraintEnforced && ~requestedPayloadTargetFeasible && ...
        isfinite(minimumPayload)
    bestObjective = minimumPayloadObjective;
    bestSources = minimumPayloadSources;
    bestDuplicates = minimumPayloadDuplicates;
    bestMaximumLoad = minimumPayloadMaximumLoad;
    payloadFeasibilityFloorActive = true;
end
if ~isfinite(bestObjective) || any(~isfinite(bestSources))
    error('DominantRoute:Infeasible', ...
        'No exact duplicate/load-constrained dominant route exists.');
end
if payloadConstraintEnforced
    selectedPayload = selectedIncrementalPayloadBytes( ...
        bestSources, baselineSources(members), senderPayloadBytes);
    maximumPayload = max( ...
        requestedIncrementalPayloadBytes, minimumPayload);
else
    selectedPayload = NaN;
    minimumPayload = NaN;
    maximumPayload = inf;
    requestedPayloadTargetFeasible = true;
end
payloadDetails = struct( ...
    'selectedIncrementalPayloadBytes', selectedPayload, ...
    'referenceIncrementalPayloadBytes', ...
        referenceIncrementalPayloadBytes, ...
    'requestedIncrementalPayloadBytes', ...
        requestedIncrementalPayloadBytes, ...
    'minimumFeasibleIncrementalPayloadBytes', minimumPayload, ...
    'maximumIncrementalPayloadBytes', maximumPayload, ...
    'requestedPayloadTargetFeasible', ...
        requestedPayloadTargetFeasible, ...
    'payloadFeasibilityFloorActive', ...
        payloadFeasibilityFloorActive);
end

function maximumSourceLoad = resolveMaximumSourceLoad( ...
        mode, options, protocol)
if strncmp(mode, 'composite-balanced', ...
        numel('composite-balanced'))
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

function senderBytes = estimateSenderPayloadBytes(context, options)
nodeCount = numel(context.localPosteriorBySensor);
senderBytes = reshape(getField( ...
    options, 'senderPayloadBytes', []), 1, []);
if isempty(senderBytes)
    senderBytes = zeros(1, nodeCount);
    for senderIdx = 1:nodeCount
        stats = estimateLmbPayloadSize( ...
            context.localPosteriorBySensor{senderIdx}, ...
            context.model, 2, struct());
        senderBytes(senderIdx) = stats.estimatedBytes;
    end
elseif numel(senderBytes) ~= nodeCount || ...
        any(~isfinite(senderBytes)) || any(senderBytes < 0)
    error(['Injected senderPayloadBytes must contain one finite ', ...
        'nonnegative value per sensor.']);
end
end

function bytes = topologyPayloadBytes(adjacency, senderPayloadBytes)
bytes = sum(sum(adjacency, 1) .* senderPayloadBytes);
end

function bytes = selectedIncrementalPayloadBytes( ...
        sources, baselineSources, senderPayloadBytes)
if any(~isfinite(sources))
    bytes = inf;
    return;
end
nonduplicateMask = sources ~= baselineSources;
bytes = sum(senderPayloadBytes(sources(nonduplicateMask)));
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
