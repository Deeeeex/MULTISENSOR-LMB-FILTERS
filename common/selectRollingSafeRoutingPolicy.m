function [adjacency, details] = ...
    selectRollingSafeRoutingPolicy(context, mode, options)
% SELECTROLLINGSAFEROUTINGPOLICY Rolling-B=3 directed routing policy.
%
% Every receiver gets exactly one source at a fixed KLA weight. The default
% intra-formation source map is a directed cycle. Optional cross-formation
% edges replace, rather than add to, the cycle edge entering a receiver.
%
% Safety has three layers:
%   1. every mature three-step formation union is strongly connected;
%   2. every mature three-step sensor union is strongly connected;
%   3. the selected state admits the next action in a registered period-3
%      reserve schedule, conditional on the current physical action set.
%
% Supported modes:
%   scheduled-burst   execute the registered tree/edge/empty reserve
%   scheduled-chunk   execute a constant-quota cyclic-chunk reserve
%   link-aware        optimize observable link-reliability advantage
%   posterior-analytic optimize truth-free posterior/link features
%   external-scores   use an injected score matrix or score callback

if nargin < 2 || isempty(mode)
    mode = 'posterior-analytic';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
sourceWeight = getField(options, 'sourceWeight', 0.70);
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('Rolling-safe sourceWeight must lie strictly inside (0,1).');
end
windowLength = max(2, round(getField( ...
    options, 'connectivityWindowLength', 3)));
if windowLength ~= 3
    error('The registered rolling-safe policy currently requires B=3.');
end

nodeCount = numel(context.localPosteriorBySensor);
groupIds = resolveGroupIds(context.model, nodeCount);
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
if groupCount < 2
    error('Rolling-safe routing needs at least two formations.');
end
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;
history = resolvePolicyHistory(context, nodeCount, windowLength);

[baselineAdjacency, baselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-balanced-cycle', ...
        struct('sourceWeight', sourceWeight, 'phase', 1));
baselineSources = reshape( ...
    baselineDetails.selectedSourcesByReceiver, 1, []);
if numel(baselineSources) ~= nodeCount || ...
        any(~isfinite(baselineSources)) || ...
        nnz(baselineAdjacency) ~= nodeCount
    error('Rolling-safe cycle backbone must cover every receiver.');
end

reserveOptions = struct( ...
    'anchorTime', getField(options, 'anchorTime', 1), ...
    'temporalPhase', getField(options, 'temporalPhase', 0), ...
    'rootFormation', getField(options, 'rootFormation', 1), ...
    'orientation', getField(options, 'orientation', 'clockwise'), ...
    'scheduleType', getField(options, ...
        'reserveScheduleType', 'burst'), ...
    'quota', getField(options, 'quota', groupCount - 1), ...
    'formationPhase', getField(options, 'formationPhase', 0));
[currentReserveFormation, nextReserveFormation, reserveDetails] = ...
    buildRollingReserveSchedule( ...
        groupCount, context.currentTime, reserveOptions);
recentFormationHistory = formationHistoryFromSensorHistory( ...
    history, groupIds);
forbiddenReceivers = receiversRequiringCycleRestoration( ...
    history, baselineSources, windowLength);

[receiverIndices, senderIndices] = ...
    enumeratePhysicalCrossEdges(physical, groupIds);
[scores, scoreDetails] = scoreCrossEdges( ...
    context, mode, options, receiverIndices, senderIndices, ...
    baselineSources, currentReserveFormation, groupIds);
[payloadDelta, baselinePayloadBytes, senderPayloadBytes] = ...
    computePayloadDeltas( ...
        context, receiverIndices, senderIndices, baselineSources);
payloadToleranceFraction = max(0, getField( ...
    options, 'payloadToleranceFraction', 0.02));
maximumPayloadIncreaseBytes = ...
    payloadToleranceFraction * baselinePayloadBytes;

projectionOptions = struct( ...
    'maximumCrossEdges', groupCount - 1, ...
    'connectivityWindowLength', windowLength, ...
    'recentFormationAdjacency', recentFormationHistory, ...
    'nextReserveFormationAdjacency', nextReserveFormation, ...
    'forbiddenReceiverIndices', forbiddenReceivers, ...
    'payloadDeltaByExample', payloadDelta, ...
    'maximumPayloadIncreaseBytes', maximumPayloadIncreaseBytes, ...
    'solverTimeLimitSeconds', getField( ...
        options, 'solverTimeLimitSeconds', 10));
if ismember(mode, {'scheduled-burst', 'scheduled-chunk'})
    projectionOptions.allowedFormationAdjacency = ...
        currentReserveFormation;
    projectionOptions.requiredFormationAdjacency = ...
        currentReserveFormation;
    projectionOptions.maximumCrossEdges = ...
        nnz(currentReserveFormation);
end

payloadEmergencyUsed = false;
payloadConstrainedError = [];
try
    selection = selectRollingFormationMatchingEdges( ...
        groupIds, receiverIndices, senderIndices, scores, ...
        projectionOptions);
catch errorInfo
    payloadConstrainedError = errorInfo;
    topologyOnlyOptions = projectionOptions;
    topologyOnlyOptions.maximumPayloadIncreaseBytes = inf;
    topologyOnlyFeasible = false;
    try
        topologyOnlySelection = ...
            selectRollingFormationMatchingEdges( ...
                groupIds, receiverIndices, senderIndices, scores, ...
                topologyOnlyOptions);
        topologyOnlyFeasible = true;
    catch
        topologyOnlySelection = struct(); %#ok<NASGU>
    end
    allowEmergency = logical(getField( ...
        options, 'allowEmergencyPayloadViolation', false));
    if topologyOnlyFeasible && allowEmergency
        selection = topologyOnlySelection;
        payloadEmergencyUsed = true;
    else
        rethrow(payloadConstrainedError);
    end
end

selectedSources = baselineSources;
selectedSources(selection.receiverIndices) = ...
    selection.senderIndices;
[adjacency, fusionWeights] = buildSelectedRoute( ...
    selectedSources, physical, sourceWeight);

sensorWindowMature = size(history, 3) >= windowLength - 1;
sensorWindowUnion = adjacency;
if ~isempty(history)
    sensorWindowUnion = sensorWindowUnion | any(history, 3);
end
sensorWindowStrong = ~sensorWindowMature || ...
    isStronglyConnected(adjacencyToSenderOrientation( ...
        sensorWindowUnion));
if ~sensorWindowStrong
    error(['Rolling-safe projection passed formation cuts but failed ', ...
        'the independent sensor-level window check.']);
end

[nextReserveAdjacency, nextReserveSelection] = ...
    constructViableReserveSuccessor( ...
        context, groupIds, physical, baselineSources, ...
        history, adjacency, nextReserveFormation, ...
        sourceWeight, windowLength, reserveOptions);
successorWindow = adjacency | nextReserveAdjacency;
if ~isempty(history)
    successorWindow = successorWindow | history(:, :, end);
end
successorSensorStrong = isStronglyConnected( ...
    adjacencyToSenderOrientation(successorWindow));
if ~successorSensorStrong
    error(['Registered reserve successor failed the independent ', ...
        'sensor-level viability check.']);
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nnz(adjacency) ~= nodeCount || nodeCount > messageBudget
    error(['Rolling-safe routing must send exactly one message per ', ...
        'receiver within the directed-message budget.']);
end
realizedPayloadDelta = ...
    sum(senderPayloadBytes(selectedSources)) - baselinePayloadBytes;
payloadLimitPassed = realizedPayloadDelta <= ...
    maximumPayloadIncreaseBytes + 1e-7;
if ~payloadLimitPassed && ~payloadEmergencyUsed
    error('Rolling-safe routing violated its payload contract.');
end

selectedFormation = selection.formationAdjacency;
proposalPairs = find(currentReserveFormation);
selectedPairs = find(selectedFormation);
keptPairCount = numel(intersect(proposalPairs, selectedPairs));
details = struct();
details.mode = ['rolling-safe-', mode];
details.objective = -selection.predictedObjective;
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = finiteSpread(scores);
details.validCandidateCount = numel(scores);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = ...
    sourceWeight * ones(1, nodeCount);
details.baselineSourcesByReceiver = baselineSources;
details.baselineAdjacency = baselineAdjacency;
details.backboneMode = 'fixed-balanced-cycle-phase1';
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
details.overrideMask = selectedSources ~= baselineSources;
details.overrideFraction = mean(details.overrideMask);
details.crossFormationMessageCount = selection.crossEdgeCount;
details.maximumCrossEdges = groupCount - 1;
details.maximumCrossSourceLoad = ...
    selection.maximumCrossSourceLoad;
details.maximumCrossReceiverLoad = ...
    selection.maximumCrossReceiverLoad;
details.proposalCrossCount = nnz(currentReserveFormation);
details.realizedCrossCount = selection.crossEdgeCount;
details.repairKeptCount = keptPairCount;
details.repairDroppedCount = ...
    numel(proposalPairs) - keptPairCount;
details.repairAddedCount = ...
    numel(selectedPairs) - keptPairCount;
details.repairTriggered = ...
    ~isequal(currentReserveFormation, selectedFormation);
details.currentReserveFormationAdjacency = ...
    currentReserveFormation;
details.nextReserveFormationAdjacency = ...
    nextReserveFormation;
details.reserveSchedule = reserveDetails;
details.formationAdjacency = selectedFormation;
details.formationWindowAdjacency = ...
    selection.windowFormationAdjacency;
details.formationWindowMature = selection.windowMature;
details.formationWindowStrongConnected = ...
    selection.rollingUnionStrongConnected;
details.reserveViabilityEnforced = ...
    selection.reserveViabilityEnforced;
details.successorFormationStrongConnected = ...
    selection.successorReserveViable;
details.sensorWindowMature = sensorWindowMature;
details.sensorWindowStrongConnected = sensorWindowStrong;
details.successorSensorStrongConnected = ...
    successorSensorStrong;
details.nextReserveAdjacency = nextReserveAdjacency;
details.nextReserveCrossCount = ...
    nextReserveSelection.crossEdgeCount;
details.previousAdjacencyHistoryCount = size(history, 3);
details.previousAdjacencyHistoryTimes = getField( ...
    context, 'previousAdjacencyHistoryTimes', []);
details.cycleRestorationReceiverCount = ...
    numel(forbiddenReceivers);
details.cycleRestorationReceivers = forbiddenReceivers;
details.baselinePayloadBytes = baselinePayloadBytes;
details.selectedPayloadDeltaBytes = realizedPayloadDelta;
details.maximumPayloadIncreaseBytes = ...
    maximumPayloadIncreaseBytes;
details.payloadToleranceFraction = payloadToleranceFraction;
details.payloadLimitPassed = payloadLimitPassed;
details.payloadEmergencyUsed = payloadEmergencyUsed;
details.scoreDetails = scoreDetails;
details.posteriorUsed = ismember(mode, ...
    {'posterior-analytic', 'external-scores'}) && ...
    logical(getField(scoreDetails, 'posteriorUsed', true));
details.truthUsed = logical(getField( ...
    options, 'truthUsed', false));
details.currentLinkReliabilityUsed = ismember(mode, ...
    {'link-aware', 'posterior-analytic'});
details.currentPhysicalActionSetUsed = true;
details.reserveFeasibilityCondition = ...
    'current-physical-action-set';
details.topologyInfeasible = false;
end

function [scores, details] = scoreCrossEdges( ...
        context, mode, options, receiverIndices, senderIndices, ...
        baselineSources, currentReserveFormation, groupIds)
exampleCount = numel(receiverIndices);
scores = zeros(exampleCount, 1);
details = struct('posteriorUsed', false);
switch mode
    case {'scheduled-burst', 'scheduled-chunk'}
        groups = unique(groupIds, 'stable');
        endpointRound = floor((context.currentTime - ...
            getField(options, 'anchorTime', 1)) / 3);
        for exampleIdx = 1:exampleCount
            receiverIdx = receiverIndices(exampleIdx);
            senderIdx = senderIndices(exampleIdx);
            senderGroup = find(groups == groupIds(senderIdx), 1);
            receiverGroup = find(groups == groupIds(receiverIdx), 1);
            senderMembers = find(groupIds == groupIds(senderIdx));
            receiverMembers = find(groupIds == groupIds(receiverIdx));
            senderRole = find(senderMembers == senderIdx, 1);
            receiverRole = find(receiverMembers == receiverIdx, 1);
            targetSenderRole = 1 + mod( ...
                endpointRound + senderGroup - 1, ...
                numel(senderMembers));
            targetReceiverRole = 1 + mod( ...
                endpointRound + receiverGroup - 1, ...
                numel(receiverMembers));
            scores(exampleIdx) = ...
                10 * currentReserveFormation( ...
                    senderGroup, receiverGroup) + ...
                0.1 * (senderRole == targetSenderRole) + ...
                0.01 * (receiverRole == targetReceiverRole) - ...
                1e-8 * senderIdx - 1e-10 * receiverIdx;
        end
    case 'link-aware'
        for exampleIdx = 1:exampleCount
            receiverIdx = receiverIndices(exampleIdx);
            senderIdx = senderIndices(exampleIdx);
            crossReliability = max(1e-6, 1 - edgeDrop( ...
                context.commConfig, senderIdx, receiverIdx, ...
                context.currentTime));
            baselineReliability = max(1e-6, 1 - edgeDrop( ...
                context.commConfig, baselineSources(receiverIdx), ...
                receiverIdx, context.currentTime));
            scores(exampleIdx) = ...
                log(crossReliability) - log(baselineReliability);
        end
    case 'posterior-analytic'
        [features, featureNames] = ...
            computeDirectedRoutingFeatures(context);
        reliabilityIdx = requireFeature( ...
            featureNames, 'link_reliability');
        qualityIdx = requireFeature( ...
            featureNames, 'node_quality_advantage');
        discrepancyIdx = requireFeature( ...
            featureNames, 'normalized_state_discrepancy');
        reliabilityWeight = getField( ...
            options, 'reliabilityWeight', 1.0);
        qualityWeight = getField( ...
            options, 'qualityWeight', 0.5);
        discrepancyWeight = getField( ...
            options, 'discrepancyWeight', 0.25);
        for exampleIdx = 1:exampleCount
            receiverIdx = receiverIndices(exampleIdx);
            senderIdx = senderIndices(exampleIdx);
            scores(exampleIdx) = ...
                reliabilityWeight * ...
                    features(receiverIdx, senderIdx, reliabilityIdx) + ...
                qualityWeight * ...
                    features(receiverIdx, senderIdx, qualityIdx) - ...
                discrepancyWeight * ...
                    features(receiverIdx, senderIdx, discrepancyIdx);
        end
        details.posteriorUsed = true;
        details.featureNames = { ...
            'link_reliability', ...
            'node_quality_advantage', ...
            'normalized_state_discrepancy'};
        details.coefficients = [ ...
            reliabilityWeight, qualityWeight, -discrepancyWeight];
    case 'external-scores'
        if isfield(options, 'edgeScoreFcn') && ...
                ~isempty(options.edgeScoreFcn)
            scores = options.edgeScoreFcn( ...
                context, receiverIndices, senderIndices, ...
                baselineSources);
        elseif isfield(options, 'edgeScoreMatrix')
            scoreMatrix = options.edgeScoreMatrix;
            if ~isequal(size(scoreMatrix), ...
                    [numel(baselineSources), numel(baselineSources)])
                error('edgeScoreMatrix must be S-by-S.');
            end
            scores = zeros(exampleCount, 1);
            for exampleIdx = 1:exampleCount
                scores(exampleIdx) = scoreMatrix( ...
                    receiverIndices(exampleIdx), ...
                    senderIndices(exampleIdx));
            end
        else
            error(['external-scores mode requires edgeScoreFcn or ', ...
                'edgeScoreMatrix.']);
        end
        scores = reshape(scores, [], 1);
        if numel(scores) ~= exampleCount
            error('Injected rolling-safe scores have the wrong length.');
        end
        details.posteriorUsed = logical(getField( ...
            options, 'posteriorUsed', true));
    otherwise
        error('Unknown rolling-safe routing mode: %s', mode);
end
scores(~isfinite(scores)) = -inf;
end

function [payloadDelta, baselineBytes, senderBytes] = ...
    computePayloadDeltas( ...
        context, receiverIndices, senderIndices, baselineSources)
nodeCount = numel(baselineSources);
senderBytes = zeros(1, nodeCount);
for senderIdx = 1:nodeCount
    stats = estimateLmbPayloadSize( ...
        context.localPosteriorBySensor{senderIdx}, ...
        context.model, 2, struct());
    senderBytes(senderIdx) = stats.estimatedBytes;
end
baselineBytes = sum(senderBytes(baselineSources));
payloadDelta = zeros(numel(receiverIndices), 1);
for exampleIdx = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(exampleIdx);
    payloadDelta(exampleIdx) = ...
        senderBytes(senderIndices(exampleIdx)) - ...
        senderBytes(baselineSources(receiverIdx));
end
end

function [adjacency, fusionWeights] = buildSelectedRoute( ...
        selectedSources, physical, sourceWeight)
nodeCount = numel(selectedSources);
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error('Rolling-safe routing selected a non-physical route.');
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
end
end

function [reserveAdjacency, reserveSelection] = ...
    constructViableReserveSuccessor( ...
        context, groupIds, physical, baselineSources, ...
        history, currentAdjacency, nextReserveFormation, ...
        sourceWeight, windowLength, reserveOptions)
nodeCount = numel(groupIds);
successorHistory = currentAdjacency;
if ~isempty(history)
    successorHistory = cat(3, ...
        history(:, :, end), currentAdjacency);
else
    successorHistory = reshape( ...
        currentAdjacency, nodeCount, nodeCount, 1);
end
if size(successorHistory, 3) > windowLength - 1
    successorHistory = successorHistory(:, :, ...
        end-windowLength+2:end);
end
forbidden = receiversRequiringCycleRestoration( ...
    successorHistory, baselineSources, windowLength);
[receiverIndices, senderIndices] = ...
    enumeratePhysicalCrossEdges(physical, groupIds);
scores = deterministicEndpointScores( ...
    groupIds, receiverIndices, senderIndices, ...
    context.currentTime + 1, reserveOptions);
selectionOptions = struct( ...
    'maximumCrossEdges', nnz(nextReserveFormation), ...
    'connectivityWindowLength', windowLength, ...
    'requireMatureWindow', false, ...
    'allowedFormationAdjacency', nextReserveFormation, ...
    'requiredFormationAdjacency', nextReserveFormation, ...
    'forbiddenReceiverIndices', forbidden, ...
    'maximumPayloadIncreaseBytes', inf);
reserveSelection = selectRollingFormationMatchingEdges( ...
    groupIds, receiverIndices, senderIndices, ...
    scores, selectionOptions);
reserveSources = baselineSources;
reserveSources(reserveSelection.receiverIndices) = ...
    reserveSelection.senderIndices;
[reserveAdjacency, ~] = buildSelectedRoute( ...
    reserveSources, physical, sourceWeight);
end

function scores = deterministicEndpointScores( ...
        groupIds, receiverIndices, senderIndices, ...
        currentTime, options)
groups = unique(groupIds, 'stable');
endpointRound = floor((currentTime - ...
    getField(options, 'anchorTime', 1)) / 3);
scores = zeros(numel(receiverIndices), 1);
for exampleIdx = 1:numel(receiverIndices)
    receiverIdx = receiverIndices(exampleIdx);
    senderIdx = senderIndices(exampleIdx);
    senderGroup = find(groups == groupIds(senderIdx), 1);
    receiverGroup = find(groups == groupIds(receiverIdx), 1);
    senderMembers = find(groupIds == groupIds(senderIdx));
    receiverMembers = find(groupIds == groupIds(receiverIdx));
    senderRole = find(senderMembers == senderIdx, 1);
    receiverRole = find(receiverMembers == receiverIdx, 1);
    targetSenderRole = 1 + mod( ...
        endpointRound + senderGroup - 1, numel(senderMembers));
    targetReceiverRole = 1 + mod( ...
        endpointRound + receiverGroup - 1, ...
        numel(receiverMembers));
    scores(exampleIdx) = ...
        0.1 * (senderRole == targetSenderRole) + ...
        0.01 * (receiverRole == targetReceiverRole) - ...
        1e-8 * senderIdx - 1e-10 * receiverIdx;
end
end

function history = resolvePolicyHistory(context, nodeCount, windowLength)
history = getField(context, 'previousAdjacencyHistory', ...
    false(nodeCount, nodeCount, 0));
if isempty(history)
    history = false(nodeCount, nodeCount, 0);
elseif ndims(history) < 3
    history = reshape(history, nodeCount, nodeCount, 1);
end
if size(history, 1) ~= nodeCount || ...
        size(history, 2) ~= nodeCount || ndims(history) > 3
    error('previousAdjacencyHistory must be S-by-S-by-H.');
end
history = logical(history);
if size(history, 3) > windowLength - 1
    history = history(:, :, end-windowLength+2:end);
end
end

function formationHistory = ...
    formationHistoryFromSensorHistory(history, groupIds)
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
formationHistory = false( ...
    groupCount, groupCount, size(history, 3));
for historyIdx = 1:size(history, 3)
    policyAdjacency = history(:, :, historyIdx);
    for receiverIdx = 1:numel(groupIds)
        receiverGroup = find(groups == groupIds(receiverIdx), 1);
        for senderIdx = reshape( ...
                find(policyAdjacency(receiverIdx, :)), 1, [])
            senderGroup = find(groups == groupIds(senderIdx), 1);
            if senderGroup ~= receiverGroup
                formationHistory( ...
                    senderGroup, receiverGroup, historyIdx) = true;
            end
        end
    end
end
end

function receivers = receiversRequiringCycleRestoration( ...
        history, baselineSources, windowLength)
if size(history, 3) < windowLength - 1
    receivers = [];
    return;
end
receivers = zeros(1, 0);
for receiverIdx = 1:numel(baselineSources)
    senderIdx = baselineSources(receiverIdx);
    present = reshape(history( ...
        receiverIdx, senderIdx, :), 1, []);
    if ~any(present)
        receivers(end + 1) = receiverIdx; %#ok<AGROW>
    end
end
end

function [receiverIndices, senderIndices] = ...
    enumeratePhysicalCrossEdges(physical, groupIds)
[receiverIndices, senderIndices] = find(physical);
crossMask = groupIds(receiverIndices) ~= groupIds(senderIndices);
receiverIndices = receiverIndices(crossMask);
senderIndices = senderIndices(crossMask);
end

function [currentAdjacency, nextAdjacency, details] = ...
    buildRollingReserveSchedule(groupCount, currentTime, options)
anchorTime = round(getField(options, 'anchorTime', 1));
scheduleType = lower(char(getField( ...
    options, 'scheduleType', 'burst')));
temporalPhase = mod(round(getField( ...
    options, 'temporalPhase', 0)), 3);
rootFormation = 1 + mod(round(getField( ...
    options, 'rootFormation', 1)) - 1, groupCount);
orientation = lower(char(getField( ...
    options, 'orientation', 'clockwise')));
cycleAdjacency = false(groupCount);
cycleSenders = zeros(1, groupCount);
cycleReceivers = zeros(1, groupCount);
for senderFormation = 1:groupCount
    if strcmp(orientation, 'clockwise')
        receiverFormation = ...
            1 + mod(senderFormation, groupCount);
    elseif strcmp(orientation, 'counter-clockwise') || ...
            strcmp(orientation, 'counterclockwise')
        receiverFormation = ...
            1 + mod(senderFormation - 2, groupCount);
    else
        error('Unknown rolling reserve orientation: %s', orientation);
    end
    cycleAdjacency(senderFormation, receiverFormation) = true;
    cycleSenders(senderFormation) = senderFormation;
    cycleReceivers(senderFormation) = receiverFormation;
end
if strcmp(scheduleType, 'burst')
    incomingSender = find(cycleAdjacency(:, rootFormation), 1);
    missingEdge = false(groupCount);
    missingEdge(incomingSender, rootFormation) = true;
    tree = cycleAdjacency & ~missingEdge;
    slot = mod(currentTime - anchorTime + temporalPhase, 3);
    nextSlot = mod(slot + 1, 3);
    currentAdjacency = reserveSlot(slot, tree, missingEdge);
    nextAdjacency = reserveSlot(nextSlot, tree, missingEdge);
    quota = NaN;
    formationPhase = NaN;
elseif strcmp(scheduleType, 'cyclic-chunk')
    quota = round(getField(options, 'quota', groupCount - 1));
    minimumQuota = ceil(groupCount / 3);
    if quota < minimumQuota || quota > groupCount - 1
        error(['Cyclic-chunk quota must lie between ceil(G/3) ', ...
            'and G-1.']);
    end
    formationPhase = mod(round(getField( ...
        options, 'formationPhase', 0)), groupCount);
    currentAdjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime, anchorTime, groupCount);
    nextAdjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime + 1, anchorTime, groupCount);
    tree = false(groupCount);
    missingEdge = false(groupCount);
    slot = NaN;
    nextSlot = NaN;
else
    error('Unknown rolling reserve schedule type: %s', scheduleType);
end
details = struct( ...
    'scheduleType', scheduleType, ...
    'anchorTime', anchorTime, ...
    'temporalPhase', temporalPhase, ...
    'rootFormation', rootFormation, ...
    'orientation', orientation, ...
    'slot', slot, ...
    'nextSlot', nextSlot, ...
    'quota', quota, ...
    'formationPhase', formationPhase, ...
    'cycleAdjacency', cycleAdjacency, ...
    'treeAdjacency', tree, ...
    'missingEdgeAdjacency', missingEdge);
end

function adjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime, anchorTime, groupCount)
startPosition = mod( ...
    formationPhase + quota * (currentTime - anchorTime), ...
    groupCount);
positions = 1 + mod( ...
    startPosition + (0:(quota - 1)), groupCount);
adjacency = false(groupCount);
for position = positions
    adjacency(cycleSenders(position), ...
        cycleReceivers(position)) = true;
end
end

function adjacency = reserveSlot(slot, tree, missingEdge)
switch slot
    case 0
        adjacency = tree;
    case 1
        adjacency = missingEdge;
    otherwise
        adjacency = false(size(tree));
end
end

function senderAdjacency = ...
    adjacencyToSenderOrientation(policyAdjacency)
senderAdjacency = policyAdjacency';
end

function valid = isStronglyConnected(senderAdjacency)
nodeCount = size(senderAdjacency, 1);
valid = all(reachableFrom(senderAdjacency, 1)) && ...
    all(reachableFrom(senderAdjacency', 1));
end

function reached = reachableFrom(adjacency, startNode)
reached = false(1, size(adjacency, 1));
frontier = startNode;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if reached(node)
        continue;
    end
    reached(node) = true;
    next = find(adjacency(node, :) & ~reached);
    frontier = [frontier, reshape(next, 1, [])]; %#ok<AGROW>
end
end

function probability = edgeDrop( ...
        config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function idx = requireFeature(names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('Missing directed-routing feature: %s', name);
end
end

function spread = finiteSpread(values)
finiteValues = values(isfinite(values));
if isempty(finiteValues)
    spread = NaN;
else
    spread = max(finiteValues) - min(finiteValues);
end
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('Rolling-safe routing needs sensorGroupIds metadata.');
end
groupIds = reshape(model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds))
    error('Rolling-safe sensorGroupIds are invalid.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
