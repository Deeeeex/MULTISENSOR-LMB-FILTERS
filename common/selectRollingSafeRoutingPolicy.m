function [adjacency, details] = ...
    selectRollingSafeRoutingPolicy(context, mode, options)
% SELECTROLLINGSAFEROUTINGPOLICY Rolling-B=3 directed routing policy.
%
% Every receiver gets exactly one source at a fixed KLA weight. The default
% intra-formation source map is a directed cycle. Optional cross-formation
% edges replace, rather than add to, the cycle edge entering a receiver.
%
% Safety has three auditable layers:
%   1. every mature three-step formation union is strongly connected;
%   2. every mature three-step sensor union is strongly connected;
%   3. once the successor B=3 window is mature, the selected state admits
%      one successor under the current physical geometry and, when enabled,
%      the current posterior-payload projection cap.
%
% Layer 3 is deliberately not called recursive feasibility: future geometry,
% future payloads and a terminal invariant set are not available online.
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
    'formationPhase', getField(options, 'formationPhase', 0), ...
    'solverTimeLimitSeconds', getField( ...
        options, 'solverTimeLimitSeconds', 10));
[currentReserveFormation, nextReserveFormation, reserveDetails] = ...
    buildRollingReserveSchedule( ...
        groupCount, context.currentTime, reserveOptions);
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
payloadToleranceFraction = getField( ...
    options, 'payloadToleranceFraction', inf);
if ~isscalar(payloadToleranceFraction) || ...
        isnan(payloadToleranceFraction) || ...
        payloadToleranceFraction < 0
    error('payloadToleranceFraction must be nonnegative or inf.');
end
payloadConstraintEnforced = isfinite(payloadToleranceFraction);
if payloadConstraintEnforced
    maximumPayloadIncreaseBytes = ...
        payloadToleranceFraction * baselinePayloadBytes;
else
    maximumPayloadIncreaseBytes = inf;
end

successorEndpointScores = deterministicEndpointScores( ...
    groupIds, receiverIndices, senderIndices, ...
    context.currentTime + 1, reserveOptions);
successorRepairScores = buildMinimumEditScores( ...
    successorEndpointScores, receiverIndices, senderIndices, ...
    groupIds, nextReserveFormation, groupCount);
jointOptions = struct( ...
    'maximumCurrentCrossEdges', groupCount - 1, ...
    'maximumSuccessorCrossEdges', groupCount - 1, ...
    'connectivityWindowLength', windowLength, ...
    'recentSensorAdjacency', history, ...
    'baselineSources', baselineSources, ...
    'payloadDeltaByExample', payloadDelta, ...
    'maximumCurrentPayloadIncreaseBytes', ...
        maximumPayloadIncreaseBytes, ...
    'maximumSuccessorPayloadIncreaseBytes', ...
        maximumPayloadIncreaseBytes, ...
    'solverTimeLimitSeconds', getField( ...
        options, 'solverTimeLimitSeconds', 10));
scheduledMode = ismember( ...
    mode, {'scheduled-burst', 'scheduled-chunk'});
nominalProjectionFeasible = true;
repairProjectionAttempted = false;
nominalOptions = jointOptions;
if scheduledMode
    % Registered controls must execute their declared current/successor
    % schedule exactly whenever it is feasible.  Online score-based arms
    % have no scientific reason to inherit this arbitrary burst successor:
    % their witness is free to use the full safe action set.
    nominalOptions.successorAllowedFormationAdjacency = ...
        nextReserveFormation;
    nominalOptions.successorRequiredFormationAdjacency = ...
        nextReserveFormation;
    nominalOptions.maximumSuccessorCrossEdges = ...
        nnz(nextReserveFormation);
    nominalOptions.currentAllowedFormationAdjacency = ...
        currentReserveFormation;
    nominalOptions.currentRequiredFormationAdjacency = ...
        currentReserveFormation;
    nominalOptions.maximumCurrentCrossEdges = ...
        nnz(currentReserveFormation);
end
try
    jointSelection = ...
        selectRollingFormationMatchingWithSuccessor( ...
            groupIds, receiverIndices, senderIndices, ...
            scores, successorEndpointScores, nominalOptions);
    payloadEmergencyUsed = false;
catch errorInfo
    if ~isInfeasibleProjectionError(errorInfo)
        rethrow(errorInfo);
    end
    nominalProjectionFeasible = false;
    repairProjectionAttempted = true;
    flexibleScores = scores;
    if scheduledMode
        repairScores = buildMinimumEditScores( ...
            scores, receiverIndices, senderIndices, groupIds, ...
            currentReserveFormation, groupCount);
        flexibleScores = repairScores;
    end
    [jointSelection, payloadEmergencyUsed] = ...
        projectJointWithOptionalPayloadEmergency( ...
            groupIds, receiverIndices, senderIndices, ...
            flexibleScores, successorRepairScores, ...
            jointOptions, options);
end
selection = jointSelection.current;
nextReserveSelection = jointSelection.successor;

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

nextReserveSources = baselineSources;
nextReserveSources(nextReserveSelection.receiverIndices) = ...
    nextReserveSelection.senderIndices;
[nextReserveAdjacency, ~] = buildSelectedRoute( ...
    nextReserveSources, physical, sourceWeight);
nextReservePayloadDelta = ...
    sum(senderPayloadBytes(nextReserveSources)) - ...
    baselinePayloadBytes;
successorScheduleConstrained = scheduledMode;
successorScheduleRepairTriggered = ...
    successorScheduleConstrained && ...
    ~isequal(nextReserveSelection.formationAdjacency, ...
        nextReserveFormation);
nextReserveProjectionDetails = struct( ...
    'nominalProjectionFeasible', nominalProjectionFeasible, ...
    'scheduleConstrained', successorScheduleConstrained, ...
    'repairTriggered', successorScheduleRepairTriggered, ...
    'payloadLimitPassed', ...
        nextReservePayloadDelta <= ...
            maximumPayloadIncreaseBytes + 1e-7, ...
    'selectedPayloadDeltaBytes', ...
        nextReservePayloadDelta, ...
    'maximumPayloadIncreaseBytes', ...
        maximumPayloadIncreaseBytes);
successorWindow = adjacency | nextReserveAdjacency;
if ~isempty(history)
    successorWindow = successorWindow | history(:, :, end);
end
oneStepTopologyReserveChecked = ...
    jointSelection.successorWindowMature;
if oneStepTopologyReserveChecked
    successorSensorStrong = isStronglyConnected( ...
        adjacencyToSenderOrientation(successorWindow));
    if ~successorSensorStrong
        error(['Registered reserve successor failed the independent ', ...
            'sensor-level viability check.']);
    end
    successorFormationHistory = formationHistoryFromSensorHistory( ...
        cat(3, successorWindow), groupIds);
    successorFormationStrong = isStronglyConnected( ...
        any(successorFormationHistory, 3));
    if ~successorFormationStrong
        error(['Registered reserve successor failed the independent ', ...
            'formation-level viability check.']);
    end
else
    % Before two selected history pages exist, the hypothetical successor
    % still precedes the first mature B=3 window. Reporting a pass here
    % would create a phantom certificate, so the check is not applicable.
    successorSensorStrong = NaN;
    successorFormationStrong = NaN;
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
if scheduledMode
    proposalCrossCount = nnz(currentReserveFormation);
    repairKeptCount = keptPairCount;
    repairDroppedCount = numel(proposalPairs) - keptPairCount;
    repairAddedCount = numel(selectedPairs) - keptPairCount;
    repairTriggered = ...
        ~isequal(currentReserveFormation, selectedFormation);
else
    proposalCrossCount = NaN;
    repairKeptCount = NaN;
    repairDroppedCount = NaN;
    repairAddedCount = NaN;
    repairTriggered = false;
end
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
details.proposalCrossCount = proposalCrossCount;
details.realizedCrossCount = selection.crossEdgeCount;
details.repairKeptCount = repairKeptCount;
details.repairDroppedCount = repairDroppedCount;
details.repairAddedCount = repairAddedCount;
details.repairTriggered = repairTriggered;
details.nominalProjectionFeasible = ...
    nominalProjectionFeasible;
details.repairProjectionAttempted = ...
    repairProjectionAttempted;
details.currentReserveFormationAdjacency = ...
    currentReserveFormation;
details.nextReserveFormationAdjacency = ...
    nextReserveFormation;
details.successorScheduleConstrained = ...
    successorScheduleConstrained;
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
    successorFormationStrong;
details.sensorWindowMature = sensorWindowMature;
details.sensorWindowStrongConnected = sensorWindowStrong;
details.successorSensorStrongConnected = ...
    successorSensorStrong;
details.oneStepTopologyReserveChecked = ...
    oneStepTopologyReserveChecked;
details.oneStepTopologyReservePassed = ...
    double(oneStepTopologyReserveChecked && ...
        successorFormationStrong && successorSensorStrong);
if ~oneStepTopologyReserveChecked
    details.oneStepTopologyReservePassed = NaN;
end
details.oneStepJointProjectionUsed = true;
details.oneStepCurrentConnectivityCutCount = ...
    jointSelection.currentConnectivityCutCount;
details.oneStepSuccessorConnectivityCutCount = ...
    jointSelection.successorConnectivityCutCount;
details.oneStepCycleRestorationConstraintCount = ...
    jointSelection.cycleRestorationConstraintCount;
details.oneStepReserveNominalFeasible = ...
    nextReserveProjectionDetails.nominalProjectionFeasible;
details.oneStepReserveRepairTriggered = ...
    nextReserveProjectionDetails.repairTriggered;
details.oneStepReservePayloadLimitPassed = ...
    nextReserveProjectionDetails.payloadLimitPassed;
details.recursiveSafetyClaimed = false;
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
details.payloadConstraintEnforced = ...
    payloadConstraintEnforced;
details.payloadLimitPassed = payloadLimitPassed;
details.payloadEmergencyUsed = payloadEmergencyUsed;
details.scoreDetails = scoreDetails;
details.posteriorUsed = ismember(mode, ...
    {'posterior-analytic', 'external-scores'}) && ...
    logical(getField(scoreDetails, 'posteriorUsed', true));
details.posteriorPayloadMetadataUsed = ...
    payloadConstraintEnforced;
details.truthUsed = logical(getField( ...
    options, 'truthUsed', false));
details.currentLinkReliabilityUsed = ismember(mode, ...
    {'link-aware', 'posterior-analytic'}) || ...
    logical(getField(options, ...
        'currentLinkReliabilityUsed', false));
details.currentPhysicalActionSetUsed = true;
if payloadConstraintEnforced
    details.reserveFeasibilityCondition = ...
        ['one-step-current-geometry-and-current-posterior-', ...
         'payload-estimate'];
else
    details.reserveFeasibilityCondition = ...
        'one-step-current-geometry-topology-only';
end
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
        reserveOptions = struct( ...
            'anchorTime', getField(options, 'anchorTime', 1), ...
            'temporalPhase', getField(options, 'temporalPhase', 0), ...
            'rootFormation', getField(options, 'rootFormation', 1), ...
            'orientation', getField(options, ...
                'orientation', 'clockwise'), ...
            'scheduleType', getField(options, ...
                'reserveScheduleType', 'burst'), ...
            'quota', getField(options, 'quota', ...
                numel(unique(groupIds, 'stable')) - 1), ...
            'formationPhase', getField(options, ...
                'formationPhase', 0));
        scores = deterministicEndpointScores( ...
            groupIds, receiverIndices, senderIndices, ...
            context.currentTime, reserveOptions);
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
        details.truthUsed = logical(getField( ...
            options, 'truthUsed', false));
        details.currentLinkReliabilityUsed = logical(getField( ...
            options, 'currentLinkReliabilityUsed', false));
        requestedHorizons = getField(options, ...
            'teacherRequestedHorizonSteps', getField( ...
                options, 'teacherHorizonSteps', []));
        details.teacherRequestedHorizonSteps = ...
            requestedHorizons;
        details.teacherEffectiveHorizonSteps = ...
            resolveEffectiveTeacherHorizons( ...
                context, requestedHorizons);
    otherwise
        error('Unknown rolling-safe routing mode: %s', mode);
end
scores(~isfinite(scores)) = -inf;
end

function effective = resolveEffectiveTeacherHorizons( ...
        context, requested)
effective = requested;
if isempty(requested) || ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, ...
            'targetTrajectories') || ...
        isempty(context.model.dynamicTopologyScenario.targetTrajectories)
    return;
end
trajectories = ...
    context.model.dynamicTopologyScenario.targetTrajectories;
if ~iscell(trajectories)
    trajectories = {trajectories};
end
timeCount = min(cellfun(@(trajectory) ...
    size(trajectory, 2), trajectories));
effective = requested( ...
    context.currentTime + requested <= timeCount);
if isempty(effective)
    effective = 0;
end
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

function [selection, payloadEmergencyUsed] = ...
    projectJointWithOptionalPayloadEmergency( ...
        groupIds, receiverIndices, senderIndices, ...
        currentScores, successorScores, ...
        projectionOptions, policyOptions)
payloadEmergencyUsed = false;
try
    selection = selectRollingFormationMatchingWithSuccessor( ...
        groupIds, receiverIndices, senderIndices, ...
        currentScores, successorScores, projectionOptions);
    return;
catch constrainedError
    if ~isInfeasibleProjectionError(constrainedError)
        rethrow(constrainedError);
    end
end
topologyOnlyOptions = projectionOptions;
topologyOnlyOptions.maximumCurrentPayloadIncreaseBytes = inf;
topologyOnlyOptions.maximumSuccessorPayloadIncreaseBytes = inf;
try
    topologyOnlySelection = ...
        selectRollingFormationMatchingWithSuccessor( ...
            groupIds, receiverIndices, senderIndices, ...
            currentScores, successorScores, topologyOnlyOptions);
catch topologyOnlyError
    if isInfeasibleProjectionError(topologyOnlyError)
        rethrow(constrainedError);
    end
    rethrow(topologyOnlyError);
end
allowEmergency = logical(getField( ...
    policyOptions, 'allowEmergencyPayloadViolation', false));
if ~allowEmergency
    rethrow(constrainedError);
end
selection = topologyOnlySelection;
payloadEmergencyUsed = true;
end

function value = isInfeasibleProjectionError(errorInfo)
identifier = '';
try
    identifier = errorInfo.identifier;
catch
    % Octave supplies a struct and MATLAB supplies an MException.  If a
    % third-party runtime exposes neither shape, this remains a hard error.
end
value = strcmp(identifier, 'RollingMatching:Infeasible');
end

function repairScores = buildMinimumEditScores( ...
        endpointScores, receiverIndices, senderIndices, groupIds, ...
        proposalFormationAdjacency, groupCount)
groups = unique(groupIds, 'stable');
proposalMask = false(numel(receiverIndices), 1);
for exampleIdx = 1:numel(receiverIndices)
    senderGroup = find( ...
        groups == groupIds(senderIndices(exampleIdx)), 1);
    receiverGroup = find( ...
        groups == groupIds(receiverIndices(exampleIdx)), 1);
    proposalMask(exampleIdx) = ...
        proposalFormationAdjacency(senderGroup, receiverGroup);
end
finiteEndpoint = endpointScores(isfinite(endpointScores));
normalizedEndpoint = zeros(size(endpointScores));
if ~isempty(finiteEndpoint)
    endpointMinimum = min(finiteEndpoint);
    endpointRange = max(finiteEndpoint) - endpointMinimum;
    if endpointRange <= eps
        normalizedEndpoint(isfinite(endpointScores)) = 0;
    else
        normalizedEndpoint(isfinite(endpointScores)) = ...
            (endpointScores(isfinite(endpointScores)) - ...
                endpointMinimum) / endpointRange;
    end
end
% One retained proposal dominates every possible collection of replacement
% edges.  Conditional on the retained count, each added pair costs one;
% endpoint preference and lexical rank only break exact edit ties.
proposalBonus = groupCount + 1;
repairScores = ...
    proposalBonus * double(proposalMask) - ...
    double(~proposalMask) + ...
    1e-3 * normalizedEndpoint - ...
    1e-9 * reshape(1:numel(endpointScores), [], 1);
repairScores(~isfinite(endpointScores)) = -inf;
end

function scores = deterministicEndpointScores( ...
        groupIds, receiverIndices, senderIndices, ...
        currentTime, options)
groups = unique(groupIds, 'stable');
activationCounts = formationEdgeActivationCounts( ...
    numel(groups), currentTime, options);
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
    activationIndex = activationCounts( ...
        senderGroup, receiverGroup);
    % The receiver role advances fastest; the sender advances after every
    % complete receiver sweep.  Thus each directed formation pair covers
    % its full Cartesian endpoint support before repeating.
    targetReceiverRole = 1 + mod( ...
        activationIndex, numel(receiverMembers));
    targetSenderRole = 1 + mod( ...
        floor(activationIndex / numel(receiverMembers)), ...
        numel(senderMembers));
    scores(exampleIdx) = ...
        0.1 * (receiverRole == targetReceiverRole) + ...
        0.01 * (senderRole == targetSenderRole) - ...
        1e-8 * senderIdx - 1e-10 * receiverIdx;
end
end

function activationCounts = formationEdgeActivationCounts( ...
        groupCount, currentTime, options)
activationCounts = zeros(groupCount);
anchorTime = round(getField(options, 'anchorTime', 1));
for timeIdx = anchorTime:(currentTime - 1)
    [formationAdjacency, ~, ~] = ...
        buildRollingReserveSchedule( ...
            groupCount, timeIdx, options);
    activationCounts = activationCounts + ...
        double(formationAdjacency);
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
