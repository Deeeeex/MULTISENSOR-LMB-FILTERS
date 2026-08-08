function [adjacency, details] = ...
    buildFormationB4V52FixedRuntimeArm(context, armId)
% BUILDFORMATIONB4V52FIXEDRUNTIMEARM Time one complete V46 pulse per B4.

timerId = tic;
protocol = getFormationB4V52RuntimeProtocol();
if nargin ~= 2 || ~strcmp(armId, protocol.candidateArmId)
    error('FormationB4V52Runtime:InvalidArm', ...
        'Only the V52 counterfactual pulse-timing arm is accepted.');
end

routeContext = buildFormationB4V49GraphOnlyRouteContext(context);
parentContext = buildParentContext( ...
    context, routeContext, numel(context.localPosteriorBySensor));
[~, parent] = buildFormationB4V46FixedRuntimeArm( ...
    parentContext, protocol.parentRouteArmId);
dominant = logical(parent.dominantAdjacency);
residual = logical(parent.residualAdjacency);
nodeCount = size(dominant, 1);
phase = mod(context.currentTime - 1, protocol.period) + 1;
[pulseAlreadyUsed, pulseTimesInWindow, windowStart] = ...
    resolvePulseLedger(context, phase, protocol, nodeCount);

decision = emptyDecision();
pulseDecisionEvaluated = false;
pulseForced = false;
if context.currentTime == protocol.bootstrapTime
    pulseExecuted = true;
    pulseForced = true;
    decisionReason = 'bootstrap';
elseif pulseAlreadyUsed
    pulseExecuted = false;
    decisionReason = 'window-credit-already-used';
elseif phase == protocol.hardDeadlinePhase
    pulseExecuted = true;
    pulseForced = true;
    decisionReason = 'hard-deadline';
elseif any(phase == protocol.decisionPhases)
    decision = ...
        selectFormationB4V52CounterfactualPulseDecision( ...
            context, dominant, residual, protocol);
    pulseDecisionEvaluated = true;
    pulseExecuted = decision.serve;
    if pulseExecuted
        decisionReason = 'counterfactual-serve';
    else
        decisionReason = 'counterfactual-hold';
    end
else
    error('FormationB4V52Runtime:InvalidPhase', ...
        'The V52 service phase is not registered.');
end

activeResidual = false(size(residual));
if pulseExecuted
    activeResidual = residual;
end
adjacency = dominant | activeResidual;
weights = buildWeights(dominant, activeResidual, protocol);
if any(adjacency(:) & ~logical(context.physicalAdjacency(:))) || ...
        nnz(dominant) ~= nodeCount || nnz(residual) ~= nodeCount || ...
        nnz(adjacency) ~= nodeCount * (1 + pulseExecuted) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12) || ...
        any(weights(:) < -1e-12)
    error('FormationB4V52Runtime:InvalidAction', ...
        'The V52 page violates its physical, budget or weight rule.');
end

schedule = struct();
schedule.contractVersion = ...
    'formation-b4-v52-runtime-schedule-v1';
schedule.currentTime = context.currentTime;
schedule.currentAbsolutePhase = phase;
schedule.period = protocol.period;
schedule.windowStartTime = windowStart;
schedule.pulseTimesPreviouslyExecutedInWindow = ...
    pulseTimesInWindow;
schedule.pulseAlreadyUsed = pulseAlreadyUsed;
schedule.pulseExecuted = pulseExecuted;
schedule.pulseForced = pulseForced;
schedule.pulseDecisionEvaluated = pulseDecisionEvaluated;
schedule.pulseDecisionReason = decisionReason;
schedule.counterfactualDecision = decision;
schedule.currentScheduledDirectedEdgeCount = nnz(adjacency);
schedule.cycleSelectionPerformed = false;
schedule.cycleSelected = false;
schedule.referenceFallbackUsed = false;
schedule.fallbackReason = decisionReason;
schedule.selectedPosteriorObjective = 0;
if pulseDecisionEvaluated
    schedule.selectedPosteriorObjective = ...
        decision.disagreementImprovementFraction;
end
schedule.truthUsed = false;
schedule.futureOutcomeUsed = false;

details = parent;
% Pulse timing changes the parent page and its weights. The installed
% adjacency and row-stochastic weights are checked directly by the filter.
details.routeAndWeightCanonicalSha256 = '';
details.contractVersion = ...
    'formation-b4-v52-fixed-runtime-policy-v1';
details.mode = ...
    'formation-b4-v52-counterfactual-pulse-timing-runtime';
details.armId = protocol.candidateArmId;
details.actionName = protocol.candidateArmId;
details.protocolId = protocol.id;
details.fusionWeightMatrix = weights;
details.dominantAdjacency = dominant;
details.residualAdjacency = activeResidual;
details.referenceAdjacency = dominant | residual;
details.scheduleCertificate = schedule;
details.currentScheduledDirectedEdgeCount = nnz(adjacency);
details.currentMessageCount = nnz(adjacency);
details.referenceMessageCount = 2 * nodeCount;
details.scheduledDirectedEdgeSavingFraction = ...
    (2 * nodeCount - nnz(adjacency)) / (2 * nodeCount);
details.messageSavingFraction = ...
    details.scheduledDirectedEdgeSavingFraction;
details.sameScheduledDirectedEdgeCountVsV46 = ...
    nnz(adjacency) == 2 * nodeCount;
details.sameOrLowerScheduledMessageCountVsV46 = true;
details.pulseTimingDecision = decision;
details.selectionSeconds = toc(timerId);
end

function [used, pulseTimes, windowStart] = ...
    resolvePulseLedger(context, phase, protocol, nodeCount)
windowStart = context.currentTime - phase + 1;
pulseTimes = zeros(1, 0);
if ~isfield(context, 'previousAdjacencyHistory') || ...
        ~isfield(context, 'previousAdjacencyHistoryTimes')
    used = false;
    return;
end
times = reshape(context.previousAdjacencyHistoryTimes, 1, []);
history = logical(context.previousAdjacencyHistory);
for idx = 1:numel(times)
    if times(idx) < windowStart || times(idx) >= context.currentTime
        continue;
    end
    count = nnz(history(:, :, idx));
    if count == 2 * nodeCount
        pulseTimes(end + 1) = times(idx); %#ok<AGROW>
    elseif count ~= nodeCount
        error('FormationB4V52Runtime:InvalidHistory', ...
            'A previous V52 page has an invalid message count.');
    end
end
if numel(pulseTimes) > 1
    error('FormationB4V52Runtime:DuplicatePulse', ...
        'More than one complete pulse was used in a B4 window.');
end
used = ~isempty(pulseTimes);
end

function weights = buildWeights(dominant, residual, protocol)
nodeCount = size(dominant, 1);
weights = zeros(nodeCount);
weights(dominant) = protocol.dominantWeight;
weights(residual) = protocol.activeResidualWeight;
weights(1:nodeCount+1:end) = 1 - sum(weights, 2)';
end

function decision = emptyDecision()
decision = struct( ...
    'contractVersion', ...
        'formation-b4-v52-counterfactual-pulse-decision-empty-v1', ...
    'serve', false, ...
    'disagreementImprovementFraction', NaN, ...
    'cardinalityGainFraction', NaN, ...
    'retentionRisk', NaN, ...
    'retentionSafe', false, ...
    'disagreementUseful', false, ...
    'cardinalityUseful', false, ...
    'truthUsed', false, ...
    'futureOutcomeUsed', false);
end

function contextOut = buildParentContext( ...
        context, routeContext, nodeCount)
contextOut = struct();
contextOut.localPosteriorBySensor = cell(1, nodeCount);
contextOut.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', routeContext.sensorGroupIds)));
contextOut.baseAdjacency = logical(context.baseAdjacency);
contextOut.physicalAdjacency = logical(routeContext.physicalAdjacency);
contextOut.positions = routeContext.positions;
contextOut.sensorPhysicalUids = routeContext.sensorPhysicalUids;
contextOut.formationPhysicalUidsBySensor = ...
    routeContext.formationPhysicalUidsBySensor;
contextOut.physicalIdentityRegistryCanonicalSha256 = ...
    context.physicalIdentityRegistryCanonicalSha256;
contextOut.commConfig = struct('pDropByEdge', routeContext.pDropByEdge);
contextOut.currentTime = routeContext.currentTime;
contextOut.directedMessageBudget = 2 * nodeCount;
contextOut.triggerConfig = struct( ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount);
contextOut.observableInputContract = context.observableInputContract;
end
