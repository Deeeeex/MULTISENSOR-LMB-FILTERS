function [adjacency, details] = ...
        selectEventWindowResidualReinforcementV258Policy(context)
% SELECTEVENTWINDOWRESIDUALREINFORCEMENTV258POLICY Reinforce one tail event.

timerId = tic;
protocol = getEventWindowResidualReinforcementV258Protocol();
active = context.currentTime >= protocol.activationWindow(1) && ...
    context.currentTime <= protocol.activationWindow(2);
if active
    options = struct( ...
        'allowedReceiverFormationIds', ...
            protocol.reinforcedReceiverFormationIds, ...
        'maximumTotalResiduals', protocol.maximumTotalResiduals);
    [adjacency, details] = ...
        selectParetoSafeResidualAugmentationV246Policy( ...
            context, options);
    sourcePolicy = 'v246-constrained-current-step-guard';
else
    [adjacency, details] = ...
        selectCausalMinimumFormationBackboneV242Policy(context);
    details.addedResidualAdjacency = false(size(adjacency));
    details.residualQuota = 0;
    details.allowedReceiverFormationIds = ...
        protocol.reinforcedReceiverFormationIds;
    details.selectedLocalReceivers = zeros(1, 0);
    details.selectedLocalSenders = zeros(1, 0);
    details.validCandidateCount = 0;
    details.selectedAddedResidualBytes = 0;
    details.addedResidualByteBudget = 0;
    details.taskAdvantage = 0;
    details.disagreementAdvantage = 0;
    details.protectedDownwardCrossingCount = 0;
    sourcePolicy = 'v242-minimum-backbone';
end

details.contractVersion = ...
    'event-window-residual-reinforcement-v258-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'posthoc-event-window-local-residual-reinforcement';
details.backboneMode = details.mode;
details.selectionSeconds = toc(timerId);
details.eventWindowActive = active;
details.activationWindow = protocol.activationWindow;
details.reinforcedReceiverFormationIds = ...
    protocol.reinforcedReceiverFormationIds;
details.maximumTotalResiduals = protocol.maximumTotalResiduals;
details.posthocScheduleUsed = true;
details.mechanismEvidenceOnly = true;
details.truthUsed = false;
details.futureOutcomeUsed = false;
details.trackingOutcomeScored = false;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'event-window-residual-reinforcement-v258-schedule-v1';
schedule.currentTime = context.currentTime;
schedule.phase = ternary(active, ...
    'v258-event-reinforcement', 'v258-minimum-backbone');
schedule.eventWindowActive = active;
schedule.activationWindow = protocol.activationWindow;
schedule.reinforcedReceiverFormationIds = ...
    protocol.reinforcedReceiverFormationIds;
schedule.maximumTotalResiduals = protocol.maximumTotalResiduals;
schedule.selectedResidualCount = nnz( ...
    details.addedResidualAdjacency);
schedule.selectedAddedResidualBytes = ...
    details.selectedAddedResidualBytes;
schedule.addedResidualByteBudget = ...
    details.addedResidualByteBudget;
schedule.currentStepParetoSafe = ...
    details.taskAdvantage >= -1e-10 && ...
    details.disagreementAdvantage >= -1e-10 && ...
    details.protectedDownwardCrossingCount == 0;
schedule.posthocScheduleUsed = true;
schedule.mechanismEvidenceOnly = true;
schedule.sourcePolicy = sourcePolicy;
details.scheduleCertificate = schedule;
end

function value = ternary(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end
