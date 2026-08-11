function context = buildMaturedHandoffV103ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDMATUREDHANDOFFV103EXECUTIONCONTEXT Frozen H=8 causal handoff.

protocol = getMaturedHandoffV103Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('MaturedHandoffV103:InvalidExecutionContext', ...
        'The V103 execution-context input is invalid.');
end

context = struct();
context.contractVersion = 'matured-handoff-v103-context-v1';
context.capability = 'matured-handoff-v103-development';
context.action = 'filter-matured-handoff-v103-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.measurementTimeCount = ...
    currentTime + protocol.horizonSteps - 1;
context.policyName = protocol.outcomePolicyName;
context.actionName = actionName;
context.scheduleEnabled = logical(scheduleEnabled);
context.onlineReselectionEnabled = false;
context.developmentEvidenceOnly = true;
end
