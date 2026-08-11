function context = buildReceiverSelectiveHandoffV104ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDRECEIVERSELECTIVEHANDOFFV104EXECUTIONCONTEXT Oracle H=8 handoff.

protocol = getReceiverSelectiveHandoffV104Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('ReceiverSelectiveV104:InvalidExecutionContext', ...
        'The V104 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'receiver-selective-v104-context-v1';
context.capability = 'receiver-selective-v104-development';
context.action = 'filter-receiver-selective-v104-development';
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
