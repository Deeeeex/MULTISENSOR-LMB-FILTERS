function context = buildExplicitLabelAbstentionV109ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDEXPLICITLABELABSTENTIONV109EXECUTIONCONTEXT Frozen attribution.

protocol = getExplicitLabelAbstentionV109Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('ExplicitLabelAbstentionV109:InvalidExecutionContext', ...
        'The V109 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'explicit-label-abstention-v109-context-v1';
context.capability = 'explicit-label-abstention-v109-development';
context.action = 'filter-explicit-label-abstention-v109-development';
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
