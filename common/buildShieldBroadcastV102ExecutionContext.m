function context = buildShieldBroadcastV102ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDSHIELDBROADCASTV102EXECUTIONCONTEXT Frozen H=6 composition.

protocol = getShieldBroadcastV102Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('ShieldBroadcastV102:InvalidExecutionContext', ...
        'The V102 execution-context input is invalid.');
end

context = struct();
context.contractVersion = 'shield-broadcast-v102-context-v1';
context.capability = 'shield-broadcast-v102-development';
context.action = 'filter-shield-broadcast-v102-development';
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
