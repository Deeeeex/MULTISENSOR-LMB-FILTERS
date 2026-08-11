function context = buildProtectionOnlyH8V105ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDPROTECTIONONLYH8V105EXECUTIONCONTEXT Frozen H=8 attribution arm.

protocol = getProtectionOnlyH8V105Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('ProtectionOnlyV105:InvalidExecutionContext', ...
        'The V105 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'protection-only-h8-v105-context-v1';
context.capability = 'protection-only-h8-v105-development';
context.action = 'filter-protection-only-h8-v105-development';
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
