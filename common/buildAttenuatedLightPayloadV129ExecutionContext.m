function context = buildAttenuatedLightPayloadV129ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDATTENUATEDLIGHTPAYLOADV129EXECUTIONCONTEXT Frozen V129 context.

protocol = getAttenuatedLightPayloadV129Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('AttenuatedLightPayloadV129:InvalidExecutionContext', ...
        'The V129 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'attenuated-light-payload-v129-context-v1';
context.capability = 'attenuated-light-payload-v129-development';
context.action = 'filter-attenuated-light-payload-v129-development';
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
