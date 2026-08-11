function context = buildInfluenceConeCarrierV113ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDINFLUENCECONECARRIERV113EXECUTIONCONTEXT Frozen V113 context.

protocol = getInfluenceConeCarrierV113Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('InfluenceConeCarrierV113:InvalidExecutionContext', ...
        'The V113 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'influence-cone-carrier-v113-context-v1';
context.capability = 'influence-cone-carrier-v113-development';
context.action = 'filter-influence-cone-carrier-v113-development';
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
