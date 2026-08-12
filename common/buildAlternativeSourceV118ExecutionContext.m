function context = buildAlternativeSourceV118ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDALTERNATIVESOURCEV118EXECUTIONCONTEXT Frozen V118 context.

protocol = getAlternativeSourceV118Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('AlternativeSourceV118:InvalidExecutionContext', ...
        'The V118 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'alternative-source-v118-context-v1';
context.capability = 'alternative-source-v118-development';
context.action = 'filter-alternative-source-v118-development';
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
