function context = buildShadowStateRollbackV126ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDSHADOWSTATEROLLBACKV126EXECUTIONCONTEXT Frozen V126 context.

protocol = getShadowStateRollbackV126Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('ShadowStateRollbackV126:InvalidExecutionContext', ...
        'The V126 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'shadow-state-rollback-v126-context-v1';
context.capability = 'shadow-state-rollback-v126-development';
context.action = 'filter-shadow-state-rollback-v126-development';
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
