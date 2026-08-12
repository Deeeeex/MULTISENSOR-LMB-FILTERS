function context = buildLocalPosteriorRollbackV127ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDLOCALPOSTERIORROLLBACKV127EXECUTIONCONTEXT Frozen V127 context.

protocol = getLocalPosteriorRollbackV127Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('LocalPosteriorRollbackV127:InvalidExecutionContext', ...
        'The V127 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'local-posterior-rollback-v127-context-v1';
context.capability = 'local-posterior-rollback-v127-development';
context.action = 'filter-local-posterior-rollback-v127-development';
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
