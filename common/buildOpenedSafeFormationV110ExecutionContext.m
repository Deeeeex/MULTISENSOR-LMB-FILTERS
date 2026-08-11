function context = buildOpenedSafeFormationV110ExecutionContext( ...
        presetName, seed, currentTime, actionName, scheduleEnabled)
% BUILDOPENEDSAFEFORMATIONV110EXECUTIONCONTEXT Frozen oracle context.

protocol = getOpenedSafeFormationV110Protocol();
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || ...
        ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(scheduleEnabled) || ...
        ~(islogical(scheduleEnabled) || isnumeric(scheduleEnabled))
    error('OpenedSafeFormationV110:InvalidExecutionContext', ...
        'The V110 execution-context input is invalid.');
end
context = struct();
context.contractVersion = 'opened-safe-formation-v110-context-v1';
context.capability = 'opened-safe-formation-v110-development';
context.action = 'filter-opened-safe-formation-v110-development';
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
